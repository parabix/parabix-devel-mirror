#include "zeroextend.h"
#include <kernels/kernel_builder.h>
#include <kernels/streamset.h>
#include <llvm/Support/raw_ostream.h>

using namespace llvm;

namespace kernel {

inline bool notProperFactorOf(const unsigned n, const unsigned m) {
    return ((n % m) != n) || (n == 1) || (n >= m);
}

inline static bool is_power_2(const unsigned n) {
    return ((n & (n - 1)) == 0) && n;
}

void ZeroExtend::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, Value * const numOfStrides) {

    const Binding & input = getInputStreamSetBinding(0);
    const auto inputFieldWidth = input.getFieldWidth();

    // TODO: support for 1,2,4 bit field widths will require specialized logic.

    // TODO: we cannot assume aligned I/O streams when handling more than one stream in a set

    if (LLVM_UNLIKELY(inputFieldWidth < 8 || !is_power_2(inputFieldWidth))) {
        report_fatal_error("ZeroExtend: input field width "
                           "must be a power of 2 greater than 4");
    }

    const Binding & output = getOutputStreamSetBinding(0);
    const auto outputFieldWidth = output.getFieldWidth();

    if (LLVM_UNLIKELY(notProperFactorOf(inputFieldWidth, outputFieldWidth))) {
        report_fatal_error("ZeroExtend: input field width "
                           "must be a proper factor of "
                           "output field width");
    }

    const auto blockWidth = b->getBitBlockWidth();

    if (LLVM_UNLIKELY(notProperFactorOf(outputFieldWidth, blockWidth))) {
        report_fatal_error("ZeroExtend: output field width "
                           "must be a proper factor of "
                           "block width");
    }

    if (LLVM_UNLIKELY(input.getNumElements() != 1 || output.getNumElements() != 1)) {
        report_fatal_error("ZeroExtend: currently only supports "
                           "single stream I/O");
    }

    const auto inputVectorSize = (blockWidth / inputFieldWidth); assert (is_power_2(inputVectorSize));
    const auto outputVectorWidth = (blockWidth / outputFieldWidth); assert (is_power_2(outputVectorWidth));

    IntegerType * const sizeTy = b->getSizeTy();

    Value * const ZERO = b->getSize(0);

    VectorType * const inputTy = VectorType::get(b->getIntNTy(inputFieldWidth), inputVectorSize);
    PointerType * const inputPtrTy = inputTy->getPointerTo();

    Value * const processed = b->getProcessedItemCount(input.getName());
    Value * const baseInputPtr = b->getRawInputPointer(input.getName(), processed);

    Value * const produced = b->getProducedItemCount(output.getName());
    Value * const baseOutputPtr = b->getRawOutputPointer(output.getName(), produced);

    BasicBlock * const entry = b->GetInsertBlock();
    BasicBlock * const loop = b->CreateBasicBlock("Loop");
    b->CreateBr(loop);

    b->SetInsertPoint(loop);
    PHINode * const index = b->CreatePHI(sizeTy, 2);
    index->addIncoming(ZERO, entry);

    std::vector<Value *> inputBuffer(inputFieldWidth);
    Constant * const log2InputSize = b->getSize(std::log2<unsigned>(inputFieldWidth));

    // read the values from the input stream
    for (unsigned i = 0; i < inputFieldWidth; ++i) {
        Value * offset = b->CreateShl(index, log2InputSize);
        offset = b->CreateAdd(offset, b->getSize(i * inputVectorSize));
        Value * ptr = b->CreateGEP(baseInputPtr, offset);
        ptr = b->CreatePointerCast(ptr, inputPtrTy);
        inputBuffer[i] = b->CreateAlignedLoad(ptr, (inputFieldWidth / CHAR_BIT));
    }

    std::vector<Value *> outputBuffer(inputFieldWidth * 2);

    std::vector<Constant *> lowerHalf(inputVectorSize);
    std::vector<Constant *> upperHalf(inputVectorSize);

    // expand by doubling repeatidly until we've reached the desired output size
    for (;;) {

        VectorType * const inputTy = cast<VectorType>(inputBuffer[0]->getType());

        const auto n = inputTy->getVectorElementType()->getIntegerBitWidth();
        const auto count = blockWidth / n;

        const auto halfCount = (count / 2);

        for (unsigned i = 0; i < halfCount; ++i) {
            lowerHalf[i * 2] = b->getInt32(i);
            lowerHalf[(i * 2) + 1] = b->getInt32(count + i);
        }
        Constant * const LOWER_MASK = ConstantVector::get(lowerHalf);

        for (unsigned i = 0; i < halfCount; ++i) {
            upperHalf[i * 2] = b->getInt32(halfCount + i);
            upperHalf[(i * 2) + 1] = b->getInt32(count + halfCount + i);
        }
        Constant * const UPPER_MASK = ConstantVector::get(upperHalf);

        VectorType * const outputTy = VectorType::get(b->getIntNTy(n * 2), halfCount);

        Constant * const ZEROES = ConstantVector::getNullValue(inputTy);
        for (unsigned i = 0; i < inputBuffer.size(); ++i) {
            Value * const lower = b->CreateShuffleVector(ZEROES, inputBuffer[i], LOWER_MASK);
            outputBuffer[i * 2] = b->CreateBitCast(lower, outputTy);
            Value * const upper = b->CreateShuffleVector(ZEROES, inputBuffer[i], UPPER_MASK);
            outputBuffer[(i * 2) + 1] = b->CreateBitCast(upper, outputTy);
        }

        if (LLVM_LIKELY(outputBuffer.size() == outputFieldWidth)) {
            break;
        }

        inputBuffer.swap(outputBuffer);
        outputBuffer.resize(inputBuffer.size() * 2);
        lowerHalf.resize(halfCount);
        upperHalf.resize(halfCount);
    }

    // write the values to the output stream
    VectorType * const outputTy = cast<VectorType>(outputBuffer[0]->getType());
    PointerType * const outputPtrTy = outputTy->getPointerTo();
    Constant * const log2OutputSize = b->getSize(std::log2<unsigned>(outputFieldWidth));
    for (unsigned i = 0; i < outputFieldWidth; ++i) {
        Value * offset = b->CreateShl(index, log2OutputSize);
        offset = b->CreateAdd(offset, b->getSize(i * outputVectorWidth));
        Value * ptr = b->CreateGEP(baseOutputPtr, offset);
        ptr = b->CreatePointerCast(ptr, outputPtrTy);
        b->CreateAlignedStore(outputBuffer[i], ptr, (outputFieldWidth / CHAR_BIT));
    }

    // loop until done
    BasicBlock * const exit = b->CreateBasicBlock("exit");
    Value * const nextIndex = b->CreateAdd(index, b->getSize(1));
    Value * const notDone = b->CreateICmpNE(nextIndex, numOfStrides);
    index->addIncoming(nextIndex, b->GetInsertBlock());
    b->CreateLikelyCondBr(notDone, loop, exit);
    b->SetInsertPoint(exit);
}

ZeroExtend::ZeroExtend(const std::unique_ptr<kernel::KernelBuilder> & b,
                       StreamSet * const input, StreamSet * const output)
: MultiBlockKernel(b, "zeroextend" + std::to_string(input->getFieldWidth()) + "x" + std::to_string(output->getFieldWidth()),
{Binding{"input", input}},
{Binding{"output", output}},
{}, {}, {}) {
    assert (input->getNumElements() == output->getNumElements());
}

}
