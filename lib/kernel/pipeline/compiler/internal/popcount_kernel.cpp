#include "popcount_kernel.h"

#include <kernel/core/kernel_builder.h>

using namespace llvm;

namespace kernel {

const std::string INPUT = "input";

const std::string OUTPUT_STREAM = "output";
const std::string POSITIVE_STREAM = "positive";
const std::string NEGATIVE_STREAM = "negative";

const std::string CURRENT_COUNT = "currentCount";
const std::string POSITIVE_COUNT = "positiveCount";
const std::string NEGATIVE_COUNT = "negativeCount";

using Rational = ProcessingRate::Rational;

bool isNotConstantOne(Value * const value) {
    return !isa<Constant>(value) || !cast<Constant>(value)->isOneValue();
}

inline static unsigned floor_log2(const unsigned v) {
    assert ("log2(0) is undefined!" && v != 0);
    return ((sizeof(unsigned) * CHAR_BIT) - 1U) - __builtin_clz(v);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateMultiBlockLogic
 ** ------------------------------------------------------------------------------------------------------------- */
void PopCountKernel::generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfStrides) {


    Constant * const ZERO = b->getSize(0);
    Constant * const ONE = b->getSize(1);
    Constant * const NEG_ONE = ConstantExpr::getNeg(ONE);
    IntegerType * const sizeTy = b->getSizeTy();

    const Binding & input = b->getInputStreamSetBinding(INPUT);
    const ProcessingRate & rate = input.getRate();
    const Rational & rv = rate.getRate();
    assert (rv.denominator() == 1);
    const auto inputWidth = rv.numerator();
    const auto blockWidth = b->getBitBlockWidth();
    const auto sizeWidth = sizeTy->getBitWidth();

    if (isNotConstantOne(b->getInputStreamSetCount(INPUT))) {
        report_fatal_error("PopCount input stream must be a single stream");
    }

    Value * position = nullptr;
    if (LLVM_LIKELY(mType != PopCountType::BOTH)) {
        position = b->getProducedItemCount(OUTPUT_STREAM);
    } else {
        position = b->getProducedItemCount(POSITIVE_STREAM);
    }

    // TODO: load the initial counts from a lookbehind
    Value * positiveArray = nullptr;
    Value * initialPositiveCount = nullptr;
    Value * negativeArray = nullptr;
    Value * initialNegativeCount = nullptr;

    if (LLVM_LIKELY(mType == PopCountType::POSITIVE || mType == PopCountType::NEGATIVE)) {
        Value * const array = b->getRawOutputPointer(OUTPUT_STREAM, position);
        Value * const count = b->CreateLoad(b->CreateInBoundsGEP(array, NEG_ONE));
        if (LLVM_LIKELY(mType == PopCountType::POSITIVE)) {
            positiveArray = array;
            initialPositiveCount = count;
        } else { // if (mType == PopCountType::NEGATIVE) {
            negativeArray = array;
            initialNegativeCount = count;
        }
    } else { // if (mType == PopCountType::BOTH) {
        positiveArray = b->getRawOutputPointer(POSITIVE_STREAM, position);
        initialPositiveCount = b->CreateLoad(b->CreateInBoundsGEP(positiveArray, NEG_ONE));
        negativeArray = b->getRawOutputPointer(NEGATIVE_STREAM, position);
        initialNegativeCount = b->CreateLoad(b->CreateInBoundsGEP(negativeArray, NEG_ONE));
    }

    BasicBlock * const entry = b->GetInsertBlock();
    BasicBlock * const popCountLoop = b->CreateBasicBlock("Loop");
    BasicBlock * const popCountExit = b->CreateBasicBlock("Exit");
    b->CreateBr(popCountLoop);

    b->SetInsertPoint(popCountLoop);
    PHINode * const index = b->CreatePHI(sizeTy, 2);
    index->addIncoming(ZERO, entry);
    PHINode * positiveSum = nullptr;
    if (positiveArray) {
        positiveSum = b->CreatePHI(sizeTy, 2);
        positiveSum->addIncoming(initialPositiveCount, entry);
    }
    PHINode * negativeSum = nullptr;
    if (negativeArray) {
        negativeSum = b->CreatePHI(sizeTy, 2);
        negativeSum->addIncoming(initialNegativeCount, entry);
    }

    Value * sum = nullptr;
    if (LLVM_UNLIKELY(inputWidth < blockWidth)) {
        report_fatal_error("popcount input width < block width is not supported yet.");
    } else {

        const auto step = inputWidth / blockWidth;
        assert (inputWidth % blockWidth == 0);
        Constant * const STEP = b->getSize(step);
        Value * const baseIndex = b->CreateMul(index, STEP);

        Value * partialSumVector = nullptr;
        // half adder will require the same number of pop count steps when n < 4
        if (step < 4) {
            for (unsigned i = 0; i < step; ++i) {
                Constant * const I = b->getSize(i);
                Value * const idx = b->CreateAdd(baseIndex, I);
                Value * value = b->loadInputStreamBlock(INPUT, ZERO, idx);
                if (LLVM_UNLIKELY(positiveSum == nullptr)) { // only negative count
                    value = b->CreateNot(value);
                }
                Value * const count = b->simd_popcount(sizeWidth, value);
                if (i == 0) {
                    partialSumVector = count;
                } else {
                    partialSumVector = b->CreateAdd(partialSumVector, count);
                }
            }
        } else {
            const auto m = floor_log2(step + 1) + 1;
            SmallVector<Value *, 64> adders(m);
            // load the first block
            Value * value = b->loadInputStreamBlock(INPUT, ZERO, baseIndex);
            if (LLVM_UNLIKELY(positiveSum == nullptr)) { // only negative count
                value = b->CreateNot(value);
            }
            adders[0] = value;
            // load and half-add the subsequent blocks
            for (unsigned i = 1; i < step; ++i) {
                Constant * const I = b->getSize(i);
                Value * const idx = b->CreateAdd(baseIndex, I);
                Value * value = b->loadInputStreamBlock(INPUT, ZERO, idx);
                if (LLVM_UNLIKELY(positiveSum == nullptr)) { // only negative count
                    value = b->CreateNot(value);
                }
                const auto k = floor_log2(i);
                for (unsigned j = 0; j <= k; ++j) {
                    Value * const sum_in = adders[j]; assert (sum_in);
                    Value * const sum_out = b->simd_xor(sum_in, value);
                    Value * const carry_out = b->simd_and(sum_in, value);
                    adders[j] = sum_out;
                    value = carry_out;
                }
                const auto l = floor_log2(i + 1);
                if (k < l) {
                    adders[l] = value;
                }
            }
            // sum the half adders
            for (unsigned i = 0; i < m; ++i) {
                Value * const count = b->simd_popcount(sizeWidth, adders[i]);
                if (i == 0) {
                    partialSumVector = count;
                } else {
                    partialSumVector = b->CreateAdd(partialSumVector, b->CreateShl(count, i));
                }
            }
        }

        const auto field = blockWidth / sizeWidth;
        assert (blockWidth % sizeWidth == 0);
        Value * const partialSum = b->hsimd_partial_sum(sizeWidth, partialSumVector);
        sum = b->mvmd_extract(sizeWidth, partialSum, field - 1);
    }

    Value * positivePartialSum = nullptr;
    if (positiveArray) {
        positivePartialSum = b->CreateAdd(positiveSum, sum);
        positiveSum->addIncoming(positivePartialSum, popCountLoop);
        Value * const ptr = b->CreateInBoundsGEP(positiveArray, index);
        b->CreateStore(positivePartialSum, ptr);

        b->CreateDprintfCall(b->getInt32(STDERR_FILENO),
                             "> pop[%" PRIu64 "] = %" PRIu64 " (0x%" PRIx64 ")\n",
                             b->CreateAdd(position, index), positivePartialSum, ptr);
    }

    Value * negativePartialSum = nullptr;
    if (negativeArray) {
        Value * negSum = sum;
        if (positiveArray) {
            Constant * const INPUT_WIDTH = b->getSize(inputWidth);
            negSum = b->CreateSub(INPUT_WIDTH, sum);
        }
        negativePartialSum = b->CreateAdd(negativeSum, negSum);
        negativeSum->addIncoming(negativePartialSum, popCountLoop);
        Value * const ptr = b->CreateInBoundsGEP(negativeArray, index);
        b->CreateStore(negativePartialSum, ptr);
    }

    BasicBlock * const popCountLoopEnd = b->GetInsertBlock();
    Value * const nextIndex = b->CreateAdd(index, ONE);
    index->addIncoming(nextIndex, popCountLoopEnd);
    Value * const done = b->CreateICmpNE(nextIndex, numOfStrides);
    b->CreateCondBr(done, popCountLoop, popCountExit);

    b->SetInsertPoint(popCountExit);
}

// TODO: is a lookbehind window of 1 sufficient here?

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructor
 ** ------------------------------------------------------------------------------------------------------------- */
PopCountKernel::PopCountKernel(BuilderRef b, const PopCountType type, const unsigned stepFactor, StreamSet * input, StreamSet * const output)
: MultiBlockKernel(b, TypeId::PopCountKernel, "PopCount" + std::string{type == PopCountType::POSITIVE ? "P" : "N"} + std::to_string(stepFactor)
// input streams
,{Binding{INPUT, input, FixedRate(stepFactor), Add1() }}
// output stream
,{Binding{OUTPUT_STREAM, output, FixedRate(), LookBehind(1) }}
// unnused I/O scalars
,{} ,{},
// internal scalar
{})
, mType(type) {
    // a block of input becomes a single integer of output
    setStride(1);
    assert (type != PopCountType::BOTH);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructor
 ** ------------------------------------------------------------------------------------------------------------- */
PopCountKernel::PopCountKernel(BuilderRef b, const PopCountType type, const unsigned stepFactor, StreamSet * input, StreamSet * const positive, StreamSet * const negative)
: MultiBlockKernel(b, TypeId::PopCountKernel, ".PopCountB" + std::to_string(stepFactor)
// input streams
,{Binding{INPUT, input, FixedRate(stepFactor), Add1() }}
// output stream
,{Binding{POSITIVE_STREAM, positive, FixedRate(), LookBehind(1) }
 ,Binding{NEGATIVE_STREAM, negative, FixedRate(), LookBehind(1) }}
// unnused I/O scalars
,{} ,{},
// internal scalar
{})
, mType(type) {
    // a block of input becomes a single integer of output
    setStride(1);
    assert (type == PopCountType::BOTH);
}

}
