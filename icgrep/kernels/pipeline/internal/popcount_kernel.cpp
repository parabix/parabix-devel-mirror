#include "popcount_kernel.h"
#include <kernels/kernel_builder.h>

using namespace llvm;

namespace kernel {

const std::string INPUT = "input";

const std::string OUTPUT_STREAM = "output";
const std::string POSITIVE_STREAM = "positive";
const std::string NEGATIVE_STREAM = "negative";

const std::string CURRENT_COUNT = "currentCount";
const std::string POSITIVE_COUNT = "positiveCount";
const std::string NEGATIVE_COUNT = "negativeCount";

using RateValue = ProcessingRate::RateValue;

bool isNotConstantOne(Value * const value) {
    return !isa<Constant>(value) || !cast<Constant>(value)->isOneValue();
}

inline static unsigned floor_log2(const unsigned v) {
    assert ("log2(0) is undefined!" && v != 0);
    return ((sizeof(unsigned) * CHAR_BIT) - 1U) - __builtin_clz(v);
}

inline static unsigned ceil_log2(const unsigned v) {
    assert ("log2(0) is undefined!" && v != 0);
    return (sizeof(unsigned) * CHAR_BIT) - __builtin_clz(v - 1U);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateMultiBlockLogic
 ** ------------------------------------------------------------------------------------------------------------- */
void PopCountKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, llvm::Value * const numOfStrides) {


    Constant * const ZERO = b->getSize(0);
    Constant * const ONE = b->getSize(1);
    IntegerType * const sizeTy = b->getSizeTy();

    const Binding & input = getInputStreamSetBinding(INPUT);
    const ProcessingRate & rate = input.getRate();
    const RateValue & rv = rate.getRate();
    assert (rv.denominator() == 1);
    const auto inputWidth = rv.numerator();
    const auto blockWidth = b->getBitBlockWidth();

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
        Value * const count = b->getScalarField(CURRENT_COUNT);
        if (LLVM_LIKELY(mType == PopCountType::POSITIVE)) {
            positiveArray = array;
            initialPositiveCount = count;
        } else { // if (mType == PopCountType::NEGATIVE) {
            negativeArray = array;
            initialNegativeCount = count;
        }
    } else { // if (mType == PopCountType::BOTH) {
        positiveArray = b->getRawOutputPointer(POSITIVE_STREAM, position);
        initialPositiveCount = b->getScalarField(POSITIVE_COUNT);
        negativeArray = b->getRawOutputPointer(NEGATIVE_STREAM, position);
        initialNegativeCount = b->getScalarField(NEGATIVE_COUNT);
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

        // half adder will require the same number of pop count steps when n < 4
        if (step < 4) {
            for (unsigned i = 0; i < step; ++i) {
                Constant * const I = b->getSize(i);
                Value * const idx = b->CreateOr(baseIndex, I);
                Value * value = b->loadInputStreamBlock(INPUT, ZERO, idx);
                if (LLVM_UNLIKELY(positiveSum == nullptr)) { // only negative count
                    value = b->CreateNot(value);
                }
                Value * const count = b->CreateZExtOrTrunc(b->bitblock_popcount(value), sizeTy);
                if (i == 0) {
                    sum = count;
                } else {
                    sum = b->CreateAdd(sum, count);
                }
            }
        } else {
            const auto m = ceil_log2(step + 2);
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
                Value * const idx = b->CreateOr(baseIndex, I);
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
                adders[l] = value;
            }
            // sum the half adders
            for (unsigned i = 0; i < m; ++i) {
                Value * const count = b->CreateZExtOrTrunc(b->bitblock_popcount(adders[i]), sizeTy);
                if (i == 0) {
                    sum = count;
                } else {
                    sum = b->CreateAdd(sum, b->CreateShl(count, i));
                }
            }
        }
    }

    Value * positivePartialSum = nullptr;
    if (positiveArray) {
        positivePartialSum = b->CreateAdd(positiveSum, sum);
        positiveSum->addIncoming(positivePartialSum, popCountLoop);
        Value * const ptr = b->CreateGEP(positiveArray, index);
        b->CreateStore(positivePartialSum, ptr);
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
        Value * const ptr = b->CreateGEP(negativeArray, index);
        b->CreateStore(negativePartialSum, ptr);
    }

    BasicBlock * const popCountLoopEnd = b->GetInsertBlock();
    Value * const nextIndex = b->CreateAdd(index, ONE);
    index->addIncoming(nextIndex, popCountLoopEnd);
    Value * const done = b->CreateICmpNE(nextIndex, numOfStrides);
    b->CreateCondBr(done, popCountLoop, popCountExit);

    b->SetInsertPoint(popCountExit);
    if (LLVM_LIKELY(mType == PopCountType::POSITIVE || mType == PopCountType::NEGATIVE)) {
        Value * const count = (mType == PopCountType::POSITIVE) ? positivePartialSum : negativePartialSum;
        b->setScalarField(CURRENT_COUNT, count);
    } else { // if (mType == PopCountType::BOTH) {
        b->setScalarField(POSITIVE_COUNT, positivePartialSum);
        b->setScalarField(NEGATIVE_COUNT, negativePartialSum);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructor
 ** ------------------------------------------------------------------------------------------------------------- */
PopCountKernel::PopCountKernel(const std::unique_ptr<kernel::KernelBuilder> & b, const PopCountType type, const unsigned stepFactor, StreamSet * input, StreamSet * const output)
    : MultiBlockKernel(b, "PopCount" + std::string{type == PopCountType::POSITIVE ? "P" : "N"} + std::to_string(stepFactor)
// input streams
,{Binding{INPUT, input, FixedRate(stepFactor)}}
// output stream
,{Binding{OUTPUT_STREAM, output, FixedRate()}}
// unnused I/O scalars
,{} ,{},
// internal scalar
{InternalScalar{b->getSizeTy(), CURRENT_COUNT}})
, mType(type) {
    // a block of input becomes a single integer of output
    setStride(1);
    assert (type != PopCountType::BOTH);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructor
 ** ------------------------------------------------------------------------------------------------------------- */
PopCountKernel::PopCountKernel(const std::unique_ptr<kernel::KernelBuilder> & b, const PopCountType type, const unsigned stepFactor, StreamSet * input, StreamSet * const positive, StreamSet * const negative)
: MultiBlockKernel(b, ".PopCountB" + std::to_string(stepFactor)
// input streams
,{Binding{INPUT, input, FixedRate(stepFactor)}}
// output stream
,{Binding{POSITIVE_STREAM, positive, FixedRate()}
 ,Binding{NEGATIVE_STREAM, negative, FixedRate()}}
// unnused I/O scalars
,{} ,{},
// internal scalar
{InternalScalar{b->getSizeTy(), POSITIVE_COUNT}
,InternalScalar{b->getSizeTy(), NEGATIVE_COUNT}})
, mType(type) {
    // a block of input becomes a single integer of output
    setStride(1);
    assert (type == PopCountType::BOTH);
}

}
