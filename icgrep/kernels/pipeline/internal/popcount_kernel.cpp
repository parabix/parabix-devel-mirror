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

bool isNotConstantOne(Value * const value) {
    return !isa<Constant>(value) || !cast<Constant>(value)->isOneValue();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateMultiBlockLogic
 ** ------------------------------------------------------------------------------------------------------------- */
void PopCountKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, llvm::Value * const numOfStrides) {


    BasicBlock * const entry = b->GetInsertBlock();
    BasicBlock * const popCountLoop = b->CreateBasicBlock("Loop");
    BasicBlock * const popCountExit = b->CreateBasicBlock("Exit");

    Constant * const ZERO = b->getSize(0);
    Constant * const ONE = b->getSize(1);
    IntegerType * const sizeTy = b->getSizeTy();

    if (isNotConstantOne(b->getInputStreamSetCount(INPUT))) {
        report_fatal_error("PopCount input stream must be a single stream");
    }

    Value * initialCount = nullptr;
    if (LLVM_LIKELY(mType != PopCountType::BOTH)) {
        initialCount = b->getProducedItemCount(OUTPUT_STREAM);
    } else {
        initialCount = b->getProducedItemCount(POSITIVE_STREAM);
    }

    // TODO: load the initial counts from a lookbehind

    Value * positiveArray = nullptr;
    Value * initialPositiveCount = nullptr;
    Value * negativeArray = nullptr;
    Value * initialNegativeCount = nullptr;

    if (LLVM_LIKELY(mType == PopCountType::POSITIVE || mType == PopCountType::NEGATIVE)) {
        Value * const array = b->getRawOutputPointer(OUTPUT_STREAM, initialCount);
        Value * const count = b->getScalarField(CURRENT_COUNT);
        if (LLVM_LIKELY(mType == PopCountType::POSITIVE)) {
            positiveArray = array;
            initialPositiveCount = count;
        } else { // if (mType == PopCountType::NEGATIVE) {
            negativeArray = array;
            initialNegativeCount = count;
        }
    } else { // if (mType == PopCountType::BOTH) {
        positiveArray = b->getRawOutputPointer(POSITIVE_STREAM, initialCount);
        initialPositiveCount = b->getScalarField(POSITIVE_COUNT);
        negativeArray = b->getRawOutputPointer(NEGATIVE_STREAM, initialCount);
        initialNegativeCount = b->getScalarField(NEGATIVE_COUNT);
    }

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
    Value * value = b->loadInputStreamBlock(INPUT, ZERO, index);
    if (LLVM_UNLIKELY(positiveSum == nullptr)) { // only negative count
        value = b->CreateNot(value);
    }

    // TODO: parallelize the partial sum when we're reasonably sure that we'll
    // have enough data to be worth it on average. Ideally, we'd also want
    // to know whether we'd ever need to finish counting sequentially.

    Value * const count = b->CreateZExtOrTrunc(b->bitblock_popcount(value), sizeTy);
    Value * positivePartialSum = nullptr;
    if (positiveArray) {
        positivePartialSum = b->CreateAdd(positiveSum, count);
        positiveSum->addIncoming(positivePartialSum, popCountLoop);
        Value * const ptr = b->CreateGEP(positiveArray, index);
        b->CreateStore(positivePartialSum, ptr);
    }

    Value * negativePartialSum = nullptr;
    if (negativeArray) {
        Value * negCount = count;
        if (positiveArray) {
            Constant * blockWidth = b->getSize(b->getBitBlockWidth());
            negCount = b->CreateSub(blockWidth, count);
        }
        negativePartialSum = b->CreateAdd(negativeSum, negCount);
        negativeSum->addIncoming(negativePartialSum, popCountLoop);
        Value * const ptr = b->CreateGEP(negativeArray, index);
        b->CreateStore(negativePartialSum, ptr);
    }

    Value * const nextIndex = b->CreateAdd(index, ONE);
    index->addIncoming(nextIndex, popCountLoop);
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
PopCountKernel::PopCountKernel(const std::unique_ptr<kernel::KernelBuilder> & b, const PopCountType type, StreamSet * input, StreamSet * const output)
: MultiBlockKernel(b, "PopCount" + std::string{type == PopCountType::POSITIVE ? "P" : "N"}
// input streams
,{Binding{INPUT, input, FixedRate(b->getBitBlockWidth())}}
// output stream
,{Binding{OUTPUT_STREAM, output, FixedRate(), Add1()}}
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
PopCountKernel::PopCountKernel(const std::unique_ptr<kernel::KernelBuilder> & b, const PopCountType type, StreamSet * input, StreamSet * const positive, StreamSet * const negative)
: MultiBlockKernel(b, ".PopCountB"
// input streams
,{Binding{INPUT, input, FixedRate(b->getBitBlockWidth())}}
// output stream
,{Binding{POSITIVE_STREAM, positive, FixedRate(), Add1()}
 ,Binding{NEGATIVE_STREAM, negative, FixedRate(), Add1()}}
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
