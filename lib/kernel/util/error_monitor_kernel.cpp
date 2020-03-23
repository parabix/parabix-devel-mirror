/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernel/util/error_monitor_kernel.h>

#include <cinttypes>
#include <kernel/core/streamset.h>
#include <kernel/core/kernel_builder.h>

using namespace llvm;

namespace kernel {

void ErrorMonitorKernel::generateMultiBlockLogic(BuilderRef b, Value * const numOfStrides) {

    const auto numErrorStreams = b->getInputStreamSet("errorStream")->getNumElements();
    const auto blockWidth = b->getBitBlockWidth();
    IntegerType * const blockWidthTy = b->getIntNTy(blockWidth);

    BasicBlock * entryBB = b->GetInsertBlock();
    BasicBlock * loopBB = b->CreateBasicBlock(mName + "_loop");
    BasicBlock * isErrorBB = b->CreateBasicBlock(mName + "_is_error");
    BasicBlock * processErrBB = b->CreateBasicBlock(mName + "_process_error");
    BasicBlock * exitBB = b->CreateBasicBlock(mName + "_exit");

    ConstantInt * ZERO = b->getSize(0);
    ConstantInt * ONE = b->getSize(1);
    ConstantInt * BLOCK_TY_ONE = b->getIntN(blockWidth, 1);
    ConstantInt * const BLOCK_WIDTH = b->getSize(blockWidth);

    b->CreateBr(loopBB);

    // Loop Block
    b->SetInsertPoint(loopBB);
    PHINode * offset = b->CreatePHI(b->getSizeTy(), 2);
    offset->addIncoming(ZERO, entryBB);

    Value * accumulator = b->loadInputStreamBlock("errorStream", ZERO, offset);
    for (uint32_t i = 1; i < numErrorStreams; ++i) {
        Value * streamBlock = b->loadInputStreamBlock("errorStream", b->getInt32(i), offset);
        accumulator = b->CreateOr(accumulator, streamBlock);
    }

    // copy blocks from in streams to out streams
    foreachMonitoredStreamSet([&](std::string const & iName, std::string const & oName) {
        const uint32_t streamCount = b->getInputStreamSet(iName)->getNumElements();
        assert(streamCount == b->getOutputStreamSet(oName)->getNumElements());
        for (uint32_t i = 0; i < streamCount; ++i) {
            Value * istreamBlock = b->loadInputStreamBlock(iName, b->getInt32(i), offset);
            b->storeOutputStreamBlock(oName, b->getInt32(i), offset, istreamBlock);
        }
    });

    Value * accumBlock = b->CreateBitCast(accumulator, blockWidthTy);

    Value * nextOffset = b->CreateAdd(ONE, offset);
    offset->addIncoming(nextOffset, loopBB);
    Value * isZero = b->CreateICmpEQ(accumBlock, b->getIntN(blockWidth, 0));
    Value * moreData = b->CreateICmpNE(nextOffset, numOfStrides);
    Value * continueLoop = b->CreateAnd(isZero, moreData);
    b->CreateCondBr(continueLoop, loopBB, isErrorBB);

    // Is Error? Block
    b->SetInsertPoint(isErrorBB);
    b->CreateCondBr(isZero, exitBB, processErrBB);

    // Process Error Block
    b->SetInsertPoint(processErrBB);
    Value * numFwZeros = b->CreateCountForwardZeroes(accumBlock, true);
    Value * errBitMask = b->CreateShl(BLOCK_TY_ONE, numFwZeros);
    Value * errCode = b->getSize(0);
    for (uint32_t i = 0; i < numErrorStreams; ++i) {
        Value * streamBlock = b->loadInputStreamBlock("errorStream", b->getInt32(i), offset);
        streamBlock = b->CreateBitCast(streamBlock, blockWidthTy);
        Value * errBit = b->CreateAnd(streamBlock, errBitMask);
        errBit = b->CreateLShr(errBit, numFwZeros);                 // err bit at idx=0
        errBit = b->CreateShl(errBit, b->getIntN(blockWidth, i));   // err bit at idx=i
        errBit = b->CreateTrunc(errBit, b->getSizeTy());
        errCode = b->CreateOr(errCode, errBit);
    }
    b->setScalarField("errorCode", errCode);

    Value * maskSize = b->CreateSub(numFwZeros, BLOCK_TY_ONE);
    Value * mask = b->bitblock_mask_to(maskSize);

    foreachMonitoredStreamSet([&](std::string const & iName, std::string const & oName) {
        const uint32_t streamCount = b->getInputStreamSet(iName)->getNumElements();
        for (uint32_t i = 0; i < streamCount; ++i) {
            Value * const streamBlock = b->loadInputStreamBlock(iName, b->getInt32(i), offset);
            Value * const maskedBlock = b->simd_and(streamBlock, mask);
            b->storeOutputStreamBlock(oName, b->getInt32(i), offset, maskedBlock);
        }
    });

    Value * produced = b->CreateMul(offset, BLOCK_WIDTH);
    if (!mStreamNames.empty()) {
        Value * prior = b->getProducedItemCount(mStreamNames.front().second);
        produced = b->CreateAdd(prior, produced);
        produced = b->CreateAdd(produced, b->CreateZExtOrTrunc(numFwZeros, b->getSizeTy()));
    }
    foreachMonitoredStreamSet([&](std::string const &, std::string const & oName) {
        b->setProducedItemCount(oName, produced);
    });
    b->setFatalTerminationSignal();
    b->CreateBr(exitBB);

    // Exit Basic Block
    b->SetInsertPoint(exitBB);
}

ErrorMonitorKernel::ErrorMonitorKernel(BuilderRef b,
                                       StreamSet * error,
                                       ErrorMonitorKernel::IOStreamBindings bindings)
: MultiBlockKernel(b, "ErrorMonitorKernel" + std::to_string(error->getNumElements()),
// inputs
{Binding{"errorStream", error}},
// outputs - populated in constructor body
{},
// input scalars
{},
// output scalars
{Binding{b->getSizeTy(), "errorCode"}},
// internal scalars
{}),
// member fields
mName("ErrorMonitorKernel" + std::to_string(error->getNumElements())),
mStreamNames()
{
    addAttribute(MayFatallyTerminate());
    addAttribute(SideEffecting());
    for (auto binding : bindings) {
        auto names = generateNewStreamSetNames();
        mStreamNames.push_back(names);
        getInputStreamSetBindings().emplace_back(names.first, binding.first);
        getOutputStreamSetBindings().emplace_back(names.second, binding.second);
    }
}

} // namespace kernel
