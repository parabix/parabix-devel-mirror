/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "kernel.h"
#include <toolchain/toolchain.h>
#include <kernels/streamset.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/MDBuilder.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_ostream.h>
#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(4, 0, 0)
#include <llvm/Bitcode/ReaderWriter.h>
#else
#include <llvm/Bitcode/BitcodeWriter.h>
#endif
#include <llvm/Transforms/Utils/Local.h>
#include <kernels/streamset.h>
#include <sstream>
#include <kernels/kernel_builder.h>
#include <boost/math/common_factor.hpp>
#include <llvm/Support/Debug.h>

using namespace llvm;
using namespace parabix;
using namespace boost::math;

namespace kernel {

const auto DO_BLOCK_SUFFIX = "_DoBlock";
const auto FINAL_BLOCK_SUFFIX = "_FinalBlock";

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateMultiBlockLogic
 ** ------------------------------------------------------------------------------------------------------------- */
void BlockOrientedKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, Value * const numOfBlocks) {

    if (LLVM_UNLIKELY(mStride != b->getBitBlockWidth())) {
        report_fatal_error(getName() + ": the Stride (" + std::to_string(mStride) + ") of BlockOrientedKernel "
                           "equal to the BitBlockWidth (" + std::to_string(b->getBitBlockWidth()) + ")");
    }

    BasicBlock * const entryBlock = b->GetInsertBlock();
    mStrideLoopBody = b->CreateBasicBlock(getName() + "_strideLoopBody");
    BasicBlock * const stridesDone = b->CreateBasicBlock(getName() + "_stridesDone");
    BasicBlock * const doFinalBlock = b->CreateBasicBlock(getName() + "_doFinalBlock");
    BasicBlock * const segmentDone = b->CreateBasicBlock(getName() + "_segmentDone");

    b->CreateUnlikelyCondBr(mIsFinal, doFinalBlock, mStrideLoopBody);

    /// BLOCK BODY

    b->SetInsertPoint(mStrideLoopBody);

    if (b->supportsIndirectBr()) {
        Value * const baseTarget = BlockAddress::get(segmentDone);
        mStrideLoopTarget = b->CreatePHI(baseTarget->getType(), 2, "strideTarget");
        mStrideLoopTarget->addIncoming(baseTarget, entryBlock);
    }

    mStrideBlockIndex = b->CreatePHI(b->getSizeTy(), 2);
    mStrideBlockIndex->addIncoming(b->getSize(0), entryBlock);

    /// GENERATE DO BLOCK METHOD

    writeDoBlockMethod(b);

    Value * const nextStrideBlockIndex = b->CreateAdd(mStrideBlockIndex, b->getSize(1));

    incrementDerivedItemCounts(b);

    BasicBlock * const bodyEnd = b->GetInsertBlock();
    if (mStrideLoopTarget) {
        mStrideLoopTarget->addIncoming(mStrideLoopTarget, bodyEnd);
    }
    mStrideBlockIndex->addIncoming(nextStrideBlockIndex, bodyEnd);
    Value * const notDone = b->CreateICmpULT(nextStrideBlockIndex, numOfBlocks);
    b->CreateCondBr(notDone, mStrideLoopBody, stridesDone);

    stridesDone->moveAfter(bodyEnd);

    /// STRIDE DONE

    b->SetInsertPoint(stridesDone);

    // Now conditionally perform the final block processing depending on the doFinal parameter.
    if (mStrideLoopTarget) {
        mStrideLoopBranch = b->CreateIndirectBr(mStrideLoopTarget, 3);
        mStrideLoopBranch->addDestination(doFinalBlock);
        mStrideLoopBranch->addDestination(segmentDone);
    } else {
        b->CreateUnlikelyCondBr(mIsFinal, doFinalBlock, segmentDone);
    }

    doFinalBlock->moveAfter(stridesDone);

    /// DO FINAL BLOCK

    b->SetInsertPoint(doFinalBlock);

    writeFinalBlockMethod(b, getRemainingItems(b));

    b->CreateBr(segmentDone);

    segmentDone->moveAfter(b->GetInsertBlock());

    b->SetInsertPoint(segmentDone);

    // Update the branch prediction metadata to indicate that the likely target will be segmentDone
    if (mStrideLoopTarget) {
        MDBuilder mdb(b->getContext());
        const auto destinations = mStrideLoopBranch->getNumDestinations();
        uint32_t weights[destinations];
        for (unsigned i = 0; i < destinations; ++i) {
            weights[i] = (mStrideLoopBranch->getDestination(i) == segmentDone) ? 100 : 1;
        }
        ArrayRef<uint32_t> bw(weights, destinations);
        mStrideLoopBranch->setMetadata(LLVMContext::MD_prof, mdb.createBranchWeights(bw));
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getRemainingItems
 ** ------------------------------------------------------------------------------------------------------------- */
Value * BlockOrientedKernel::incrementDerivedItemCounts(const std::unique_ptr<KernelBuilder> & b) {

    Value * const nextIndex = b->CreateAdd(mStrideBlockIndex, b->getSize(1));

    mTreatUnsafeKernelOperationsAsErrors = false;

    // Update the processed item counts
    for (unsigned i = 0; i < mStreamSetInputs.size(); ++i) {
        const Binding & input = mStreamSetInputs[i];
        if (hasDerivedItemCount(input)) {
            const ProcessingRate & rate = input.getRate();
            Value * offset = nullptr;
            if (rate.isFixed()) {
                offset = b->CreateMul(nextIndex, mInputStrideLength[i]);
            } else { // if (rate.isPopCount() || rate.isNegatedPopCount())
                offset = getPopCountRateItems(b, rate, nextIndex);
            }
            Value * const processed = b->CreateAdd(mInitialProcessedItemCount[i], offset);
            b->setNonDeferredProcessedItemCount(input, processed);
        }
    }

    // Update the produced item counts
    for (unsigned i = 0; i < mStreamSetOutputs.size(); ++i) {
        const Binding & output = mStreamSetOutputs[i];
        if (hasDerivedItemCount(output)) {
            const ProcessingRate & rate = output.getRate();
            Value * offset = nullptr;
            if (rate.isFixed()) {
                offset = b->CreateMul(nextIndex, mOutputStrideLength[i]);
            } else { // if (rate.isPopCount() || rate.isNegatedPopCount())
                offset = getPopCountRateItems(b, rate, nextIndex);
            }
            Value * const produced = b->CreateAdd(mInitialProducedItemCount[i], offset);
            b->setNonDeferredProducedItemCount(output, produced);
        }
    }

    mTreatUnsafeKernelOperationsAsErrors = true;

    return nextIndex;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getRemainingItems
 ** ------------------------------------------------------------------------------------------------------------- */
Value * BlockOrientedKernel::getRemainingItems(const std::unique_ptr<KernelBuilder> & b) {
    Value * remainingItems = nullptr;
    const auto count = mStreamSetInputs.size();
    if (count == 1) {
        return mAccessibleInputItems[0];
    } else {
        for (unsigned i = 0; i < count; i++) {
            if (mStreamSetInputs[i].isPrincipal()) {
                return mAccessibleInputItems[i];
            }
        }
        for (unsigned i = 0; i < count; ++i) {
            const ProcessingRate & r = mStreamSetInputs[i].getRate();
            if (r.isFixed()) {
                Value * ic = b->CreateCeilUDiv2(mAccessibleInputItems[i], r.getRate());
                if (remainingItems) {
                    remainingItems = b->CreateUMin(remainingItems, ic);
                } else {
                    remainingItems = ic;
                }
            }
        }
    }
    return remainingItems;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeDoBlockMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void BlockOrientedKernel::writeDoBlockMethod(const std::unique_ptr<KernelBuilder> & b) {

    Value * const self = getInstance();
    Function * const cp = mCurrentMethod;
    auto ip = b->saveIP();
    std::vector<Value *> availableItemCount(0);

    /// Check if the do block method is called and create the function if necessary
    if (!b->supportsIndirectBr()) {

        std::vector<Type *> params;
        params.reserve(1 + mAccessibleInputItems.size());
        params.push_back(self->getType());
        for (Value * avail : mAccessibleInputItems) {
            params.push_back(avail->getType());
        }

        FunctionType * const type = FunctionType::get(b->getVoidTy(), params, false);
        mCurrentMethod = Function::Create(type, GlobalValue::InternalLinkage, getName() + DO_BLOCK_SUFFIX, b->getModule());
        mCurrentMethod->setCallingConv(CallingConv::C);
        mCurrentMethod->setDoesNotThrow();
        auto args = mCurrentMethod->arg_begin();
        args->setName("self");
        setInstance(&*args);
        availableItemCount.reserve(mAccessibleInputItems.size());
        while (++args != mCurrentMethod->arg_end()) {
            availableItemCount.push_back(&*args);
        }
        assert (availableItemCount.size() == mAccessibleInputItems.size());
        mAccessibleInputItems.swap(availableItemCount);
        b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", mCurrentMethod));
    }

    generateDoBlockMethod(b); // must be implemented by the BlockOrientedKernelBuilder subtype

    if (!b->supportsIndirectBr()) {
        // Restore the DoSegment function state then call the DoBlock method
        b->CreateRetVoid();
        mDoBlockMethod = mCurrentMethod;
        b->restoreIP(ip);
        setInstance(self);
        mCurrentMethod = cp;
        mAccessibleInputItems.swap(availableItemCount);
        CreateDoBlockMethodCall(b);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeFinalBlockMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void BlockOrientedKernel::writeFinalBlockMethod(const std::unique_ptr<KernelBuilder> & b, Value * remainingItems) {

    Value * const self = getInstance();
    Function * const cp = mCurrentMethod;
    Value * const remainingItemCount = remainingItems;
    auto ip = b->saveIP();
    std::vector<Value *> availableItemCount(0);

    if (!b->supportsIndirectBr()) {
        std::vector<Type *> params;
        params.reserve(2 + mAccessibleInputItems.size());
        params.push_back(self->getType());
        params.push_back(b->getSizeTy());
        for (Value * avail : mAccessibleInputItems) {
            params.push_back(avail->getType());
        }
        FunctionType * const type = FunctionType::get(b->getVoidTy(), params, false);
        mCurrentMethod = Function::Create(type, GlobalValue::InternalLinkage, getName() + FINAL_BLOCK_SUFFIX, b->getModule());
        mCurrentMethod->setCallingConv(CallingConv::C);
        mCurrentMethod->setDoesNotThrow();
        auto args = mCurrentMethod->arg_begin();
        args->setName("self");
        setInstance(&*args);
        remainingItems = &*(++args);
        remainingItems->setName("remainingItems");
        availableItemCount.reserve(mAccessibleInputItems.size());
        while (++args != mCurrentMethod->arg_end()) {
            availableItemCount.push_back(&*args);
        }
        assert (availableItemCount.size() == mAccessibleInputItems.size());
        mAccessibleInputItems.swap(availableItemCount);
        b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", mCurrentMethod));
    }

    generateFinalBlockMethod(b, remainingItems); // may be implemented by the BlockOrientedKernel subtype

    if (!b->supportsIndirectBr()) {
        b->CreateRetVoid();
        b->restoreIP(ip);
        setInstance(self);
        mAccessibleInputItems.swap(availableItemCount);
        // Restore the DoSegment function state then call the DoFinal method
        std::vector<Value *> args;
        args.reserve(2 + mAccessibleInputItems.size());
        args.push_back(self);
        args.push_back(remainingItemCount);
        args.insert(args.end(), mAccessibleInputItems.begin(), mAccessibleInputItems.end());
        b->CreateCall(mCurrentMethod, args);
        mCurrentMethod = cp;
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateFinalBlockMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void BlockOrientedKernel::generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> & b, Value * /* remainingItems */) {
    //  The default finalBlock method simply dispatches to the doBlock routine.
    CreateDoBlockMethodCall(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief CreateDoBlockMethodCall
 ** ------------------------------------------------------------------------------------------------------------- */
void BlockOrientedKernel::CreateDoBlockMethodCall(const std::unique_ptr<KernelBuilder> & b) {
    if (b->supportsIndirectBr()) {
        BasicBlock * const bb = b->CreateBasicBlock("resume");
        mStrideLoopBranch->addDestination(bb);
        BasicBlock * const current = b->GetInsertBlock();
        mStrideLoopTarget->addIncoming(BlockAddress::get(bb), current);
        mStrideBlockIndex->addIncoming(b->getSize(0), current);
        b->CreateBr(mStrideLoopBody);
        bb->moveAfter(current);
        b->SetInsertPoint(bb);
    } else {
        std::vector<Value *> args;
        args.reserve(1 + mAccessibleInputItems.size());
        args.push_back(getInstance());
        args.insert(args.end(), mAccessibleInputItems.begin(), mAccessibleInputItems.end());
        b->CreateCall(mDoBlockMethod, args);
    }
}

// CONSTRUCTOR
BlockOrientedKernel::BlockOrientedKernel(std::string && kernelName,
                                         Bindings && stream_inputs,
                                         Bindings && stream_outputs,
                                         Bindings && scalar_parameters,
                                         Bindings && scalar_outputs,
                                         Bindings && internal_scalars)
: MultiBlockKernel(std::move(kernelName), std::move(stream_inputs), std::move(stream_outputs), std::move(scalar_parameters), std::move(scalar_outputs), std::move(internal_scalars))
, mDoBlockMethod(nullptr)
, mStrideLoopBody(nullptr)
, mStrideLoopBranch(nullptr)
, mStrideLoopTarget(nullptr)
, mStrideBlockIndex(nullptr) {

}


}
