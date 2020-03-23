#include <kernel/core/block_kernel_compiler.h>
#include <kernel/core/kernel_builder.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/MDBuilder.h>

using namespace llvm;

namespace kernel {

const auto DO_BLOCK_SUFFIX = "_DoBlock";
const auto FINAL_BLOCK_SUFFIX = "_FinalBlock";

#define TARGET (reinterpret_cast<BlockOrientedKernel *>(mTarget))

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateMultiBlockLogic
 ** ------------------------------------------------------------------------------------------------------------- */
void BlockKernelCompiler::generateMultiBlockLogic(BuilderRef b, Value * const numOfBlocks) {

    const auto stride = mTarget->getStride();

    if (LLVM_UNLIKELY(stride != b->getBitBlockWidth())) {
        SmallVector<char, 256> tmp;
        raw_svector_ostream out(tmp);
        out << getName() << ": the Stride (" << stride << ") of BlockOrientedKernel "
               "equal to the BitBlockWidth (" << b->getBitBlockWidth() << ")";
        report_fatal_error(out.str());
    }

    BasicBlock * const entryBlock = b->GetInsertBlock();
    mStrideLoopBody = b->CreateBasicBlock(getName() + "_strideLoopBody");
    BasicBlock * const incrementCountableItems = b->CreateBasicBlock(getName() + "_incrementCountableItems");
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
    Value * noMore = b->CreateICmpEQ(nextStrideBlockIndex, numOfBlocks);
    if (canSetTerminateSignal()) {
        noMore = b->CreateOr(noMore, b->getTerminationSignal());
    }
    b->CreateUnlikelyCondBr(noMore, stridesDone, incrementCountableItems);

    b->SetInsertPoint(incrementCountableItems);
    incrementCountableItemCounts(b);
    BasicBlock * const bodyEnd = b->GetInsertBlock();
    if (mStrideLoopTarget) {
        mStrideLoopTarget->addIncoming(mStrideLoopTarget, bodyEnd);
    }
    mStrideBlockIndex->addIncoming(nextStrideBlockIndex, bodyEnd);

    b->CreateBr(mStrideLoopBody);

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
        SmallVector<uint32_t, 16> weights(destinations);
        for (unsigned i = 0; i < destinations; ++i) {
            weights[i] = (mStrideLoopBranch->getDestination(i) == segmentDone) ? 100 : 1;
        }
        mStrideLoopBranch->setMetadata(LLVMContext::MD_prof, mdb.createBranchWeights(weights));
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief incrementCountableItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void BlockKernelCompiler::incrementCountableItemCounts(BuilderRef b) {
    // Update the processed item counts

    const auto stride = mTarget->getStride();

    for (const Binding & input : getInputStreamSetBindings()) {
        if (isCountable(input)) {
            const ProcessingRate & rate = input.getRate();
            Value * offset = nullptr;
            if (rate.isFixed()) {
                offset = b->getSize(ceiling(getUpperBound(input) * stride));
            } else { // if (rate.isPopCount() || rate.isNegatedPopCount())
                offset = getPopCountRateItemCount(b, rate);
            }
            Value * const initial = b->getProcessedItemCount(input.getName());
            Value * const processed = b->CreateAdd(initial, offset);
            b->setProcessedItemCount(input.getName(), processed);
        }
    }
    // Update the produced item counts
    for (const Binding & output : getOutputStreamSetBindings()) {
        if (isCountable(output)) {
            const ProcessingRate & rate = output.getRate();
            Value * offset = nullptr;
            if (rate.isFixed()) {
                offset = b->getSize(ceiling(getUpperBound(output) * stride));
            } else { // if (rate.isPopCount() || rate.isNegatedPopCount())
                offset = getPopCountRateItemCount(b, rate);
            }
            Value * const initial = b->getProducedItemCount(output.getName());
            Value * const produced = b->CreateAdd(initial, offset);
            b->setProducedItemCount(output.getName(), produced);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getPopCountRateItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
Value * BlockKernelCompiler::getPopCountRateItemCount(BuilderRef b,
                                                      const ProcessingRate & rate) {
    assert (rate.isPopCount() || rate.isNegatedPopCount());
    const auto refPort = getStreamPort(rate.getReference());
    assert (refPort.Type == PortType::Input);
    Value * array = nullptr;
    if (rate.isNegatedPopCount()) {
        array = mNegatedPopCountRateArray[refPort.Number];
    } else {
        array = mPopCountRateArray[refPort.Number];
    }
    assert (array && "missing pop count array attribute");
    Value * const currentSum = b->CreateLoad(b->CreateInBoundsGEP(array, mStrideBlockIndex));
    Value * const priorIndex = b->CreateSub(mStrideBlockIndex, b->getSize(1));
    Value * const priorSum = b->CreateLoad(b->CreateInBoundsGEP(array, priorIndex));
    return b->CreateSub(currentSum, priorSum);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getRemainingItems
 ** ------------------------------------------------------------------------------------------------------------- */
Value * BlockKernelCompiler::getRemainingItems(BuilderRef b) {
    const auto count = mInputStreamSets.size();
    assert (count > 0);
    for (unsigned i = 0; i < count; i++) {
        if (mInputStreamSets[i].isPrincipal()) {
            return mAccessibleInputItems[i];
        }
    }
    return mAccessibleInputItems[0];
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeDoBlockMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void BlockKernelCompiler::writeDoBlockMethod(BuilderRef b) {

    Value * const self = b->getHandle();
    Function * const cp = mCurrentMethod;
    auto ip = b->saveIP();
    Vec<Value *> availableItemCount;

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
        setHandle(&*args);
        availableItemCount.reserve(mAccessibleInputItems.size());
        while (++args != mCurrentMethod->arg_end()) {
            availableItemCount.push_back(&*args);
        }
        assert (availableItemCount.size() == mAccessibleInputItems.size());
        mAccessibleInputItems.swap(availableItemCount);
        b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", mCurrentMethod));
    }

    TARGET->generateDoBlockMethod(b); // must be implemented by the BlockOrientedKernelBuilder subtype

    if (!b->supportsIndirectBr()) {
        // Restore the DoSegment function state then call the DoBlock method
        b->CreateRetVoid();
        mDoBlockMethod = mCurrentMethod;
        b->restoreIP(ip);
        setHandle(self);
        mCurrentMethod = cp;
        mAccessibleInputItems.swap(availableItemCount);
        generateDefaultFinalBlockMethod(b);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeFinalBlockMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void BlockKernelCompiler::writeFinalBlockMethod(BuilderRef b, Value * remainingItems) {

    Value * const self = b->getHandle();
    Function * const cp = mCurrentMethod;
    Value * const remainingItemCount = remainingItems;
    auto ip = b->saveIP();
    Vec<Value *> availableItemCount;

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
        setHandle(&*args);
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

    TARGET->generateFinalBlockMethod(b, remainingItems); // may be implemented by the BlockOrientedKernel subtype

    if (!b->supportsIndirectBr()) {
        b->CreateRetVoid();
        b->restoreIP(ip);
        setHandle(self);
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
 * @brief generateDefaultFinalBlockMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void BlockKernelCompiler::generateDefaultFinalBlockMethod(BuilderRef b) {
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
        args.push_back(b->getHandle());
        args.insert(args.end(), mAccessibleInputItems.begin(), mAccessibleInputItems.end());
        b->CreateCall(mDoBlockMethod, args);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructor
 ** ------------------------------------------------------------------------------------------------------------- */
BlockKernelCompiler::BlockKernelCompiler(BlockOrientedKernel * const kernel) noexcept
: KernelCompiler(kernel) {

}

}
