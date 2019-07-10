/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "kernel.h"
#include <toolchain/toolchain.h>
#include <kernels/kernel_builder.h>
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
#include <llvm/Support/Debug.h>
#include <boost/graph/adjacency_list.hpp>
#include <util/extended_boost_graph_containers.h>
#include <sstream>
#include <functional>

using namespace llvm;
using namespace boost;
using boost::container::flat_set;

namespace kernel {

using AttrId = Attribute::KindId;
using RateId = ProcessingRate::KindId;

// TODO: Break the BlockOrientedKernel into two classes, one with an explicit DoFinal block and another that
// calls the DoBlock method with optional preamble and postamble hooks. By doing so, we can remove the indirect
// branches (or function calls) from the following kernel and simplify the cognitive load for the kernel
// programmer. This is less general than the current method but no evidence that being able to reenter the
// DoBlock method multiple times from the DoFinal block would ever be useful.

// Can we eliminate some of the kernel state (e.g., EOF) by having a general preamble that can create a stack-
// allocated struct?

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
void BlockOrientedKernel::incrementCountableItemCounts(const std::unique_ptr<KernelBuilder> & b) {
    // Update the processed item counts
    for (const Binding & input : getInputStreamSetBindings()) {
        if (isCountable(input)) {
            const ProcessingRate & rate = input.getRate();
            Value * offset = nullptr;
            if (rate.isFixed()) {
                offset = b->getSize(ceiling(getUpperBound(input) * getStride()));
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
                offset = b->getSize(ceiling(getUpperBound(output) * getStride()));
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
Value * BlockOrientedKernel::getPopCountRateItemCount(const std::unique_ptr<KernelBuilder> & b,
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
    Value * const currentSum = b->CreateLoad(b->CreateGEP(array, mStrideBlockIndex));
    Value * const priorIndex = b->CreateSub(mStrideBlockIndex, b->getSize(1));
    Value * const priorSum = b->CreateLoad(b->CreateGEP(array, priorIndex));
    return b->CreateSub(currentSum, priorSum);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getRemainingItems
 ** ------------------------------------------------------------------------------------------------------------- */
Value * BlockOrientedKernel::getRemainingItems(const std::unique_ptr<KernelBuilder> & b) {
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
inline void BlockOrientedKernel::writeDoBlockMethod(const std::unique_ptr<KernelBuilder> & b) {

    Value * const self = getHandle();
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
        setHandle(b, &*args);
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
        setHandle(b, self);
        mCurrentMethod = cp;
        mAccessibleInputItems.swap(availableItemCount);
        CreateDoBlockMethodCall(b);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeFinalBlockMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void BlockOrientedKernel::writeFinalBlockMethod(const std::unique_ptr<KernelBuilder> & b, Value * remainingItems) {

    Value * const self = getHandle();
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
        setHandle(b, &*args);
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
        setHandle(b, self);
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
        args.push_back(getHandle());
        args.insert(args.end(), mAccessibleInputItems.begin(), mAccessibleInputItems.end());
        b->CreateCall(mDoBlockMethod, args);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief annotateInputBindingsWithPopCountArrayAttributes
 ** ------------------------------------------------------------------------------------------------------------- */
void BlockOrientedKernel::annotateInputBindingsWithPopCountArrayAttributes() {
    const unsigned n = mInputStreamSets.size();
    const unsigned m = mOutputStreamSets.size();

    if (LLVM_UNLIKELY(n == 0 || (n + m) < 2)) {
        return;
    }

    enum : int {
        POSITIVE = 0
        , NEGATIVE = 1
    };

    using Graph = adjacency_list<vecS, vecS, directedS>;

    Graph G(std::max(n, 2U));

    auto checkPopCount = [&](const Binding & binding) {
        const ProcessingRate & rate = binding.getRate();
        if (LLVM_UNLIKELY(rate.isPopCount() || rate.isNegatedPopCount())) {
            const auto refPort = getStreamPort(rate.getReference());
            assert ("pop count rate cannot refer to an output stream" && (refPort.Type == PortType::Input));
            const auto targetType = rate.isPopCount() ? POSITIVE : NEGATIVE;
            add_edge(refPort.Number, targetType, G);
        }
    };

    for (unsigned i = 0; i < n; ++i) {
        checkPopCount(mInputStreamSets[i]);
    }
    for (unsigned i = 0; i < m; ++i) {
        checkPopCount(mOutputStreamSets[i]);
    }

    for (const auto e : make_iterator_range(edges(G))) {
        const auto i = source(e, G);
        assert (i < mInputStreamSets.size());
        Binding & input = mInputStreamSets[i];
        switch (target(e, G)) {
            case POSITIVE:
                input.addAttribute(RequiresPopCountArray());
                break;
            case NEGATIVE:
                input.addAttribute(RequiresNegatedPopCountArray());
                break;
            default: llvm_unreachable("unhandled pop count type!");
        }
    }
}


// CONSTRUCTOR
BlockOrientedKernel::BlockOrientedKernel(
    const std::unique_ptr<KernelBuilder> & b,
    std::string && kernelName,
    Bindings && stream_inputs,
    Bindings && stream_outputs,
    Bindings && scalar_parameters,
    Bindings && scalar_outputs,
    InternalScalars && internal_scalars)
: MultiBlockKernel(b,
    TypeId::BlockOriented,
    std::move(kernelName),
    std::move(stream_inputs),
    std::move(stream_outputs),
    std::move(scalar_parameters),
    std::move(scalar_outputs),
    std::move(internal_scalars))
, mDoBlockMethod(nullptr)
, mStrideLoopBody(nullptr)
, mStrideLoopBranch(nullptr)
, mStrideLoopTarget(nullptr)
, mStrideBlockIndex(nullptr) {
    annotateInputBindingsWithPopCountArrayAttributes();
}


}
