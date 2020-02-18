#ifndef KERNEL_EXECUTION_LOGIC_HPP
#define KERNEL_EXECUTION_LOGIC_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeRegularKernelCall
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeKernelCall(BuilderRef b) {

    // WARNING: any change to this must be reflected in Kernel::addDoSegmentDeclaration, Kernel::getDoSegmentFields,
    // Kernel::setDoSegmentProperties and Kernel::getDoSegmentProperties.

    // TODO: consider whether we should share internally synchronized item counts via state?

    // TODO: add MProtect to buffers and their handles.

    // TODO: send in the # of output items we want in the external buffers

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mKernelHandle, CBuilder::Protect::NONE);
    }

    Value * releaseLock = nullptr;

    if (mKernelIsInternallySynchronized) {
        updateProcessedAndProducedItemCounts(b);
        writeUpdatedItemCounts(b, ItemCountSource::ComputedAtKernelCall);
        writeTerminationSignal(b, mIsFinalInvocationPhi);
        BasicBlock * const lastPartialSegment = b->CreateBasicBlock("", mKernelTerminationCheck);
        BasicBlock * const afterSyncLock = b->CreateBasicBlock("", mKernelTerminationCheck);

        Constant * const maxStrides = b->getSize(mMaximumNumOfStrides);
        releaseLock = b->CreateICmpEQ(mUpdatedNumOfStrides, maxStrides);
        releaseLock = b->CreateOr(b->CreateIsNotNull(mIsFinalInvocationPhi), releaseLock);

        b->CreateCondBr(releaseLock, lastPartialSegment, afterSyncLock);

        b->SetInsertPoint(lastPartialSegment);
        releaseSynchronizationLock(b, LockType::ItemCheck);
        b->CreateBr(afterSyncLock);

        b->SetInsertPoint(afterSyncLock);
    }

    const auto args = buildKernelCallArgumentList(b);

    #ifdef PRINT_DEBUG_MESSAGES
    const auto prefix = makeKernelName(mKernelIndex);
    debugPrint(b, "* " + prefix + "_executing = %" PRIu64, mNumOfLinearStrides);
    debugHalt(b);
    #endif
    startCycleCounter(b, CycleCounter::BEFORE_KERNEL_CALL);
    Value * const doSegment = getKernelDoSegmentFunction(b);
    Value * doSegmentRetVal = nullptr;
    if (mRethrowException) {
        BasicBlock * const invokeOk = b->CreateBasicBlock("", mKernelTerminationCheck);
        doSegmentRetVal = b->CreateInvoke(doSegment, invokeOk, mRethrowException, args);
        b->SetInsertPoint(invokeOk);
    } else {
        doSegmentRetVal = b->CreateCall(doSegment, args);
    }
    updateCycleCounter(b, CycleCounter::BEFORE_KERNEL_CALL, CycleCounter::AFTER_KERNEL_CALL);
    #ifdef PRINT_DEBUG_MESSAGES
    debugResume(b);
    #endif

    mTerminatedExplicitly = mKernelCanTerminateEarly ? doSegmentRetVal : nullptr;

    if (mKernelIsInternallySynchronized) {

        BasicBlock * const lastPartialSegment = b->CreateBasicBlock("", mKernelTerminationCheck);
        BasicBlock * const afterSyncLock = b->CreateBasicBlock("", mKernelTerminationCheck);

        b->CreateCondBr(releaseLock, lastPartialSegment, afterSyncLock);

        b->SetInsertPoint(lastPartialSegment);
        startCycleCounter(b, CycleCounter::BEFORE_SYNCHRONIZATION);
        acquireSynchronizationLock(b, LockType::Segment, CycleCounter::BEFORE_SYNCHRONIZATION);
        b->CreateBr(afterSyncLock);

        b->SetInsertPoint(afterSyncLock);
    } else {
        updateProcessedAndProducedItemCounts(b);
    }

    readReturnedOutputVirtualBaseAddresses(b);

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mKernelHandle, CBuilder::Protect::WRITE);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief buildKernelCallArgumentList
 ** ------------------------------------------------------------------------------------------------------------- */
ArgVec PipelineCompiler::buildKernelCallArgumentList(BuilderRef b) {

    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);

    ArgVec args;
    args.reserve(4 + (numOfInputs + numOfOutputs) * 4);
    if (LLVM_LIKELY(mKernel->isStateful())) {
        args.push_back(mKernelHandle); assert (mKernelHandle);
    }
    if (LLVM_UNLIKELY(mKernel->hasThreadLocal())) {
        args.push_back(b->CreateLoad(getThreadLocalHandlePtr(b, mKernelIndex)));
    }
    args.push_back(mNumOfLinearStrides); assert (mNumOfLinearStrides);
    if (LLVM_LIKELY(!mHasExplicitFinalPartialStride)) {
        args.push_back(b->CreateIsNotNull(mIsFinalInvocationPhi));
    }
    // If a kernel is internally synchronized, pass the segno to
    // allow the kernel to initialize its current "position"
    if (mKernelIsInternallySynchronized) {
        const auto prefix = makeKernelName(mKernelIndex);
        Value * const internalSegNoPtr = b->getScalarFieldPtr(prefix + CURRENT_LOGICAL_SEGMENT_NUMBER);
        Value * const segNo = b->CreateLoad(internalSegNoPtr);
        b->CreateStore(b->CreateAdd(segNo, b->getSize(1)), internalSegNoPtr);
        args.push_back(segNo);
    }
    if (mFixedRateFactorPhi) {
        args.push_back(mFixedRateFactorPhi);
    }

    SmallVector<ParamVec, 16> inputs(in_degree(mKernelIndex, mBufferGraph));

    for (const auto e : make_iterator_range(in_edges(mKernelIndex, mBufferGraph))) {
        const BufferRateData & rt = mBufferGraph[e];
        assert (rt.Port.Type == PortType::Input);

        if (LLVM_LIKELY(rt.Port.Reason == ReasonType::Explicit)) {

            // calculate the deferred processed item count
            PHINode * processed = nullptr;
            bool deferred = false;

            if (mAlreadyProcessedDeferredPhi(rt.Port)) {
                processed = mAlreadyProcessedDeferredPhi(rt.Port);
                deferred = true;
            } else {
                processed = mAlreadyProcessedPhi(rt.Port);
            }

            ParamVec & inputArgs = inputs[rt.Port.Number];

            const Binding & input = rt.Binding;

            const auto buffer = source(e, mBufferGraph);
            const BufferNode & bn = mBufferGraph[buffer];

            #ifndef NDEBUG
            assert ("input buffer type mismatch?" && (input.getType() == bn.Buffer->getBaseType()));
            #endif

            inputArgs.push_back(getVirtualBaseAddress(b, rt.Binding, bn.Buffer, processed, mIsInputZeroExtended(rt.Port)));

            mReturnedProcessedItemCountPtr(rt.Port) = addItemCountArg(b, input, deferred, processed, inputArgs);

            if (LLVM_UNLIKELY(requiresItemCount(input))) {
                // calculate how many linear items are from the *deferred* position
                Value * inputItems = mLinearInputItemsPhi(rt.Port);
                if (deferred) {
                    Value * diff = b->CreateSub(mAlreadyProcessedPhi(rt.Port), mAlreadyProcessedDeferredPhi(rt.Port));
                    inputItems = b->CreateAdd(inputItems, diff);
                }
                inputArgs.push_back(inputItems); assert (inputItems);
            }
        }
    }

    for (ParamVec & inputArgs : inputs) {
        args.insert(args.end(), inputArgs.begin(), inputArgs.end());
    }


    SmallVector<ParamVec, 16> outputs(out_degree(mKernelIndex, mBufferGraph));

    for (const auto e : make_iterator_range(out_edges(mKernelIndex, mBufferGraph))) {
        const BufferRateData & rt = mBufferGraph[e];
        assert (rt.Port.Reason == ReasonType::Explicit);
        assert (rt.Port.Type == PortType::Output);
        const auto i = rt.Port.Number;

        PHINode * const produced = mAlreadyProducedPhi(rt.Port);
        const auto buffer = target(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[buffer];
        const Binding & output = rt.Binding;

        ParamVec & outputArgs = outputs[i];

        assert ("output buffer type mismatch?" && (output.getType() == bn.Buffer->getBaseType()));

        if (LLVM_UNLIKELY(bn.Type == BufferType::ManagedByKernel)) {
            mReturnedOutputVirtualBaseAddressPtr(rt.Port) = addVirtualBaseAddressArg(b, bn.Buffer, outputArgs);
        } else {
            outputArgs.push_back(getVirtualBaseAddress(b, output, bn.Buffer, produced, nullptr));
        }
        mReturnedProducedItemCountPtr(rt.Port) = addItemCountArg(b, output, mKernelCanTerminateEarly, produced, outputArgs);
        // TODO:  consider whether we should pass a requested amount to source streams?
        if (requiresItemCount(output)) {
            outputArgs.push_back(mLinearOutputItemsPhi(rt.Port));  assert (mLinearOutputItemsPhi(rt.Port));
        }
        if (LLVM_UNLIKELY(bn.Type == BufferType::ManagedByKernel)) {
            outputArgs.push_back(mConsumedItemCount(rt.Port)); assert (mConsumedItemCount(rt.Port));
        }
    }

    for (ParamVec & outputArgs : outputs) {
        args.insert(args.end(), outputArgs.begin(), outputArgs.end());
    }

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mKernelHandle, CBuilder::Protect::NONE);
    }

    return args;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updateProcessedAndProducedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::updateProcessedAndProducedItemCounts(BuilderRef b) {

    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);

    // calculate or read the item counts (assuming this kernel did not terminate)
    for (unsigned i = 0; i < numOfInputs; ++i) {
        Value * processed = nullptr;
        const auto inputPort = StreamSetPort{PortType::Input, i};
        const Binding & input = getInputBinding(inputPort);
        const ProcessingRate & rate = input.getRate();
        if (LLVM_LIKELY(rate.isFixed() || rate.isPartialSum() || rate.isGreedy())) {
            processed = b->CreateAdd(mAlreadyProcessedPhi(inputPort), mLinearInputItemsPhi(inputPort));
            if (mAlreadyProcessedDeferredPhi(inputPort)) {
                assert (mReturnedProcessedItemCountPtr(inputPort));
                mProcessedDeferredItemCount(inputPort) = b->CreateLoad(mReturnedProcessedItemCountPtr(inputPort));
                #ifdef PRINT_DEBUG_MESSAGES
                const auto prefix = makeBufferName(mKernelIndex, inputPort);
                debugPrint(b, prefix + "_processed_deferred' = %" PRIu64, mProcessedDeferredItemCount(inputPort));
                #endif
                if (LLVM_UNLIKELY(mCheckAssertions)) {
                    Value * const deferred = mProcessedDeferredItemCount(inputPort);
                    Value * const isDeferred = b->CreateICmpULE(deferred, processed);
                    Value * const isFinal = mIsFinalInvocationPhi;
                    // TODO: workaround now for ScanMatch; if it ends with a match on a
                    // block-aligned boundary the start of the next match seems to be one
                    // after? Revise the logic to only perform a 0-item final block on
                    // kernels that may produce Add'ed data? Define the final/non-final
                    // contract first.
                    Value * const isDeferredOrFinal = b->CreateOr(isDeferred, b->CreateIsNotNull(isFinal));
                    b->CreateAssert(isDeferredOrFinal,
                                    "%s.%s: deferred processed item count (%" PRIu64 ") "
                                    "exceeds non-deferred (%" PRIu64 ")",
                                    mKernelAssertionName,
                                    b->GetString(input.getName()),
                                    deferred, processed);
                }
            }
        } else if (rate.isBounded() || rate.isUnknown()) {
            assert (mReturnedProcessedItemCountPtr(inputPort));
            processed = b->CreateLoad(mReturnedProcessedItemCountPtr(inputPort));
        } else {
            SmallVector<char, 256> tmp;
            raw_svector_ostream out(tmp);
            out << "Kernel " << mKernel->getName() << ":" << input.getName()
                << " has an " << "input" << " rate that is not properly handled by the PipelineKernel";
            report_fatal_error(out.str());
        }
        mProcessedItemCount(inputPort) = processed; assert (processed);
        #ifdef PRINT_DEBUG_MESSAGES
        const auto prefix = makeBufferName(mKernelIndex, inputPort);
        debugPrint(b, prefix + "_processed' = %" PRIu64, mProcessedItemCount(inputPort));
        #endif
    }

    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto outputPort = StreamSetPort{PortType::Output, i};
        const Binding & output = getOutputBinding(outputPort);
        const ProcessingRate & rate = output.getRate();
        Value * produced = nullptr;
        if (LLVM_LIKELY(rate.isFixed() || rate.isPartialSum())) {
            produced = b->CreateAdd(mAlreadyProducedPhi(outputPort), mLinearOutputItemsPhi(outputPort));
            if (mAlreadyProducedDeferredPhi(outputPort)) {
                assert (mReturnedProducedItemCountPtr(outputPort));
                mProducedDeferredItemCount(outputPort) = b->CreateLoad(mReturnedProducedItemCountPtr(outputPort));
                #ifdef PRINT_DEBUG_MESSAGES
                const auto prefix = makeBufferName(mKernelIndex, outputPort);
                debugPrint(b, prefix + "_produced_deferred' = %" PRIu64, mProcessedDeferredItemCount(outputPort));
                #endif
                if (LLVM_UNLIKELY(mCheckAssertions)) {
                    Value * const deferred = mProducedDeferredItemCount(outputPort);
                    Value * const isDeferred = b->CreateICmpULE(deferred, produced);
                    Value * const isFinal = mIsFinalInvocationPhi;
                    // TODO: workaround now for ScanMatch; if it ends with a match on a
                    // block-aligned boundary the start of the next match seems to be one
                    // after? Revise the logic to only perform a 0-item final block on
                    // kernels that may produce Add'ed data? Define the final/non-final
                    // contract first.
                    Value * const isDeferredOrFinal = b->CreateOr(isDeferred, b->CreateIsNotNull(isFinal));
                    b->CreateAssert(isDeferredOrFinal,
                                    "%s.%s: deferred processed item count (%" PRIu64 ") "
                                    "exceeds non-deferred (%" PRIu64 ")",
                                    mKernelAssertionName,
                                    b->GetString(output.getName()),
                                    deferred, produced);
                }
            }
        } else if (rate.isBounded() || rate.isUnknown()) {
            assert (mReturnedProducedItemCountPtr(outputPort));
            produced = b->CreateLoad(mReturnedProducedItemCountPtr(outputPort));
        } else {
            SmallVector<char, 256> tmp;
            raw_svector_ostream out(tmp);
            out << "Kernel " << mKernel->getName() << ":" << output.getName()
                << " has an " << "output" << " rate that is not properly handled by the PipelineKernel";
            report_fatal_error(out.str());
        }
        #ifdef PRINT_DEBUG_MESSAGES
        const auto prefix = makeBufferName(mKernelIndex, StreamSetPort{PortType::Output, i});
        debugPrint(b, prefix + "_produced' = %" PRIu64, produced);
        #endif
        mProducedItemCount(outputPort) = produced;
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addItemCountArg
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::addItemCountArg(BuilderRef b, const Binding & binding,
                                          const bool forceAddressability,
                                          PHINode * const itemCount,
                                          ParamVec & args) {
    const ProcessingRate & rate = binding.getRate();
    if (LLVM_UNLIKELY(rate.isRelative())) {
        return nullptr;
    }
    Value * ptr = nullptr;
    if (forceAddressability || isAddressable(binding)) {
        if (LLVM_UNLIKELY(mNumOfAddressableItemCount == mAddressableItemCountPtr.size())) {
            auto aic = b->CreateAllocaAtEntryPoint(b->getSizeTy());
            mAddressableItemCountPtr.push_back(aic);
        }
        ptr = mAddressableItemCountPtr[mNumOfAddressableItemCount++];
        b->CreateStore(itemCount, ptr);
        args.push_back(ptr);
    } else if (isCountable(binding)) {
        args.push_back(itemCount);
    }
    return ptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addVirtualBaseAddressArg
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::addVirtualBaseAddressArg(BuilderRef b, const StreamSetBuffer * buffer,
                                                   ParamVec & args) {
    if (LLVM_UNLIKELY(mNumOfVirtualBaseAddresses == mVirtualBaseAddressPtr.size())) {
        auto vba = b->CreateAllocaAtEntryPoint(b->getVoidPtrTy());
        mVirtualBaseAddressPtr.push_back(vba);
    }
    Value * ptr = mVirtualBaseAddressPtr[mNumOfVirtualBaseAddresses++]; assert (isa<ExternalBuffer>(buffer));
    ptr = b->CreatePointerCast(ptr, buffer->getPointerType()->getPointerTo());
    b->CreateStore(buffer->getBaseAddress(b.get()), ptr);
    args.push_back(ptr);
    return ptr;
}

}


#endif // KERNEL_EXECUTION_LOGIC_HPP
