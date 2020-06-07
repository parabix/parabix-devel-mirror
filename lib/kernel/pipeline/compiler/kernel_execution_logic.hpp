#ifndef KERNEL_EXECUTION_LOGIC_HPP
#define KERNEL_EXECUTION_LOGIC_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeKernelCall
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeKernelCall(BuilderRef b) {

    // TODO: add MProtect to buffers and their handles.

    // TODO: send in the # of output items we want in the external buffers

    Value * const doSegment = getKernelDoSegmentFunction(b);

    #ifndef NDEBUG
    mKernelDoSegmentFunctionType = cast<FunctionType>(doSegment->getType()->getPointerElementType());
    #endif

    const auto args = buildKernelCallArgumentList(b);

    #ifdef PRINT_DEBUG_MESSAGES
    debugHalt(b);
    #endif

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mKernelSharedHandle, CBuilder::Protect::WRITE);
    }

    Value * const beforeKernelCall = startCycleCounter(b);
    Value * doSegmentRetVal = nullptr;
    if (mRethrowException) {
        const auto prefix = makeKernelName(mKernelId);
        BasicBlock * const invokeOk = b->CreateBasicBlock(prefix + "_invokeOk", mKernelCompletionCheck);
        doSegmentRetVal = b->CreateInvoke(doSegment, invokeOk, mRethrowException, args);
        b->SetInsertPoint(invokeOk);
    } else {
        doSegmentRetVal = b->CreateCall(doSegment, args);
    }
    updateCycleCounter(b, mKernelId, beforeKernelCall, CycleCounter::KERNEL_EXECUTION);

    #ifdef PRINT_DEBUG_MESSAGES
    debugResume(b);
    #endif

    if (mKernelCanTerminateEarly) {
        mTerminatedExplicitly = doSegmentRetVal;
        #ifdef PRINT_DEBUG_MESSAGES
        const auto prefix = makeKernelName(mKernelId);
        debugPrint(b, "* " + prefix + "_terminatedExplicitly = %" PRIu64, mTerminatedExplicitly);
        #endif
    } else {
        mTerminatedExplicitly = nullptr;
    }

    readOrUpdateProcessedAndProducedItemCounts(b);
    readReturnedOutputVirtualBaseAddresses(b);

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mKernelSharedHandle, CBuilder::Protect::NONE);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief buildKernelCallArgumentList
 ** ------------------------------------------------------------------------------------------------------------- */
ArgVec PipelineCompiler::buildKernelCallArgumentList(BuilderRef b) {

    // WARNING: any change to this must be reflected in Kernel::addDoSegmentDeclaration, Kernel::getDoSegmentFields,
    // Kernel::setDoSegmentProperties and Kernel::getDoSegmentProperties.

    const auto numOfInputs = in_degree(mKernelId, mBufferGraph);
    const auto numOfOutputs = out_degree(mKernelId, mBufferGraph);

    ArgVec args;

    auto addNextArg = [&](Value * arg) {

        #ifndef NDEBUG
        assert ("null argument" && arg);

        const auto n = mKernelDoSegmentFunctionType->getNumParams();
        if (LLVM_UNLIKELY(args.size() >= n)) {
            SmallVector<char, 256> tmp;
            raw_svector_ostream out(tmp);
            out << mKernel->getName() << ": "
                   "was given too many arguments";
            throw std::runtime_error(out.str().str());
        }

        Type * const argTy = mKernelDoSegmentFunctionType->getParamType(args.size());
        if (LLVM_UNLIKELY(argTy != arg->getType())) {
            SmallVector<char, 256> tmp;
            raw_svector_ostream out(tmp);
            out << mKernel->getName() << ": "
                "invalid argument type for #" << args.size()
                << ": expected ";
            argTy->print(out);
            out << " but got ";
            arg->getType()->print(out);
            throw std::runtime_error(out.str().str());
        }
        #endif

        args.push_back(arg);
    };

    auto addItemCountArg = [&](const BufferPort & port,
                               Value * const itemCount,
                               Value * const itemCountPtrInStateObject) -> Value * {
        const Binding & binding = port.Binding;
        const ProcessingRate & rate = binding.getRate();
        if (LLVM_UNLIKELY(rate.isRelative())) {
            return nullptr;
        }
        if (port.Addressable) {
            if (port.DirectlyUpdatesInternalState) {
                assert (itemCountPtrInStateObject);
                addNextArg(itemCountPtrInStateObject);
                return itemCountPtrInStateObject;
            } else {                
                if (LLVM_UNLIKELY(mNumOfAddressableItemCount == mAddressableItemCountPtr.size())) {
                    auto aic = b->CreateAllocaAtEntryPoint(b->getSizeTy());
                    mAddressableItemCountPtr.push_back(aic);
                }
                assert (itemCountPtrInStateObject == nullptr);
                Value * const temporaryItemCountPtr = mAddressableItemCountPtr[mNumOfAddressableItemCount++];
                b->CreateStore(itemCount, temporaryItemCountPtr);
                addNextArg(temporaryItemCountPtr);
                return temporaryItemCountPtr;
            }
        } else if (isCountable(binding)) {
            addNextArg(itemCount);
        }
        return nullptr;
    };

    args.reserve(4 + (numOfInputs + numOfOutputs) * 4);
    if (LLVM_LIKELY(mKernelSharedHandle)) {
        addNextArg(mKernelSharedHandle);
    }
    if (LLVM_UNLIKELY(mKernelThreadLocalHandle)) {
        addNextArg(mKernelThreadLocalHandle);
    }

    // If a kernel is internally synchronized, pass the segno to
    // allow the kernel to initialize its current "position"
    const auto greedy = mKernel->isGreedy();
    if (mKernelIsInternallySynchronized || greedy) {
        if (mKernelIsInternallySynchronized) {
            addNextArg(mSegNo);
        }
        addNextArg(mKernelIsFinal);
    } else {
        addNextArg(mNumOfLinearStrides);
        #ifdef PRINT_DEBUG_MESSAGES
        const auto prefix = makeKernelName(mKernelId);
        debugPrint(b, "* " + prefix + "_executing = %" PRIu64, mNumOfLinearStrides);
        #endif
        if (mFixedRateFactorPhi) {
            addNextArg(mFixedRateFactorPhi);
        }
    }

    PointerType * const voidPtrTy = b->getVoidPtrTy();

    for (unsigned i = 0; i < numOfInputs; ++i) {
        const auto port = getInput(mKernelId, StreamSetPort(PortType::Input, i));
        const BufferPort & rt = mBufferGraph[port];
        const auto inputPort = rt.Port;

        if (LLVM_LIKELY(inputPort.Reason == ReasonType::Explicit)) {

            const auto streamSet = source(port, mBufferGraph);
            const BufferNode & bn = mBufferGraph[streamSet];
            const StreamSetBuffer * const buffer = bn.Buffer;

            const auto  deferred = rt.IsDeferred;

            Value * processed = nullptr;

            if (LLVM_UNLIKELY(mKernelIsInternallySynchronized)) {

                // As an outside observer to an internally synchronized kernel, we cannot
                // know exactly how many items have been processed within the kernel itself.
                // Moreover without any synchronization locks guarding this kernel code,
                // we could easily have two or more threads simultaneously executing this
                // code. However, we still want a uniform model for how we represent virtual
                // base addresses (VBAs) since the address we're passing into this kernel
                // could be external to the pipeline itself. However, this still means the
                // invoked kernel will still have to modify the VBA to point to the correct
                // position once it begins to use it.

                if (deferred) {
                    processed = mInitiallyProcessedDeferredItemCount[inputPort];
                } else {
                    processed = mInitiallyProcessedItemCount[inputPort];
                }

            } else {

                if (deferred) {
                    processed = mAlreadyProcessedDeferredPhi[inputPort];
                } else {
                    processed = mAlreadyProcessedPhi[inputPort];
                }
            }
            assert (processed);

            Value * addr = nullptr;
            if (LLVM_UNLIKELY(mKernelIsInternallySynchronized)) {
                assert ("internally synchronized I/O must be linear!" && !bn.NonLinear);
                addr = getVirtualBaseAddress(b, rt, buffer, processed);
            } else {
                addr = mInputVirtualBaseAddressPhi[inputPort];
            }

            addNextArg(b->CreatePointerCast(addr, voidPtrTy));

            Value * processedItemCountInStateObject = nullptr;
            if (rt.DirectlyUpdatesInternalState) {
                if (LLVM_UNLIKELY(deferred)) {
                    processedItemCountInStateObject = mProcessedDeferredItemCountPtr[inputPort];
                } else {
                    processedItemCountInStateObject = mProcessedItemCountPtr[inputPort];
                }
            }

            mReturnedProcessedItemCountPtr[inputPort] = addItemCountArg(rt, processed, processedItemCountInStateObject);

            if (mKernelIsInternallySynchronized) {

                #warning change accessible to avail for safety?

                const auto streamSet = source(port, mBufferGraph);
                Value * const avail = mLocallyAvailableItems[streamSet]; assert (avail);
                Value * const accessible = b->CreateSub(avail, processed);
                addNextArg(accessible);
            } else if (LLVM_UNLIKELY(requiresItemCount(rt.Binding))) {
                // calculate how many linear items are from the *deferred* position
                Value * inputItems = mLinearInputItemsPhi[rt.Port]; assert (inputItems);
                if (deferred) {
                    Value * diff = b->CreateSub(mAlreadyProcessedPhi[rt.Port], mAlreadyProcessedDeferredPhi[rt.Port]);
                    inputItems = b->CreateAdd(inputItems, diff);
                }
                addNextArg(inputItems);
            }
        }
    }

    auto addVirtualBaseAddressArg = [&](const StreamSetBuffer * buffer) {
        PointerType * const voidPtrTy = b->getVoidPtrTy();
        if (LLVM_UNLIKELY(mNumOfVirtualBaseAddresses == mVirtualBaseAddressPtr.size())) {
            auto vba = b->CreateAllocaAtEntryPoint(voidPtrTy);
            mVirtualBaseAddressPtr.push_back(vba);
        }
        Value * ptr = mVirtualBaseAddressPtr[mNumOfVirtualBaseAddresses++];
        ptr = b->CreatePointerCast(ptr, buffer->getPointerType()->getPointerTo());
        b->CreateStore(buffer->getBaseAddress(b.get()), ptr);
        addNextArg(b->CreatePointerCast(ptr, voidPtrTy->getPointerTo()));
        return ptr;
    };


    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto port = getOutput(mKernelId, StreamSetPort(PortType::Output, i));
        const BufferPort & rt = mBufferGraph[port];

        assert (rt.Port.Reason == ReasonType::Explicit);
        assert (rt.Port.Type == PortType::Output);

        const auto streamSet = target(port, mBufferGraph);
        const BufferNode & bn = mBufferGraph[streamSet];
        const StreamSetBuffer * const buffer = bn.Buffer;

        Value * produced = nullptr;
        if (LLVM_UNLIKELY(mKernelIsInternallySynchronized)) {
            assert ("internally synchronized I/O must be linear!" && !bn.NonLinear);
            produced = mInitiallyProducedItemCount[streamSet];
        } else {
            produced = mAlreadyProducedPhi[rt.Port];
        }
        const auto managed = rt.IsShared || mKernelIsInternallySynchronized || rt.IsManaged;
        if (LLVM_UNLIKELY(rt.IsShared)) {
            addNextArg(b->CreatePointerCast(buffer->getHandle(), voidPtrTy));
        } else if (LLVM_UNLIKELY(managed)) {
            mReturnedOutputVirtualBaseAddressPtr[rt.Port] = addVirtualBaseAddressArg(buffer);
        } else {
            addNextArg(b->CreatePointerCast(getVirtualBaseAddress(b, rt, buffer, produced), voidPtrTy));
        }

        Value * producedItemCountInStateObject = nullptr;
        if (rt.DirectlyUpdatesInternalState) {
            if (LLVM_UNLIKELY(rt.IsDeferred)) {
                producedItemCountInStateObject = mProducedDeferredItemCountPtr[rt.Port];
            } else {
                producedItemCountInStateObject = mProducedItemCountPtr[rt.Port];
            }
        }
        mReturnedProducedItemCountPtr[rt.Port] = addItemCountArg(rt, produced, producedItemCountInStateObject);

        if (LLVM_UNLIKELY(managed)) {
            addNextArg(mInitialConsumedItemCount[streamSet]);
        } else if (requiresItemCount(rt.Binding)) {
            addNextArg(mLinearOutputItemsPhi[rt.Port]);
        }

    }

    assert (args.size() == mKernelDoSegmentFunctionType->getNumParams());

    return args;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readOrUpdateProcessedAndProducedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::readOrUpdateProcessedAndProducedItemCounts(BuilderRef b) {

    // calculate or read the item counts (assuming this kernel did not terminate)
    for (const auto input : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {

        const BufferPort & bp = mBufferGraph[input];
        const auto inputPort = bp.Port;
        Value * processed = nullptr;

        if (bp.Countable && !mKernelIsInternallySynchronized) {
            assert (mLinearInputItemsPhi[inputPort]);
            processed = b->CreateAdd(mAlreadyProcessedPhi[inputPort], mLinearInputItemsPhi[inputPort]);
            if (bp.IsDeferred) {
                assert (mReturnedProcessedItemCountPtr[inputPort]);
                mProcessedDeferredItemCount[inputPort] = b->CreateLoad(mReturnedProcessedItemCountPtr[inputPort]);
                #ifdef PRINT_DEBUG_MESSAGES
                const auto prefix = makeBufferName(mKernelId, inputPort);
                debugPrint(b, prefix + "_processed_deferred' = %" PRIu64, mProcessedDeferredItemCount[inputPort]);
                #endif
                if (LLVM_UNLIKELY(CheckAssertions)) {
                    const Binding & input = bp.Binding;
                    Value * const deferred = mProcessedDeferredItemCount[inputPort];
                    Value * const isDeferred = b->CreateICmpULE(deferred, processed);
                    Value * const isFinal = b->CreateIsNotNull(mIsFinalInvocationPhi);
                    // TODO: workaround now for ScanMatch; if it ends with a match on a
                    // block-aligned boundary the start of the next match seems to be one
                    // after? Revise the logic to only perform a 0-item final block on
                    // kernels that may produce Add'ed data? Define the final/non-final
                    // contract first.
                    Value * const isDeferredOrFinal = b->CreateOr(isDeferred, isFinal);
                    b->CreateAssert(isDeferredOrFinal,
                                    "%s.%s: deferred processed item count (%" PRIu64 ") "
                                    "exceeds non-deferred (%" PRIu64 ")",
                                    mCurrentKernelName,
                                    b->GetString(input.getName()),
                                    deferred, processed);
                }
            }
        } else { // read the processed count from the kernel
            processed = b->CreateLoad(mReturnedProcessedItemCountPtr[inputPort]);
        }

        if (LLVM_UNLIKELY(CheckAssertions)) {

            Value * prior = nullptr;
            const auto  deferred = bp.IsDeferred;
            if (LLVM_UNLIKELY(mKernelIsInternallySynchronized)) {
                if (deferred) {
                    prior = mInitiallyProcessedDeferredItemCount[inputPort];
                } else {
                    prior = mInitiallyProcessedItemCount[inputPort];
                }
            } else {
                if (deferred) {
                    prior = mAlreadyProcessedDeferredPhi[inputPort];
                } else {
                    prior = mAlreadyProcessedPhi[inputPort];
                }
            }
            assert (prior);

            const Binding & input = bp.Binding;
            Constant * const bindingName = b->GetString(input.getName());

            Value * const valid = b->CreateICmpULE(prior, processed);
            b->CreateAssert(valid, "%s.%s processed item count (%" PRId64 ") must be non-decreasing (%" PRId64 ")",
                            mCurrentKernelName, bindingName, processed, prior);

        }

        mProcessedItemCount[inputPort] = processed; assert (processed);
        #ifdef PRINT_DEBUG_MESSAGES
        const auto prefix = makeBufferName(mKernelId, inputPort);
        debugPrint(b, prefix + "_processed' = %" PRIu64, mProcessedItemCount[inputPort]);
        #endif
    }

    for (const auto output : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const BufferPort & bp = mBufferGraph[output];
        const auto outputPort = bp.Port;
        Value * produced = nullptr;

        if (bp.Countable && !mKernelIsInternallySynchronized) {
            assert (mLinearOutputItemsPhi[outputPort]);
            produced = b->CreateAdd(mAlreadyProducedPhi[outputPort], mLinearOutputItemsPhi[outputPort]);
            if (bp.IsDeferred) {
                mProducedDeferredItemCount[outputPort] = b->CreateLoad(mReturnedProducedItemCountPtr[outputPort]);
                #ifdef PRINT_DEBUG_MESSAGES
                const auto prefix = makeBufferName(mKernelId, outputPort);
                debugPrint(b, prefix + "_produced_deferred' = %" PRIu64, mProcessedDeferredItemCount[outputPort]);
                #endif
                if (LLVM_UNLIKELY(CheckAssertions)) {
                    const Binding & output = bp.Binding;
                    Value * const deferred = mProducedDeferredItemCount[outputPort];
                    Value * const isDeferred = b->CreateICmpULE(deferred, produced);
                    Value * const isFinal = b->CreateIsNotNull(mIsFinalInvocationPhi);
                    // TODO: workaround now for ScanMatch; if it ends with a match on a
                    // block-aligned boundary the start of the next match seems to be one
                    // after? Revise the logic to only perform a 0-item final block on
                    // kernels that may produce Add'ed data? Define the final/non-final
                    // contract first.
                    Value * const isDeferredOrFinal = b->CreateOr(isDeferred, isFinal);
                    b->CreateAssert(isDeferredOrFinal,
                                    "%s.%s: deferred produced item count (%" PRIu64 ") "
                                    "exceeds non-deferred (%" PRIu64 ")",
                                    mCurrentKernelName,
                                    b->GetString(output.getName()),
                                    deferred, produced);
                }
            }
        } else { // read the produced count from the kernel
            produced = b->CreateLoad(mReturnedProducedItemCountPtr[outputPort]);
        }

        if (LLVM_UNLIKELY(CheckAssertions)) {
            Value * prior = nullptr;
            if (LLVM_UNLIKELY(mKernelIsInternallySynchronized)) {
                const auto streamSet = target(output, mBufferGraph);
                prior = mInitiallyProducedItemCount[streamSet];
            } else {
                prior = mAlreadyProducedPhi[outputPort];
            }
            assert (prior);

            const Binding & output = bp.Binding;
            Constant * const bindingName = b->GetString(output.getName());

            Value * const valid = b->CreateICmpULE(prior, produced);
            b->CreateAssert(valid, "%s.%s produced item count (%" PRId64 ") must be non-decreasing (%" PRId64 ")",
                            mCurrentKernelName, bindingName, produced, prior);

        }

        #ifdef PRINT_DEBUG_MESSAGES
        const auto prefix = makeBufferName(mKernelId, outputPort);
        debugPrint(b, prefix + "_produced' = %" PRIu64, produced);
        #endif
        mProducedItemCount[outputPort] = produced;
    }

}

}


#endif // KERNEL_EXECUTION_LOGIC_HPP
