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
        #if LLVM_VERSION_MAJOR >= 11
            //no matching member function for call to 'CreateInvoke'
            auto *calleePtrType = llvm::cast<llvm::PointerType>(doSegment->getType());
            auto *calleeType = llvm::cast<llvm::FunctionType>(calleePtrType->getElementType());
            doSegmentRetVal = b->CreateInvoke(calleeType, doSegment, invokeOk, mRethrowException, args);
        #else
            doSegmentRetVal = b->CreateInvoke(doSegment, invokeOk, mRethrowException, args);
        #endif
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

    updateProcessedAndProducedItemCounts(b);
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

    args.reserve(4 + (numOfInputs + numOfOutputs) * 4);
    if (LLVM_LIKELY(mKernelSharedHandle)) {
        addNextArg(mKernelSharedHandle);
    }
    if (LLVM_UNLIKELY(mKernelThreadLocalHandle)) {
        addNextArg(mKernelThreadLocalHandle);
    }

    // If a kernel is internally synchronized, pass the segno to
    // allow the kernel to initialize its current "position"
    #ifdef PRINT_DEBUG_MESSAGES
    const auto prefix = makeKernelName(mKernelId);
    #endif

    const auto greedy = mKernel->isGreedy();
    if (mKernelIsInternallySynchronized || greedy) {
        if (mKernelIsInternallySynchronized) {
            addNextArg(mSegNo);
            // addNextArg(mMaximumNumOfStrides);
        }
        addNextArg(mKernelIsFinal);
    } else {
        addNextArg(mNumOfLinearStrides);
        #ifdef PRINT_DEBUG_MESSAGES
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

        if (LLVM_LIKELY(rt.Port.Reason == ReasonType::Explicit)) {

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
                    processed = mInitiallyProcessedDeferredItemCount[rt.Port];
                } else {
                    processed = mInitiallyProcessedItemCount[rt.Port];
                }

            } else {

                if (deferred) {
                    processed = mAlreadyProcessedDeferredPhi[rt.Port];
                } else {
                    processed = mAlreadyProcessedPhi[rt.Port];
                }
            }
            assert (processed);

            Value * addr = nullptr;
            if (LLVM_UNLIKELY(mKernelIsInternallySynchronized)) {
                assert ("internally synchronized I/O must be linear!" && !bn.NonLinear);
                addr = getVirtualBaseAddress(b, rt, buffer, processed);
            } else {
                addr = mInputVirtualBaseAddressPhi[rt.Port];
            }

            addNextArg(b->CreatePointerCast(addr, voidPtrTy));

            if (LLVM_UNLIKELY(mKernelIsInternallySynchronized)) {
                Value * ptr = nullptr;
                if (LLVM_UNLIKELY(rt.IsDeferred)) {
                    ptr = mProcessedDeferredItemCountPtr[rt.Port];
                } else {
                    ptr = mProcessedItemCountPtr[rt.Port];
                }
                mReturnedProcessedItemCountPtr[rt.Port] = ptr;
                addNextArg(ptr);
            } else {
                mReturnedProcessedItemCountPtr[rt.Port] = addItemCountArg(b, rt, deferred, processed, args);
            }

            if (mKernelIsInternallySynchronized) {
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
            mReturnedOutputVirtualBaseAddressPtr[rt.Port] = addVirtualBaseAddressArg(b, buffer, args);
        } else {
            addNextArg(b->CreatePointerCast(getVirtualBaseAddress(b, rt, buffer, produced), voidPtrTy));
        }

        if (LLVM_UNLIKELY(mKernelIsInternallySynchronized)) {
            Value * ptr = nullptr;
            if (LLVM_UNLIKELY(rt.IsDeferred)) {
                ptr = mProducedDeferredItemCountPtr[rt.Port];
            } else {
                ptr = mProducedItemCountPtr[rt.Port];
            }
            mReturnedProducedItemCountPtr[rt.Port] = ptr;
            addNextArg(ptr);
        } else {
            mReturnedProducedItemCountPtr[rt.Port] = addItemCountArg(b, rt, mKernelCanTerminateEarly, produced, args);
        }

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
 * @brief updateProcessedAndProducedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::updateProcessedAndProducedItemCounts(BuilderRef b) {

    const auto numOfInputs = in_degree(mKernelId, mBufferGraph);
    const auto numOfOutputs = out_degree(mKernelId, mBufferGraph);

    // calculate or read the item counts (assuming this kernel did not terminate)
    for (unsigned i = 0; i < numOfInputs; ++i) {
        Value * processed = nullptr;
        const auto inputPort = StreamSetPort{PortType::Input, i};
        if (LLVM_UNLIKELY(mKernelIsInternallySynchronized)) {
            assert (mReturnedProcessedItemCountPtr[inputPort]);
            processed = b->CreateLoad(mReturnedProcessedItemCountPtr[inputPort]);
        } else {
            const Binding & input = getInputBinding(inputPort);
            const ProcessingRate & rate = input.getRate();
            if (LLVM_LIKELY(rate.isFixed() || rate.isPartialSum() || rate.isGreedy())) {
                processed = b->CreateAdd(mAlreadyProcessedPhi[inputPort], mLinearInputItemsPhi[inputPort]);
                if (mAlreadyProcessedDeferredPhi[inputPort]) {
                    assert (mReturnedProcessedItemCountPtr[inputPort]);
                    mProcessedDeferredItemCount[inputPort] = b->CreateLoad(mReturnedProcessedItemCountPtr[inputPort]);
                    #ifdef PRINT_DEBUG_MESSAGES
                    const auto prefix = makeBufferName(mKernelId, inputPort);
                    debugPrint(b, prefix + "_processed_deferred' = %" PRIu64, mProcessedDeferredItemCount[inputPort]);
                    #endif
                    if (LLVM_UNLIKELY(CheckAssertions)) {
                        Value * const deferred = mProcessedDeferredItemCount[inputPort];
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
                                        mCurrentKernelName,
                                        b->GetString(input.getName()),
                                        deferred, processed);
                    }
                }
            } else if (rate.isBounded() || rate.isUnknown()) {
                assert (mReturnedProcessedItemCountPtr[inputPort]);
                processed = b->CreateLoad(mReturnedProcessedItemCountPtr[inputPort]);
            } else {
                SmallVector<char, 256> tmp;
                raw_svector_ostream out(tmp);
                out << "Kernel " << mKernel->getName() << ":" << input.getName()
                    << " has an " << "input" << " rate that is not properly handled by the PipelineKernel";
                report_fatal_error(out.str());
            }
        }

        mProcessedItemCount[inputPort] = processed; assert (processed);
        #ifdef PRINT_DEBUG_MESSAGES
        const auto prefix = makeBufferName(mKernelId, inputPort);
        debugPrint(b, prefix + "_processed' = %" PRIu64, mProcessedItemCount[inputPort]);
        #endif
    }

    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto outputPort = StreamSetPort{PortType::Output, i};
        Value * produced = nullptr;
        if (LLVM_UNLIKELY(mKernelIsInternallySynchronized)) {
            assert (mReturnedProducedItemCountPtr[outputPort]);
            produced = b->CreateLoad(mReturnedProducedItemCountPtr[outputPort]);
        } else {
            const Binding & output = getOutputBinding(outputPort);
            const ProcessingRate & rate = output.getRate();
            if (LLVM_LIKELY(rate.isFixed() || rate.isPartialSum())) {
                produced = b->CreateAdd(mAlreadyProducedPhi[outputPort], mLinearOutputItemsPhi[outputPort]);
                if (mAlreadyProducedDeferredPhi[outputPort]) {
                    assert (mReturnedProducedItemCountPtr[outputPort]);
                    mProducedDeferredItemCount[outputPort] = b->CreateLoad(mReturnedProducedItemCountPtr[outputPort]);
                    #ifdef PRINT_DEBUG_MESSAGES
                    const auto prefix = makeBufferName(mKernelId, outputPort);
                    debugPrint(b, prefix + "_produced_deferred' = %" PRIu64, mProcessedDeferredItemCount[outputPort]);
                    #endif
                    if (LLVM_UNLIKELY(CheckAssertions)) {
                        Value * const deferred = mProducedDeferredItemCount[outputPort];
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
                                        mCurrentKernelName,
                                        b->GetString(output.getName()),
                                        deferred, produced);
                    }
                }
            } else if (rate.isBounded() || rate.isUnknown()) {
                assert (mReturnedProducedItemCountPtr[outputPort]);
                produced = b->CreateLoad(mReturnedProducedItemCountPtr[outputPort]);
            } else {
                SmallVector<char, 256> tmp;
                raw_svector_ostream out(tmp);
                out << "Kernel " << mKernel->getName() << ":" << output.getName()
                    << " has an " << "output" << " rate that is not properly handled by the PipelineKernel";
                report_fatal_error(out.str());
            }
        }
        #ifdef PRINT_DEBUG_MESSAGES
        const auto prefix = makeBufferName(mKernelId, StreamSetPort{PortType::Output, i});
        debugPrint(b, prefix + "_produced' = %" PRIu64, produced);
        #endif
        mProducedItemCount[outputPort] = produced;
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addItemCountArg
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::addItemCountArg(BuilderRef b, const BufferPort & port,
                                          const bool forceAddressability,
                                          Value * const itemCount,
                                          ArgVec & args) {
    const Binding & binding = port.Binding;
    const ProcessingRate & rate = binding.getRate();
    if (LLVM_UNLIKELY(rate.isRelative())) {
        return nullptr;
    }

    auto addNextArg = [&](Value * arg) {
        assert ("null argument" && arg);
        assert ("too many arguments?" && args.size() < mKernelDoSegmentFunctionType->getNumParams());
        assert ("invalid argument type" && (mKernelDoSegmentFunctionType->getParamType(args.size()) == arg->getType()));
        args.push_back(arg);
    };

    Value * ptr = nullptr;

    if (forceAddressability || isAddressable(binding)) {
        if (LLVM_UNLIKELY(mNumOfAddressableItemCount == mAddressableItemCountPtr.size())) {
            auto aic = b->CreateAllocaAtEntryPoint(b->getSizeTy());
            mAddressableItemCountPtr.push_back(aic);
        }
        ptr = mAddressableItemCountPtr[mNumOfAddressableItemCount++];
        b->CreateStore(itemCount, ptr);
        addNextArg(ptr);
    } else if (isCountable(binding)) {
        addNextArg(itemCount);
    }
    return ptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addVirtualBaseAddressArg
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::addVirtualBaseAddressArg(BuilderRef b, const StreamSetBuffer * buffer, ArgVec & args) {
    PointerType * const voidPtrTy = b->getVoidPtrTy();
    if (LLVM_UNLIKELY(mNumOfVirtualBaseAddresses == mVirtualBaseAddressPtr.size())) {
        auto vba = b->CreateAllocaAtEntryPoint(voidPtrTy);
        mVirtualBaseAddressPtr.push_back(vba);
    }

    auto addNextArg = [&](Value * arg) {
        assert ("null argument" && arg);
        assert ("too many arguments?" && args.size() < mKernelDoSegmentFunctionType->getNumParams());
        assert ("invalid argument type" && (mKernelDoSegmentFunctionType->getParamType(args.size()) == arg->getType()));
        args.push_back(arg);
    };

    Value * ptr = mVirtualBaseAddressPtr[mNumOfVirtualBaseAddresses++];
    ptr = b->CreatePointerCast(ptr, buffer->getPointerType()->getPointerTo());
    b->CreateStore(buffer->getBaseAddress(b.get()), ptr);
    addNextArg(b->CreatePointerCast(ptr, voidPtrTy->getPointerTo()));
    return ptr;
}

}


#endif // KERNEL_EXECUTION_LOGIC_HPP
