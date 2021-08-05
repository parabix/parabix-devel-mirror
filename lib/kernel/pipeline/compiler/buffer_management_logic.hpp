#ifndef BUFFER_ALLOCATION_HPP
#define BUFFER_ALLOCATION_HPP

#include "pipeline_compiler.hpp"

#include <llvm/Support/ErrorHandling.h>

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addHandlesToPipelineKernel
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addBufferHandlesToPipelineKernel(BuilderRef b, const unsigned index) {
    for (const auto e : make_iterator_range(out_edges(index, mBufferGraph))) {
        const auto streamSet = target(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[streamSet];
        // external buffers already have a buffer handle
        if (LLVM_UNLIKELY(bn.isExternal())) {
            continue;
        }
        assert (parent(streamSet, mBufferGraph) != PipelineInput);
        const BufferPort & rd = mBufferGraph[e];
        const auto handleName = makeBufferName(index, rd.Port);
        StreamSetBuffer * const buffer = bn.Buffer;
        Type * const handleType = buffer->getHandleType(b);

        #ifdef PERMIT_BUFFER_MEMORY_REUSE
        // We automatically assign the buffer memory according to the buffer start position
        if (bn.Locality == BufferLocality::ThreadLocal) {
            assert (bn.isOwned());
            mTarget->addNonPersistentScalar(handleType, handleName);
        } else
        #endif
        if (LLVM_LIKELY(bn.isOwned())) {
            mTarget->addInternalScalar(handleType, handleName);
//            if (bn.Locality == BufferLocality::GloballyShared) {
//                mTarget->addInternalScalar(handleType, handleName);
//            } else {
//                mTarget->addThreadLocalScalar(handleType, handleName);
//            }
        } else {
            mTarget->addNonPersistentScalar(handleType, handleName);
            mTarget->addInternalScalar(buffer->getPointerType(), handleName + LAST_GOOD_VIRTUAL_BASE_ADDRESS, index);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief loadExternalStreamSetHandles
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::loadExternalStreamSetHandles(BuilderRef /* b */) {

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief loadInternalStreamSetHandles
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::loadInternalStreamSetHandles(BuilderRef b, const bool nonLocal) {
    for (auto streamSet = FirstStreamSet; streamSet <= LastStreamSet; ++streamSet) {
        const BufferNode & bn = mBufferGraph[streamSet];
        // external buffers already have a buffer handle
        StreamSetBuffer * const buffer = bn.Buffer;
        if (LLVM_UNLIKELY(bn.isExternal())) {
            assert (isFromCurrentFunction(b, buffer->getHandle()));
        } else if (bn.isNonThreadLocal() == nonLocal) {
            assert (bn.isInternal());
            const auto pe = in_edge(streamSet, mBufferGraph);
            const auto producer = source(pe, mBufferGraph);
            const BufferPort & rd = mBufferGraph[pe];
            const auto handleName = makeBufferName(producer, rd.Port);
            Value * const handle = b->getScalarFieldPtr(handleName);
            assert (buffer->getHandle() == nullptr);
            buffer->setHandle(handle);
            if (bn.Locality == BufferLocality::ThreadLocal && mThreadLocalStreamSetBaseAddress) {
                assert (RequiredThreadLocalStreamSetMemory > 0);
                assert (isa<StaticBuffer>(buffer));
                assert ((bn.BufferStart % b->getCacheAlignment()) == 0);
                Value * const startOffset = b->CreateMul(mExpectedNumOfStridesMultiplier, b->getSize(bn.BufferStart));
                Value * const baseAddress = b->CreateGEP(mThreadLocalStreamSetBaseAddress, startOffset);
                if (LLVM_UNLIKELY(CheckAssertions)) {
                    DataLayout DL(b->getModule());
                    Type * const intPtrTy = DL.getIntPtrType(baseAddress->getType());
                    Value * const intPtrVal = b->CreatePtrToInt(baseAddress, intPtrTy);

                    Value * const align = b->getSize(b->getCacheAlignment());
                    Value * const offset = b->CreateURem(intPtrVal, align);
                    Value * const valid = b->CreateIsNull(offset);
                    SmallVector<char, 256> tmp;
                    raw_svector_ostream out(tmp);
                    out << "%s: thread local buffer 0x%" PRIx64 " "
                           "is not cache aligned (%" PRIu64 ")";
                    b->CreateAssert(valid, out.str(), mCurrentKernelName, intPtrVal, align);
                }
                const auto baseCapacity = bn.RequiredCapacity * b->getBitBlockWidth();
                assert (baseCapacity > 0);
                Value * const capacity = b->CreateMul(mExpectedNumOfStridesMultiplier, b->getSize(baseCapacity));
                buffer->setBaseAddress(b, b->CreatePointerCast(baseAddress, buffer->getPointerType()));
                buffer->setCapacity(b, capacity);
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief allocateOwnedBuffers
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::allocateOwnedBuffers(BuilderRef b, Value * const expectedNumOfStrides, const bool nonLocal) {
    assert (expectedNumOfStrides);

    if (LLVM_UNLIKELY(CheckAssertions)) {
        Value * const valid = b->CreateIsNotNull(expectedNumOfStrides);
        b->CreateAssert(valid,
           "%s: expected number of strides for internally allocated buffers is 0",
           b->GetString(mTarget->getName()));
    }

    // recursively allocate any internal buffers for the nested kernels, giving them the correct
    // num of strides it should expect to perform
    for (auto i = FirstKernel; i <= LastKernel; ++i) {
        const Kernel * const kernelObj = getKernel(i);
        if (LLVM_UNLIKELY(kernelObj->allocatesInternalStreamSets())) {
            if (nonLocal || kernelObj->hasThreadLocal()) {
                setActiveKernel(b, i, !nonLocal);
                assert (mKernel == kernelObj);
                SmallVector<Value *, 3> params;
                if (LLVM_LIKELY(mKernelSharedHandle)) {
                    params.push_back(mKernelSharedHandle);
                }
                Value * func = nullptr;
                if (nonLocal) {
                    func = getKernelAllocateSharedInternalStreamSetsFunction(b);
                } else {
                    func = getKernelAllocateThreadLocalInternalStreamSetsFunction(b);
                    params.push_back(mKernelThreadLocalHandle);
                }

                const auto scale = MaximumNumOfStrides[i] * Rational{mNumOfThreads};
                params.push_back(b->CreateCeilUMulRational(expectedNumOfStrides, scale));
                b->CreateCall(func, params);
            }
        }

        // and allocate any output buffers
        for (const auto e : make_iterator_range(out_edges(i, mBufferGraph))) {
            const auto streamSet = target(e, mBufferGraph);
            const BufferNode & bn = mBufferGraph[streamSet];
            if (bn.isUnowned() || bn.isShared() || (bn.isNonThreadLocal() != nonLocal)) {
                continue;
            }

            StreamSetBuffer * const buffer = bn.Buffer;
            if (LLVM_LIKELY(bn.isInternal())) {
                const BufferPort & rd = mBufferGraph[e];
                const auto handleName = makeBufferName(i, rd.Port);
                Value * const handle = b->getScalarFieldPtr(handleName);
                buffer->setHandle(handle);
            }
            assert ("a threadlocal buffer cannot be external" && (bn.isInternal() || nonLocal));
            assert (buffer->getHandle());

            assert (isFromCurrentFunction(b, buffer->getHandle(), false));

            if (bn.Locality == BufferLocality::ThreadLocal) {
                continue;
            }


            buffer->allocateBuffer(b, expectedNumOfStrides);

            #ifdef PRINT_DEBUG_MESSAGES
            const BufferPort & rd = mBufferGraph[e];
            const auto prefix = makeBufferName(i, rd.Port);
            debugPrint(b, prefix + ".inital malloc range = [%" PRIx64 ",%" PRIx64 ")",
                       buffer->getMallocAddress(b), buffer->getOverflowAddress(b));
            #endif

            if (LLVM_UNLIKELY(CheckAssertions)) {
                DataLayout DL(b->getModule());
                Value * const mAddr = buffer->getMallocAddress(b);
                Type * const intPtrTy = DL.getIntPtrType(mAddr->getType());
                Value * const mAddrInt = b->CreatePtrToInt(mAddr, intPtrTy);

                const BufferPort & rd = mBufferGraph[e];
                const auto prefix = makeBufferName(i, rd.Port);

                Constant * const prefixName = b->GetString(prefix);

                Constant * const cacheAlign = ConstantInt::get(intPtrTy, b->getCacheAlignment());
                Constant * const blockAlign = ConstantInt::get(intPtrTy, b->getBitBlockWidth() / 8);

                b->CreateAssertZero(b->CreateURem(mAddrInt, cacheAlign),
                                    "%s: malloc addr is not cache-aligned", prefixName);

                Value * const mOverInt = b->CreatePtrToInt(buffer->getOverflowAddress(b), intPtrTy);

                b->CreateAssertZero(b->CreateURem(mOverInt, blockAlign),
                                    "%s: overflow addr is not block-aligned", prefixName);
            }


        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief releaseOwnedBuffers
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::releaseOwnedBuffers(BuilderRef b, const bool nonLocal) {
    loadInternalStreamSetHandles(b, nonLocal);
    for (auto streamSet = FirstStreamSet; streamSet <= LastStreamSet; ++streamSet) {
        const BufferNode & bn = mBufferGraph[streamSet];
        #ifdef PERMIT_BUFFER_MEMORY_REUSE
        if (bn.Locality == BufferLocality::ThreadLocal) {
            continue;
        }
        #endif
        if (bn.isUnowned() || bn.isShared() || bn.isNonThreadLocal() != nonLocal) {
            continue;
        }
        StreamSetBuffer * const buffer = bn.Buffer;
        assert (isFromCurrentFunction(b, buffer->getHandle(), false));
        buffer->releaseBuffer(b);

        // TODO: TraceDynamicBuffers needs to be fixed to permit thread local buffers.

        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::TraceDynamicBuffers))) {
            if (isa<DynamicBuffer>(buffer)) {

                const auto pe = in_edge(streamSet, mBufferGraph);
                const auto p = source(pe, mBufferGraph);
                const BufferPort & rd = mBufferGraph[pe];
                const auto prefix = makeBufferName(p, rd.Port);

                Value * const traceData = b->getScalarFieldPtr(prefix + STATISTICS_BUFFER_EXPANSION_SUFFIX);
                Constant * const ZERO = b->getInt32(0);
                b->CreateFree(b->CreateLoad(b->CreateInBoundsGEP(traceData, {ZERO, ZERO})));
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief resetInternalBufferHandles
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::resetInternalBufferHandles() {
    #ifndef NDEBUG
    for (auto i = FirstStreamSet; i <= LastStreamSet; ++i) {
        const BufferNode & bn = mBufferGraph[i];
        if (LLVM_LIKELY(bn.isInternal())) {
            StreamSetBuffer * const buffer = bn.Buffer;
            buffer->setHandle(nullptr);
        }
    }
    #endif
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructStreamSetBuffers
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::constructStreamSetBuffers(BuilderRef /* b */) {

    mStreamSetInputBuffers.clear();
    const auto numOfInputStreams = out_degree(PipelineInput, mBufferGraph);
    mStreamSetInputBuffers.resize(numOfInputStreams);
    for (const auto e : make_iterator_range(out_edges(PipelineInput, mBufferGraph))) {
        const BufferPort & rd = mBufferGraph[e];
        const auto i = rd.Port.Number;
        const auto streamSet = target(e, mBufferGraph);
        assert (mBufferGraph[streamSet].isExternal());
        const auto j = streamSet - FirstStreamSet;
        StreamSetBuffer * const buffer = mInternalBuffers[j].release();
        assert (buffer == mBufferGraph[streamSet].Buffer);
        mStreamSetInputBuffers[i].reset(buffer);
    }

    mStreamSetOutputBuffers.clear();
    const auto numOfOutputStreams = in_degree(PipelineOutput, mBufferGraph);
    mStreamSetOutputBuffers.resize(numOfOutputStreams);
    for (const auto e : make_iterator_range(in_edges(PipelineOutput, mBufferGraph))) {
        const BufferPort & rd = mBufferGraph[e];
        const auto i = rd.Port.Number;
        const auto streamSet = source(e, mBufferGraph);
        assert (mBufferGraph[streamSet].isExternal());
        const auto j = streamSet - FirstStreamSet;
        StreamSetBuffer * const buffer = mInternalBuffers[j].release();
        assert (buffer == mBufferGraph[streamSet].Buffer);
        mStreamSetOutputBuffers[i].reset(buffer);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readProcessedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::readProcessedItemCounts(BuilderRef b) {
    for (const auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
        const BufferPort & br = mBufferGraph[e];
        const auto inputPort = br.Port;
        const auto prefix = makeBufferName(mKernelId, inputPort);
        Value * const processed = b->getScalarFieldPtr(prefix + ITEM_COUNT_SUFFIX);
        mProcessedItemCountPtr[inputPort] = processed;
        mInitiallyProcessedItemCount[inputPort] = b->CreateLoad(processed);
        if (br.IsDeferred) {
            Value * const deferred = b->getScalarFieldPtr(prefix + DEFERRED_ITEM_COUNT_SUFFIX);
            mProcessedDeferredItemCountPtr[inputPort] = deferred;
            mInitiallyProcessedDeferredItemCount[inputPort] = b->CreateLoad(deferred);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readProducedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::readProducedItemCounts(BuilderRef b) {

    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const auto streamSet = target(e, mBufferGraph);
        const BufferPort & br = mBufferGraph[e];
        const auto outputPort = br.Port;
        const auto prefix = makeBufferName(mKernelId, outputPort);
        Value * const produced = b->getScalarFieldPtr(prefix + ITEM_COUNT_SUFFIX);
        mProducedItemCountPtr[outputPort] = produced;
        mInitiallyProducedItemCount[streamSet] = b->CreateLoad(produced);
        if (br.IsDeferred) {
            Value * const deferred = b->getScalarField(prefix + DEFERRED_ITEM_COUNT_SUFFIX);
            mProducedDeferredItemCountPtr[outputPort] = deferred;
            mInitiallyProducedDeferredItemCount[streamSet] = b->CreateLoad(deferred);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeLocallyAvailableItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::initializeLocallyAvailableItemCounts(BuilderRef b, BasicBlock * const entryBlock) {
    IntegerType * const sizeTy = b->getSizeTy();
    for (auto streamSet = FirstStreamSet; streamSet <= LastStreamSet; ++streamSet) {
        PHINode * const phi = b->CreatePHI(sizeTy, 2);
        phi->addIncoming(mLocallyAvailableItems[streamSet], entryBlock);
        mInitiallyAvailableItemsPhi[streamSet] = phi;
        mLocallyAvailableItems[streamSet] = phi;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updateLocallyAvailableItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::updateLocallyAvailableItemCounts(BuilderRef b, BasicBlock * const entryBlock) {
    for (auto streamSet = FirstStreamSet; streamSet <= LastStreamSet; ++streamSet) {
        PHINode * const phi = mInitiallyAvailableItemsPhi[streamSet];
        phi->addIncoming(mLocallyAvailableItems[streamSet], entryBlock);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getTotalItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getLocallyAvailableItemCount(BuilderRef /* b */, const StreamSetPort inputPort) const {
    const auto streamSet = getInputBufferVertex(inputPort);
    return mLocallyAvailableItems[streamSet];
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getTotalItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::setLocallyAvailableItemCount(BuilderRef /* b */, const StreamSetPort outputPort, Value * const available) {
    const auto streamSet = getOutputBufferVertex(outputPort);
    mLocallyAvailableItems[streamSet] = available;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeUpdatedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeUpdatedItemCounts(BuilderRef b) {

    if (mKernelIsInternallySynchronized) {
        return;
    }

    for (const auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
        const BufferPort & br = mBufferGraph[e];
        const StreamSetPort inputPort = br.Port;
        b->CreateStore(mUpdatedProcessedPhi[inputPort], mProcessedItemCountPtr[inputPort]);
        #ifdef PRINT_DEBUG_MESSAGES
        const auto prefix = makeBufferName(mKernelId, inputPort);
        debugPrint(b, " @ writing " + prefix + "_processed = %" PRIu64, mUpdatedProcessedPhi[inputPort]);
        #endif

        if (br.IsDeferred) {
            b->CreateStore(mUpdatedProcessedDeferredPhi[inputPort], mProcessedDeferredItemCountPtr[inputPort]);
            #ifdef PRINT_DEBUG_MESSAGES
            debugPrint(b, " @ writing " + prefix + "_processed(deferred) = %" PRIu64, mUpdatedProcessedDeferredPhi[inputPort]);
            #endif
        }
    }

    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const BufferPort & br = mBufferGraph[e];
        const StreamSetPort outputPort = br.Port;
        b->CreateStore(mUpdatedProducedPhi[outputPort], mProducedItemCountPtr[outputPort]);
        #ifdef PRINT_DEBUG_MESSAGES
        const auto prefix = makeBufferName(mKernelId, outputPort);
        debugPrint(b, " @ writing " + prefix + "_produced = %" PRIu64, mUpdatedProducedPhi[outputPort]);
        #endif
        if (br.IsDeferred) {
            b->CreateStore(mUpdatedProducedDeferredPhi[outputPort], mProducedDeferredItemCountPtr[outputPort]);
            #ifdef PRINT_DEBUG_MESSAGES
            debugPrint(b, " @ writing " + prefix + "_produced(deferred) = %" PRIu64, mUpdatedProducedDeferredPhi[outputPort]);
            #endif
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief recordFinalProducedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::recordFinalProducedItemCounts(BuilderRef b) {
    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const BufferPort & br = mBufferGraph[e];
        const auto outputPort = br.Port;
        Value * fullyProduced = nullptr;
        if (LLVM_UNLIKELY(mKernelIsInternallySynchronized)) {
            fullyProduced = mProducedItemCount[outputPort];
        } else {
            fullyProduced = mFullyProducedItemCount[outputPort];
        }

        #ifdef PRINT_DEBUG_MESSAGES
        SmallVector<char, 256> tmp;
        raw_svector_ostream out(tmp);
        const auto prefix = makeBufferName(mKernelId, outputPort);
        out << " * -> " << prefix << "_avail = %" PRIu64;
        debugPrint(b, out.str(), fullyProduced);
        #endif

        setLocallyAvailableItemCount(b, outputPort, fullyProduced);
        initializeConsumedItemCount(b, outputPort, fullyProduced);
        #ifdef PRINT_DEBUG_MESSAGES
        const auto streamSet = getOutputBufferVertex(outputPort);
        Value * const producedDelta = b->CreateSub(fullyProduced, mInitiallyProducedItemCount[streamSet]);
        debugPrint(b, prefix + "_producedΔ = %" PRIu64, producedDelta);
        #endif
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readReturnedOutputVirtualBaseAddresses
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::readReturnedOutputVirtualBaseAddresses(BuilderRef b) const {
    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const auto streamSet = target(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[streamSet];
        // owned or external buffers do not have a mutable vba
        if (LLVM_LIKELY(bn.isOwned() || bn.isExternal())) {
            continue;
        }
        #ifdef PERMIT_BUFFER_MEMORY_REUSE
        assert (bn.Locality != BufferLocality::ThreadLocal);
        #endif
        const BufferPort & rd = mBufferGraph[e];
        const StreamSetPort port(rd.Port.Type, rd.Port.Number);
        Value * const ptr = mReturnedOutputVirtualBaseAddressPtr[port]; assert (ptr);
        Value * vba = b->CreateLoad(ptr);
        StreamSetBuffer * const buffer = bn.Buffer;
        vba = b->CreatePointerCast(vba, buffer->getPointerType());
        buffer->setBaseAddress(b.get(), vba);
        buffer->setCapacity(b.get(), mProducedItemCount[port]);
        const auto handleName = makeBufferName(mKernelId, port);
        #ifdef PRINT_DEBUG_MESSAGES
        debugPrint(b, "%s_updatedVirtualBaseAddress = 0x%" PRIx64, b->GetString(handleName), buffer->getBaseAddress(b));
        #endif
        b->setScalarField(handleName + LAST_GOOD_VIRTUAL_BASE_ADDRESS, vba);
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief loadLastGoodVirtualBaseAddressesOfUnownedBuffers
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::loadLastGoodVirtualBaseAddressesOfUnownedBuffers(BuilderRef b, const size_t kernelId) const {
    for (const auto e : make_iterator_range(out_edges(kernelId, mBufferGraph))) {
        const auto streamSet = target(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[streamSet];
        // owned or external buffers do not have a mutable vba
        if (LLVM_LIKELY(bn.isOwned() || bn.isExternal())) {
            continue;
        }
        #ifdef PERMIT_BUFFER_MEMORY_REUSE
        assert (bn.Locality != BufferLocality::ThreadLocal);
        #endif
        const BufferPort & rd = mBufferGraph[e];
        const auto handleName = makeBufferName(kernelId, rd.Port);
        Value * const vba = b->getScalarField(handleName + LAST_GOOD_VIRTUAL_BASE_ADDRESS);
        StreamSetBuffer * const buffer = bn.Buffer;
        buffer->setBaseAddress(b.get(), vba);
        #ifdef PRINT_DEBUG_MESSAGES
        debugPrint(b, "%s_loadPriorVirtualBaseAddress = 0x%" PRIx64, b->GetString(handleName), buffer->getBaseAddress(b));
        #endif
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeLookBehindLogic
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeLookBehindLogic(BuilderRef b) {
    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const auto streamSet = target(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[streamSet];
        const StreamSetBuffer * const buffer = bn.Buffer;
        if (bn.LookBehind) {
            const BufferPort & br = mBufferGraph[e];
            Constant * const underflow = b->getSize(bn.LookBehind);
            Value * const produced = mAlreadyProducedPhi[br.Port];
            Value * const capacity = buffer->getCapacity(b);
            Value * const producedOffset = b->CreateURem(produced, capacity);
            Value * const needsCopy = b->CreateICmpULE(producedOffset, underflow);
            #ifdef PRINT_DEBUG_MESSAGES
            const auto handleName = makeBufferName(mKernelId, br.Port);
            debugPrint(b, "%s_needsLookBehind = %" PRIx8, b->GetString(handleName), needsCopy);
            #endif
            copy(b, CopyMode::LookBehind, needsCopy, br.Port, buffer, bn.LookBehind);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeLookBehindReflectionLogic
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeDelayReflectionLogic(BuilderRef b) {
    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const BufferPort & br = mBufferGraph[e];
        if (br.Delay) {
            const auto streamSet = target(e, mBufferGraph);
            const BufferNode & bn = mBufferGraph[streamSet];
            const StreamSetBuffer * const buffer = bn.Buffer;
            Value * const capacity = buffer->getCapacity(b);
            Value * const produced = mAlreadyProducedPhi[br.Port];
            const auto size = round_up_to(br.Delay, b->getBitBlockWidth());
            Constant * const reflection = b->getSize(size);
            Value * const producedOffset = b->CreateURem(produced, capacity);
            Value * const needsCopy = b->CreateICmpULT(producedOffset, reflection);
            copy(b, CopyMode::Delay, needsCopy, br.Port, buffer, br.Delay);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeCopyBackLogic
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeCopyBackLogic(BuilderRef b) {
    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const auto streamSet = target(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[streamSet];
        if (bn.CopyBack) {
            const StreamSetBuffer * const buffer = bn.Buffer;
            const BufferPort & br = mBufferGraph[e];
            Value * const capacity = buffer->getCapacity(b);
            Value * const alreadyProduced = mAlreadyProducedPhi[br.Port];
            Value * const priorOffset = b->CreateURem(alreadyProduced, capacity);
            Value * const produced = mProducedItemCount[br.Port];
            Value * const producedOffset = b->CreateURem(produced, capacity);
            Value * const nonCapacityAlignedWrite = b->CreateIsNotNull(producedOffset);
            Value * const wroteToOverflow = b->CreateICmpULT(producedOffset, priorOffset);
            Value * const needsCopy = b->CreateAnd(nonCapacityAlignedWrite, wroteToOverflow);
            #ifdef PRINT_DEBUG_MESSAGES
            const auto handleName = makeBufferName(mKernelId, br.Port);
            debugPrint(b, "%s_needsCopyBack = %" PRIx8, b->GetString(handleName), needsCopy);
            #endif
            copy(b, CopyMode::CopyBack, needsCopy, br.Port, buffer, bn.CopyBack);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeLookAheadLogic
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeLookAheadLogic(BuilderRef b) {
    // Unless we modified the portion of data that ought to be reflected in the overflow region, do not copy
    // any data. To do so would incur extra writes and pollute the cache with potentially unnecessary data.
    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const auto streamSet = target(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[streamSet];
        if (bn.CopyForwards) {

            const StreamSetBuffer * const buffer = bn.Buffer;
            const BufferPort & br = mBufferGraph[e];
            Value * const capacity = buffer->getCapacity(b);
            Value * const initial = mInitiallyProducedItemCount[streamSet];
            Value * const produced = mUpdatedProducedPhi[br.Port];

            // If we wrote anything and it was not our first write to the buffer ...
            Value * overwroteData = b->CreateICmpUGT(produced, capacity);
            const Binding & output = getOutputBinding(br.Port);
            const ProcessingRate & rate = output.getRate();
            const Rational ONE(1, 1);
            bool mayProduceZeroItems = false;
            if (rate.getLowerBound() < ONE) {
                mayProduceZeroItems = true;
            } else if (rate.isRelative()) {
                const Binding & ref = getBinding(getReference(br.Port));
                const ProcessingRate & refRate = ref.getRate();
                mayProduceZeroItems = (rate.getLowerBound() * refRate.getLowerBound()) < ONE;
            }
            if (mayProduceZeroItems) {
                Value * const producedAnyOutput = b->CreateICmpNE(initial, produced);
                overwroteData = b->CreateAnd(overwroteData, producedAnyOutput);
            }

            // And we started writing within the first block ...
            Constant * const overflowSize = b->getSize(bn.CopyForwards);
            Value * const initialOffset = b->CreateURem(initial, capacity);
            Value * const startedWithinFirstBlock = b->CreateICmpULT(initialOffset, overflowSize);
            Value * const wroteToFirstBlock = b->CreateAnd(overwroteData, startedWithinFirstBlock);

            // And we started writing at the end of the buffer but wrapped over to the start of it,
            Value * const producedOffset = b->CreateURem(produced, capacity);
            Value * const wroteFromEndToStart = b->CreateICmpULT(producedOffset, initialOffset);

            // Then mirror the data in the overflow region.
            Value * const needsCopy = b->CreateOr(wroteToFirstBlock, wroteFromEndToStart);

            // TODO: optimize this further to ensure that we don't copy data that was just copied back from
            // the overflow. Should be enough just to have a "copyback flag" phi node to say it that was the
            // last thing it did to the buffer.
            #ifdef PRINT_DEBUG_MESSAGES
            const auto handleName = makeBufferName(mKernelId, br.Port);
            debugPrint(b, "%s_needsLookAhead = %" PRIx8, b->GetString(handleName), needsCopy);
            #endif
            copy(b, CopyMode::LookAhead, needsCopy, br.Port, buffer, bn.CopyForwards);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeOverflowCopy
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::copy(BuilderRef b, const CopyMode mode, Value * cond,
                            const StreamSetPort outputPort, const StreamSetBuffer * const buffer,
                            const unsigned itemsToCopy) {
    auto makeSuffix = [](CopyMode mode) {
        switch (mode) {
            case CopyMode::LookAhead: return "LookAhead";
            case CopyMode::CopyBack: return "CopyBack";
            case CopyMode::LookBehind: return "LookBehind";
            case CopyMode::Delay: return "Delay";
        }
        llvm_unreachable("unknown copy mode!");
    };

    const auto prefix = makeBufferName(mKernelId, outputPort) + "_" + makeSuffix(mode);

    const auto itemWidth = getItemWidth(buffer->getBaseType());
    assert (is_power_2(itemWidth));
    const auto blockWidth = b->getBitBlockWidth();

    const auto bitsToCopy = round_up_to(itemsToCopy * itemWidth, blockWidth);
    const auto bitsPerStream = round_up_to(itemsToCopy, blockWidth) * itemWidth;

    Value * const numOfStreams = buffer->getStreamSetCount(b);
    ConstantInt * const bytesToCopy = b->getSize(bitsToCopy / 8);

    BasicBlock * const copyStart = b->CreateBasicBlock(prefix, mKernelExit);
    BasicBlock * copyLoop = nullptr;
    if ((bitsToCopy < bitsPerStream) && !(isa<ConstantInt>(numOfStreams) && cast<ConstantInt>(numOfStreams)->isOne())) {
        copyLoop = b->CreateBasicBlock(prefix + "Loop", mKernelExit);
    }
    BasicBlock * const copyExit = b->CreateBasicBlock(prefix + "Exit", mKernelExit);

    b->CreateUnlikelyCondBr(cond, copyStart, copyExit);

    b->SetInsertPoint(copyStart);
    #ifdef ENABLE_PAPI
    readPAPIMeasurement(b, mKernelId, PAPIReadBeforeMeasurementArray);
    #endif
    Value * const beforeCopy = startCycleCounter(b);

    Value * source = buffer->getOverflowAddress(b);
    Value * target = buffer->getMallocAddress(b);

    PointerType * const int8PtrTy = b->getInt8PtrTy();
    source = b->CreatePointerCast(source, int8PtrTy);
    target = b->CreatePointerCast(target, int8PtrTy);

    ConstantInt * const bytesPerStream = b->getSize(bitsPerStream / 8);

    Value * const totalBytesPerStreamSetBlock = b->CreateMul(bytesPerStream, numOfStreams);

    if (mode == CopyMode::LookBehind || mode == CopyMode::Delay) {
        Value * const offset = b->CreateNeg(totalBytesPerStreamSetBlock);
        source = b->CreateInBoundsGEP(source, offset);
        target = b->CreateInBoundsGEP(target, offset);
    }

    if (mode == CopyMode::LookAhead || mode == CopyMode::Delay) {
        std::swap(target, source);
    }

    assert (bitsToCopy >= blockWidth);

    const auto align = blockWidth / 8;

    if (copyLoop) {

        BasicBlock * recordCopyCycleCount = nullptr;
        if (EnableCycleCounter || EnablePAPICounters) {
            recordCopyCycleCount = b->CreateBasicBlock(prefix + "RecordCycleCount", copyExit);
        }

        b->CreateBr(copyLoop);

        b->SetInsertPoint(copyLoop);
        PHINode * const idx = b->CreatePHI(b->getSizeTy(), 2);
        idx->addIncoming(b->getSize(0), copyStart);
        Value * const offset = b->CreateMul(idx, bytesPerStream);
        Value * const sourcePtr = b->CreateGEP(source, offset);
        Value * const targetPtr = b->CreateGEP(target, offset);

        #ifdef PRINT_DEBUG_MESSAGES
        debugPrint(b, prefix + "_copying %" PRIu64 " bytes from %" PRIx64 " to %" PRIx64 " (align=%" PRIu64 ")", bytesToCopy, sourcePtr, targetPtr, b->getSize(align));
        #endif

        b->CreateMemCpy(targetPtr, sourcePtr, bytesToCopy, align);

        Value * const nextIdx = b->CreateAdd(idx, b->getSize(1));
        idx->addIncoming(nextIdx, copyLoop);
        Value * const done = b->CreateICmpEQ(nextIdx, numOfStreams);

        BasicBlock * const loopExit = EnableCycleCounter ? recordCopyCycleCount : copyExit;
        b->CreateCondBr(done, loopExit, copyLoop);

        if (EnableCycleCounter || EnablePAPICounters) {
            b->SetInsertPoint(recordCopyCycleCount);
            updateCycleCounter(b, mKernelId, beforeCopy, CycleCounter::BUFFER_COPY);
            #ifdef ENABLE_PAPI
            accumPAPIMeasurementWithoutReset(b, PAPIReadBeforeMeasurementArray, mKernelId, PAPIKernelCounter::PAPI_BUFFER_COPY);
            #endif
            b->CreateBr(copyExit);
        }

    } else {

        #ifdef PRINT_DEBUG_MESSAGES
        debugPrint(b, prefix + "_segment_copying %" PRIu64 "x%" PRIu64 "=%" PRIu64 " bytes "
                      "from %" PRIx64 " to %" PRIx64 " (align=%" PRIu64 ")",
                   bytesPerStream, numOfStreams, totalBytesPerStreamSetBlock, source, target, b->getSize(align));
        #endif

        b->CreateMemCpy(target, source, totalBytesPerStreamSetBlock, align);
        if (EnableCycleCounter || EnablePAPICounters) {
            updateCycleCounter(b, mKernelId, beforeCopy, CycleCounter::BUFFER_COPY);
            #ifdef ENABLE_PAPI
            accumPAPIMeasurementWithoutReset(b, PAPIReadBeforeMeasurementArray, mKernelId, PAPIKernelCounter::PAPI_BUFFER_COPY);
            #endif
        }
        b->CreateBr(copyExit);

    }

    b->SetInsertPoint(copyExit);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief prepareLinearBuffers
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::prepareLinearThreadLocalOutputBuffers(BuilderRef b) {
    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const auto streamSet = target(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[streamSet];
        if (LLVM_UNLIKELY(bn.Locality == BufferLocality::ThreadLocal && bn.IsLinear)) {
            Value * const produced = mInitiallyProducedItemCount[streamSet];
            // purely threadlocal buffers are guaranteed to consume every produced
            // item each segment.
            bn.Buffer->copyBackLinearOutputBuffer(b, produced);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getVirtualBaseAddress
 *
 * Returns the address of the "zeroth" item of the (logically-unbounded) stream set.
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getVirtualBaseAddress(BuilderRef b,
                                                const BufferPort & rateData,
                                                const BufferNode & bufferNode,
                                                Value * position,
                                                Value * isFinal) const {


    const StreamSetBuffer * const buffer = bufferNode.Buffer;
    assert ("buffer cannot be null!" && buffer);
    Value * const baseAddress = buffer->getBaseAddress(b);
    if (bufferNode.isUnowned()) {
        assert (bufferNode.Locality != BufferLocality::ThreadLocal);
        return baseAddress;
    }

    Constant * const LOG_2_BLOCK_WIDTH = b->getSize(floor_log2(b->getBitBlockWidth()));
    Constant * const ZERO = b->getSize(0);
    PointerType * const bufferType = buffer->getPointerType();
    Value * const blockIndex = b->CreateLShr(position, LOG_2_BLOCK_WIDTH);

    if (LLVM_UNLIKELY(CheckAssertions)) {
        const Binding & binding = rateData.Binding;
        b->CreateAssert(baseAddress, "%s.%s: baseAddress cannot be null",
                        mCurrentKernelName,
                        b->GetString(binding.getName()));
    }

    Value * const address = buffer->getStreamLogicalBasePtr(b, baseAddress, ZERO, blockIndex);
    return b->CreatePointerCast(address, bufferType);
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputVirtualBaseAddresses
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::getInputVirtualBaseAddresses(BuilderRef b, Vec<Value *> & baseAddresses) const {
    for (const auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
        const BufferPort & rt = mBufferGraph[e];
        PHINode * processed = nullptr;
        if (mAlreadyProcessedDeferredPhi[rt.Port]) {
            processed = mAlreadyProcessedDeferredPhi[rt.Port];
        } else {
            processed = mAlreadyProcessedPhi[rt.Port];
        }
        const auto buffer = source(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[buffer];
        assert (isFromCurrentFunction(b, bn.Buffer->getHandle()));
        baseAddresses[rt.Port.Number] = getVirtualBaseAddress(b, rt, bn, processed, nullptr);
    }
}

} // end of kernel namespace

#endif // BUFFER_ALLOCATION_HPP
