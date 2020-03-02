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
        const BufferRateData & rd = mBufferGraph[e];
        const auto handleName = makeBufferName(index, rd.Port);
        StreamSetBuffer * const buffer = bn.Buffer;
        Type * const handleType = buffer->getHandleType(b);



        if (LLVM_LIKELY(bn.isOwned())) {
            //if (LLVM_UNLIKELY(bn.NonLocal)) {
                mTarget->addInternalScalar(handleType, handleName);
            //} else {
            //    mTarget->addThreadLocalScalar(handleType, handleName);
            //}
        } else {
            mTarget->addNonPersistentScalar(handleType, handleName);
            mTarget->addInternalScalar(buffer->getPointerType(), handleName + LAST_GOOD_VIRTUAL_BASE_ADDRESS);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief loadInternalStreamSetHandles
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::loadInternalStreamSetHandles(BuilderRef b) {
    for (auto streamSet = FirstStreamSet; streamSet <= LastStreamSet; ++streamSet) {
        const BufferNode & bn = mBufferGraph[streamSet];
        // external buffers already have a buffer handle
        StreamSetBuffer * const buffer = bn.Buffer;
        if (LLVM_UNLIKELY(bn.isExternal())) {
            assert (isFromCurrentFunction(b, buffer->getHandle()));
            continue;
        }
        const auto pe = in_edge(streamSet, mBufferGraph);
        const auto producer = source(pe, mBufferGraph);
        const BufferRateData & rd = mBufferGraph[pe];
        const auto handleName = makeBufferName(producer, rd.Port);
        Value * const handle = b->getScalarFieldPtr(handleName);
        assert (buffer->getHandle() == nullptr);
        buffer->setHandle(handle);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief allocateOwnedBuffers
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::allocateOwnedBuffers(BuilderRef b, Value * const expectedNumOfStrides, const bool nonLocal) {
    if (!nonLocal) return;

    assert (expectedNumOfStrides);

    if (LLVM_UNLIKELY(mCheckAssertions)) {
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
                setActiveKernel(b, i);
                assert (mKernel == kernelObj);
                SmallVector<Value *, 3> params;
                if (LLVM_LIKELY(kernelObj->isStateful())) {
                    params.push_back(mKernelHandle);
                }
                Value * func = nullptr;
                if (nonLocal) {
                    func = getKernelAllocateSharedInternalStreamSetsFunction(b);
                } else {
                    func = getKernelAllocateThreadLocalInternalStreamSetsFunction(b);
                    params.push_back(b->CreateLoad(getThreadLocalHandlePtr(b, mKernelId)));
                }
                params.push_back(b->CreateCeilUMulRate(expectedNumOfStrides, MaximumNumOfStrides[i]));
                b->CreateCall(func, params);
            }
        }
        // and allocate any output buffers
        for (const auto e : make_iterator_range(out_edges(i, mBufferGraph))) {
            const auto j = target(e, mBufferGraph);
            const BufferNode & bn = mBufferGraph[j];
            if (LLVM_UNLIKELY(bn.isOwned())) { // && bn.NonLocal == nonLocal
                StreamSetBuffer * const buffer = bn.Buffer;
                if (LLVM_LIKELY(bn.isInternal())) {
                    const BufferRateData & rd = mBufferGraph[e];
                    const auto handleName = makeBufferName(i, rd.Port);
                    Value * const handle = b->getScalarFieldPtr(handleName);
                    buffer->setHandle(handle);
                }
                assert (isFromCurrentFunction(b, buffer->getHandle(), false));
                buffer->allocateBuffer(b, expectedNumOfStrides);
            }
        }
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief releaseOwnedBuffers
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::releaseOwnedBuffers(BuilderRef b, const bool nonLocal) {
    if (!nonLocal) return;
    for (auto i = FirstStreamSet; i <= LastStreamSet; ++i) {
        const BufferNode & bn = mBufferGraph[i];
        if (LLVM_LIKELY(bn.isOwned())) { // && bn.NonLocal == nonLocal
            StreamSetBuffer * const buffer = bn.Buffer;
            assert (isFromCurrentFunction(b, buffer->getHandle(), false));
            buffer->releaseBuffer(b);
            if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::TraceDynamicBuffers))) {
                if (isa<DynamicBuffer>(buffer)) {

                    const auto pe = in_edge(i, mBufferGraph);
                    const auto p = source(pe, mBufferGraph);
                    const BufferRateData & rd = mBufferGraph[pe];
                    const auto prefix = makeBufferName(p, rd.Port);

                    Value * const traceData = b->getScalarFieldPtr(prefix + STATISTICS_BUFFER_EXPANSION_SUFFIX);
                    Constant * const ZERO = b->getInt32(0);
                    b->CreateFree(b->CreateLoad(b->CreateInBoundsGEP(traceData, {ZERO, ZERO})));
                }
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
        const BufferRateData & rd = mBufferGraph[e];
        const auto i = rd.Port.Number;
        const auto b = target(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[b];
        assert (bn.isExternal());
        mStreamSetInputBuffers[i].reset(bn.Buffer);
    }

    mStreamSetOutputBuffers.clear();
    const auto numOfOutputStreams = in_degree(PipelineOutput, mBufferGraph);
    mStreamSetOutputBuffers.resize(numOfOutputStreams);
    for (const auto e : make_iterator_range(in_edges(PipelineOutput, mBufferGraph))) {
        const BufferRateData & rd = mBufferGraph[e];
        const auto i = rd.Port.Number;
        const auto b = source(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[b];
        assert (bn.isExternal());
        mStreamSetOutputBuffers[i].reset(bn.Buffer);
    }

    mInternalBuffers.reserve(LastStreamSet - FirstStreamSet + 1);
    for (auto i = FirstStreamSet; i <= LastStreamSet; ++i) {
        const BufferNode & bn = mBufferGraph[i];
        if (LLVM_LIKELY(bn.isInternal())) {
            mInternalBuffers.emplace_back(bn.Buffer);
        }
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readProcessedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::readProcessedItemCounts(BuilderRef b) {
    for (const auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
        const BufferRateData & br = mBufferGraph[e];
        const auto inputPort = br.Port;
        const Binding & input = br.Binding;
        const auto prefix = makeBufferName(mKernelId, inputPort);
        Value * const processed = b->getScalarField(prefix + ITEM_COUNT_SUFFIX);
        mInitiallyProcessedItemCount(mKernelId, inputPort) = processed;
        if (input.isDeferred()) {
            Value * const deferred = b->getScalarField(prefix + DEFERRED_ITEM_COUNT_SUFFIX);
            mInitiallyProcessedDeferredItemCount(mKernelId, inputPort) = deferred;
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readProducedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::readProducedItemCounts(BuilderRef b) {

    for (const auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
        const BufferRateData & br = mBufferGraph[e];
        const auto outputPort = br.Port;
        const Binding & output = br.Binding;
        const auto prefix = makeBufferName(mKernelId, outputPort);
        const auto streamSet = getOutputBufferVertex(mKernelId, outputPort);
        mInitiallyProducedItemCount[streamSet] = b->getScalarField(prefix + ITEM_COUNT_SUFFIX);
        #ifdef PRINT_DEBUG_MESSAGES
        debugPrint(b, prefix + "_initiallyProduced = %" PRIu64, mInitiallyProducedItemCount[streamSet]);
        #endif
        if (output.isDeferred()) {
            Value * const deferred = b->getScalarField(prefix + DEFERRED_ITEM_COUNT_SUFFIX);
            mInitiallyProducedDeferredItemCount(mKernelId, outputPort) = deferred;
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeUpdatedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeUpdatedItemCounts(BuilderRef b, const ItemCountSource source) {

    auto getProcessedArg = [&](const StreamSetPort port) -> Value * {
        switch (source) {
            case ItemCountSource::ComputedAtKernelCall:
                return mProcessedItemCount(mKernelId, port);
            case ItemCountSource::UpdatedItemCountsFromLoopExit:
                return mUpdatedProcessedPhi(mKernelId, port);
        }
        llvm_unreachable("unknown source type");
    };
    auto getProcessedDeferredArg = [&](const StreamSetPort port) -> Value * {
        switch (source) {
            case ItemCountSource::ComputedAtKernelCall:
                return mProcessedDeferredItemCount(mKernelId, port);
            case ItemCountSource::UpdatedItemCountsFromLoopExit:
                return mUpdatedProcessedDeferredPhi(mKernelId, port);
        }
        llvm_unreachable("unknown source type");
    };
    auto getProducedArg = [&](const StreamSetPort port) -> Value * {
        switch (source) {
            case ItemCountSource::ComputedAtKernelCall:
                return mProducedItemCount(mKernelId, port);
            case ItemCountSource::UpdatedItemCountsFromLoopExit:
                return mUpdatedProducedPhi(mKernelId, port);
        }
        llvm_unreachable("unknown source type");
    };
    auto getProducedDeferredArg = [&](const StreamSetPort port) -> Value * {
        switch (source) {
            case ItemCountSource::ComputedAtKernelCall:
                return mProducedDeferredItemCount(mKernelId, port);
            case ItemCountSource::UpdatedItemCountsFromLoopExit:
                return mUpdatedProducedDeferredPhi(mKernelId, port);
        }
        llvm_unreachable("unknown source type");
    };

    const auto numOfInputs = getNumOfStreamInputs(mKernelId);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const StreamSetPort inputPort{PortType::Input, i};
        const Binding & input = getInputBinding(inputPort);
        const auto prefix = makeBufferName(mKernelId, inputPort);
        b->setScalarField(prefix + ITEM_COUNT_SUFFIX, getProcessedArg(inputPort));
        if (input.isDeferred()) {
            b->setScalarField(prefix + DEFERRED_ITEM_COUNT_SUFFIX, getProcessedDeferredArg(inputPort));
        }
    }

    const auto numOfOutputs = getNumOfStreamOutputs(mKernelId);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const StreamSetPort outputPort{PortType::Output, i};
        const Binding & output = getOutputBinding(outputPort);
        const auto prefix = makeBufferName(mKernelId, outputPort);
        b->setScalarField(prefix + ITEM_COUNT_SUFFIX, getProducedArg(outputPort));
        if (output.isDeferred()) {
            b->setScalarField(prefix + DEFERRED_ITEM_COUNT_SUFFIX, getProducedDeferredArg(outputPort));
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief recordFinalProducedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::recordFinalProducedItemCounts(BuilderRef b) {
    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const auto outputPort = mBufferGraph[e].Port;
        Value * const fullyProduced = mFullyProducedItemCount(outputPort);
        setLocallyAvailableItemCount(b, outputPort, fullyProduced);
        initializeConsumedItemCount(b, outputPort, fullyProduced);
        #ifdef PRINT_DEBUG_MESSAGES
        const auto prefix = makeBufferName(mKernelId, outputPort);
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
        const BufferRateData & rd = mBufferGraph[e];
        const StreamSetPort port(rd.Port.Type, rd.Port.Number);
        Value * const ptr = mReturnedOutputVirtualBaseAddressPtr(mKernelId, port);
        assert (ptr);
        Value * vba = b->CreateLoad(ptr);
        StreamSetBuffer * const buffer = bn.Buffer;
        vba = b->CreatePointerCast(vba, buffer->getPointerType());
        buffer->setBaseAddress(b.get(), vba);
        buffer->setCapacity(b.get(), mProducedItemCount(mKernelId, port));
        const auto handleName = makeBufferName(mKernelId, port);
        #ifdef PRINT_DEBUG_MESSAGES
        debugPrint(b, handleName + "_virtualBaseAddress = %" PRIu64, vba);
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
        const BufferRateData & rd = mBufferGraph[e];
        const auto handleName = makeBufferName(kernelId, rd.Port);
        Value * const vba = b->getScalarField(handleName + LAST_GOOD_VIRTUAL_BASE_ADDRESS);
        StreamSetBuffer * const buffer = bn.Buffer;
        buffer->setBaseAddress(b.get(), vba);
    }
}

// TODO: copyback/copyforward ought to reflect exact num of items; not upper bound of space

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief requiresCopyBack
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool PipelineCompiler::requiresCopyBack(const unsigned bufferVertex) const {
    return getCopyBack(bufferVertex) != 0;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getCopyBack
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned PipelineCompiler::getCopyBack(const unsigned bufferVertex) const {
    return mBufferGraph[bufferVertex].CopyBack;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief requiresLookAhead
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool PipelineCompiler::requiresLookAhead(const unsigned bufferVertex) const {
    return getLookAhead(bufferVertex) != 0;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getLookAhead
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned PipelineCompiler::getLookAhead(const unsigned bufferVertex) const {
    return mBufferGraph[bufferVertex].LookAhead;
}

#warning Linear buffers do not correctly handle lookbehind

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeLookBehindLogic
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeLookBehindLogic(BuilderRef b) {
    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const auto streamSet = target(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[streamSet];
        if (bn.LookBehind) {
            const StreamSetBuffer * const buffer = bn.Buffer;
            const BufferRateData & br = mBufferGraph[e];
            Value * needsCopy = nullptr;
            Constant * const underflow = b->getSize(bn.LookBehind);
            if (LLVM_UNLIKELY(buffer->isLinear())) {
                Value * const consumed = mConsumedItemCount[streamSet];
                needsCopy = b->CreateICmpUGT(consumed, underflow);
            } else {
                Value * const produced = mAlreadyProducedPhi(br.Port);
                Value * const capacity = buffer->getCapacity(b);
                Value * const producedOffset = b->CreateURem(produced, capacity);
                needsCopy = b->CreateICmpULT(producedOffset, underflow);
            }
            copy(b, CopyMode::LookBehind, needsCopy, br.Port, bn.Buffer, bn.LookBehind);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeLookBehindReflectionLogic
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeLookBehindReflectionLogic(BuilderRef b) {
    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const auto streamSet = target(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[streamSet];
        if (bn.LookBehindReflection) {
            const StreamSetBuffer * const buffer = bn.Buffer;
            Value * const capacity = buffer->getCapacity(b);
            const BufferRateData & br = mBufferGraph[e];
            Value * const produced = mAlreadyProducedPhi(mKernelId, br.Port);
            Constant * const reflection = b->getSize(bn.LookBehindReflection);
            Value * const producedOffset = b->CreateURem(produced, capacity);
            Value * const needsCopy = b->CreateICmpULT(producedOffset, reflection);
            copy(b, CopyMode::LookBehindReflection, needsCopy, br.Port, bn.Buffer, bn.LookBehindReflection);
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
            Value * const capacity = buffer->getCapacity(b);
            const BufferRateData & br = mBufferGraph[e];
            Value * const alreadyProduced = mAlreadyProducedPhi(mKernelId, br.Port);
            Value * const priorOffset = b->CreateURem(alreadyProduced, capacity);
            Value * const produced = mProducedItemCount(mKernelId, br.Port);
            Value * const producedOffset = b->CreateURem(produced, capacity);
            Value * const nonCapacityAlignedWrite = b->CreateIsNotNull(producedOffset);
            Value * const wroteToOverflow = b->CreateICmpULT(producedOffset, priorOffset);
            Value * const needsCopy = b->CreateAnd(nonCapacityAlignedWrite, wroteToOverflow);
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
        if (bn.LookAhead) {
            const StreamSetBuffer * const buffer = bn.Buffer;
            Value * const capacity = buffer->getCapacity(b);
            const BufferRateData & br = mBufferGraph[e];
            Value * const initial = mInitiallyProducedItemCount[streamSet];
            Value * const produced = mUpdatedProducedPhi(br.Port);

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
            if (LLVM_LIKELY(mayProduceZeroItems)) {
                Value * const producedOutput = b->CreateICmpNE(initial, produced);
                overwroteData = b->CreateAnd(overwroteData, producedOutput);
            }

            // And we started writing within the first block ...
            assert (bn.LookAhead <= buffer->getOverflowCapacity(b));
            Constant * const overflowSize = b->getSize(bn.LookAhead);
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

            copy(b, CopyMode::LookAhead, needsCopy, br.Port, buffer, bn.LookAhead);
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
            case CopyMode::LookBehindReflection: return "Reflection";
        }
        llvm_unreachable("unknown copy mode!");
    };

    const auto prefix = makeBufferName(mKernelId, outputPort) + "_copy" + makeSuffix(mode);

    BasicBlock * const copyStart = b->CreateBasicBlock(prefix, mKernelExit);
    BasicBlock * const copyExit = b->CreateBasicBlock(prefix + "Exit", mKernelExit);

    b->CreateUnlikelyCondBr(cond, copyStart, copyExit);

    b->SetInsertPoint(copyStart);

    startCycleCounter(b, CycleCounter::BEFORE_COPY);

    const auto itemWidth = getItemWidth(buffer->getBaseType());
    const auto blockWidth = b->getBitBlockWidth();
    const auto bitsToCopy = round_up_to(itemsToCopy * itemWidth, blockWidth);

    Value * const numOfStreams = buffer->getStreamSetCount(b);
    Value * const bytesPerSteam = b->getSize(bitsToCopy / 8);
    Value * const bytesToCopy = b->CreateMul(bytesPerSteam, numOfStreams);

    #ifdef PRINT_DEBUG_MESSAGES
    debugPrint(b, prefix + std::to_string(itemsToCopy) + "_bytesToCopy = %" PRIu64, bytesToCopy);
    #endif

    Value * source = nullptr;
    Value * target = nullptr;

    PointerType * const int8PtrTy = b->getInt8PtrTy();

    // TODO: Wrong base address here. Needs base from prior to linearize

    if (buffer->isLinear()) {
        source = buffer->getBaseAddress(b);
        source = b->CreatePointerCast(source, int8PtrTy);
        const auto streamSet = getOutputBufferVertex(outputPort);
        Value * offset = b->CreateMul(mConsumedItemCount[streamSet], bytesPerSteam);
        source = b->CreateGEP(source, offset);
        target = buffer->getMallocAddress(b);
    } else {
        source = buffer->getOverflowAddress(b);
        target = buffer->getBaseAddress(b);
    }

    if (mode == CopyMode::LookBehind || mode == CopyMode::LookBehindReflection) {
        DataLayout DL(b->getModule());
        Type * const intPtrTy = DL.getIntPtrType(source->getType());
        Value * const offset = b->CreateNeg(b->CreateZExt(bytesToCopy, intPtrTy));
        source = b->CreatePointerCast(source, int8PtrTy);
        source = b->CreateInBoundsGEP(source, offset);
        target = b->CreatePointerCast(target, int8PtrTy);
        target = b->CreateInBoundsGEP(target, offset);
    }

    if (mode == CopyMode::LookAhead || mode == CopyMode::LookBehindReflection) {
        std::swap(target, source);
    }

    b->CreateMemCpy(target, source, bytesToCopy, blockWidth / 8);

    updateCycleCounter(b, CycleCounter::BEFORE_COPY, CycleCounter::AFTER_COPY);

    b->CreateBr(copyExit);

    b->SetInsertPoint(copyExit);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief prepareLinearBuffers
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::prepareLinearBuffers(BuilderRef b) const {
    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const auto streamSet = target(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[streamSet];
        const StreamSetBuffer * const buffer = bn.Buffer;
        if (buffer->isLinear()) {
            Value * const produced = mInitiallyProducedItemCount[streamSet];
            Value * const consumed = mConsumedItemCount[streamSet];
            #ifdef PRINT_DEBUG_MESSAGES
            const BufferRateData & br = mBufferGraph[e];
            const auto prefix = makeBufferName(mKernelId, br.Port);
            debugPrint(b, prefix + "_initiallyProduced = %" PRIu64, produced);
            debugPrint(b, prefix + "_consumed = %" PRIu64, consumed);
            #endif
            buffer->prepareLinearBuffer(b, produced, consumed);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getVirtualBaseAddress
 *
 * Returns the address of the "zeroth" item of the (logically-unbounded) stream set.
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getVirtualBaseAddress(BuilderRef b,
                                                const Binding & binding,
                                                const StreamSetBuffer * const buffer,
                                                Value * const position) const {
    assert ("buffer cannot be null!" && buffer);
    Constant * const LOG_2_BLOCK_WIDTH = b->getSize(floor_log2(b->getBitBlockWidth()));
    Constant * const ZERO = b->getSize(0);
    PointerType * const bufferType = buffer->getPointerType();
    Value * const blockIndex = b->CreateLShr(position, LOG_2_BLOCK_WIDTH);
    Value * const baseAddress = buffer->getBaseAddress(b);
    if (LLVM_UNLIKELY(mCheckAssertions)) {
        b->CreateAssert(baseAddress, "%s.%s: baseAddress cannot be null",
                        mKernelAssertionName,
                        b->GetString(binding.getName()));
    }
    Value * address = buffer->getStreamLogicalBasePtr(b, baseAddress, ZERO, blockIndex);
    return b->CreatePointerCast(address, bufferType);
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputVirtualBaseAddresses
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::getInputVirtualBaseAddresses(BuilderRef b, Vec<Value *> & baseAddresses) const {
    for (const auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
        const BufferRateData & rt = mBufferGraph[e];
        assert (rt.Port.Type == PortType::Input);
        PHINode * processed = nullptr;
        if (mAlreadyProcessedDeferredPhi(rt.Port)) {
            processed = mAlreadyProcessedDeferredPhi(rt.Port);
        } else {
            processed = mAlreadyProcessedPhi(rt.Port);
        }
        const auto buffer = source(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[buffer];
        assert (isFromCurrentFunction(b, bn.Buffer->getHandle()));
        baseAddresses[rt.Port.Number] = getVirtualBaseAddress(b, rt.Binding, bn.Buffer, processed);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputBufferVertex
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned PipelineCompiler::getInputBufferVertex(const StreamSetPort inputPort) const {
    return getInputBufferVertex(mKernelId, inputPort);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputBufferVertex
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned PipelineCompiler::getInputBufferVertex(const size_t kernelVertex, const StreamSetPort inputPort) const {
    return source(getInput(kernelVertex, inputPort), mBufferGraph);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputBuffer
 ** ------------------------------------------------------------------------------------------------------------- */
inline StreamSetBuffer * PipelineCompiler::getInputBuffer(const StreamSetPort inputPort) const {
    return getInputBuffer(mKernelId, inputPort);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputBuffer
 ** ------------------------------------------------------------------------------------------------------------- */
StreamSetBuffer * PipelineCompiler::getInputBuffer(const size_t kernelVertex, const StreamSetPort inputPort) const {
    return mBufferGraph[getInputBufferVertex(kernelVertex, inputPort)].Buffer;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputBinding
 ** ------------------------------------------------------------------------------------------------------------- */
const Binding & PipelineCompiler::getInputBinding(const size_t kernelVertex, const StreamSetPort inputPort) const {

    RelationshipGraph::vertex_descriptor v;
    RelationshipGraph::edge_descriptor e;

    graph_traits<RelationshipGraph>::in_edge_iterator ei, ei_end;
    std::tie(ei, ei_end) = in_edges(kernelVertex, mStreamGraph);
    assert (inputPort.Number < static_cast<size_t>(std::distance(ei, ei_end)));
    e = *(ei + inputPort.Number);
    v = source(e, mStreamGraph);

    assert (static_cast<StreamSetPort>(mStreamGraph[e]) == inputPort);
    const RelationshipNode & rn = mStreamGraph[v];
    assert (rn.Type == RelationshipNode::IsBinding);
    return rn.Binding;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputBinding
 ** ------------------------------------------------------------------------------------------------------------- */
inline const Binding & PipelineCompiler::getInputBinding(const StreamSetPort inputPort) const {
    return getInputBinding(mKernelId, inputPort);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getProducerOutputBinding
 ** ------------------------------------------------------------------------------------------------------------- */
inline const Binding & PipelineCompiler::getProducerOutputBinding(const StreamSetPort inputPort) const {
    const auto buffer = getInputBufferVertex(inputPort);
    const BufferRateData & br = mBufferGraph[in_edge(buffer, mBufferGraph)];
    return br.Binding;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInput
 ** ------------------------------------------------------------------------------------------------------------- */
inline const BufferGraph::edge_descriptor PipelineCompiler::getInput(const size_t kernelVertex, const StreamSetPort inputPort) const {
    assert (inputPort.Type == PortType::Input);
    assert (inputPort.Number < in_degree(kernelVertex, mBufferGraph));
    for (const auto e : make_iterator_range(in_edges(kernelVertex, mBufferGraph))) {
        const BufferRateData & br = mBufferGraph[e];
        if (br.Port.Number == inputPort.Number) {
            return e;
        }
    }
    llvm_unreachable("could not find input port");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputBufferVertex
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned PipelineCompiler::getOutputBufferVertex(const StreamSetPort outputPort) const {
    return getOutputBufferVertex(mKernelId, outputPort);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputBufferVertex
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned PipelineCompiler::getOutputBufferVertex(const size_t kernelVertex, const StreamSetPort outputPort) const {
    return target(getOutput(kernelVertex, outputPort), mBufferGraph);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputBinding
 ** ------------------------------------------------------------------------------------------------------------- */
const Binding & PipelineCompiler::getOutputBinding(const size_t kernelVertex, const StreamSetPort outputPort) const {

    RelationshipGraph::vertex_descriptor v;
    RelationshipGraph::edge_descriptor e;

    graph_traits<RelationshipGraph>::out_edge_iterator ei, ei_end;
    std::tie(ei, ei_end) = out_edges(kernelVertex, mStreamGraph);
    assert (outputPort.Number < static_cast<size_t>(std::distance(ei, ei_end)));
    e = *(ei + outputPort.Number);
    v = target(e, mStreamGraph);

    assert (static_cast<StreamSetPort>(mStreamGraph[e]) == outputPort);

    const RelationshipNode & rn = mStreamGraph[v];
    assert (rn.Type == RelationshipNode::IsBinding);
    return rn.Binding;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputBinding
 ** ------------------------------------------------------------------------------------------------------------- */
inline const Binding & PipelineCompiler::getOutputBinding(const StreamSetPort outputPort) const {
    return getOutputBinding(mKernelId, outputPort);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputBuffer
 ** ------------------------------------------------------------------------------------------------------------- */
inline StreamSetBuffer * PipelineCompiler::getOutputBuffer(const StreamSetPort outputPort) const {
    return getOutputBuffer(mKernelId, outputPort);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputBuffer
 ** ------------------------------------------------------------------------------------------------------------- */
StreamSetBuffer * PipelineCompiler::getOutputBuffer(const size_t kernelVertex, const StreamSetPort outputPort) const {
    return mBufferGraph[getOutputBufferVertex(kernelVertex, outputPort)].Buffer;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInput
 ** ------------------------------------------------------------------------------------------------------------- */
inline const BufferGraph::edge_descriptor PipelineCompiler::getOutput(const size_t kernelVertex, const StreamSetPort outputPort) const {
    assert (outputPort.Type == PortType::Output);
    assert (outputPort.Number < out_degree(kernelVertex, mBufferGraph));
    for (const auto e : make_iterator_range(out_edges(kernelVertex, mBufferGraph))) {
        const BufferRateData & br = mBufferGraph[e];
        if (br.Port.Number == outputPort.Number) {
            return e;
        }
    }
    llvm_unreachable("could not find output port");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNumOfStreamInputs
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned PipelineCompiler::getNumOfStreamInputs(const unsigned kernel) const {
    return in_degree(kernel, mStreamGraph);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNumOfStreamOutputs
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned PipelineCompiler::getNumOfStreamOutputs(const unsigned kernel) const {
    return out_degree(kernel, mStreamGraph);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getBinding
 ** ------------------------------------------------------------------------------------------------------------- */
inline const Binding & PipelineCompiler::getBinding(const StreamSetPort port) const {
    return getBinding(mKernelId, port);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getBinding
 ** ------------------------------------------------------------------------------------------------------------- */
const Binding & PipelineCompiler::getBinding(const unsigned kernel, const StreamSetPort port) const {
    if (port.Type == PortType::Input) {
        return getInputBinding(kernel, port);
    } else if (port.Type == PortType::Output) {
        return getOutputBinding(kernel, port);
    }
    llvm_unreachable("unknown port binding type!");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputBuffer
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned PipelineCompiler::getBufferIndex(const unsigned bufferVertex) const {
    assert (bufferVertex >= FirstStreamSet);
    assert (bufferVertex <= LastStreamSet);
    return bufferVertex - FirstStreamSet;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getKernel
 ** ------------------------------------------------------------------------------------------------------------- */
inline const Kernel * PipelineCompiler::getKernel(const unsigned index) const {
    assert (PipelineInput <= index && index <= PipelineOutput);
    return mStreamGraph[index].Kernel;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputPortIndex
 ** ------------------------------------------------------------------------------------------------------------- */
BOOST_FORCEINLINE unsigned PipelineCompiler::getInputPortIndex(const unsigned kernel, const StreamSetPort port) const {
    assert (port.Type == PortType::Input);
    const auto key = std::pair<unsigned, unsigned>(kernel, port.Number);
    const auto f = mInputPortSet.lower_bound(key);
    assert(f != mInputPortSet.end() && *f == key);
    const auto i = f - mInputPortSet.begin();
    assert(mInputPortSet.nth(i) == f);
    return i;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputPortIndex
 ** ------------------------------------------------------------------------------------------------------------- */
BOOST_FORCEINLINE unsigned PipelineCompiler::getOutputPortIndex(const unsigned kernel, const StreamSetPort port) const {
    assert(port.Type == PortType::Output);
    const auto key = std::pair<unsigned, unsigned>(kernel, port.Number);
    const auto f = mOutputPortSet.lower_bound(key);
    assert(f != mOutputPortSet.end() && *f == key);
    const auto i = f - mOutputPortSet.begin();
    assert(mOutputPortSet.nth(i) == f);
    return i;
}




} // end of kernel namespace

#endif // BUFFER_ALLOCATION_HPP
