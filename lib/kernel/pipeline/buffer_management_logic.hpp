#ifndef BUFFER_ALLOCATION_HPP
#define BUFFER_ALLOCATION_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addHandlesToPipelineKernel
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addBufferHandlesToPipelineKernel(BuilderRef b, const unsigned index) {
    for (const auto & e : make_iterator_range(out_edges(index, mBufferGraph))) {
        const auto bufferVertex = target(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[bufferVertex];
        if (LLVM_LIKELY(bn.Type != BufferType::Managed)) {
            const BufferRateData & rd = mBufferGraph[e];
            const auto prefix = makeBufferName(index, rd.Port);
            mPipelineKernel->addInternalScalar(bn.Buffer->getHandleType(b), prefix);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructBuffers
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::constructBuffers(BuilderRef b) {

    const auto firstBuffer = PipelineOutput + 1;
    const auto lastBuffer = num_vertices(mBufferGraph);

    b->setKernel(mPipelineKernel);

    for (unsigned i = firstBuffer; i < lastBuffer; ++i) {
        const BufferNode & bn = mBufferGraph[i];
        if (LLVM_LIKELY(bn.Type == BufferType::Internal)) {
            const auto pe = in_edge(i, mBufferGraph);
            const auto p = source(pe, mBufferGraph);
            const BufferRateData & rd = mBufferGraph[pe];
            const auto prefix = makeBufferName(p, rd.Port);
            Value * const handle = b->getScalarFieldPtr(prefix);
            StreamSetBuffer * const buffer = bn.Buffer;
            buffer->setHandle(b, handle);
            buffer->allocateBuffer(b);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief loadBufferHandles
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::loadBufferHandles(BuilderRef b) {
    assert (getKernel(mKernelIndex) == mKernel);
    for (const auto pe : make_iterator_range(out_edges(mKernelIndex, mBufferGraph))) {
        const auto bufferVertex = target(pe, mBufferGraph);
        const auto outputPort = mBufferGraph[pe].outputPort();
        const Binding & output = getOutputBinding(outputPort);
        const BufferNode & bn = mBufferGraph[bufferVertex];
        StreamSetBuffer * const buffer = bn.Buffer;
        if (LLVM_LIKELY(bn.Type == BufferType::Internal)) {
            b->setKernel(mPipelineKernel);
            Value * const scalar = b->getScalarFieldPtr(makeBufferName(mKernelIndex, StreamPort{PortType::Output, outputPort}));
            buffer->setHandle(b, scalar);
        } else if (bn.Type == BufferType::Managed) {
            b->setKernel(mKernel);
            assert (mKernel->getHandle());
            assert (mKernel->getHandle()->getType());
            Value * const scalar = b->getScalarFieldPtr(output.getName() + BUFFER_HANDLE_SUFFIX);
            buffer->setHandle(b, scalar);
        }
        assert (buffer->getHandle());
    }
    b->setKernel(mKernel);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief releaseBuffers
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::releaseBuffers(BuilderRef b) {

    b->setKernel(mPipelineKernel);

    const auto firstBuffer = PipelineOutput + 1;
    const auto lastBuffer = num_vertices(mBufferGraph);
    for (auto i = firstBuffer; i != lastBuffer; ++i) {
        const BufferNode & bn = mBufferGraph[i];
        if (LLVM_LIKELY(bn.Type == BufferType::Internal)) {
            bn.Buffer->releaseBuffer(b);

            if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::TraceDynamicBuffers))) {
                if (isa<DynamicBuffer>(bn.Buffer)) {

                    const auto pe = in_edge(i, mBufferGraph);
                    const auto p = source(pe, mBufferGraph);
                    const BufferRateData & rd = mBufferGraph[pe];
                    const auto prefix = makeBufferName(p, rd.Port);

                    Value * const traceData = b->getScalarFieldPtr(prefix + STATISTICS_BUFFER_EXPANSION_SUFFIX);
                    Constant * const ZERO = b->getInt32(0);
                    b->CreateFree(b->CreateLoad(b->CreateGEP(traceData, {ZERO, ZERO})));
                }
            }

        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readInitialItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::readInitialItemCounts(BuilderRef b) {
    b->setKernel(mPipelineKernel);
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = getInputBinding(i);
        const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Input, i});
        Value * const processed = b->getScalarField(prefix + ITEM_COUNT_SUFFIX);
        mInitiallyProcessedItemCount[i] = processed;
        if (input.isDeferred()) {
            mInitiallyProcessedDeferredItemCount[i] = b->getScalarField(prefix + DEFERRED_ITEM_COUNT_SUFFIX);
        }
    }
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = getOutputBinding(i);
        const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Output, i});
        mInitiallyProducedItemCount[i] = b->getScalarField(prefix + ITEM_COUNT_SUFFIX);
        if (output.isDeferred()) {
            mInitiallyProducedDeferredItemCount[i] = b->getScalarField(prefix + DEFERRED_ITEM_COUNT_SUFFIX);
        }
    }
    b->setKernel(mKernel);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeUpdatedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::writeUpdatedItemCounts(BuilderRef b, const bool final) {
    b->setKernel(mPipelineKernel);
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = getInputBinding(i);
        const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Input, i});
        b->setScalarField(prefix + ITEM_COUNT_SUFFIX, final ? mFinalProcessedPhi[i] : mUpdatedProcessedPhi[i]);
        if (input.isDeferred()) {
            b->setScalarField(prefix + DEFERRED_ITEM_COUNT_SUFFIX, final ? mFinalProcessedPhi[i] : mUpdatedProcessedDeferredPhi[i]);
        }
    }
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = getOutputBinding(i);
        const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Output, i});
        b->setScalarField(prefix + ITEM_COUNT_SUFFIX, final ? mFinalProducedPhi[i] : mUpdatedProducedPhi[i]);
        if (output.isDeferred()) {
            b->setScalarField(prefix + DEFERRED_ITEM_COUNT_SUFFIX, final ? mFinalProducedPhi[i] : mUpdatedProducedDeferredPhi[i]);
        }
    }
    b->setKernel(mKernel);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readFinalProducedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::readFinalProducedItemCounts(BuilderRef b) {
    for (const auto e : make_iterator_range(out_edges(mKernelIndex, mBufferGraph))) {
        const auto bufferVertex = target(e, mBufferGraph);
        const auto outputPort = mBufferGraph[e].outputPort();
        Value * fullyProduced = mFullyProducedItemCount[outputPort];
        mLocallyAvailableItems[getBufferIndex(bufferVertex)] = fullyProduced;
        initializeConsumedItemCount(b, outputPort, fullyProduced);
        #ifdef PRINT_DEBUG_MESSAGES
        const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Output, outputPort});
        Value * const producedDelta = b->CreateSub(fullyProduced, mInitiallyProducedItemCount[outputPort]);
        b->CallPrintInt(prefix + "_producedΔ", producedDelta);
        #endif
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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputBufferType
 ** ------------------------------------------------------------------------------------------------------------- */
BufferType PipelineCompiler::getOutputBufferType(const unsigned outputPort) const {
    const auto bufferVertex = getOutputBufferVertex(outputPort);
    return mBufferGraph[bufferVertex].Type;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeLookBehindLogic
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeLookBehindLogic(BuilderRef b) {
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto bufferVertex = getOutputBufferVertex(i);
        const BufferNode & bn = mBufferGraph[bufferVertex];
        if (bn.LookBehind) {
            const StreamSetBuffer * const buffer = bn.Buffer;
            Value * const capacity = buffer->getCapacity(b);
            Value * const produced = mAlreadyProducedPhi[i];
            Value * const producedOffset = b->CreateURem(produced, capacity);
            Constant * const underflow = b->getSize(bn.LookBehind);
            Value * const needsCopy = b->CreateICmpULT(producedOffset, underflow);
            copy(b, CopyMode::LookBehind, needsCopy, i, bn.Buffer, bn.LookBehind);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeCopyBackLogic
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::writeCopyBackLogic(BuilderRef b) {
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto bufferVertex = getOutputBufferVertex(i);
        const BufferNode & bn = mBufferGraph[bufferVertex];
        if (bn.CopyBack) {
            const StreamSetBuffer * const buffer = bn.Buffer;
            Value * const capacity = buffer->getCapacity(b);
            Value * const priorOffset = b->CreateURem(mAlreadyProducedPhi[i], capacity);
            Value * const produced = mProducedItemCount[i];
            Value * const producedOffset = b->CreateURem(produced, capacity);
            Value * const nonCapacityAlignedWrite = b->CreateIsNotNull(producedOffset);
            Value * const wroteToOverflow = b->CreateICmpULT(producedOffset, priorOffset);
            Value * const needsCopy = b->CreateAnd(nonCapacityAlignedWrite, wroteToOverflow);
            copy(b, CopyMode::CopyBack, needsCopy, i, buffer, bn.CopyBack);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeLookAheadLogic
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeLookAheadLogic(BuilderRef b) {
    // Unless we modified the portion of data that ought to be reflected in the overflow region, do not copy
    // any data. To do so would incur extra writes and pollute the cache with potentially unnecessary data.
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {

        const auto bufferVertex = getOutputBufferVertex(i);
        const BufferNode & bn = mBufferGraph[bufferVertex];

        if (bn.LookAhead) {

            const StreamSetBuffer * const buffer = bn.Buffer;

            Value * const capacity = buffer->getCapacity(b);
            Value * const initial = mInitiallyProducedItemCount[i];
            Value * const produced = mUpdatedProducedPhi[i];

            // If we wrote anything and it was not our first write to the buffer ...
            Value * overwroteData = b->CreateICmpUGT(produced, capacity);
            const Binding & output = getOutputBinding(i);
            const ProcessingRate & rate = output.getRate();
            const RateValue ONE(1, 1);
            bool mayProduceZeroItems = false;
            if (rate.getLowerBound() < ONE) {
                mayProduceZeroItems = true;
            } else if (rate.isRelative()) {
                const Binding & ref = getBinding(getReference(StreamPort{PortType::Output, i}));
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

            copy(b, CopyMode::LookAhead, needsCopy, i, buffer, bn.LookAhead);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeOverflowCopy
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::copy(BuilderRef b, const CopyMode mode, Value * cond,
                            const unsigned outputPort, const StreamSetBuffer * const buffer,
                            const unsigned itemsToCopy) {

    auto makeSuffix = [](CopyMode mode) {
        switch (mode) {
            case CopyMode::LookAhead: return "LookAhead";
            case CopyMode::CopyBack: return "CopyBack";
            case CopyMode::LookBehind: return "LookBehind";
        }
        llvm_unreachable("unknown copy mode!");
    };

    const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Output, outputPort}) + "_copy" + makeSuffix(mode);

    BasicBlock * const copyStart = b->CreateBasicBlock(prefix, mKernelExit);
    BasicBlock * const copyExit = b->CreateBasicBlock(prefix + "Exit", mKernelExit);

    b->CreateUnlikelyCondBr(cond, copyStart, copyExit);

    b->SetInsertPoint(copyStart);

    startCycleCounter(b, CycleCounter::BEFORE_COPY);

    const auto itemWidth = getItemWidth(buffer->getBaseType());
    const auto blockWidth = b->getBitBlockWidth();
    assert ((itemsToCopy % blockWidth) == 0);
    Value * const numOfStreams = buffer->getStreamSetCount(b);
    Value * const overflowSize = b->getSize(itemsToCopy * itemWidth / 8);
    Value * const bytesToCopy = b->CreateMul(overflowSize, numOfStreams);
    Value * source = nullptr;
    Value * target = nullptr;

    if (mode == CopyMode::LookAhead) {
        source = buffer->getBaseAddress(b);
        target = buffer->getOverflowAddress(b);
    } else  {
        source = buffer->getOverflowAddress(b);
        target = buffer->getBaseAddress(b);
        if (mode == CopyMode::LookBehind) {
            DataLayout DL(b->getModule());
            Type * const intPtrTy = DL.getIntPtrType(source->getType());
            Value * offset = b->CreateNeg(b->CreateZExt(bytesToCopy, intPtrTy));
            PointerType * const int8PtrTy = b->getInt8PtrTy();
            source = b->CreatePointerCast(source, int8PtrTy);
            source = b->CreateGEP(source, offset);
            target = b->CreatePointerCast(target, int8PtrTy);
            target = b->CreateGEP(target, offset);
        }
    }

    b->CreateMemCpy(target, source, bytesToCopy, blockWidth / 8);

    updateCycleCounter(b, CycleCounter::BEFORE_COPY, CycleCounter::AFTER_COPY);

    b->CreateBr(copyExit);

    b->SetInsertPoint(copyExit);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief epoch
 *
 * Returns the address of the "zeroth" item of the (logically-unbounded) stream set.
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::epoch(BuilderRef b,
                                const Binding & binding,
                                const StreamSetBuffer * const buffer,
                                Value * const position,
                                Value * const zeroExtended) const {
    assert ("epoch buffer cannot be null!" && buffer);
    Constant * const LOG_2_BLOCK_WIDTH = b->getSize(floor_log2(b->getBitBlockWidth()));
    Constant * const ZERO = b->getSize(0);
    PointerType * const bufferType = buffer->getPointerType();
    Value * const blockIndex = b->CreateLShr(position, LOG_2_BLOCK_WIDTH);
    Value * const baseAddress = buffer->getBaseAddress(b);
    Value * address = buffer->getStreamLogicalBasePtr(b, baseAddress, ZERO, blockIndex);
    if (zeroExtended) {
        // prepareLocalZeroExtendSpace guarantees this will be large enough to satisfy the kernel
        ExternalBuffer tmp(b, binding.getType(), true, buffer->getAddressSpace());
        assert (mZeroExtendBufferPhi);
        Value * zeroExtension = b->CreatePointerCast(mZeroExtendBufferPhi, bufferType);
        zeroExtension = tmp.getStreamBlockPtr(b, zeroExtension, ZERO, b->CreateNeg(blockIndex));
        address = b->CreateSelect(zeroExtended, zeroExtension, address);
    }
    return b->CreatePointerCast(address, bufferType);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateInputEpochAddresses
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::calculateInputEpochAddresses(BuilderRef b) {
    RelationshipType prior_in{};
    for (const auto & e : make_iterator_range(in_edges(mKernelIndex, mBufferGraph))) {
        const BufferRateData & rt = mBufferGraph[e];
        assert (rt.Port.Type == PortType::Input);
        assert (prior_in < rt.Port);
        prior_in = rt.Port;
        PHINode * processed = nullptr;
        const auto i = rt.Port.Number;
        if (mAlreadyProcessedDeferredPhi[i]) {
            processed = mAlreadyProcessedDeferredPhi[i];
        } else {
            processed = mAlreadyProcessedPhi[i];
        }
        const auto buffer = source(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[buffer];
        mInputEpoch[i] = epoch(b, rt.Binding, bn.Buffer, processed, mIsInputZeroExtended[i]);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputBufferVertex
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned PipelineCompiler::getInputBufferVertex(const unsigned inputPort) const {
    return getInputBufferVertex(mKernelIndex, inputPort);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputBufferVertex
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned PipelineCompiler::getInputBufferVertex(const unsigned kernelVertex, const unsigned inputPort) const {
    return source(getInput(kernelVertex, inputPort), mBufferGraph);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputBuffer
 ** ------------------------------------------------------------------------------------------------------------- */
inline StreamSetBuffer * PipelineCompiler::getInputBuffer(const unsigned inputPort) const {
    return mBufferGraph[getInputBufferVertex(inputPort)].Buffer;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputBinding
 ** ------------------------------------------------------------------------------------------------------------- */
const Binding & PipelineCompiler::getInputBinding(const unsigned kernelVertex, const unsigned inputPort) const {

    RelationshipGraph::vertex_descriptor v;
    RelationshipGraph::edge_descriptor e;
    if (LLVM_UNLIKELY(kernelVertex == PipelineInput || kernelVertex == PipelineOutput)) {
        graph_traits<RelationshipGraph>::out_edge_iterator ei, ei_end;
        std::tie(ei, ei_end) = out_edges(kernelVertex, mStreamGraph);
        assert (inputPort < std::distance(ei, ei_end));
        e = *(ei + inputPort);
        v = target(e, mStreamGraph);
    } else {
        graph_traits<RelationshipGraph>::in_edge_iterator ei, ei_end;
        std::tie(ei, ei_end) = in_edges(kernelVertex, mStreamGraph);
        assert (inputPort < std::distance(ei, ei_end));
        e = *(ei + inputPort);
        v = source(e, mStreamGraph);
    }

    assert (mStreamGraph[e].Type == PortType::Input);
    assert (mStreamGraph[e].Number == inputPort);
    const RelationshipNode & rn = mStreamGraph[v];
    assert (rn.Type == RelationshipNode::IsBinding);
    return rn.Binding;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputBinding
 ** ------------------------------------------------------------------------------------------------------------- */
inline const Binding & PipelineCompiler::getInputBinding(const unsigned inputPort) const {
    return getInputBinding(mKernelIndex, inputPort);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isInputExplicit
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool PipelineCompiler::isInputExplicit(const unsigned inputPort) const {
    const auto vertex = getInput(mKernelIndex, inputPort);
    const BufferRateData & rd = mBufferGraph[vertex];
    return rd.Port.Reason == ReasonType::Explicit;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getProducerOutputBinding
 ** ------------------------------------------------------------------------------------------------------------- */
inline const Binding & PipelineCompiler::getProducerOutputBinding(const unsigned inputPort) const {
    const auto buffer = getInputBufferVertex(inputPort);
    const BufferRateData & br = mBufferGraph[in_edge(buffer, mBufferGraph)];
    return br.Binding;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInput
 ** ------------------------------------------------------------------------------------------------------------- */
inline const BufferGraph::edge_descriptor PipelineCompiler::getInput(const unsigned kernelVertex, const unsigned inputPort) const {
    assert (inputPort < in_degree(kernelVertex, mBufferGraph));
    const auto e = *(in_edges(kernelVertex, mBufferGraph).first + inputPort);
    // assert (mBufferGraph[e].inputPort() == inputPort);
    return e;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputBufferVertex
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned PipelineCompiler::getOutputBufferVertex(const unsigned outputPort) const {
    return getOutputBufferVertex(mKernelIndex, outputPort);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputBufferVertex
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned PipelineCompiler::getOutputBufferVertex(const unsigned kernelVertex, const unsigned outputPort) const {
    return target(getOutput(kernelVertex, outputPort), mBufferGraph);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputBinding
 ** ------------------------------------------------------------------------------------------------------------- */
const Binding & PipelineCompiler::getOutputBinding(const unsigned kernelVertex, const unsigned outputPort) const {

    RelationshipGraph::vertex_descriptor v;
    RelationshipGraph::edge_descriptor e;
    if (LLVM_UNLIKELY(kernelVertex == PipelineInput || kernelVertex == PipelineOutput)) {
        graph_traits<RelationshipGraph>::in_edge_iterator ei, ei_end;
        std::tie(ei, ei_end) = in_edges(kernelVertex, mStreamGraph);
        assert (outputPort < std::distance(ei, ei_end));
        e = *(ei + outputPort);
        v = source(e, mStreamGraph);
    } else {
        graph_traits<RelationshipGraph>::out_edge_iterator ei, ei_end;
        std::tie(ei, ei_end) = out_edges(kernelVertex, mStreamGraph);
        assert (outputPort < std::distance(ei, ei_end));
        e = *(ei + outputPort);
        v = target(e, mStreamGraph);
    }

    assert (mStreamGraph[e].Type == PortType::Output);
    assert (mStreamGraph[e].Number == outputPort);

    const RelationshipNode & rn = mStreamGraph[v];
    assert (rn.Type == RelationshipNode::IsBinding);
    return rn.Binding;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputBinding
 ** ------------------------------------------------------------------------------------------------------------- */
inline const Binding & PipelineCompiler::getOutputBinding(const unsigned outputPort) const {
    return getOutputBinding(mKernelIndex, outputPort);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputBuffer
 ** ------------------------------------------------------------------------------------------------------------- */
inline StreamSetBuffer * PipelineCompiler::getOutputBuffer(const unsigned outputPort) const {
    return mBufferGraph[getOutputBufferVertex(outputPort)].Buffer;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInput
 ** ------------------------------------------------------------------------------------------------------------- */
inline const BufferGraph::edge_descriptor PipelineCompiler::getOutput(const unsigned kernelVertex, const unsigned outputPort) const {
    assert (outputPort < out_degree(kernelVertex, mBufferGraph));
    const auto e = *(out_edges(kernelVertex, mBufferGraph).first + outputPort);
    // assert (mBufferGraph[e].outputPort() == outputPort);
    return e;
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
inline const Binding & PipelineCompiler::getBinding(const StreamPort port) const {
    return getBinding(mKernelIndex, port);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getBinding
 ** ------------------------------------------------------------------------------------------------------------- */
const Binding & PipelineCompiler::getBinding(const unsigned kernel, const StreamPort port) const {
    if (port.Type == PortType::Input) {
        return getInputBinding(kernel, port.Number);
    } else if (port.Type == PortType::Output) {
        return getOutputBinding(kernel, port.Number);
    }
    llvm_unreachable("unknown port binding type!");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputBuffer
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned PipelineCompiler::getBufferIndex(const unsigned bufferVertex) const {
    return bufferVertex - (PipelineOutput + 1);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getKernel
 ** ------------------------------------------------------------------------------------------------------------- */
inline const Kernel * PipelineCompiler::getKernel(const unsigned index) const {
    assert (PipelineInput <= index && index <= PipelineOutput);
    return mStreamGraph[index].Kernel;
}

} // end of kernel namespace

#endif // BUFFER_ALLOCATION_HPP
