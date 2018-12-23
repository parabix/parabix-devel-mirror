#ifndef BUFFER_ALLOCATION_HPP
#define BUFFER_ALLOCATION_HPP

#include "pipeline_compiler.hpp"

// TODO: any buffers that exist only to satisfy the output dependencies are unnecessary.
// We could prune away kernels if none of their outputs are needed but we'd want some
// form of "fake" buffer for output streams in which only some are unnecessary. Making a
// single static buffer thats large enough for one segment and using it as "scratch space"
// is possible but that could cause unnecessary cache-sharing in theaded models.
// For threading, we'd want thread local buffers.

namespace kernel {

namespace {

bool isConsistentRate(const Kernel * kernel, const Binding & binding) {
    const ProcessingRate & rate = binding.getRate();
    if (LLVM_UNLIKELY(rate.isRelative())) {
        return isConsistentRate(kernel, kernel->getStreamBinding(rate.getReference()));
    }
    return rate.isFixed() || binding.hasAttribute(AttrId::BlockSize);
}

inline RateValue div_by_non_zero(const RateValue & num, const RateValue & denom) {
    return  (denom.numerator() == 0) ? num : (num / denom);
}

inline RateValue getOverflowSize(const Kernel * kernel, const Binding & binding, const BufferRateData & rate) {
    if (binding.hasAttribute(AttrId::BlockSize)) {
        return 0;
    }
    return std::min(rate.Maximum - rate.Minimum, upperBound(kernel, binding));
}

inline RateValue getInputOverflowSize(const Kernel * kernel, const Binding & binding, const BufferRateData & rate) {
    return getOverflowSize(kernel, binding, rate);
}

inline RateValue getOutputOverflowSize(const Kernel * kernel, const Binding & binding, const BufferRateData & rate) {
    RateValue overflow{getOverflowSize(kernel, binding, rate)};
    if (LLVM_UNLIKELY(binding.hasLookahead())) {
        overflow += binding.getLookahead();
    }
    return overflow;
}

} // end of anonymous namespace

inline const Binding & PipelineCompiler::getOutputBinding(const Kernel * const producer, const unsigned index) const {
    if (producer == mPipelineKernel) {
        return mPipelineKernel->getInputStreamSetBinding(index);
    } else {
        return producer->getOutputStreamSetBinding(index);
    }
}

inline const Binding & PipelineCompiler::getInputBinding(const Kernel * const consumer, const unsigned index) const {
    if (consumer == mPipelineKernel) {
        return mPipelineKernel->getOutputStreamSetBinding(index);
    } else {
        return consumer->getInputStreamSetBinding(index);
    }
}

inline BufferRateData PipelineCompiler::getBufferRateData(const unsigned index, const unsigned port, bool input) {
    const Kernel * kernel = nullptr;
    if (index < mPipeline.size()) {
        kernel = mPipeline[index];
    } else {
        assert (index == mPipeline.size());
        kernel = mPipelineKernel;
        input = !input;
    }
    const Binding & binding = input ? kernel->getInputStreamSetBinding(port) : kernel->getOutputStreamSetBinding(port);
    const auto ub = upperBound(kernel, binding);
    const auto lb = isConsistentRate(kernel, binding) ? ub : lowerBound(kernel, binding);
    return BufferRateData{port, lb, ub};
}

void PipelineCompiler::enumerateBufferProducerBindings(const unsigned producer, const Bindings & bindings, BufferGraph & G, BufferMap & M) {
    const auto n = bindings.size();
    for (unsigned i = 0; i < n; ++i) {
        const StreamSet * const rel = cast<StreamSet>(getRelationship(bindings[i]));
        assert (M.count(rel) == 0);
        const auto buffer = add_vertex(G);
        add_edge(producer, buffer, getBufferRateData(producer, i, false), G); // producer -> buffer ordering
        M.emplace(rel, buffer);
    }
}

void PipelineCompiler::enumerateBufferConsumerBindings(const unsigned consumer, const Bindings & bindings, BufferGraph & G, BufferMap & M) {
    const auto n = bindings.size();
    for (unsigned i = 0; i < n; ++i) {
        const StreamSet * const rel = cast<StreamSet>(getRelationship(bindings[i]));
        const auto f = M.find(rel); assert (f != M.end());
        const auto buffer = f->second;
        add_edge(buffer, consumer, getBufferRateData(consumer, i, true), G); // buffer -> consumer ordering
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makePipelineBufferGraph
 *
 * Return a cyclic bi-partite graph indicating the I/O relationships between the kernels and their buffers.
 *
 * Ordering: producer -> buffer -> consumer
 ** ------------------------------------------------------------------------------------------------------------- */
BufferGraph PipelineCompiler::makeBufferGraph(BuilderRef b) {

    const auto numOfKernels = mPipeline.size();
    const auto pipelineVertex = numOfKernels;
    BufferGraph G(numOfKernels + 1);
    BufferMap M;

    // make an edge from the pipeline input to a buffer vertex
    enumerateBufferProducerBindings(pipelineVertex, mPipelineKernel->getInputStreamSetBindings(), G, M);
    G[pipelineVertex].Kernel = mPipelineKernel;
    // make an edge from each producing kernel to a buffer vertex
    for (unsigned i = 0; i < numOfKernels; ++i) {
        const auto & producer = mPipeline[i];
        enumerateBufferProducerBindings(i, producer->getOutputStreamSetBindings(), G, M);
        G[i].Kernel = producer;
    }
    // make an edge from each buffer to its consuming kernel(s)
    for (unsigned i = 0; i < numOfKernels; ++i) {
        const auto & consumer = mPipeline[i];
        enumerateBufferConsumerBindings(i, consumer->getInputStreamSetBindings(), G, M);
    }
    // make an edge from a buffer vertex to each pipeline output
    enumerateBufferConsumerBindings(pipelineVertex, mPipelineKernel->getOutputStreamSetBindings(), G, M);

    const auto firstBuffer = numOfKernels + 1;
    const auto lastBuffer = num_vertices(G);

    // Since we do not want to create an artifical bottleneck by constructing output buffers that
    // cannot accommodate the full amount of data we could produce given the expected inputs, the
    // next loop will resize them accordingly.

    // compute how much data each kernel could consume/produce per iteration.
    for (unsigned i = 0; i < numOfKernels; ++i) {
        if (LLVM_LIKELY(in_degree(i, G) > 0)) {

            RateValue lower{std::numeric_limits<unsigned>::max()};
            RateValue upper{std::numeric_limits<unsigned>::max()};

            BufferNode & kn = G[i];
            assert(kn.Kernel);

            for (const auto ce : make_iterator_range(in_edges(i, G))) {
                // current consuming edge of this buffer
                const BufferRateData & cd = G[ce];
                const auto buffer = source(ce, G);
                // producing edge of this buffer
                const auto pe = in_edge(buffer, G);
                const BufferRateData & pd = G[pe];

                const auto min = div_by_non_zero(pd.Minimum, cd.Maximum);
                lower = std::min(lower, min);

                const auto max = div_by_non_zero(pd.Maximum, cd.Minimum);
                upper = std::min(upper, max);
            }

            kn.Lower = lower;
            kn.Upper = upper;

            for (const auto e : make_iterator_range(out_edges(i, G))) {
                BufferRateData & rd = G[e];
                rd.Minimum = lower * rd.Minimum;
                rd.Maximum = upper * rd.Maximum;
            }

        }
    }

    // fill in any known pipeline I/O buffers
    for (const auto e : make_iterator_range(out_edges(pipelineVertex, G))) {
        const auto bufferVertex = target(e, G);
        BufferNode & bn = G[bufferVertex];
        assert (bn.Buffer == nullptr);
        const auto inputPort = G[e].Port;
        bn.Buffer = mPipelineKernel->getInputStreamSetBuffer(inputPort);
        bn.Type = BufferType::External;
    }

    for (const auto e : make_iterator_range(in_edges(pipelineVertex, G))) {
        const auto bufferVertex = source(e, G);
        BufferNode & bn = G[bufferVertex];
        assert (bn.Buffer == nullptr);
        const auto outputPort = G[e].Port;
        bn.Buffer = mPipelineKernel->getOutputStreamSetBuffer(outputPort);
        bn.Type = BufferType::External;
    }

    // then construct the rest
    for (unsigned i = firstBuffer; i != lastBuffer; ++i) {

        // Is this a pipeline I/O buffer?
        BufferNode & bn = G[i];

        if (LLVM_UNLIKELY(bn.Buffer != nullptr)) {
            continue;
        }

        const auto pe = in_edge(i, G);
        const auto producerVertex = source(pe, G);


        const BufferNode & producerNode = G[producerVertex];
        const auto & producer = producerNode.Kernel;
        const BufferRateData & producerRate = G[pe];
        const Binding & output = getOutputBinding(producer, producerRate.Port);

        StreamSetBuffer * buffer = nullptr;

        const auto isUnknown = producerRate.Maximum.numerator() == 0;
        const auto isManaged = output.hasAttribute(AttrId::ManagedBuffer);

        BufferType bufferType = BufferType::Internal;

        if (LLVM_UNLIKELY(isUnknown || isManaged)) {
            buffer = new ExternalBuffer(b, output.getType());
            bufferType = BufferType::Managed;
        } else {

            RateValue requiredSpace{producerRate.Maximum};
            RateValue overflowSpace{getInputOverflowSize(producer, output, producerRate)};
            RateValue facsimileSpace{0};

            bool dynamic = false;

            for (const auto ce : make_iterator_range(out_edges(i, G))) {
                const BufferRateData & consumerRate = G[ce];
                requiredSpace = lcm(requiredSpace, consumerRate.Maximum);
                const auto c = target(ce, G);
                const BufferNode & consumerNode = G[c];
                const Kernel * const consumer = consumerNode.Kernel; assert (consumer);
                const Binding & input = getInputBinding(consumer, consumerRate.Port);
                facsimileSpace = std::max(facsimileSpace, getOutputOverflowSize(consumer, input, consumerRate));
                // Could the consumption rate be less than the production rate?
                if ((consumerNode.Lower * consumerRate.Minimum) < producerRate.Maximum) {
                    dynamic = true;
                }
            }

            // calculate overflow (copyback) and fascimile (copyforward) space
            const auto blockWidth = b->getBitBlockWidth();
            overflowSpace = lcm(overflowSpace, blockWidth);
            assert (overflowSpace.denominator() == 1);
            facsimileSpace = lcm(facsimileSpace, blockWidth);
            assert (facsimileSpace.denominator() == 1);
            bn.Overflow = overflowSpace.numerator();
            bn.Fasimile = facsimileSpace.numerator();
            const auto overflowSize = std::max(bn.Overflow, bn.Fasimile);

            // compute the buffer size
            const auto bufferMod = overflowSize ? overflowSize : blockWidth;
            const auto bufferSpace = lcm(requiredSpace, bufferMod);
            assert (bufferSpace.denominator() == 1);
            const auto bufferSize = bufferSpace.numerator() * mPipelineKernel->getNumOfThreads();
            // A DynamicBuffer is necessary when we cannot bound the amount of unconsumed data a priori.
            if (dynamic) {
                buffer = new DynamicBuffer(b, output.getType(), bufferSize, overflowSize);
            } else {
                buffer = new StaticBuffer(b, output.getType(), bufferSize, overflowSize);
            }
        }

        bn.Buffer = buffer;
        bn.Type = bufferType;

        mOwnedBuffers.emplace_back(buffer);
    }

//    printBufferGraph(G, errs());

    return G;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printBufferGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::printBufferGraph(const BufferGraph & G, raw_ostream & out) {

    const auto numOfKernels = mPipeline.size();
    const auto firstBuffer = numOfKernels + 1;
    const auto lastBuffer = num_vertices(G);

    out << "digraph G {\n";

    for (unsigned i = 0; i < numOfKernels; ++i) {
        const Kernel * const kernel = mPipeline[i]; assert(kernel);
        out << "v" << i << " [label=\"" << i << ": " <<  kernel->getName()  << "\" shape=box];\n";
    }

    out << "v" << numOfKernels << " [label=\"" << numOfKernels << ": pipeline\" shape=box];\n";

    for (unsigned i = firstBuffer; i != lastBuffer; ++i) {
        out << "v" << i << " [label=\"";
        const BufferNode & bn = G[i];
        if (bn.Buffer == nullptr) {
            out << '?';
        } else if (isa<ExternalBuffer>(bn.Buffer)) {
            out << 'E';
        } else if (isa<DynamicBuffer>(bn.Buffer)) {
            out << 'D';
        } else if (isa<StaticBuffer>(bn.Buffer)) {
            out << 'S';
        }
        if (bn.Overflow || bn.Fasimile) {
            out << " (O:" << bn.Overflow << ",F:" << bn.Fasimile << ')';
        }
        out << "\"];\n";
    }

    std::vector<Kernel *> P(mPipeline);
    P.push_back(mPipelineKernel);

    for (auto e : make_iterator_range(edges(G))) {
        const auto s = source(e, G);
        const auto t = target(e, G);
        out << "v" << s << " -> v" << t;
        const BufferRateData & pd = G[e];

        out << " [label=\"";
        if (pd.Minimum.denominator() > 1 || pd.Maximum.denominator() > 1) {
            out << pd.Minimum.numerator() << "/" << pd.Minimum.denominator()
                << " - "
                << pd.Maximum.numerator() << "/" << pd.Maximum.denominator();
        } else {
            out << pd.Minimum.numerator() << " - " << pd.Maximum.numerator();
        }
        out << '\n';

        if (s <= numOfKernels) {
            // producer edge
            const Kernel * const k = G[s].Kernel;
            out << k->getName() << "." << getOutputBinding(k, pd.Port).getName();
        } else {
            assert (t <= numOfKernels);
            const Kernel * const k = G[t].Kernel;
            out << k->getName() << "." << getInputBinding(k, pd.Port).getName();
        }

        out << "\"];\n";
    }

    out << "}\n\n";
    out.flush();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addHandlesToPipelineKernel
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addBufferHandlesToPipelineKernel(BuilderRef b, const unsigned index) {
    IntegerType * const sizeTy = b->getSizeTy();
    const Kernel * const kernel = mPipeline[index];
    if (!kernel->hasFamilyName()) {
        PointerType * kernelPtrTy = kernel->getKernelType()->getPointerTo(0);
        mPipelineKernel->addInternalScalar(kernelPtrTy, makeKernelName(index));
    }
    for (const auto e : make_iterator_range(out_edges(index, mBufferGraph))) {
        const auto bufferVertex = target(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[bufferVertex];
        if (LLVM_UNLIKELY(bn.Type == BufferType::Managed)) {
            continue;
        }
        const Binding & output = kernel->getOutputStreamSetBinding(mBufferGraph[e].Port);
        const auto bufferName = makeBufferName(index, output);
        mPipelineKernel->addInternalScalar(bn.Buffer->getHandleType(b), bufferName);
        mPipelineKernel->addInternalScalar(sizeTy, bufferName + CONSUMED_ITEM_COUNT_SUFFIX);
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructBuffers
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::constructBuffers(BuilderRef b) {

    const auto numOfKernels = mPipeline.size();
    const auto firstBuffer = numOfKernels + 1;
    const auto lastBuffer = num_vertices(mBufferGraph);

    b->setKernel(mPipelineKernel);

    for (unsigned i = firstBuffer; i < lastBuffer; ++i) {
        const BufferNode & bn = mBufferGraph[i];
        if (LLVM_LIKELY(bn.Type == BufferType::Internal)) {
            const auto pe = in_edge(i, mBufferGraph);
            const auto p = source(pe, mBufferGraph);
            const auto & producer = mPipeline[p];
            const Binding & output = producer->getOutputStreamSetBinding(mBufferGraph[pe].Port);
            const auto name = makeBufferName(p, output);
            Value * const handle = b->getScalarFieldPtr(name);
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
    assert (mPipeline[mKernelIndex] == mKernel);
    for (const auto pe : make_iterator_range(out_edges(mKernelIndex, mBufferGraph))) {
        const auto bufferVertex = target(pe, mBufferGraph);
        const auto outputPort = mBufferGraph[pe].Port;
        const Binding & output = mKernel->getOutputStreamSetBinding(outputPort);
        const BufferNode & bn = mBufferGraph[bufferVertex];
        StreamSetBuffer * const buffer = bn.Buffer;
        if (LLVM_LIKELY(bn.Type == BufferType::Internal)) {
            b->setKernel(mPipelineKernel);
            buffer->setHandle(b, b->getScalarFieldPtr(makeBufferName(mKernelIndex, output)));
        } else if (bn.Type == BufferType::Managed) {
            b->setKernel(mKernel);
            buffer->setHandle(b, b->getScalarFieldPtr(output.getName() + BUFFER_HANDLE_SUFFIX));
        }
        assert (buffer->getHandle());
    }
    b->setKernel(mKernel);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief releaseBuffers
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::releaseBuffers(BuilderRef b) {
    const auto firstBuffer = mPipeline.size() + 1;
    const auto lastBuffer = num_vertices(mBufferGraph);
    for (auto bufferVertex = firstBuffer; bufferVertex != lastBuffer; ++bufferVertex) {
        const BufferNode & bn = mBufferGraph[bufferVertex];
        if (LLVM_LIKELY(bn.Type == BufferType::Internal)) {
            bn.Buffer->releaseBuffer(b);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readInitialItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::readInitialItemCounts(BuilderRef b) {
    b->setKernel(mPipelineKernel);
    const auto numOfInputs = mKernel->getNumOfStreamInputs();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = mKernel->getInputStreamSetBinding(i);
        const auto prefix = makeBufferName(mKernelIndex, input);
        mInitiallyProcessedItemCount[i] = b->getScalarField(prefix + ITEM_COUNT_SUFFIX);
        #ifdef PRINT_DEBUG_MESSAGES
        b->CallPrintInt(prefix + "_initialProcessed", mInitiallyProcessedItemCount[i]);
        #endif
        if (input.isDeferred()) {
            mInitiallyProcessedDeferredItemCount[i] = b->getScalarField(prefix + DEFERRED_ITEM_COUNT_SUFFIX);
            #ifdef PRINT_DEBUG_MESSAGES
            b->CallPrintInt(prefix + "_initialProducedDeferred", mInitiallyProcessedDeferredItemCount[i]);
            #endif
        }
    }
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = mKernel->getOutputStreamSetBinding(i);
        const auto prefix = makeBufferName(mKernelIndex, output);
        mInitiallyProducedItemCount[i] = b->getScalarField(prefix + ITEM_COUNT_SUFFIX);
        #ifdef PRINT_DEBUG_MESSAGES
        b->CallPrintInt(prefix + "_initialProduced", mInitiallyProducedItemCount[i]);
        #endif
    }
    b->setKernel(mKernel);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeUpdatedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::writeUpdatedItemCounts(BuilderRef b) {
    b->setKernel(mPipelineKernel);
    const auto numOfInputs = mKernel->getNumOfStreamInputs();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = mKernel->getInputStreamSetBinding(i);
        const auto prefix = makeBufferName(mKernelIndex, input);
        b->setScalarField(prefix + ITEM_COUNT_SUFFIX, mUpdatedProcessedPhi[i]);
        if (input.isDeferred()) {
            b->setScalarField(prefix + DEFERRED_ITEM_COUNT_SUFFIX, mUpdatedProcessedDeferredPhi[i]);
        }
    }
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = mKernel->getOutputStreamSetBinding(i);
        const auto prefix = makeBufferName(mKernelIndex, output);
        b->setScalarField(prefix + ITEM_COUNT_SUFFIX, mUpdatedProducedPhi[i]);
    }
    b->setKernel(mKernel);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readFinalProducedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::readFinalProducedItemCounts(BuilderRef b) {
    for (const auto e : make_iterator_range(out_edges(mKernelIndex, mBufferGraph))) {
        const auto bufferVertex = target(e, mBufferGraph);
        const auto outputPort = mBufferGraph[e].Port;
        Value * fullyProduced = mFullyProducedItemCount[outputPort];
        BufferNode & bn = mBufferGraph[bufferVertex];
        assert (bn.TotalItems == nullptr);
        bn.TotalItems = fullyProduced;
        initializeConsumedItemCount(b, bufferVertex, fullyProduced);
        initializePopCountReferenceItemCount(b, bufferVertex, fullyProduced);
        #ifdef PRINT_DEBUG_MESSAGES
        const auto & output = mKernel->getOutputStreamSetBinding(outputPort);
        const auto prefix = makeBufferName(mKernelIndex, output);
        b->CallPrintInt(prefix + "_fullyProduced", fullyProduced);
        #endif
    }
}

#warning TODO: copyback/copyforward ought to reflect exact num of items; not upper bound of space

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
    return mBufferGraph[bufferVertex].Overflow;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief requiresFacsimile
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool PipelineCompiler::requiresFacsimile(const unsigned bufferVertex) const {
    return getFacsimile(bufferVertex) != 0;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getFacsimile
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned PipelineCompiler::getFacsimile(const unsigned bufferVertex) const {
    return mBufferGraph[bufferVertex].Fasimile;

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputBufferType
 ** ------------------------------------------------------------------------------------------------------------- */
BufferType PipelineCompiler::getOutputBufferType(const unsigned outputPort) const {
    const auto bufferVertex = getOutputBufferVertex(outputPort);
    return mBufferGraph[bufferVertex].Type;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeCopyBackLogic
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::writeCopyBackLogic(BuilderRef b) {
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        if (requiresCopyBack(getOutputBufferVertex(i))) {
            const StreamSetBuffer * const buffer = getOutputBuffer(i);
            Value * const capacity = buffer->getCapacity(b.get());
            Value * const priorOffset = b->CreateURem(mAlreadyProducedPhi[i], capacity);
            Value * const produced = mProducedItemCount[i];
            Value * const producedOffset = b->CreateURem(produced, capacity);
            Value * const nonCapacityAlignedWrite = b->CreateIsNotNull(producedOffset);
            Value * const wroteToOverflow = b->CreateICmpULT(producedOffset, priorOffset);
            Value * const needsCopyBack = b->CreateAnd(nonCapacityAlignedWrite, wroteToOverflow);
            const Binding & output = mKernel->getOutputStreamSetBinding(i);
            writeOverflowCopy(b, OverflowCopy::Backwards, needsCopyBack, output, buffer, producedOffset);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeCopyToOverflowLogic
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeCopyForwardLogic(BuilderRef b) {
    // Unless we modified the portion of data that ought to be reflected in the overflow region, do not copy
    // any data. To do so would incur extra writes and pollute the cache with potentially unnecessary data.
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        if (requiresFacsimile(getOutputBufferVertex(i))) {

            const StreamSetBuffer * const buffer = getOutputBuffer(i);

            Value * const capacity = buffer->getCapacity(b.get());
            Value * const initial = mInitiallyProducedItemCount[i];
            Value * const produced = mUpdatedProducedPhi[i];

            // If we wrote anything and it was not our first write to the buffer ...
            const Binding & output = mKernel->getOutputStreamSetBinding(i);
            Value * overwroteData = b->CreateICmpUGT(produced, capacity);
            if (LLVM_LIKELY(mKernel->getLowerBound(output) < 1)) {
                Value * const producedOutput = b->CreateICmpNE(initial, produced);
                overwroteData = b->CreateAnd(overwroteData, producedOutput);
            }
            // And we started writing within the first block ...
            const auto requiredOverflowSize = getFacsimile(getOutputBufferVertex(i));
            assert (requiredOverflowSize <= buffer->getOverflowCapacity(b));
            Constant * const overflowSize = b->getSize(requiredOverflowSize);
            Value * const initialOffset = b->CreateURem(initial, capacity);
            Value * const startedWithinFirstBlock = b->CreateICmpULT(initialOffset, overflowSize);
            Value * const wroteToFirstBlock = b->CreateAnd(overwroteData, startedWithinFirstBlock);

            // And we started writing at the end of the buffer but wrapped over to the start of it,
            Value * const producedOffset = b->CreateURem(produced, capacity);
            Value * const wroteFromEndToStart = b->CreateICmpULT(producedOffset, initialOffset);

            // Then mirror the data in the overflow region.
            Value * const needsCopyForward = b->CreateOr(wroteToFirstBlock, wroteFromEndToStart);

            // TODO: optimize this further to ensure that we don't copy data that was just copied back from
            // the overflow. Should be enough just to have a "copyback flag" phi node to say it that was the
            // last thing it did to the buffer.

            writeOverflowCopy(b, OverflowCopy::Forwards, needsCopyForward, output, buffer, overflowSize);
        }
    }
}



/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeOverflowCopy
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeOverflowCopy(BuilderRef b, const OverflowCopy direction, Value * cond, const Binding & binding, const StreamSetBuffer * const buffer, Value * const itemsToCopy) const {

    const auto prefix = makeBufferName(mKernelIndex, binding)
        + ((direction == OverflowCopy::Forwards) ? "_copyForward" : "_copyBack");

    BasicBlock * const copyLoop = b->CreateBasicBlock(prefix + "Loop", mKernelExit);
    BasicBlock * const copyExit = b->CreateBasicBlock(prefix + "Exit", mKernelExit);

    Value * const count = buffer->getStreamSetCount(b.get());
    Value * blocksToCopy = b->CreateMul(itemsToCopy, count);
    const auto itemWidth = getItemWidth(buffer->getBaseType());
    const auto blockWidth = b->getBitBlockWidth();
    if (LLVM_LIKELY(itemWidth < blockWidth)) {
        blocksToCopy = b->CreateCeilUDiv(blocksToCopy, b->getSize(blockWidth / itemWidth));
    } else if (LLVM_UNLIKELY(blockWidth > itemWidth)) {
        blocksToCopy = b->CreateMul(blocksToCopy, b->getSize(itemWidth / blockWidth));
    }
    Value * const base = buffer->getBaseAddress(b.get());
    Value * const overflow = buffer->getOverflowAddress(b.get());
    Value * const source = (direction == OverflowCopy::Forwards) ? base : overflow;
    Value * const target = (direction == OverflowCopy::Forwards) ? overflow : base;

    BasicBlock * const entryBlock = b->GetInsertBlock();
    b->CreateUnlikelyCondBr(cond, copyLoop, copyExit);

    b->SetInsertPoint(copyLoop);
    PHINode * const index = b->CreatePHI(b->getSizeTy(), 2);
    index->addIncoming(b->getSize(0), entryBlock);
    Value * const val = b->CreateBlockAlignedLoad(b->CreateGEP(source, index));
    b->CreateBlockAlignedStore(val, b->CreateGEP(target, index));
    Value * const nextIndex = b->CreateAdd(index, b->getSize(1));
    index->addIncoming(nextIndex, b->GetInsertBlock());
    Value * const notDone =b->CreateICmpNE(nextIndex, blocksToCopy);
    b->CreateCondBr(notDone, copyLoop, copyExit);

    b->SetInsertPoint(copyExit);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getLogicalInputBaseAddress
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getLogicalInputBaseAddress(BuilderRef b, const unsigned inputPort) {
    const Binding & input = mKernel->getInputStreamSetBinding(inputPort);
    const StreamSetBuffer * const buffer = getInputBuffer(inputPort);
    Value * const processed = mAlreadyProcessedPhi[inputPort];
    return calculateLogicalBaseAddress(b, input, buffer, processed);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getLogicalOutputBaseAddress
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getLogicalOutputBaseAddress(BuilderRef b, const unsigned outputPort) {
    const Binding & output = mKernel->getOutputStreamSetBinding(outputPort);
    const StreamSetBuffer * const buffer = getOutputBuffer(outputPort);
    Value * const produced = mAlreadyProducedPhi[outputPort];
    return calculateLogicalBaseAddress(b, output, buffer, produced);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateLogicalBaseAddress
 *
 * Returns the address of the "zeroth" item of the (logically-unbounded) stream set.
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::calculateLogicalBaseAddress(BuilderRef b, const Binding & binding, const StreamSetBuffer * const buffer, Value * const itemCount) {
    Constant * const LOG_2_BLOCK_WIDTH = b->getSize(floor_log2(b->getBitBlockWidth()));
    Constant * const ZERO = b->getSize(0);
    Value * const blockIndex = b->CreateLShr(itemCount, LOG_2_BLOCK_WIDTH);
    Value * address = buffer->getStreamLogicalBasePtr(b.get(), ZERO, blockIndex);
    address = b->CreatePointerCast(address, buffer->getPointerType());
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        const auto prefix = mKernel->getName() + "." + binding.getName();
        ExternalBuffer tmp(b, binding.getType());
        Value * const handle = b->CreateAlloca(tmp.getHandleType(b));
        tmp.setHandle(b, handle);
        tmp.setBaseAddress(b.get(), address);
        Value * const A0 = buffer->getStreamBlockPtr(b.get(), ZERO, blockIndex);
        Value * const B0 = tmp.getStreamBlockPtr(b.get(), ZERO, blockIndex);
        Value * const C0 = b->CreatePointerCast(B0, A0->getType());
        b->CreateAssert(b->CreateICmpEQ(A0, C0), prefix + ": logical base address is incorrect");
        Value * upToIndex = b->CreateAdd(blockIndex, b->CreateSub(mNumOfLinearStrides, b->getSize(1)));
        upToIndex = b->CreateSelect(b->CreateICmpEQ(mNumOfLinearStrides, ZERO), blockIndex, upToIndex);
        Value * const A1 = buffer->getStreamBlockPtr(b.get(), ZERO, upToIndex);
        Value * const B1 = tmp.getStreamBlockPtr(b.get(), ZERO, upToIndex);
        Value * const C1 = b->CreatePointerCast(B1, A1->getType());
        b->CreateAssert(b->CreateICmpEQ(A1, C1), prefix + ": logical base address is incorrect");
    }
    return address;
}

} // end of kernel namespace

#endif // BUFFER_ALLOCATION_HPP
