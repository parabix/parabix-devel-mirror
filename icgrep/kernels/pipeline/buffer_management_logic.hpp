#ifndef BUFFER_ALLOCATION_HPP
#define BUFFER_ALLOCATION_HPP

#include "pipeline_compiler.hpp"
#include <boost/algorithm/string/replace.hpp>

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

BufferRateData PipelineCompiler::getBufferRateData(const StreamPort port, const Kernel * const kernel, const Binding & binding) const {
    const auto ub = upperBound(kernel, binding);
    const auto lb = isConsistentRate(kernel, binding) ? ub : lowerBound(kernel, binding);
    return BufferRateData{port, lb, ub};
}

void PipelineCompiler::enumerateBufferProducerBindings(const Port type, const unsigned producer, const Bindings & bindings, BufferGraph & G, BufferMap & M) const {
    const auto n = bindings.size();
    const Kernel * const kernel = mPipeline[producer];
    for (unsigned i = 0; i < n; ++i) {
        const StreamSet * const rel = cast<StreamSet>(getRelationship(bindings[i]));
        assert (M.count(rel) == 0);
        const auto buffer = add_vertex(G);
        add_edge(producer, buffer, getBufferRateData(StreamPort{type, i}, kernel, bindings[i]), G); // producer -> buffer ordering
        M.emplace(rel, buffer);
    }
}

void PipelineCompiler::enumerateBufferConsumerBindings(const Port type, const unsigned consumer, const Bindings & bindings, BufferGraph & G, BufferMap & M) const {
    const auto n = bindings.size();
    const Kernel * const kernel = mPipeline[consumer];
    for (unsigned i = 0; i < n; ++i) {
        const StreamSet * const rel = cast<StreamSet>(getRelationship(bindings[i]));
        const auto f = M.find(rel); assert (f != M.end());
        const auto buffer = f->second;
        add_edge(buffer, consumer, getBufferRateData(StreamPort{type, i}, kernel, bindings[i]), G); // buffer -> consumer ordering
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makePipelineBufferGraph
 *
 * Return an acyclic bi-partite graph indicating the I/O relationships between the kernels and their buffers.
 *
 * Ordering: producer -> buffer -> consumer
 ** ------------------------------------------------------------------------------------------------------------- */
BufferGraph PipelineCompiler::makeBufferGraph(BuilderRef b) {

    const auto firstBuffer = mPipelineOutput + 1;

    BufferGraph G(mLastKernel + 1);
    BufferMap M;

    // make an edge from each producing kernel to a buffer vertex
    enumerateBufferProducerBindings(Port::Input, mPipelineInput, mPipelineKernel->getInputStreamSetBindings(), G, M);
    for (unsigned i = mFirstKernel; i < mLastKernel; ++i) {
        enumerateBufferProducerBindings(Port::Output, i, mPipeline[i]->getOutputStreamSetBindings(), G, M);
    }
    // make an edge from each buffer to its consuming kernel(s)
    for (unsigned i = mFirstKernel; i < mLastKernel; ++i) {
        enumerateBufferConsumerBindings(Port::Input, i, mPipeline[i]->getInputStreamSetBindings(), G, M);
    }
    // make an edge from a buffer vertex to each pipeline output
    enumerateBufferConsumerBindings(Port::Output, mPipelineOutput, mPipelineKernel->getOutputStreamSetBindings(), G, M);

    const auto lastBuffer = num_vertices(G);

    // Since we do not want to create an artifical bottleneck by constructing output buffers that
    // cannot accommodate the full amount of data we could produce given the expected inputs, the
    // next loop will resize them accordingly.

    // compute how much data each kernel could consume/produce per iteration.
    for (unsigned i = mFirstKernel; i < mLastKernel; ++i) {
        if (LLVM_LIKELY(in_degree(i, G) > 0)) {

            RateValue lower{std::numeric_limits<unsigned>::max()};
            RateValue upper{std::numeric_limits<unsigned>::max()};

            BufferNode & kn = G[i];

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
    for (const auto e : make_iterator_range(out_edges(mPipelineInput, G))) {
        const auto bufferVertex = target(e, G);
        BufferNode & bn = G[bufferVertex];
        assert (bn.Buffer == nullptr);
        bn.Buffer = mPipelineKernel->getInputStreamSetBuffer(G[e].inputPort());
        bn.Type = BufferType::External;
    }

    for (const auto e : make_iterator_range(in_edges(mPipelineOutput, G))) {
        const auto bufferVertex = source(e, G);
        BufferNode & bn = G[bufferVertex];
        assert (bn.Buffer == nullptr);
        bn.Buffer = mPipelineKernel->getOutputStreamSetBuffer(G[e].outputPort());
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


        const Kernel * const producer = mPipeline[producerVertex];
        const BufferRateData & producerRate = G[pe];
        const Binding & output = getBinding(producer, producerRate.Port);

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
                const Kernel * const consumer = mPipeline[c]; assert (consumer);
                const Binding & input = getBinding(consumer, consumerRate.Port);
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
            // compute the buffer size
            const auto overflowSize = std::max(bn.Overflow, bn.Fasimile);
            const auto bufferMod = overflowSize ? overflowSize : blockWidth;
            const auto bufferSpace = lcm(requiredSpace, bufferMod);
            assert (bufferSpace.denominator() == 1);
            const auto bufferSize = bufferSpace.numerator() * mPipelineKernel->getNumOfThreads();
            // ensure any Add/RoundUpTo attributes are safely handled
            unsigned additionalSpace = 0;
            if (LLVM_UNLIKELY(output.hasAttribute(AttrId::Add))) {
                const auto & add = output.findAttribute(AttrId::Add);
                additionalSpace = boost::lcm(add.amount(), blockWidth);
            }
            if (LLVM_UNLIKELY(output.hasAttribute(AttrId::RoundUpTo))) {
                const auto & roundUpTo = output.findAttribute(AttrId::RoundUpTo);
                const auto amount = roundUpTo.amount();
                const auto roundingSpace = boost::lcm(bufferSize % amount, blockWidth);
                additionalSpace = std::max(additionalSpace, roundingSpace);
            }
            additionalSpace = std::max(overflowSize, additionalSpace);
            // A DynamicBuffer is necessary when we cannot bound the amount of unconsumed data a priori.
            if (dynamic) {
                buffer = new DynamicBuffer(b, output.getType(), bufferSize, additionalSpace);
            } else {
                buffer = new StaticBuffer(b, output.getType(), bufferSize, additionalSpace);
            }
        }

        bn.Buffer = buffer;
        bn.Type = bufferType;
    }

    // printBufferGraph(G, errs());

    return G;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printBufferGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::printBufferGraph(const BufferGraph & G, raw_ostream & out) {


    out << "digraph G {\n"
           "v" << mPipelineInput << " [label=\"P_{in}\" shape=box];\n";

    for (unsigned i = mFirstKernel; i < mLastKernel; ++i) {
        const Kernel * const kernel = mPipeline[i]; assert(kernel);
        std::string name = kernel->getName();
        boost::replace_all(name, "\"", "\\\"");

        out << "v" << i << " [label=\"[" << i << "] " << name << "\" shape=box];\n";
    }

    out << "v" << mPipelineOutput << " [label=\"P_{out}\" shape=box];\n";

    const auto firstBuffer = mPipelineOutput + 1;
    const auto lastBuffer = num_vertices(G);

    for (unsigned i = firstBuffer; i != lastBuffer; ++i) {
        out << "v" << i << " [label=\"";
        const BufferNode & bn = G[i];
        if (bn.Buffer == nullptr) {
            out << '?';
        } else if (isa<ExternalBuffer>(bn.Buffer)) {
            out << 'E';
        } else if (DynamicBuffer * buffer = dyn_cast<DynamicBuffer>(bn.Buffer)) {
            out << 'D' << buffer->getInitialCapacity() << 'x' << buffer->getNumOfStreams();
        } else if (StaticBuffer * buffer = dyn_cast<StaticBuffer>(bn.Buffer)) {
            out << 'S' << buffer->getCapacity() << 'x' << buffer->getNumOfStreams();
        }
        if (bn.Overflow || bn.Fasimile) {
            out << " (O:" << bn.Overflow << ",F:" << bn.Fasimile << ')';
        }
        out << "\"];\n";
    }

    for (auto e : make_iterator_range(edges(G))) {
        const auto s = source(e, G);
        const auto t = target(e, G);
        out << "v" << s << " -> v" << t;
        const BufferRateData & pd = G[e];

        out << " [label=\"(" << pd.Port.second << ") ";
        if (pd.Minimum.denominator() > 1 || pd.Maximum.denominator() > 1) {
            out << pd.Minimum.numerator() << "/" << pd.Minimum.denominator()
                << " - "
                << pd.Maximum.numerator() << "/" << pd.Maximum.denominator();
        } else {
            out << pd.Minimum.numerator() << " - " << pd.Maximum.numerator();
        }

        std::string name = getBinding(mPipeline[s < t ? s : t], pd.Port).getName();
        boost::replace_all(name, "\"", "\\\"");
        out << '\n' << name << "\"];\n";
    }

    out << "}\n\n";
    out.flush();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addHandlesToPipelineKernel
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addBufferHandlesToPipelineKernel(BuilderRef b, const unsigned index) {
    const Kernel * const kernel = mPipeline[index];
    for (const auto e : make_iterator_range(out_edges(index, mBufferGraph))) {
        const auto bufferVertex = target(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[bufferVertex];
        if (LLVM_LIKELY(bn.Type != BufferType::Managed)) {
            const auto outputPort = mBufferGraph[e].outputPort();
            const Binding & output = kernel->getOutputStreamSetBinding(outputPort);
            const auto prefix = makeBufferName(index, output);
            mPipelineKernel->addInternalScalar(bn.Buffer->getHandleType(b), prefix);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructBuffers
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::constructBuffers(BuilderRef b) {

    const auto firstBuffer = mLastKernel + 1;
    const auto lastBuffer = num_vertices(mBufferGraph);

    b->setKernel(mPipelineKernel);

    for (unsigned i = firstBuffer; i < lastBuffer; ++i) {
        const BufferNode & bn = mBufferGraph[i];
        if (LLVM_LIKELY(bn.Type == BufferType::Internal)) {
            const auto pe = in_edge(i, mBufferGraph);
            const auto p = source(pe, mBufferGraph);
            const Kernel * const producer = mPipeline[p];
            const auto outputPort = mBufferGraph[pe].outputPort();
            const Binding & output = producer->getOutputStreamSetBinding(outputPort);
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
        const auto outputPort = mBufferGraph[pe].outputPort();
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
    const auto firstBuffer = mLastKernel + 1;
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
        if (input.isDeferred()) {
            mInitiallyProcessedDeferredItemCount[i] = b->getScalarField(prefix + DEFERRED_ITEM_COUNT_SUFFIX);
        }
    }
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = mKernel->getOutputStreamSetBinding(i);
        const auto prefix = makeBufferName(mKernelIndex, output);
        mInitiallyProducedItemCount[i] = b->getScalarField(prefix + ITEM_COUNT_SUFFIX);
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
        const auto outputPort = mBufferGraph[e].outputPort();
        Value * fullyProduced = mFullyProducedItemCount[outputPort];
        mTotalItems[getBufferIndex(bufferVertex)] = fullyProduced;
        initializeConsumedItemCount(bufferVertex, fullyProduced);
        initializePopCountReferenceItemCount(b, bufferVertex, fullyProduced);
        #ifdef PRINT_DEBUG_MESSAGES
        const auto & output = mKernel->getOutputStreamSetBinding(outputPort);
        const auto prefix = makeBufferName(mKernelIndex, output);
        b->CallPrintInt(prefix + "_fullyProduced", fullyProduced);
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

    BasicBlock * const copyStart = b->CreateBasicBlock(prefix + "Start", mKernelExit);
    BasicBlock * const copyLoop = b->CreateBasicBlock(prefix + "Loop", mKernelExit);
    BasicBlock * const copyExit = b->CreateBasicBlock(prefix + "Exit", mKernelExit);

    b->CreateUnlikelyCondBr(cond, copyStart, copyExit);

    b->SetInsertPoint(copyStart);
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
    PointerType * const copyTy = b->getBitBlockType()->getPointerTo();
    Value * const source =
        b->CreatePointerCast((direction == OverflowCopy::Forwards) ? base : overflow, copyTy);
    Value * const target =
        b->CreatePointerCast((direction == OverflowCopy::Forwards) ? overflow : base, copyTy);

    BasicBlock * const entryBlock = b->GetInsertBlock();
    b->CreateBr(copyLoop);

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
 * @brief epoch
 *
 * Returns the address of the "zeroth" item of the (logically-unbounded) stream set.
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::epoch(BuilderRef b,
                                const Binding & binding,
                                const StreamSetBuffer * const buffer,
                                Value * const position,
                                Value * const available) const {

    Constant * const LOG_2_BLOCK_WIDTH = b->getSize(floor_log2(b->getBitBlockWidth()));
    Constant * const ZERO = b->getSize(0);
    Value * const blockIndex = b->CreateLShr(position, LOG_2_BLOCK_WIDTH);
    Value * address = buffer->getStreamLogicalBasePtr(b.get(), ZERO, blockIndex);
    address = b->CreatePointerCast(address, buffer->getPointerType());
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        // This assertion is not thread safe; a dynamic input stream could be expanded between the time that we
        // take the buffer address and test the assertion here.
        if (!isa<DynamicBuffer>(buffer) || mPipelineKernel->getNumOfThreads() == 1) {
            const auto prefix = makeBufferName(mKernelIndex, binding);
            ExternalBuffer tmp(b, binding.getType());
            Value * const handle = b->CreateAlloca(tmp.getHandleType(b));
            tmp.setHandle(b, handle);
            tmp.setBaseAddress(b.get(), address);
            Value * const capacity = b->CreateAdd(position, available);
            tmp.setCapacity(b.get(), capacity);
            Value * const A0 = buffer->getStreamBlockPtr(b.get(), ZERO, blockIndex);
            Value * const B0 = tmp.getStreamBlockPtr(b.get(), ZERO, blockIndex);
            Value * const C0 = b->CreatePointerCast(B0, A0->getType());
            b->CreateAssert(b->CreateICmpEQ(A0, C0), prefix + ": logical start address is incorrect");
        }
    }
    return address;
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
    for (const auto e : make_iterator_range(in_edges(kernelVertex, mBufferGraph))) {
        if (mBufferGraph[e].inputPort() == inputPort) {
            return source(e, mBufferGraph);
        }
    }
    llvm_unreachable("input buffer not found");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputBuffer
 ** ------------------------------------------------------------------------------------------------------------- */
inline StreamSetBuffer * PipelineCompiler::getInputBuffer(const unsigned inputPort) const {
    return mBufferGraph[getInputBufferVertex(inputPort)].Buffer;
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
    for (const auto e : make_iterator_range(out_edges(kernelVertex, mBufferGraph))) {
        if (mBufferGraph[e].outputPort() == outputPort) {
            return target(e, mBufferGraph);
        }
    }
    llvm_unreachable("output buffer not found");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputBuffer
 ** ------------------------------------------------------------------------------------------------------------- */
inline StreamSetBuffer * PipelineCompiler::getOutputBuffer(const unsigned outputPort) const {
    return mBufferGraph[getOutputBufferVertex(outputPort)].Buffer;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputBuffer
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned PipelineCompiler::getBufferIndex(const unsigned bufferVertex) const {
    return bufferVertex - (mPipelineOutput + 1);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isPipelineInput
 ** ------------------------------------------------------------------------------------------------------------- */
bool PipelineCompiler::isPipelineInput(const unsigned kernel, const unsigned inputPort) const {
    return !is_parent(getInputBufferVertex(kernel, inputPort), mPipelineInput, mBufferGraph);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isPipelineOutput
 ** ------------------------------------------------------------------------------------------------------------- */
bool PipelineCompiler::isPipelineOutput(const unsigned kernel, const unsigned outputPort) const {
    return !has_child(getOutputBufferVertex(kernel, outputPort), mPipelineOutput, mBufferGraph);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief nestedPipeline
 ** ------------------------------------------------------------------------------------------------------------- */
bool PipelineCompiler::nestedPipeline() const {
    return out_degree(mPipelineInput, mBufferGraph) != 0 || in_degree(mPipelineOutput, mBufferGraph) != 0;
}


} // end of kernel namespace

#endif // BUFFER_ALLOCATION_HPP
