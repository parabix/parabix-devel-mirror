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
    return std::min(rate.maximum - rate.minimum, upperBound(kernel, binding));
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
    return producer == mPipelineKernel ? mPipelineKernel->getInputStreamSetBinding(index) : producer->getOutputStreamSetBinding(index);
}

inline const Binding & PipelineCompiler::getInputBinding(const Kernel * const consumer, const unsigned index) const {
    return consumer == mPipelineKernel ? mPipelineKernel->getOutputStreamSetBinding(index) : consumer->getInputStreamSetBinding(index);
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
 ** ------------------------------------------------------------------------------------------------------------- */
BufferGraph PipelineCompiler::makeBufferGraph(BuilderRef b) {

    const auto numOfKernels = mPipeline.size();
    const auto pipelineVertex = numOfKernels;
    BufferGraph G(numOfKernels + 1);
    BufferMap M;

    // make an edge from the pipeline input to a buffer vertex
    enumerateBufferProducerBindings(pipelineVertex, mPipelineKernel->getInputStreamSetBindings(), G, M);
    G[pipelineVertex].kernel = mPipelineKernel;
    // make an edge from each producing kernel to a buffer vertex
    for (unsigned i = 0; i < numOfKernels; ++i) {
        const auto & producer = mPipeline[i];
        enumerateBufferProducerBindings(i, producer->getOutputStreamSetBindings(), G, M);
        G[i].kernel = producer;
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

            const Kernel * const kernel = kn.kernel; assert(kernel);

            for (const auto ce : make_iterator_range(in_edges(i, G))) {
                // current consuming edge of this buffer
                const BufferRateData & cd = G[ce];
                const auto buffer = source(ce, G);
                // producing edge of this buffer
                const auto pe = in_edge(buffer, G);
                const BufferRateData & pd = G[pe];

                const auto min = div_by_non_zero(pd.minimum, cd.maximum);
                lower = std::min(lower, min);

                const auto max = div_by_non_zero(pd.maximum, cd.minimum);
                upper = std::min(upper, max);
            }

            kn.lower = lower;
            kn.upper = upper;

            for (const auto e : make_iterator_range(out_edges(i, G))) {
                BufferRateData & rd = G[e];
                rd.minimum = lower * rd.minimum;
                rd.maximum = upper * rd.maximum;
            }

        }
    }

//    printBufferGraph(G, errs());

    // fill in any known pipeline I/O buffers
    for (const auto e : make_iterator_range(out_edges(pipelineVertex, G))) {
        const auto bufferVertex = target(e, G);
        assert (G[bufferVertex].buffer == nullptr);
        StreamSetBuffer * const buffer = mPipelineKernel->getInputStreamSetBuffer(G[e].port);
        mIsPipelineIO.insert(buffer);
        G[bufferVertex].buffer = buffer;
    }

    for (const auto e : make_iterator_range(in_edges(pipelineVertex, G))) {
        const auto bufferVertex = source(e, G);
        assert (G[bufferVertex].buffer == nullptr);
        StreamSetBuffer * const buffer = mPipelineKernel->getOutputStreamSetBuffer(G[e].port);
        mIsPipelineIO.insert(buffer);
        G[bufferVertex].buffer = buffer;
    }

    // then construct the rest
    for (unsigned i = firstBuffer; i != lastBuffer; ++i) {

        // Is this a pipeline I/O buffer?
        if (LLVM_UNLIKELY(G[i].buffer != nullptr)) {
            continue;
        }

        const auto pe = in_edge(i, G);
        const auto producerVertex = source(pe, G);


        const BufferNode & producerNode = G[producerVertex];
        const auto & producer = producerNode.kernel;
        const BufferRateData & producerRate = G[pe];
        const Binding & output = getOutputBinding(producer, producerRate.port);

        StreamSetBuffer * buffer = nullptr;

        const auto isUnknown = producerRate.maximum.numerator() == 0;
        const auto isManaged = output.hasAttribute(AttrId::ManagedBuffer);
        if (LLVM_UNLIKELY(isUnknown || isManaged)) {
            buffer = new ExternalBuffer(b, output.getType());
        } else {

            RateValue requiredSpace{producerRate.maximum};
            RateValue overflowSpace{getInputOverflowSize(producer, output, producerRate)};
            RateValue facsimileSpace{0};
            bool dynamic = (producerRate.minimum < producerRate.maximum);

            for (const auto ce : make_iterator_range(out_edges(i, G))) {
                const BufferRateData & consumerRate = G[ce];
                requiredSpace = lcm(requiredSpace, consumerRate.maximum);
                const auto c = target(ce, G);
                const BufferNode & consumerNode = G[c];
                const Kernel * const consumer = consumerNode.kernel; assert (consumer);
                const Binding & input = getInputBinding(consumer, consumerRate.port);
                facsimileSpace = std::max(facsimileSpace, getOutputOverflowSize(consumer, input, consumerRate));
                // Could the consumption rate be less than the production rate?
                if ((consumerNode.lower * consumerRate.minimum) < producerRate.maximum) {
                    dynamic = true;
                }
            }

            const auto bufferSize = lcm(requiredSpace, b->getBitBlockWidth()) * mPipelineKernel->getNumOfThreads();
            assert (bufferSize.denominator() == 1);
            const auto overflowSize = lcm(std::max(overflowSpace, facsimileSpace), b->getBitBlockWidth());
            assert (overflowSize.denominator() == 1);

            // A DynamicBuffer is necessary when we cannot bound the amount of unconsumed data a priori.
            if (dynamic) {
                buffer = new DynamicBuffer(b, output.getType(), bufferSize.numerator(), overflowSize.numerator());
            } else {
                buffer = new StaticBuffer(b, output.getType(), bufferSize.numerator(), overflowSize.numerator());
            }
            if (LLVM_UNLIKELY(overflowSize > 0)) {
                mOverflowRequirements.emplace(buffer, OverflowRequirement{ceiling(overflowSpace), ceiling(facsimileSpace)});
            }
        }

        G[i].buffer = buffer;
        mOwnedBuffers.emplace_back(buffer);
    }

    // printBufferGraph(G, errs());

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
        out << "v" << i << " [label=\"" << i << ": " <<  mPipeline[i]->getName()  << "\" shape=box];\n";
    }

    out << "v" << numOfKernels << " [label=\"" << numOfKernels << ": pipeline\" shape=box];\n";

    for (unsigned i = firstBuffer; i != lastBuffer; ++i) {
        out << "v" << i << " [label=\"";
        if (G[i].buffer == nullptr) {
            out << '?';
        } else if (isa<ExternalBuffer>(G[i].buffer)) {
            out << 'E';
        } else if (isa<DynamicBuffer>(G[i].buffer)) {
            out << 'D';
        } else if (isa<StaticBuffer>(G[i].buffer)) {
            out << 'S';
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
        if (pd.minimum.denominator() > 1 || pd.maximum.denominator() > 1) {
            out << pd.minimum.numerator() << "/" << pd.minimum.denominator()
                << " - "
                << pd.maximum.numerator() << "/" << pd.maximum.denominator();
        } else {
            out << pd.minimum.numerator() << " - " << pd.maximum.numerator();
        }
        out << '\n';

        if (s <= numOfKernels) {
            // producer edge
            const Kernel * const k = G[s].kernel;
            out << k->getName() << "." << getOutputBinding(k, pd.port).getName();
        } else {
            assert (t <= numOfKernels);
            const Kernel * const k = G[t].kernel;
            out << k->getName() << "." << getInputBinding(k, pd.port).getName();
        }

        out << "\"];\n";
    }

    out << "}\n\n";
    out.flush();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addHandlesToPipelineKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::addHandlesToPipelineKernel(BuilderRef b) {
    const auto numOfKernels = mPipeline.size();
    b->setKernel(mPipelineKernel);
    IntegerType * const sizeTy = b->getSizeTy();
    for (unsigned i = 0; i < numOfKernels; ++i) {
        const auto & kernel = mPipeline[i];
        if (!kernel->hasFamilyName()) {
            PointerType * kernelPtrTy = kernel->getKernelType()->getPointerTo(0);
            mPipelineKernel->addInternalScalar(kernelPtrTy, makeKernelName(i));
        }
        for (const auto e : make_iterator_range(out_edges(i, mBufferGraph))) {
            const auto bufferVertex = target(e, mBufferGraph);
            const StreamSetBuffer * const buffer = mBufferGraph[bufferVertex].buffer;
            if (LLVM_UNLIKELY(isa<ExternalBuffer>(buffer))) {
                continue;
            }
            const Binding & output = kernel->getOutputStreamSetBinding(mBufferGraph[e].port);
            const auto bufferName = makeBufferName(i, output);
            mPipelineKernel->addInternalScalar(buffer->getHandleType(b), bufferName);
            mPipelineKernel->addInternalScalar(sizeTy, bufferName + CONSUMED_ITEM_COUNT_SUFFIX);
        }
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
        StreamSetBuffer * const buffer = mBufferGraph[i].buffer;
        if (LLVM_UNLIKELY(isa<ExternalBuffer>(buffer))) {
            continue;
        }
        const auto pe = in_edge(i, mBufferGraph);
        const auto p = source(pe, mBufferGraph);
        const auto & producer = mPipeline[p];
        const Binding & output = producer->getOutputStreamSetBinding(mBufferGraph[pe].port);
        const auto name = makeBufferName(p, output);
        Value * const handle = b->getScalarFieldPtr(name);
        buffer->setHandle(b, handle);
        buffer->allocateBuffer(b);
    }
}



/** ------------------------------------------------------------------------------------------------------------- *
 * @brief loadBufferHandles
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::loadBufferHandles(BuilderRef b) {
    assert (mPipeline[mKernelIndex] == mKernel);
    assert (b->getKernel() == mKernel);
    for (const auto pe : make_iterator_range(out_edges(mKernelIndex, mBufferGraph))) {
        const auto bufferVertex = target(pe, mBufferGraph);
        StreamSetBuffer * const buffer = mBufferGraph[bufferVertex].buffer;
        const Binding & output = mKernel->getOutputStreamSetBinding(mBufferGraph[pe].port);
        if (LLVM_UNLIKELY(isa<ExternalBuffer>(buffer))) {
            // if the handle was already set, it must be a pipeline I/O stream
            if (LLVM_LIKELY(buffer->getHandle() == nullptr)) {
                buffer->setHandle(b, b->getScalarFieldPtr(output.getName() + BUFFER_HANDLE_SUFFIX));
            }
        } else {
            b->setKernel(mPipelineKernel);
            buffer->setHandle(b, b->getScalarFieldPtr(makeBufferName(mKernelIndex, output)));
            b->setKernel(mKernel);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief releaseBuffers
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::releaseBuffers(BuilderRef b) {
    for (const auto & buffer : mOwnedBuffers) {
        buffer->releaseBuffer(b);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readCurrentProducedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::readCurrentProducedItemCounts(BuilderRef b) {
    for (const auto e : make_iterator_range(out_edges(mKernelIndex, mBufferGraph))) {
        const Binding & output = mKernel->getOutputStreamSetBinding(mBufferGraph[e].port);
        Value * const produced = b->getProducedItemCount(output.getName());
        const auto bufferIndex = target(e, mBufferGraph);
        const StreamSetBuffer * const buffer = mBufferGraph[bufferIndex].buffer;
        assert (mTotalItemCount.count(buffer) == 0);
        mTotalItemCount.emplace(buffer, produced);
        initializeConsumedItemCount(b, bufferIndex, produced);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief requiresCopyBack
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool PipelineCompiler::requiresCopyBack(const StreamSetBuffer * const buffer) const {
    const auto f = mOverflowRequirements.find(buffer);
    if (f == mOverflowRequirements.end()) return false;
    const OverflowRequirement & r = f->second;
    return r.copyBack != 0;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getCopyBack
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned PipelineCompiler::getCopyBack(const StreamSetBuffer * const buffer) const {
    const auto f = mOverflowRequirements.find(buffer);
    if (LLVM_LIKELY(f == mOverflowRequirements.end())) {
        return 0;
    }
    const OverflowRequirement & r = f->second;
    return r.copyBack;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief requiresFacsimile
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool PipelineCompiler::requiresFacsimile(const StreamSetBuffer * const buffer) const {
    const auto f = mOverflowRequirements.find(buffer);
    if (f == mOverflowRequirements.end()) return false;
    const OverflowRequirement & r = f->second;
    return r.facsimile != 0;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getFacsimile
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned PipelineCompiler::getFacsimile(const StreamSetBuffer * const buffer) const {
    const auto f = mOverflowRequirements.find(buffer);
    if (LLVM_LIKELY(f == mOverflowRequirements.end())) {
        return 0;
    }
    const OverflowRequirement & r = f->second;
    return r.facsimile;

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isPipelineIO
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool PipelineCompiler::isPipelineIO(const StreamSetBuffer * const buffer) const {
    return mIsPipelineIO.count(buffer) != 0;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief storeCopyBackProducedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::storeCopyBackProducedItemCounts(BuilderRef b) {
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
    mCopyBackProducedOutputItems.resize(numOfOutputs);
    std::fill(mCopyBackProducedOutputItems.begin(), mCopyBackProducedOutputItems.end(), nullptr);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const StreamSetBuffer * const buffer = getOutputBuffer(i);
        if (requiresCopyBack(buffer)) {
            const Binding & output = mKernel->getOutputStreamSetBinding(i);
            mCopyBackProducedOutputItems[i] = b->getNonDeferredProducedItemCount(output);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeCopyBackLogic
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::writeCopyBackLogic(BuilderRef b) {
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        if (mCopyBackProducedOutputItems[i]) {
            const Binding & output = mKernel->getOutputStreamSetBinding(i);
            const auto prefix = mKernel->getName() + "_" + output.getName();
            BasicBlock * const copyBack = b->CreateBasicBlock(prefix + "_copyBack", mKernelExit);
            BasicBlock * const copyExit = b->CreateBasicBlock(prefix + "_copyBackExit", mKernelExit);
            const StreamSetBuffer * const buffer = getOutputBuffer(i);
            Value * const capacity = buffer->getCapacity(b.get());

            Value * const priorOffset = b->CreateURem(mCopyBackProducedOutputItems[i], capacity);
            Value * const produced = b->getNonDeferredProducedItemCount(output);
            Value * const producedOffset = b->CreateURem(produced, capacity);
            Value * const nonCapacityAlignedWrite = b->CreateIsNotNull(producedOffset);
            Value * const wroteToOverflow = b->CreateICmpULT(producedOffset, priorOffset);
            Value * const needsCopyBack = b->CreateAnd(nonCapacityAlignedWrite, wroteToOverflow);
            b->CreateUnlikelyCondBr(needsCopyBack, copyBack, copyExit);

            b->SetInsertPoint(copyBack);
            Value * blocksToCopy = producedOffset;
            const auto itemWidth = getItemWidth(buffer->getBaseType());
            const auto blockWidth = b->getBitBlockWidth();
            if (LLVM_LIKELY(itemWidth < blockWidth)) {
                blocksToCopy = b->CreateCeilUDiv(blocksToCopy, b->getSize(blockWidth / itemWidth));
            } else if (LLVM_UNLIKELY(blockWidth > itemWidth)) {
                blocksToCopy = b->CreateMul(blocksToCopy, b->getSize(itemWidth / blockWidth));
            }
            blocksToCopy = b->CreateMul(blocksToCopy, buffer->getStreamSetCount(b.get()));
            const auto bytesPerBlock = blockWidth / 8;
            Value * const bytesToCopy = b->CreateMul(blocksToCopy, b->getSize(bytesPerBlock));
            Value * const source = buffer->getOverflowAddress(b.get());
            Value * const target = buffer->getBaseAddress(b.get());
            b->CreateMemCpy(target, source, bytesToCopy, bytesPerBlock);
            b->CreateBr(copyExit);

            b->SetInsertPoint(copyExit);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief storeCopyForwardProducedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::storeCopyForwardProducedItemCounts(BuilderRef b) {
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
    mCopyForwardProducedOutputItems.resize(numOfOutputs);
    std::fill(mCopyForwardProducedOutputItems.begin(), mCopyForwardProducedOutputItems.end(), nullptr);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const StreamSetBuffer * const buffer = getOutputBuffer(i);
        if (requiresFacsimile(buffer)) {
            const Binding & output = mKernel->getOutputStreamSetBinding(i);
            mCopyForwardProducedOutputItems[i] = b->getNonDeferredProducedItemCount(output);
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
        if (mCopyForwardProducedOutputItems[i]) {

            const Binding & output = mKernel->getOutputStreamSetBinding(i);
            const auto prefix = mKernel->getName() + "_" + output.getName();
            BasicBlock * const copyForward = b->CreateBasicBlock(prefix + "_copyForward", mKernelExit);
            BasicBlock * const copyExit = b->CreateBasicBlock(prefix + "_copyForwardExit", mKernelExit);
            const StreamSetBuffer * const buffer = getOutputBuffer(i);

            Value * const capacity = buffer->getCapacity(b.get());
            Value * const initial = mCopyForwardProducedOutputItems[i];
            Value * const produced = b->getNonDeferredProducedItemCount(output);

            // If we wrote anything and it was not our first write to the buffer ...
            Value * overwroteData = b->CreateICmpUGT(produced, capacity);
            if (LLVM_LIKELY(mKernel->getLowerBound(output) < 1)) {
                Value * const producedOutput = b->CreateICmpNE(initial, produced);
                overwroteData = b->CreateAnd(overwroteData, producedOutput);
            }
            // And we started writing within the first block ...
            const auto requiredOverflowSize = getFacsimile(buffer);
            assert (requiredOverflowSize <= buffer->getOverflowCapacity(b));
            Constant * const overflowSize = b->getSize(requiredOverflowSize);
            Value * const initialOffset = b->CreateURem(initial, capacity);
            Value * const startedWithinFirstBlock = b->CreateICmpULT(initialOffset, overflowSize);
            Value * const wroteToFirstBlock = b->CreateAnd(overwroteData, startedWithinFirstBlock);

            // Or we started writing at the end of the buffer but wrapped over to the start of it,
            Value * const producedOffset = b->CreateURem(produced, capacity);
            Value * const wroteFromEndToStart = b->CreateICmpULT(producedOffset, initialOffset);

            // Then mirror the data in the overflow region.
            Value * const needsCopyForward = b->CreateOr(wroteToFirstBlock, wroteFromEndToStart);
            b->CreateUnlikelyCondBr(needsCopyForward, copyForward, copyExit);

            // TODO: optimize this further to ensure that we don't copy data that was just copied back from
            // the overflow. Should be enough just to have a "copyback flag" phi node to say it that was the
            // last thing it did to the buffer.

            // TODO: look into non-cache-polluting writes? How big does the buffer need to be before it helps?

            b->SetInsertPoint(copyForward);
            Value * blocksToCopy = overflowSize;
            const auto itemWidth = getItemWidth(buffer->getBaseType());
            const auto blockWidth = b->getBitBlockWidth();
            if (LLVM_LIKELY(itemWidth < blockWidth)) {
                blocksToCopy = b->CreateCeilUDiv(blocksToCopy, b->getSize(blockWidth / itemWidth));
            } else if (LLVM_UNLIKELY(blockWidth > itemWidth)) {
                blocksToCopy = b->CreateMul(blocksToCopy, b->getSize(itemWidth / blockWidth));
            }
            blocksToCopy = b->CreateMul(blocksToCopy, buffer->getStreamSetCount(b.get()));

            const auto bytesPerBlock = blockWidth / 8;
            Value * const bytesToCopy = b->CreateMul(blocksToCopy, b->getSize(bytesPerBlock));
            Value * const source = buffer->getBaseAddress(b.get());
            Value * const target = buffer->getOverflowAddress(b.get());
            b->CreateMemCpy(target, source, bytesToCopy, bytesPerBlock);
            b->CreateBr(copyExit);

            b->SetInsertPoint(copyExit);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getLogicalInputBaseAddress
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getLogicalInputBaseAddress(BuilderRef b, const unsigned index) const {
    const Binding & input = mKernel->getInputStreamSetBinding(index);
    const StreamSetBuffer * const buffer = getInputBuffer(index);
    Value * const processed = b->getNonDeferredProcessedItemCount(input);
    assert (mAccessibleInputItems[index]);
    return calculateLogicalBaseAddress(b, input, buffer, processed);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getLogicalOutputBaseAddress
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getLogicalOutputBaseAddress(BuilderRef b, const unsigned index) const {
    const Binding & output = mKernel->getOutputStreamSetBinding(index);
    const StreamSetBuffer * const buffer = getOutputBuffer(index);
    Value * const produced = b->getNonDeferredProducedItemCount(output);
    return calculateLogicalBaseAddress(b, output, buffer, produced);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateLogicalBaseAddress
 *
 * Returns the address of the "zeroth" item of the (logically-unbounded) stream set.
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::calculateLogicalBaseAddress(BuilderRef b, const Binding & binding, const StreamSetBuffer * const buffer, Value * const itemCount) const {
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
        b->CreateAssert(b->CreateICmpEQ(A0, B0), prefix + ": logical base address is incorrect");
    }
    return address;
}

} // end of kernel namespace

#endif // BUFFER_ALLOCATION_HPP
