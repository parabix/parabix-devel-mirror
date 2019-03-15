#ifndef BUFFER_ALLOCATION_HPP
#define BUFFER_ALLOCATION_HPP

#include "pipeline_compiler.hpp"
#include <boost/algorithm/string/replace.hpp>
#include <boost/interprocess/mapped_region.hpp>

// TODO: any buffers that exist only to satisfy the output dependencies are unnecessary.
// We could prune away kernels if none of their outputs are needed but we'd want some
// form of "fake" buffer for output streams in which only some are unnecessary. Making a
// single static buffer thats large enough for one segment and using it as "scratch space"
// is possible but that could cause unnecessary cache-sharing in theaded models.
// For threading, we'd want thread local buffers.

namespace kernel {

namespace {

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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makePipelineBufferGraph
 *
 * Return an acyclic bi-partite graph indicating the I/O relationships between the kernels and their buffers.
 *
 * Ordering: producer -> buffer -> consumer
 ** ------------------------------------------------------------------------------------------------------------- */
BufferGraph PipelineCompiler::makeBufferGraph(BuilderRef b) {

    const auto firstBuffer = PipelineOutput + 1;
    const auto lastBuffer = PipelineOutput + (LastStreamSet - FirstStreamSet + 1);
    BufferGraph G(lastBuffer + 1);

    // construct the base buffer graph with edges sorted by port number
    assert (PipelineOutput < FirstScalar);
    assert (LastScalar < FirstStreamSet);

    SmallVector<std::pair<RelationshipType, unsigned>, 16> ports;
    for (unsigned i = PipelineInput; i <= PipelineOutput; ++i) {
        const Kernel * const kernel = mPipelineGraph[i].Kernel; assert (kernel);

        std::function<bool(const Binding &)> dataIsConsumedAtVariableRate = [&](const Binding & binding) {
                const ProcessingRate & rate = binding.getRate();
                if (rate.isFixed() || binding.hasAttribute(AttrId::BlockSize)) {
                    return false;
                }
                if (LLVM_UNLIKELY(rate.isRelative())) {
                    return dataIsConsumedAtVariableRate(kernel->getStreamBinding(rate.getReference()));
                }
                return true;
            };

        auto addBinding = [&](const std::string & name, Relationship * relationship, ProcessingRate rate) -> Binding & {
            assert (relationship);
            Binding * const binding = new Binding(name, relationship, rate);
            mImplicitBindings.emplace_back(binding);
            return *binding;
        };

        // note: explicit lambda return type required for refs
        auto getBinding = [&](const RelationshipType port, Relationship * relationship) -> const Binding & {
            assert (relationship);
            if (port.Reason == ReasonType::Explicit) {
                if (port.Type == PortType::Input) {
                    return kernel->getInputStreamSetBinding(port.Number);
                } else if (port.Type == PortType::Output) {
                    return kernel->getOutputStreamSetBinding(port.Number);
                }
            } else if (port.Reason == ReasonType::ImplicitPopCount) {
                Binding & popCount = addBinding("#popcount", relationship, FixedRate({1, b->getBitBlockWidth()}));
                popCount.addAttribute(LookBehind(1));
                return popCount;
            } else if (port.Reason == ReasonType::ImplicitRegionSelector) {
                return addBinding("#regionselector", relationship, FixedRate());
            }
            llvm_unreachable("unknown port binding type!");
        };

        auto computeBufferRateBounds = [&](const RelationshipType port, Relationship * relationship) {
            unsigned strideLength = kernel->getStride();
            if (i == PipelineInput || i == PipelineOutput) {
                const unsigned pageSize = boost::interprocess::mapped_region::get_page_size();
                strideLength = boost::lcm(pageSize, strideLength);
            }
            const Binding & binding = getBinding(port, relationship);
            const RateValue ub = kernel->getUpperBound(binding) * strideLength;
            RateValue lb{ub};
            if (dataIsConsumedAtVariableRate(binding)) {
                lb = kernel->getLowerBound(binding) * strideLength;
            }
            return BufferRateData{port, binding, lb, ub, port.Reason};
        };

        // add in any inputs
        for (const auto & e : make_iterator_range(in_edges(i, mPipelineGraph))) {
            const auto k = source(e, mBufferGraph);
            assert (k >= FirstScalar && k <= LastStreamSet);
            if (k < FirstStreamSet) continue;
            const auto buffer = firstBuffer + k - FirstStreamSet;
            assert (firstBuffer <= buffer && buffer <= lastBuffer);
            const RelationshipType & portNum = mPipelineGraph[e];
            ports.emplace_back(portNum, buffer);
        }
        std::sort(ports.begin(), ports.end());
        for (const auto & e : ports) {
            RelationshipType port;
            unsigned buffer;
            std::tie(port, buffer) = e;
            const auto k = FirstStreamSet + buffer - firstBuffer;
            const RelationshipNode & rn = mPipelineGraph[k];
            add_edge(buffer, i, computeBufferRateBounds(port, rn.Relationship), G);
        }
        ports.clear();

        // and any outputs
        for (const auto & e : make_iterator_range(out_edges(i, mPipelineGraph))) {
            const auto k = target(e, mBufferGraph);
            assert (k >= FirstScalar && k <= LastStreamSet);
            if (k < FirstStreamSet) continue;
            const auto buffer = firstBuffer + k - FirstStreamSet;
            assert (firstBuffer <= buffer && buffer <= lastBuffer);
            assert (in_degree(buffer, G) == 0);
            const auto portNum = mPipelineGraph[e];
            ports.emplace_back(portNum, buffer);
        }
        std::sort(ports.begin(), ports.end());
        for (const auto & e : ports) {
            RelationshipType port;
            unsigned buffer;
            std::tie(port, buffer) = e;
            const auto k = FirstStreamSet + buffer - firstBuffer;
            const RelationshipNode & rn = mPipelineGraph[k];
            add_edge(i, buffer, computeBufferRateBounds(port, rn.Relationship), G);
        }
        ports.clear();
    }

    // Since we do not want to create an artifical bottleneck by constructing output buffers that
    // cannot accommodate the full amount of data we could produce given the expected inputs, the
    // next loop will resize them accordingly.

    // compute how much data each kernel could consume/produce per iteration.
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        if (LLVM_LIKELY(in_degree(i, G) > 0)) {
            RateValue lower{std::numeric_limits<unsigned>::max()};
            RateValue upper{std::numeric_limits<unsigned>::max()};
            for (const auto & ce : make_iterator_range(in_edges(i, G))) {
                const BufferRateData & consumptionRate = G[ce];
                const BufferRateData & productionRate = G[in_edge(source(ce, G), G)];
                const auto min = div_by_non_zero(productionRate.Minimum, consumptionRate.Maximum);
                lower = std::min(lower, min);
                const auto max = div_by_non_zero(productionRate.Maximum, consumptionRate.Minimum);
                upper = std::min(upper, max);
            }
            BufferNode & kn = G[i];
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
    for (const auto e : make_iterator_range(out_edges(PipelineInput, G))) {
        const auto bufferVertex = target(e, G);
        BufferNode & bn = G[bufferVertex];
        assert (bn.Buffer == nullptr);
        bn.Buffer = mPipelineKernel->getInputStreamSetBuffer(G[e].inputPort());
        bn.Type = BufferType::External;
    }

    for (const auto e : make_iterator_range(in_edges(PipelineOutput, G))) {
        const auto bufferVertex = source(e, G);
        BufferNode & bn = G[bufferVertex];
        assert (bn.Buffer == nullptr);
        bn.Buffer = mPipelineKernel->getOutputStreamSetBuffer(G[e].outputPort());
        bn.Type = BufferType::External;
    }

    const auto isOpenSystem = out_degree(PipelineInput, G) != 0 || in_degree(PipelineOutput, G) != 0;

    // then construct the rest
    for (unsigned i = firstBuffer; i <= lastBuffer; ++i) {

        // Is this a pipeline I/O buffer?
        BufferNode & bn = G[i];

        if (LLVM_UNLIKELY(bn.Buffer != nullptr)) {
            continue;
        }

        const auto pe = in_edge(i, G);
        const auto producerVertex = source(pe, G);
        const Kernel * const producer = getKernel(producerVertex);
        const BufferRateData & producerRate = G[pe];
        const Binding & output = producerRate.Binding;

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
            unsigned underflowSize = 0;
            bool unboundedLookbehind = false;

            // If we have an open system, then the input rate to this pipeline cannot
            // be bounded a priori. Just make all internal buffers dynamic to simplify
            // the process.

            // TODO: during initialization, we could pass a "suggestion" argument to
            // indicate what the outer pipeline believes its I/O rates will be.

            bool dynamic = isOpenSystem;
            for (const auto ce : make_iterator_range(out_edges(i, G))) {
                const BufferRateData & consumerRate = G[ce];
                requiredSpace = lcm(requiredSpace, consumerRate.Maximum);
                const auto c = target(ce, G);
                const BufferNode & consumerNode = G[c];
                const Kernel * const consumer = getKernel(c);
                const Binding & input = consumerRate.Binding;
                facsimileSpace = std::max(facsimileSpace, getOutputOverflowSize(consumer, input, consumerRate));
                // Could the consumption rate be less than the production rate?
                if ((consumerNode.Lower * consumerRate.Minimum) < producerRate.Maximum) {
                    dynamic = true;
                }
                // If we have a lookbehind attribute,
                if (LLVM_UNLIKELY(input.hasAttribute(AttrId::LookBehind))) {
                    const auto & lookBehind = input.findAttribute(AttrId::LookBehind);
                    const auto amount = lookBehind.amount();
                    if (amount == 0) {
                        unboundedLookbehind = true;
                    } else {
                        underflowSize = std::max(underflowSize, lookBehind.amount());
                    }
                }
            }
            bn.Underflow = underflowSize;
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
            const auto threads = mPipelineKernel->getNumOfThreads();
            assert (threads > 0);
            const auto segmentSize = bufferSpace.numerator();
            // TODO: investigate whether this affects thrashing
            const auto additional = std::max(threads > 1 ? 1U : 0U, ((bn.Fasimile + segmentSize - 1) / segmentSize));
            const auto bufferSize = segmentSize * (threads + additional);
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
            underflowSize = (underflowSize + blockWidth - 1) & -blockWidth;
            if (LLVM_UNLIKELY(unboundedLookbehind)) {
                buffer = new LinearBuffer(b, output.getType(), bufferSize, additionalSpace, underflowSize, 0);
            } else if (dynamic) {
                // A DynamicBuffer is necessary when we cannot bound the amount of unconsumed data a priori.
                buffer = new DynamicBuffer(b, output.getType(), bufferSize, additionalSpace, underflowSize, 0);
            } else {
                buffer = new StaticBuffer(b, output.getType(), bufferSize, additionalSpace, underflowSize, 0);
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
           "v" << PipelineInput << " [label=\"[" << PipelineInput << "] P_{in}\" shape=box];\n";

    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        const Kernel * const kernel = getKernel(i);
        std::string name = kernel->getName();
        boost::replace_all(name, "\"", "\\\"");

        out << "v" << i << " [label=\"[" << i << "] " << name << "\" shape=box];\n";
    }

    out << "v" << PipelineOutput << " [label=\"[" << PipelineOutput << "]P_{out}\" shape=box];\n";

    const auto firstBuffer = PipelineOutput + 1;
    const auto lastBuffer = num_vertices(G);

    for (unsigned i = firstBuffer; i != lastBuffer; ++i) {
        out << "v" << i << " [label=\""
               "(" << i << ") ";

        const BufferNode & bn = G[i];
        if (bn.Buffer == nullptr) {
            out << '?';
        } else if (isa<ExternalBuffer>(bn.Buffer)) {
            out << 'E';
        } else if (DynamicBuffer * buffer = dyn_cast<DynamicBuffer>(bn.Buffer)) {
            out << 'D' << buffer->getInitialCapacity() << 'x' << buffer->getNumOfStreams();
        } else if (LinearBuffer * buffer = dyn_cast<LinearBuffer>(bn.Buffer)) {
            out << 'L' << buffer->getInitialCapacity() << 'x' << buffer->getNumOfStreams();
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

        out << " [label=\"#"
            << pd.Port.Number << ": ";

        if (pd.Minimum.denominator() > 1 || pd.Maximum.denominator() > 1) {
            out << pd.Minimum.numerator() << "/" << pd.Minimum.denominator()
                << " - "
                << pd.Maximum.numerator() << "/" << pd.Maximum.denominator();
        } else {
            out << pd.Minimum.numerator() << " - " << pd.Maximum.numerator();
        }

        std::string name = pd.Binding.get().getName();
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
    for (const auto & e : make_iterator_range(out_edges(index, mBufferGraph))) {
        const auto bufferVertex = target(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[bufferVertex];
        if (LLVM_LIKELY(bn.Type != BufferType::Managed)) {
            const BufferRateData & rd = mBufferGraph[e];
            const auto prefix = makeBufferName(index, rd.Binding);
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
            const auto name = makeBufferName(p, rd.Binding);
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
    assert (getKernel(mKernelIndex) == mKernel);
    for (const auto pe : make_iterator_range(out_edges(mKernelIndex, mBufferGraph))) {
        const auto bufferVertex = target(pe, mBufferGraph);
        const auto outputPort = mBufferGraph[pe].outputPort();
        const Binding & output = getOutputBinding(outputPort);
        const BufferNode & bn = mBufferGraph[bufferVertex];
        StreamSetBuffer * const buffer = bn.Buffer;
        if (LLVM_LIKELY(bn.Type == BufferType::Internal)) {
            b->setKernel(mPipelineKernel);
            Value * const scalar = b->getScalarFieldPtr(makeBufferName(mKernelIndex, output));
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
    const auto firstBuffer = PipelineOutput + 1;
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
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = getInputBinding(i);
        const auto prefix = makeBufferName(mKernelIndex, input);
        Value * const processed = b->getScalarField(prefix + ITEM_COUNT_SUFFIX);
        mInitiallyProcessedItemCount[i] = processed;
        if (input.isDeferred()) {
            mInitiallyProcessedDeferredItemCount[i] = b->getScalarField(prefix + DEFERRED_ITEM_COUNT_SUFFIX);
        }
    }
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = getOutputBinding(i);
        const auto prefix = makeBufferName(mKernelIndex, output);
        Value * const produced = b->getScalarField(prefix + ITEM_COUNT_SUFFIX);
        mInitiallyProducedItemCount[i] = produced;
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
        const auto prefix = makeBufferName(mKernelIndex, input);
        b->setScalarField(prefix + ITEM_COUNT_SUFFIX, final ? mFinalProcessedPhi[i] : mUpdatedProcessedPhi[i]);
        if (input.isDeferred()) {
            b->setScalarField(prefix + DEFERRED_ITEM_COUNT_SUFFIX, final ? mFinalProcessedPhi[i] : mUpdatedProcessedDeferredPhi[i]);
        }
    }
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = getOutputBinding(i);
        const auto prefix = makeBufferName(mKernelIndex, output);
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
        initializeConsumedItemCount(bufferVertex, fullyProduced);
        initializePopCountReferenceItemCount(b, bufferVertex, fullyProduced);
        #ifdef PRINT_DEBUG_MESSAGES
        const auto & output = getOutputBinding(outputPort);
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
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto bufferVertex = getOutputBufferVertex(i);
        const BufferNode & bn = mBufferGraph[bufferVertex];
        if (bn.Overflow) {
            const StreamSetBuffer * const buffer = bn.Buffer;
            Value * const capacity = buffer->getCapacity(b.get());
            Value * const priorOffset = b->CreateURem(mAlreadyProducedPhi[i], capacity);
            Value * const produced = mProducedItemCount[i];
            Value * const producedOffset = b->CreateURem(produced, capacity);
            Value * const nonCapacityAlignedWrite = b->CreateIsNotNull(producedOffset);
            Value * const wroteToOverflow = b->CreateICmpULT(producedOffset, priorOffset);
            Value * const needsCopy = b->CreateAnd(nonCapacityAlignedWrite, wroteToOverflow);
            copy(b, CopyMode::Overflow, needsCopy, getOutputBinding(i), buffer, producedOffset);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeCopyToOverflowLogic
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeCopyForwardLogic(BuilderRef b) {
    // Unless we modified the portion of data that ought to be reflected in the overflow region, do not copy
    // any data. To do so would incur extra writes and pollute the cache with potentially unnecessary data.
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {

        const auto bufferVertex = getOutputBufferVertex(i);
        const BufferNode & bn = mBufferGraph[bufferVertex];

        if (bn.Underflow || bn.Fasimile) {

            const StreamSetBuffer * const buffer = bn.Buffer;

            Value * const capacity = buffer->getCapacity(b.get());
            Value * const initial = mInitiallyProducedItemCount[i];
            Value * const produced = mUpdatedProducedPhi[i];

            const Binding & output = getOutputBinding(i);

            if (bn.Fasimile) {

                // If we wrote anything and it was not our first write to the buffer ...
                Value * overwroteData = b->CreateICmpUGT(produced, capacity);
                if (LLVM_LIKELY(mKernel->getLowerBound(output) < 1)) {
                    Value * const producedOutput = b->CreateICmpNE(initial, produced);
                    overwroteData = b->CreateAnd(overwroteData, producedOutput);
                }
                // And we started writing within the first block ...
                assert (bn.Fasimile <= buffer->getOverflowCapacity(b));
                Constant * const overflowSize = b->getSize(bn.Fasimile);
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

                copy(b, CopyMode::Fasimile, needsCopy, output, buffer, overflowSize);

            }

            if (bn.Underflow) {
                Value * const producedOffset = b->CreateURem(produced, capacity);
                Constant * underflow = b->getSize(bn.Underflow);
                Value * const limit = b->CreateSub(capacity, underflow);
                Value * const needsCopy = b->CreateICmpUGE(producedOffset, limit);
                Value * const toCopy = b->CreateSub(producedOffset, limit);
                copy(b, CopyMode::Underflow, needsCopy, output, buffer, toCopy);
            }
        }
    }
}



/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeOverflowCopy
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::copy(BuilderRef b, const CopyMode mode, Value * cond,
                            const Binding & binding, const StreamSetBuffer * const buffer,
                            Value * const itemsToCopy) const {

    auto makeSuffix = [](CopyMode mode) {
        switch (mode) {
            case CopyMode::Fasimile: return "Fasimile";
            case CopyMode::Overflow: return "Overflow";
            case CopyMode::Underflow: return "Underflow";
        }
    };

    const auto prefix = makeBufferName(mKernelIndex, binding) + "_copy" + makeSuffix(mode);

    BasicBlock * const copyStart = b->CreateBasicBlock(prefix, mKernelExit);
    BasicBlock * const copyExit = b->CreateBasicBlock(prefix + "Exit", mKernelExit);

    b->CreateUnlikelyCondBr(cond, copyStart, copyExit);

    b->SetInsertPoint(copyStart);
    const auto itemWidth = getItemWidth(buffer->getBaseType());
    const auto blockWidth = b->getBitBlockWidth();
    Value * blocksToCopy = itemsToCopy;
    if (LLVM_LIKELY(itemWidth < blockWidth)) {
        blocksToCopy = b->CreateCeilUDiv(blocksToCopy, b->getSize(blockWidth / itemWidth));
    } else if (LLVM_UNLIKELY(blockWidth < itemWidth)) {
        blocksToCopy = b->CreateMul(blocksToCopy, b->getSize(itemWidth / blockWidth));
    }
    Value * const scale = b->CreateMul(buffer->getStreamSetCount(b.get()), b->getSize(blockWidth / 8));
    Value * const bytesToCopy = b->CreateMul(blocksToCopy, scale);

    Value * source = nullptr;
    Value * target = nullptr;

    if (mode == CopyMode::Fasimile) {
        source = buffer->getBaseAddress(b.get());
        target = buffer->getOverflowAddress(b.get());
    } else  {
        source = buffer->getOverflowAddress(b.get());
        target = buffer->getBaseAddress(b.get());
        if (mode == CopyMode::Underflow) {
            Constant * underflow = ConstantExpr::getNeg(b->getSize(buffer->getUnderflow()));
            source = b->CreateGEP(source, underflow);
            target = b->CreateGEP(target, underflow);
        }
    }

    b->CreateMemCpy(target, source, bytesToCopy, blockWidth / 8);

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

    Constant * const LOG_2_BLOCK_WIDTH = b->getSize(floor_log2(b->getBitBlockWidth()));
    Constant * const ZERO = b->getSize(0);
    PointerType * const bufferType = buffer->getPointerType();
    Value * const blockIndex = b->CreateLShr(position, LOG_2_BLOCK_WIDTH);
    Value * baseAddress = buffer->getBaseAddress(b.get());
    baseAddress = buffer->getStreamLogicalBasePtr(b.get(), baseAddress, ZERO, blockIndex);
    if (zeroExtended) {
        // prepareLocalZeroExtendSpace guarantees this will be large enough to satisfy the kernel
        ExternalBuffer tmp(b, binding.getType());
        Value * zeroExtension = b->CreatePointerCast(mZeroExtendBufferPhi, bufferType);
        zeroExtension = tmp.getStreamBlockPtr(b.get(), zeroExtension, ZERO, b->CreateNeg(blockIndex));
        baseAddress = b->CreateSelect(zeroExtended, zeroExtension, baseAddress);
    }
    baseAddress = b->CreatePointerCast(baseAddress, bufferType);
    return baseAddress;
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
inline const Binding & PipelineCompiler::getInputBinding(const unsigned kernelVertex, const unsigned inputPort) const {
    return mBufferGraph[getInput(kernelVertex, inputPort)].Binding;
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
    return mBufferGraph[getInput(mKernelIndex, inputPort)].Reason == ReasonType::Explicit;
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
inline const Binding & PipelineCompiler::getOutputBinding(const unsigned kernelVertex, const unsigned outputPort) const {
    return mBufferGraph[getOutput(kernelVertex, outputPort)].Binding;
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
    return in_degree(kernel, mBufferGraph);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNumOfStreamOutputs
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned PipelineCompiler::getNumOfStreamOutputs(const unsigned kernel) const {
    return out_degree(kernel, mBufferGraph);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getBinding
 ** ------------------------------------------------------------------------------------------------------------- */
const Binding & PipelineCompiler::getBinding(const StreamPort port) const {
    if (port.Type == PortType::Input) {
        return getInputBinding(port.Number);
    } else if (port.Type == PortType::Output) {
        return getOutputBinding(port.Number);
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
    return mPipelineGraph[index].Kernel;
}

} // end of kernel namespace

#endif // BUFFER_ALLOCATION_HPP
