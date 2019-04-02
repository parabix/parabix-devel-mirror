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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makePipelineBufferGraph
 *
 * Return an acyclic bi-partite graph indicating the I/O relationships between the kernels and their buffers.
 *
 * Ordering: producer -> buffer -> consumer
 ** ------------------------------------------------------------------------------------------------------------- */
BufferGraph PipelineCompiler::makeBufferGraph(BuilderRef b) {

    BufferGraph G(LastStreamSet + 1);

    for (unsigned i = PipelineInput; i <= PipelineOutput; ++i) {
        const Kernel * const kernel = mStreamGraph[i].Kernel; assert (kernel);

        auto computeBufferRateBounds = [&](const RelationshipType port, const RelationshipNode & bindingNode) {
            unsigned strideLength = kernel->getStride();
            if (i == PipelineInput || i == PipelineOutput) {
                strideLength = boost::lcm(codegen::SegmentSize, strideLength);
            }
            assert (bindingNode.Type == RelationshipNode::IsBinding);
            const Binding & binding = bindingNode.Binding;
            const ProcessingRate & rate = binding.getRate();
            RateValue lb{rate.getLowerBound()};
            RateValue ub{rate.getUpperBound()};

            if (LLVM_UNLIKELY(rate.isRelative())) {
                const Binding & ref = getBinding(getReference(port));
                const ProcessingRate & refRate = ref.getRate();
                lb *= refRate.getLowerBound();
                ub *= refRate.getUpperBound();
            }
            if (LLVM_UNLIKELY(binding.isDeferred())) {
                lb = RateValue{0};
            }
            return BufferRateData{port, binding, lb * strideLength, ub * strideLength};
        };

        // add in any inputs
        RelationshipType prior_in{};
        for (const auto & e : make_iterator_range(in_edges(i, mStreamGraph))) {
            const RelationshipType & port = mStreamGraph[e];
            assert (prior_in < port);
            prior_in = port;
            const auto binding = source(e, mStreamGraph);
            const RelationshipNode & rn = mStreamGraph[binding];
            assert (rn.Type == RelationshipNode::IsBinding);
            const auto f = first_in_edge(binding, mStreamGraph);
            assert (mStreamGraph[f].Reason != ReasonType::Reference);
            unsigned streamSet = source(f, mStreamGraph);
            assert (mStreamGraph[streamSet].Type == RelationshipNode::IsRelationship);
            assert (isa<StreamSet>(mStreamGraph[streamSet].Relationship));
            add_edge(streamSet, i, computeBufferRateBounds(port, rn), G);
        }

        // and any outputs
        RelationshipType prior_out{};
        for (const auto & e : make_iterator_range(out_edges(i, mStreamGraph))) {
            const RelationshipType & port = mStreamGraph[e];
            assert (prior_out < port);
            prior_out = port;
            const auto binding = target(e, mStreamGraph);
            const RelationshipNode & rn = mStreamGraph[binding];
            assert (rn.Type == RelationshipNode::IsBinding);
            const auto f = out_edge(binding, mStreamGraph);
            assert (mStreamGraph[f].Reason != ReasonType::Reference);
            unsigned streamSet = target(f, mStreamGraph);
            assert (mStreamGraph[streamSet].Type == RelationshipNode::IsRelationship);
            assert (isa<StreamSet>(mStreamGraph[streamSet].Relationship));
            add_edge(i, streamSet, computeBufferRateBounds(port, rn), G);
        }
    }

    // Since we do not want to create an artifical bottleneck by constructing output buffers that
    // cannot accommodate the full amount of data we could produce given the expected inputs, the
    // next loop will resize them accordingly.

    auto div_by_non_zero = [](const RateValue & num, const RateValue & denom) -> RateValue {
        return  (denom.numerator() == 0) ? num : (num / denom);
    };

    // compute how much data each kernel could consume/produce per iteration.
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        BufferNode & kn = G[i];
        if (LLVM_UNLIKELY(in_degree(i, G) == 0)) {
            kn.Lower = RateValue{1, 1};
            kn.Upper = RateValue{1, 1};
        } else {
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

    const auto requiredThreadSegments = ((codegen::ThreadNum > 1U) ? codegen::ThreadNum + 1U : 1U);
    const auto numOfSegments = std::max(codegen::BufferSegments, requiredThreadSegments);

    // then construct the rest
    for (unsigned i = FirstStreamSet; i <= LastStreamSet; ++i) {

        BufferNode & bn = G[i];
        const auto pe = in_edge(i, G);
        const auto producerVertex = source(pe, G);
        const Kernel * const producer = getKernel(producerVertex);
        const BufferRateData & producerRate = G[pe];
        const Binding & output = producerRate.Binding;

        // Verify all consumers expect the same input type as the produced output type.
        Type * const baseType = output.getType();
        for (const auto & e : make_iterator_range(out_edges(i, G))) {
            const BufferRateData & consumerRate = G[e];
            const Binding & input = consumerRate.Binding;
            if (LLVM_UNLIKELY(baseType != input.getType())) {
                std::string tmp;
                raw_string_ostream msg(tmp);
                msg << producer->getName() << ':' << output.getName()
                    << " produces a ";
                baseType->print(msg);
                const Kernel * const consumer = getKernel(target(e, G));
                msg << " but "
                    << consumer->getName() << ':' << input.getName()
                    << " expects ";
                input.getType()->print(msg);
                report_fatal_error(msg.str());
            }
        }

        // Is this a pipeline I/O buffer?
        assert ((bn.Buffer == nullptr) ^ (bn.Type == BufferType::External));
        if (bn.Type == BufferType::External) {
            continue;
        }
        const auto isUnknown = producerRate.Maximum.numerator() == 0;
        const auto isManaged = output.hasAttribute(AttrId::ManagedBuffer);

        StreamSetBuffer * buffer = nullptr;
        BufferType bufferType = BufferType::Internal;
        if (LLVM_UNLIKELY(isUnknown || isManaged)) {
            buffer = new ExternalBuffer(b, baseType);
            bufferType = BufferType::Managed;
        } else {

            auto getOverflowSize = [&](const Kernel * kernel, const Binding & binding, const BufferRateData & rd) -> RateValue {
                if (binding.hasAttribute(AttrId::BlockSize)) {
                    return 0;
                }
                const auto deviation = rd.Maximum - rd.Minimum;
                if (deviation.numerator() == 0) {
                    return deviation;
                }
                const ProcessingRate & pr = binding.getRate();
                RateValue ub{pr.getUpperBound()};
                if (LLVM_UNLIKELY(pr.isRelative())) {
                    const Binding & ref = getBinding(getReference(rd.Port));
                    const ProcessingRate & rate = ref.getRate();
                    ub *= rate.getUpperBound();
                }
                return std::min(deviation, ub * kernel->getStride());
            };

            RateValue requiredSpace{producerRate.Maximum};
            RateValue overflowSpace{getOverflowSize(producer, output, producerRate)};
            RateValue facsimileSpace{0};
            unsigned underflowSize = 0;
            bool unboundedLookbehind = false;

            if (LLVM_UNLIKELY(output.hasAttribute(AttrId::LookBehind))) {
                const auto & lookBehind = output.findAttribute(AttrId::LookBehind);
                const auto amount = lookBehind.amount();
                if (amount == 0) {
                    unboundedLookbehind = true;
                } else {
                    underflowSize = lookBehind.amount();
                }
            }

            // TODO: If we have an open system, then the input rate to this pipeline cannot
            // be bounded a priori. During initialization, we could pass a "suggestion"
            // argument to indicate what the outer pipeline believes its I/O rates will be.

            bool dynamic = false;
            for (const auto ce : make_iterator_range(out_edges(i, G))) {
                const BufferRateData & consumerRate = G[ce];
                requiredSpace = lcm(requiredSpace, consumerRate.Maximum);
                const auto c = target(ce, G);
                const BufferNode & consumerNode = G[c];
                const Kernel * const consumer = getKernel(c);
                const Binding & input = consumerRate.Binding;

                // get output overflow size
                auto overflow = getOverflowSize(consumer, input, consumerRate);
                if (LLVM_UNLIKELY(input.hasLookahead())) {
                    overflow += input.getLookahead();
                }
                facsimileSpace = std::max(facsimileSpace, overflow);

                // Could the consumption rate be less than the production rate?
                if ((consumerNode.Lower * consumerRate.Minimum) < producerRate.Maximum) {
                    dynamic = true;
                }

                // If we have a lookbehind attribute, make sure we have enough underflow
                // space to satisfy the processing rate.
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

            // calculate overflow (copyback) and fascimile (copyforward) space
            const auto blockWidth = b->getBitBlockWidth();

            // ensure any Add/RoundUpTo attributes are safely handled
            if (LLVM_UNLIKELY(output.hasAttribute(AttrId::Add))) {
                const auto & add = output.findAttribute(AttrId::Add);
                const RateValue amount(add.amount());
                overflowSpace = std::max(overflowSpace, amount);
            }
            if (LLVM_UNLIKELY(output.hasAttribute(AttrId::RoundUpTo))) {
                const auto & roundUpTo = output.findAttribute(AttrId::RoundUpTo);
                auto mod = [](const RateValue & a, const RateValue & b) -> RateValue {
                    RateValue n(a.numerator() * b.denominator(), b.numerator() * a.denominator());
                    return a - n * b;
                };
                RateValue r(roundUpTo.amount());
                const auto a = mod(producerRate.Minimum, r);
                const auto b = mod(producerRate.Maximum, r);
                overflowSpace = std::max(overflowSpace, std::max(a, b));
            }

            underflowSize = (underflowSize + blockWidth - 1) & -blockWidth;
            overflowSpace = lcm(overflowSpace, blockWidth);
            facsimileSpace = lcm(facsimileSpace, blockWidth);

            bn.LookBehind = underflowSize;
            bn.CopyBack = overflowSpace.numerator();
            assert (overflowSpace.denominator() == 1);
            bn.LookAhead = facsimileSpace.numerator();
            assert (facsimileSpace.denominator() == 1);

            unsigned overflowSize = std::max(bn.CopyBack, bn.LookAhead);
            // compute the buffer size
            if (underflowSize || overflowSize) {
                const RateValue requiredOverflow(std::max(underflowSize, overflowSize) * 2);
                requiredSpace = std::max(requiredSpace, requiredOverflow);
            }
            const auto bufferSpace = lcm(requiredSpace, blockWidth);
            assert (bufferSpace.denominator() == 1);
            const auto bufferSize = bufferSpace.numerator() * numOfSegments;
            assert (codegen::BufferSegments > 0);
            assert (codegen::ThreadNum > 0);

            if (LLVM_UNLIKELY(unboundedLookbehind)) {
                buffer = new LinearBuffer(b, baseType, bufferSize, overflowSize, underflowSize, 0);
            } else if (dynamic) {
                // A DynamicBuffer is necessary when we cannot bound the amount of unconsumed data a priori.
                buffer = new DynamicBuffer(b, baseType, bufferSize, overflowSize, underflowSize, 0);
            } else {
                buffer = new StaticBuffer(b, baseType, bufferSize, overflowSize, underflowSize, 0);
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

    using KindId = StreamSetBuffer::BufferKind;

    out << "digraph G {\n"
           "v" << PipelineInput << " [label=\"[" << PipelineInput << "] P_{in}\" peripheries=2, shape=rect];\n";

    auto rate_range = [&out](const RateValue & a, const RateValue & b) {
        if (a.denominator() > 1 || b.denominator() > 1) {
            out << a.numerator() << "/" << a.denominator()
                << " - "
                << b.numerator() << "/" << b.denominator();
        } else {
            out << a.numerator() << " - " << b.numerator();
        }
    };

    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        const Kernel * const kernel = getKernel(i);
        std::string name = kernel->getName();
        boost::replace_all(name, "\"", "\\\"");

        const BufferNode & bn = G[i];

        out << "v" << i <<
               " [label=\"[" << i << "] " << name << '\n';
        rate_range(bn.Lower, bn.Upper);
        out << "\" peripheries=2, shape=rect];\n";
    }

    out << "v" << PipelineOutput << " [label=\"[" << PipelineOutput << "]P_{out}\" peripheries=2, shape=rect];\n";

    const auto firstBuffer = PipelineOutput + 1;
    const auto lastBuffer = num_vertices(G);

    for (unsigned i = firstBuffer; i != lastBuffer; ++i) {
        out << "v" << i << " [shape=record, label=\""
               << i << "|{";

        const BufferNode & bn = G[i];
        const StreamSetBuffer * const buffer = bn.Buffer;
        if (buffer == nullptr) {
            out << '?';
        } else {
            char bufferType = '?';
            switch (buffer->getBufferKind()) {
                case KindId::ExternalBuffer: bufferType = 'E'; break;
                case KindId::StaticBuffer: bufferType = 'S'; break;
                case KindId::DynamicBuffer: bufferType = 'D'; break;
                case KindId::LinearBuffer: bufferType = 'L'; break;
                default: llvm_unreachable("unknown buffer type");
            }

            Type * ty = buffer->getBaseType();
            out << bufferType << ':'
                << ty->getArrayNumElements() << 'x';
            ty = ty->getArrayElementType();
            ty = ty->getVectorElementType();
            out << ty->getIntegerBitWidth();
        }

        out << "|{";

        if (buffer && buffer->getBufferKind() != KindId::ExternalBuffer) {
            switch (buffer->getBufferKind()) {
                case KindId::StaticBuffer:
                    out << cast<StaticBuffer>(buffer)->getCapacity();
                    break;
                case KindId::DynamicBuffer:
                    out << cast<DynamicBuffer>(buffer)->getInitialCapacity();
                    break;
                case KindId::LinearBuffer:
                    out << cast<LinearBuffer>(buffer)->getInitialCapacity();
                    break;
                default: llvm_unreachable("unknown buffer type");
            }
        }

        if (bn.LookBehind) {
            out << "|U:" << bn.LookBehind;
        }
        if (bn.CopyBack) {
            out << "|O:" << bn.CopyBack;
        }
        if (bn.LookAhead) {
            out << "|F:" << bn.LookAhead;
        }

        out << "}}\"];\n";
    }

    for (auto e : make_iterator_range(edges(G))) {
        const auto s = source(e, G);
        const auto t = target(e, G);
        out << "v" << s << " -> v" << t;
        const BufferRateData & pd = G[e];
        out << " [label=\"#" << pd.Port.Number << ": ";
        rate_range(pd.Minimum, pd.Maximum);
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
            const auto name = makeBufferName(p, rd.Port);
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
            Value * const capacity = buffer->getCapacity(b.get());
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
            Value * const capacity = buffer->getCapacity(b.get());
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

            Value * const capacity = buffer->getCapacity(b.get());
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
                            const unsigned itemsToCopy) const {

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
    const auto itemWidth = getItemWidth(buffer->getBaseType());
    const auto blockWidth = b->getBitBlockWidth();
    assert ((itemsToCopy % blockWidth) == 0);
    Value * const numOfStreams = buffer->getStreamSetCount(b.get());
    Value * const overflowSize = b->getSize(itemsToCopy * itemWidth / 8);
    Value * const bytesToCopy = b->CreateMul(overflowSize, numOfStreams);
    Value * source = nullptr;
    Value * target = nullptr;

    if (mode == CopyMode::LookAhead) {
        source = buffer->getBaseAddress(b.get());
        target = buffer->getOverflowAddress(b.get());
    } else  {
        source = buffer->getOverflowAddress(b.get());
        target = buffer->getBaseAddress(b.get());
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
