#ifndef BUFFER_ANALYSIS_HPP
#define BUFFER_ANALYSIS_HPP

#include "pipeline_analysis.hpp"

// TODO: any buffers that exist only to satisfy the output dependencies are unnecessary.
// We could prune away kernels if none of their outputs are needed but we'd want some
// form of "fake" buffer for output streams in which only some are unnecessary. Making a
// single static thread local buffer thats large enough for one segment.

// TODO: can we "combine" static stream sets that are used together and use fixed offsets
// from the first set? Would this improve data locality or prefetching?

// TODO: generate thread local buffers when we can guarantee all produced data is consumed
// within the same segment "iteration"? We can eliminate synchronization for kernels that
// consume purely local data.

// TODO: if we can prove the liveness of two streams never overlaps, can we reuse the
// memory space.

// TODO: if an external buffer is marked as managed, have it allocate and manage the
// buffer but not deallocate it.

namespace kernel {

inline static unsigned ceil_udiv(const unsigned x, const unsigned y) {
    assert (is_power_2(y));
    return (((x - 1) | (y - 1)) + 1) / y;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addStreamSetsToBufferGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::addStreamSetsToBufferGraph(BuilderRef b) {

    // Identify all External I/O buffers
    SmallFlatSet<BufferGraph::vertex_descriptor, 16> IsExternal;

    IsExternal.reserve(out_degree(PipelineInput, mBufferGraph) + in_degree(PipelineOutput, mBufferGraph));
    for (const auto e : make_iterator_range(out_edges(PipelineInput, mBufferGraph))) {
        IsExternal.insert(target(e, mBufferGraph));
    }


    for (const auto e : make_iterator_range(in_edges(PipelineOutput, mBufferGraph))) {
        IsExternal.insert(source(e, mBufferGraph));
    }

    auto internalOrExternal = [&IsExternal](BufferGraph::vertex_descriptor streamSet) -> BufferType {
        if (LLVM_LIKELY(IsExternal.count(streamSet) == 0)) {
            return BufferType::Internal;
        } else {
            return BufferType::External;
        }
    };

    // fill in any known managed buffers
    for (auto kernel = FirstKernel; kernel <= LastKernel; ++kernel) {
        const Kernel * const kernelObj = getKernel(kernel);
        const auto internallySynchronized = kernelObj->hasAttribute(AttrId::InternallySynchronized);
        for (const auto e : make_iterator_range(out_edges(kernel, mBufferGraph))) {
            const BufferRateData & producerRate = mBufferGraph[e];
            const Binding & output = producerRate.Binding;
            if (LLVM_UNLIKELY(internallySynchronized || Kernel::isLocalBuffer(output))) {
                const auto streamSet = target(e, mBufferGraph);
                BufferNode & bn = mBufferGraph[streamSet];
                // Every managed buffer is considered linear to the pipeline
                bn.Buffer = new ExternalBuffer(b, output.getType(), true, 0);
                bn.Type = BufferType::Unowned | internalOrExternal(streamSet);
            }
        }
    }

    // fill in any unmanaged pipeline input buffers
    for (const auto e : make_iterator_range(out_edges(PipelineInput, mBufferGraph))) {
        const auto streamSet = target(e, mBufferGraph);
        BufferNode & bn = mBufferGraph[streamSet];
        if (LLVM_LIKELY(bn.Buffer == nullptr)) {
            const BufferRateData & rate = mBufferGraph[e];
            const Binding & input = rate.Binding;
            bn.Buffer = new ExternalBuffer(b, input.getType(), true, 0);
            bn.Type = BufferType::UnownedExternal;
        }
    }

    // and pipeline output buffers ...
    for (const auto e : make_iterator_range(in_edges(PipelineOutput, mBufferGraph))) {
        const auto streamSet = source(e, mBufferGraph);
        BufferNode & bn = mBufferGraph[streamSet];
        if (LLVM_LIKELY(bn.Buffer == nullptr)) {
            const BufferRateData & rate = mBufferGraph[e];
            const Binding & output = rate.Binding;
            if (LLVM_UNLIKELY(Kernel::isLocalBuffer(output))) {
                continue;
            }
            bn.Buffer = new ExternalBuffer(b, output.getType(), true, 0);
            bn.Type = BufferType::UnownedExternal;
        }
    }

    const auto blockWidth = b->getBitBlockWidth();

    mInternalBuffers.resize(LastStreamSet - FirstStreamSet + 1);

    // then construct the rest
    for (auto streamSet = FirstStreamSet; streamSet <= LastStreamSet; ++streamSet) {

        BufferNode & bn = mBufferGraph[streamSet];
        const auto producerOutput = in_edge(streamSet, mBufferGraph);
        const BufferRateData & producerRate = mBufferGraph[producerOutput];
        const Binding & output = producerRate.Binding;

        bool nonLocal = false;

        // Does this stream cross a partition boundary?
        const auto producer = source(producerOutput, mBufferGraph);
        if (producer == PipelineInput) {
            nonLocal = true;
        }
        const auto producerPartitionId = KernelPartitionId[producer];
        for (const auto ce : make_iterator_range(out_edges(streamSet, mBufferGraph))) {
            const auto consumer = target(ce, mBufferGraph);
            if (producerPartitionId != KernelPartitionId[consumer]) {
                nonLocal = true;
                break;
            }
        }

        if (LLVM_LIKELY(bn.Buffer == nullptr)) { // is internal buffer

            // TODO: If we have an open system, then the input rate to this pipeline cannot
            // be bounded a priori. During initialization, we could pass a "suggestion"
            // argument to indicate what the outer pipeline believes its I/O rates will be.

            const auto bufferType = internalOrExternal(streamSet);



            bn.Type = bufferType;

            // If this buffer is externally used, we cannot analyze the dataflow rate of
            // external consumers. Default to dynamic for such buffers.

            // Similarly if any internal consumer has a deferred rate, we cannot analyze
            // any consumption rates.

            bool dynamic = nonLocal || (bufferType == BufferType::External) || producerRate.IsDeferred;

            auto maxDelay = producerRate.Delay;
            auto maxLookAhead = producerRate.LookAhead;
            auto maxLookBehind = producerRate.LookBehind;


//            const auto pMin = producerRate.Minimum * MinimumNumOfStrides[producer];
            const auto pMax = producerRate.Maximum * MaximumNumOfStrides[producer];

            assert (producerRate.Maximum >= producerRate.Minimum);

            if (producerRate.Minimum < producerRate.Maximum) {
                nonLocal = true;
            }

            // TODO: dataflow analysis must take lookahead/delay into account to permit
            // this optimization.

            Rational consumeMin(std::numeric_limits<unsigned>::max());
            Rational consumeMax(std::numeric_limits<unsigned>::min());

            for (const auto ce : make_iterator_range(out_edges(streamSet, mBufferGraph))) {

                const BufferRateData & consumerRate = mBufferGraph[ce];
                const Binding & input = consumerRate.Binding;

                const auto consumer = target(ce, mBufferGraph);

                const auto cMin = consumerRate.Minimum * MinimumNumOfStrides[consumer];
                const auto cMax = consumerRate.Maximum * MaximumNumOfStrides[consumer];

                // Could we consume less data than we produce?
                if (consumerRate.Minimum < consumerRate.Maximum) {
                    dynamic = true;
                // Or is the data consumption rate unpredictable despite its type?
                } else if (LLVM_UNLIKELY(consumerRate.IsDeferred)) {
                    dynamic = true;
                }
                assert (consumerRate.Maximum >= consumerRate.Minimum);

                consumeMin = std::min(consumeMin, cMin);
                consumeMax = std::max(consumeMax, cMax);

                assert (consumeMax >= consumeMin);

                const Binding & inputBinding = consumerRate.Binding;
//                // Get output overflow size
                auto lookAhead = consumerRate.LookAhead;
                if (consumerRate.Maximum > consumerRate.Minimum) {
                    lookAhead += ceiling(consumerRate.Maximum - consumerRate.Minimum);
                }

                maxDelay = std::max(maxDelay, consumerRate.Delay);
                maxLookAhead = std::max(maxLookAhead, lookAhead);
                maxLookBehind = std::max(maxLookBehind, consumerRate.LookBehind);

            }

            bn.LookAhead = maxLookAhead;
            bn.LookBehind = maxLookBehind;

//            reqOverflow = std::max(bn.MaxAdd, maxLookAhead);
//            if (bn.MaxAdd) {
//                reqOverflow = std::max(reqOverflow, bn.MaxAdd);
//            }
//            if (reqOverflow > 0)  {
//                reqOverflow = std::max(reqOverflow, ceiling(consumeMax));
//            }

            // calculate overflow (copyback) and fascimile (copyforward) space
            const auto overflowSize = round_up_to(std::max(bn.MaxAdd, bn.LookAhead), blockWidth) / blockWidth;
            const auto underflowSize = round_up_to(std::max(bn.LookBehind, maxDelay), blockWidth) / blockWidth;
            const auto spillover = std::max((consumeMax * Rational{2}) - consumeMin, pMax);
            const auto reqSize0 = round_up_to(ceiling(spillover), blockWidth) / blockWidth;
            const auto reqSize1 = 2 * (overflowSize + underflowSize);
            const auto requiredSize = std::max(reqSize0, reqSize1);


            // if this buffer is "stateful", we cannot make it *thread* local
            if (maxLookBehind || maxDelay || bn.CopyBack || maxLookAhead) {
                nonLocal = true;
            }

            Type * const baseType = output.getType();

            // A DynamicBuffer is necessary when we cannot bound the amount of unconsumed data a priori.
            StreamSetBuffer * buffer = nullptr;
            if (dynamic || nonLocal) {
                // TODO: we can make some buffers static despite crossing a partition but only if we can guarantee
                // an upper bound to the buffer size for all potential inputs. Build a dataflow analysis to
                // determine this.
                const auto bufferSize = requiredSize * mNumOfThreads;
                buffer = new DynamicBuffer(b, baseType, bufferSize, overflowSize, underflowSize, true, 0U); // !bn.NonLinear
            } else {
                assert (!bn.NonLinear);
                buffer = new StaticBuffer(b, baseType, requiredSize, overflowSize, underflowSize, true, 0U);
            }
            bn.Buffer = buffer;
        }

        assert ("missing buffer?" && bn.Buffer);

        bn.NonLocal = nonLocal;
        mInternalBuffers[streamSet - FirstStreamSet].reset(bn.Buffer);
    }   
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateInitialBufferGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::generateInitialBufferGraph() {

    mBufferGraph = BufferGraph(LastStreamSet + 1U);

    using Graph = adjacency_list<hash_setS, vecS, bidirectionalS, RelationshipGraph::edge_descriptor>;
    using Vertex = graph_traits<Graph>::vertex_descriptor;

    for (auto i = PipelineInput; i <= PipelineOutput; ++i) {
        const RelationshipNode & node = mStreamGraph[i];
        const Kernel * const kernelObj = node.Kernel; assert (kernelObj);

        auto makeBufferPort = [&](const RelationshipType port,
                                  const RelationshipNode & bindingNode,
                                  const unsigned streamSet) {
            assert (bindingNode.Type == RelationshipNode::IsBinding);
            const Binding & binding = bindingNode.Binding;
            const ProcessingRate & rate = binding.getRate();
            Rational lb{rate.getLowerBound()};
            Rational ub{rate.getUpperBound()};
            if (LLVM_UNLIKELY(rate.isGreedy())) {
                if (LLVM_UNLIKELY(port.Type == PortType::Output)) {
                    SmallVector<char, 0> tmp;
                    raw_svector_ostream out(tmp);
                    out << "Greedy rate cannot be applied an output port: "
                        << kernelObj->getName() << "." << binding.getName();
                    report_fatal_error(out.str());
                }
                const auto e = in_edge(streamSet, mBufferGraph);
                const BufferRateData & producerBr = mBufferGraph[e];
                ub = std::max(lb, producerBr.Maximum);
            } else {
                const auto strideLength = kernelObj->getStride();
                if (LLVM_UNLIKELY(rate.isRelative())) {
                    const Binding & ref = getBinding(i, getReference(i, port));
                    const ProcessingRate & refRate = ref.getRate();
                    lb *= refRate.getLowerBound();
                    ub *= refRate.getUpperBound();
                }
                lb *= strideLength;
                ub *= strideLength;
            }

            auto add = 0U;
            auto delay = 0U;
            auto lookAhead = 0U;
            auto lookBehind = 0U;
            auto truncate = 0U;
            auto isPrincipal = false;
            auto isDeferred = false;

//            if (lb != ub) {
//                lookAhead = ceiling(ub - lb) - 1U;
//            }

            for (const Attribute & attr : binding.getAttributes()) {
                switch (attr.getKind()) {
                    case AttrId::Add:
                        add = std::max(add, attr.amount());
                        break;
                    case AttrId::Delayed:
                        delay = std::max(delay, attr.amount());
                        break;
                    case AttrId::LookAhead:
                        lookAhead = std::max(lookAhead, attr.amount());
                        break;
                    case AttrId::LookBehind:
                        lookBehind = std::max(lookBehind, attr.amount());
                        break;
                    case AttrId::Truncate:
                        truncate = std::max(truncate, attr.amount());
                        break;
                    case AttrId::Principal:
                        isPrincipal = true;
                        break;
                    case AttrId::Deferred:
                        isDeferred = true;
                        break;
                    default: break;
                }
            }



            return BufferRateData{port, binding, lb, ub, add, truncate,
                                  delay, lookAhead, lookBehind,
                                  isPrincipal, isDeferred};
        };

        // Evaluate the input/output ordering here and ensure that any reference port is stored first.
        const auto numOfInputs = in_degree(i, mStreamGraph);
        const auto numOfOutputs = out_degree(i, mStreamGraph);

        const auto numOfPorts = numOfInputs + numOfOutputs;

        if (LLVM_UNLIKELY(numOfPorts == 0)) {
            continue;
        }

        Graph E(numOfPorts);

        #ifndef NDEBUG
        RelationshipType prior_in{};
        #endif
        for (auto e : make_iterator_range(in_edges(i, mStreamGraph))) {
            const RelationshipType & port = mStreamGraph[e];
            #ifndef NDEBUG
            assert (prior_in < port);
            prior_in = port;
            #endif
            const auto binding = source(e, mStreamGraph);
            const RelationshipNode & rn = mStreamGraph[binding];
            assert (rn.Type == RelationshipNode::IsBinding);
            E[port.Number] = e;
            if (LLVM_UNLIKELY(in_degree(binding, mStreamGraph) != 1)) {
                for (const auto f : make_iterator_range(in_edges(binding, mStreamGraph))) {
                    const RelationshipType & ref = mStreamGraph[f];
                    if (ref.Reason == ReasonType::Reference) {
                        if (LLVM_UNLIKELY(port.Type == PortType::Output)) {
                            SmallVector<char, 256> tmp;
                            raw_svector_ostream out(tmp);
                            out << "Error: input reference for binding " <<
                                   kernelObj->getName() << "." << rn.Binding.get().getName() <<
                                   " refers to an output stream.";
                            report_fatal_error(out.str());
                        }
                        add_edge(ref.Number, port.Number, E);
                        break;
                    }
                }
            }
        }

        #ifndef NDEBUG
        RelationshipType prior_out{};
        #endif
        for (auto e : make_iterator_range(out_edges(i, mStreamGraph))) {
            const RelationshipType & port = mStreamGraph[e];
            #ifndef NDEBUG
            assert (prior_out < port);
            prior_out = port;
            #endif
            const auto binding = target(e, mStreamGraph);
            const RelationshipNode & rn = mStreamGraph[binding];
            assert (rn.Type == RelationshipNode::IsBinding);
            const auto portNum = port.Number + numOfInputs;
            E[portNum] = e;
            if (LLVM_UNLIKELY(in_degree(binding, mStreamGraph) != 1)) {
                for (const auto f : make_iterator_range(in_edges(binding, mStreamGraph))) {
                    const RelationshipType & ref = mStreamGraph[f];
                    if (ref.Reason == ReasonType::Reference) {
                        auto refPort = ref.Number;
                        if (LLVM_UNLIKELY(ref.Type == PortType::Output)) {
                            refPort += numOfInputs;
                        }
                        add_edge(refPort, portNum, E);
                        break;
                    }
                }
            }
        }

        BitVector V(numOfPorts);
        std::queue<Vertex> Q;

        auto add_edge_if_no_induced_cycle = [&](const Vertex s, const Vertex t) {
            // If s-t exists, skip adding this edge
            if (LLVM_UNLIKELY(edge(s, t, E).second || s == t)) {
                return;
            }

            // If G is a DAG and there is a t-s path, adding s-t will induce a cycle.
            if (in_degree(s, E) > 0) {
                // do a BFS to search for a t-s path
                V.reset();
                assert (Q.empty());
                Q.push(t);
                for (;;) {
                    const auto u = Q.front();
                    Q.pop();
                    for (auto e : make_iterator_range(out_edges(u, E))) {
                        const auto v = target(e, E);
                        if (LLVM_UNLIKELY(v == s)) {
                            // we found a t-s path
                            return;
                        }
                        if (LLVM_LIKELY(!V.test(v))) {
                            V.set(v);
                            Q.push(v);
                        }
                    }
                    if (Q.empty()) {
                        break;
                    }
                }
            }
            add_edge(s, t, E);
        };

        for (unsigned j = 1; j < numOfPorts; ++j) {
            add_edge_if_no_induced_cycle(j - 1, j);
        }

        SmallVector<Graph::vertex_descriptor, 16> ordering;
        ordering.reserve(numOfPorts);
        lexical_ordering(E, ordering);

        for (const auto k : ordering) {
            const auto e = E[k];
            const RelationshipType & port = mStreamGraph[e];
            if (port.Type == PortType::Input) {
                const auto binding = source(e, mStreamGraph);
                const RelationshipNode & rn = mStreamGraph[binding];
                assert (rn.Type == RelationshipNode::IsBinding);
                const auto f = first_in_edge(binding, mStreamGraph);
                assert (mStreamGraph[f].Reason != ReasonType::Reference);
                const auto streamSet = source(f, mStreamGraph);
                assert (mStreamGraph[streamSet].Type == RelationshipNode::IsRelationship);
                assert (isa<StreamSet>(mStreamGraph[streamSet].Relationship));
                add_edge(streamSet, i, makeBufferPort(port, rn, streamSet), mBufferGraph);
            } else {
                const auto binding = target(e, mStreamGraph);
                const RelationshipNode & rn = mStreamGraph[binding];
                assert (rn.Type == RelationshipNode::IsBinding);
                const auto f = out_edge(binding, mStreamGraph);
                assert (mStreamGraph[f].Reason != ReasonType::Reference);
                const auto streamSet = target(f, mStreamGraph);
                assert (mStreamGraph[streamSet].Type == RelationshipNode::IsRelationship);
                assert (isa<StreamSet>(mStreamGraph[streamSet].Relationship));
                add_edge(i, streamSet, makeBufferPort(port, rn, streamSet), mBufferGraph);
            }
        }
    }

#if 0
    Rational ONE{1};

    for (auto i = FirstStreamSet; i <= LastStreamSet; ++i) {
        const auto e = in_edge(i, mBufferGraph);
        const BufferRateData & br = mBufferGraph[e];
        BufferNode & bn = mBufferGraph[i];
        bn.CopyBack = ceiling(br.Maximum - br.Minimum);
        auto peekable = br.Delay;
        auto maxLookAhead = 0U;

        auto maxLookBehind = br.LookBehind;

        for (const auto e : make_iterator_range(out_edges(i, mBufferGraph))) {
            const BufferRateData & br = mBufferGraph[e];
            if (br.Maximum != br.Minimum) {
                const auto diff = br.Maximum - br.Minimum;
                peekable = std::max(peekable, ceiling(diff) - 1U);
            }
            maxLookAhead = std::max(maxLookAhead, br.LookAhead);
            maxLookBehind = std::max(maxLookBehind, br.LookBehind);
        }
        bn.CopyBackReflection = std::max(maxLookAhead, peekable);
        bn.LookBehind = maxLookBehind;
    }

    for (auto i = FirstStreamSet; i <= LastStreamSet; ++i) {
        const auto e = in_edge(i, mBufferGraph);
        const BufferRateData & br = mBufferGraph[e];
        auto copyBack = br.LookAhead;
        for (const auto e : make_iterator_range(out_edges(i, mBufferGraph))) {
            const BufferRateData & br = mBufferGraph[e];
            copyBack = std::max(copyBack, br.LookAhead);
        }
        BufferNode & bn = mBufferGraph[i];
        bn.CopyBack = copyBack;
    }
#endif
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief identifyLinearBuffers
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::identifyLinearBuffers() {

    auto isNonLinear = [](const Binding & binding) {
        const ProcessingRate & rate = binding.getRate();
        const auto isFixed = rate.isFixed();
        if (LLVM_LIKELY(isFixed)) {
            for (const Attribute & attr : binding.getAttributes()) {
                switch(attr.getKind()) {
                    case AttrId::Linear:
                    case AttrId::Deferred:
                    case AttrId::LookBehind:
                        return false;
                    default: break;
                }
            }
        }
        return !isFixed;
    };

    for (auto streamSet = FirstStreamSet; streamSet <= LastStreamSet; ++streamSet) {
        BufferNode & N = mBufferGraph[streamSet];
        const auto output = in_edge(streamSet, mBufferGraph);
        const BufferRateData & br = mBufferGraph[output];
        N.NonLinear = isNonLinear(br.Binding);
    }

    // All pipeline I/O must be linear
    for (const auto e : make_iterator_range(out_edges(PipelineInput, mBufferGraph))) {
        const auto streamSet = source(e, mBufferGraph);
        BufferNode & N = mBufferGraph[streamSet];
        N.NonLinear = false;
    }

    for (const auto e : make_iterator_range(in_edges(PipelineOutput, mBufferGraph))) {
        const auto streamSet = source(e, mBufferGraph);
        BufferNode & N = mBufferGraph[streamSet];
        N.NonLinear = false;
    }

    // Any kernel that is internally synchronized or has a greedy rate input
    // requires that all of its inputs are linear.
    for (auto i = FirstKernel; i <= LastKernel; ++i) {
        const Kernel * const kernelObj = getKernel(i);
        bool mustBeLinear = false;
        if (LLVM_UNLIKELY(kernelObj->hasAttribute(AttrId::InternallySynchronized))) {
            // An internally synchronized kernel requires that all I/O is linear
            for (const auto e : make_iterator_range(out_edges(i, mBufferGraph))) {
                const auto streamSet = target(e, mBufferGraph);
                BufferNode & N = mBufferGraph[streamSet];
                N.NonLinear = false;
            }
            mustBeLinear = true;
        } else {
            for (const auto e : make_iterator_range(in_edges(i, mBufferGraph))) {
                const BufferRateData & rateData = mBufferGraph[e];
                const Binding & binding = rateData.Binding;
                const ProcessingRate & rate = binding.getRate();
                if (LLVM_UNLIKELY(rate.isGreedy())) {
                    mustBeLinear = true;
                    break;
                }
            }
        }
        if (LLVM_UNLIKELY(mustBeLinear)) {
            for (const auto e : make_iterator_range(in_edges(i, mBufferGraph))) {
                const auto streamSet = source(e, mBufferGraph);
                BufferNode & N = mBufferGraph[streamSet];
                N.NonLinear = false;
            }
        }
    }

    // If the binding attributes of the producer/consumer(s) of a streamSet indicate
    // that the kernel requires linear input, mark it accordingly.
    for (auto streamSet = FirstStreamSet; streamSet <= LastStreamSet; ++streamSet) {

        BufferNode & N = mBufferGraph[streamSet];
        if (!N.NonLinear) {
            continue;
        }

        const auto binding = in_edge(streamSet, mBufferGraph);
        const BufferRateData & producerRate = mBufferGraph[binding];
        const Binding & output = producerRate.Binding;


        const auto producer = source(binding, mBufferGraph);
        const auto partitionId = KernelPartitionId[producer];

        bool nonLinear = true;
        if (LLVM_UNLIKELY(isNonLinear(output))) {
            nonLinear = false;
        } else {
            bool samePartition = true;

            for (const auto binding : make_iterator_range(out_edges(streamSet, mBufferGraph))) {
                const BufferRateData & consumerRate = mBufferGraph[binding];
                const Binding & input = consumerRate.Binding;
                if (LLVM_UNLIKELY(isNonLinear(input))) {
                    nonLinear = false;
                    break;
                }
                const auto consumer = target(binding, mBufferGraph);
                samePartition &= (KernelPartitionId[consumer] == partitionId);
           }
           nonLinear |= !samePartition;
        }       
        N.NonLinear = nonLinear;
    }

#if 0
    // Any ImplicitPopCount/RegionSelector inputs must be linear to ensure
    // we can easily access all of the rate information.
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        for (const auto e : make_iterator_range(in_edges(i, mStreamGraph))) {
            const RelationshipType & rt = mStreamGraph[e];
            switch (rt.Reason) {
                case ReasonType::ImplicitPopCount:
                case ReasonType::ImplicitRegionSelector:
                    BEGIN_SCOPED_REGION
                    const auto binding = source(e, mStreamGraph);
                    const auto streamSet = parent(binding, mStreamGraph);
                    BufferNode & N = G[streamSet];
                    N.Linear = true;
                    END_SCOPED_REGION
                default: break;
            }
        }
    }
#endif
}

}

#endif
