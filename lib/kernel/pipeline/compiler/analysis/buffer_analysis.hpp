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
        for (const auto e : make_iterator_range(out_edges(kernel, mBufferGraph))) {
            const BufferRateData & producerRate = mBufferGraph[e];
            const Binding & output = producerRate.Binding;
            if (LLVM_UNLIKELY(Kernel::isLocalBuffer(output))) {
                const auto streamSet = target(e, mBufferGraph);
                BufferNode & bn = mBufferGraph[streamSet];
                const auto linear = output.hasAttribute(AttrId::Linear);
                bn.Buffer = new ExternalBuffer(b, output.getType(), linear, 0);
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
            if (LLVM_UNLIKELY(Kernel::isLocalBuffer(output))) continue;
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

            const BufferType bufferType = internalOrExternal(streamSet);

            bn.Type = bufferType;

            // If this buffer is externally used, we cannot analyze the dataflow rate of
            // external consumers. Default to dynamic for such buffers.

            // Similarly if any internal consumer has a deferred rate, we cannot analyze
            // any consumption rates.

            bool dynamic = nonLocal || (bufferType == BufferType::External) || output.hasAttribute(AttrId::Deferred);

            unsigned lookAhead = 0;
            unsigned lookBehind = 0;
            unsigned reflection = 0;

            const Binding & outputBinding = producerRate.Binding;
            for (const Attribute & attr : outputBinding.getAttributes()) {
                switch (attr.getKind()) {
                    case AttrId::LookBehind:
                        lookBehind = std::max(lookBehind, attr.amount());
                        break;
                    case AttrId::Delayed:
                        reflection = std::max(reflection, attr.amount());
                        break;
                    default: break;
                }
            }

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
                } else if (LLVM_UNLIKELY(input.hasAttribute(AttrId::Deferred))) {
                    dynamic = true;
                }
                assert (consumerRate.Maximum >= consumerRate.Minimum);

                consumeMin = std::min(consumeMin, cMin);
                consumeMax = std::max(consumeMax, cMax);

                assert (consumeMax >= consumeMin);

                const Binding & inputBinding = consumerRate.Binding;
                // Get output overflow size
                unsigned tmpLookAhead = 0;
                for (const Attribute & attr : inputBinding.getAttributes()) {
                    switch (attr.getKind()) {
                        case AttrId::LookAhead:
                            tmpLookAhead = std::max(tmpLookAhead, attr.amount());
                            break;
                        case AttrId::LookBehind:
                            lookBehind = std::max(lookBehind, attr.amount());
                            break;
                        default: break;
                    }
                }
                if (consumerRate.Maximum > consumerRate.Minimum) {
                    tmpLookAhead += ceiling(consumerRate.Maximum - consumerRate.Minimum);
                }
                lookAhead = std::max(tmpLookAhead, lookAhead);
            }

            const auto copyBack = ceiling(producerRate.Maximum - producerRate.Minimum);

            auto reqOverflow = std::max(copyBack, lookAhead);
            if (bn.Add > 0) {
                reqOverflow = std::max(reqOverflow, bn.Add);
            }

            // calculate overflow (copyback) and fascimile (copyforward) space
            const auto overflowSize = round_up_to(reqOverflow, blockWidth) / blockWidth;
            const auto underflowSize = round_up_to(std::max(lookBehind, reflection), blockWidth) / blockWidth;
            const auto spillover = std::max((consumeMax * Rational{2}) - consumeMin, pMax);
            const auto reqSize0 = round_up_to(ceiling(spillover), blockWidth) / blockWidth;
            const auto reqSize1 = 2 * (overflowSize + underflowSize);
            const auto requiredSize = std::max(reqSize0, reqSize1);


            // if this buffer is "stateful", we cannot make it *thread* local
            if (dynamic || lookBehind || reflection || copyBack || lookAhead) {
                nonLocal = true;
            }

            bn.LookBehind = lookBehind;
            bn.LookBehindReflection = reflection;
            bn.CopyBack = copyBack;
            bn.LookAhead = lookAhead;

            Type * const baseType = output.getType();

            #ifdef PERMIT_THREAD_LOCAL_BUFFERS
            const auto bufferFactor = nonLocal ? mNumOfThreads : 1U;
            #else
            const auto bufferFactor = mNumOfThreads;
            #endif

            const auto bufferSize = requiredSize * bufferFactor;

            // A DynamicBuffer is necessary when we cannot bound the amount of unconsumed data a priori.
            StreamSetBuffer * buffer = nullptr;
            if (dynamic) {
                buffer = new DynamicBuffer(b, baseType, bufferSize, overflowSize, underflowSize, !bn.NonLinear, 0U);
            } else {
                buffer = new StaticBuffer(b, baseType, bufferSize, overflowSize, underflowSize, !bn.NonLinear, 0U);
            }
            bn.Buffer = buffer;
        }
        bn.NonLocal = nonLocal;
        mInternalBuffers[streamSet - FirstStreamSet].reset(bn.Buffer);
    }

    verifyIOStructure();
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

        auto computeBufferRateBounds = [&](const RelationshipType port,
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
            return BufferRateData{port, binding, lb, ub};
        };

        // Evaluate the input/output ordering here and ensure that any reference port is stored first.

        const auto numOfInputs = in_degree(i, mStreamGraph);
        const auto numOfPorts = numOfInputs + out_degree(i, mStreamGraph);

        if (numOfPorts == 0) {
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
                add_edge(streamSet, i, computeBufferRateBounds(port, rn, streamSet), mBufferGraph);
            } else {
                const auto binding = target(e, mStreamGraph);
                const RelationshipNode & rn = mStreamGraph[binding];
                assert (rn.Type == RelationshipNode::IsBinding);
                const auto f = out_edge(binding, mStreamGraph);
                assert (mStreamGraph[f].Reason != ReasonType::Reference);
                const auto streamSet = target(f, mStreamGraph);
                assert (mStreamGraph[streamSet].Type == RelationshipNode::IsRelationship);
                assert (isa<StreamSet>(mStreamGraph[streamSet].Relationship));
                add_edge(i, streamSet, computeBufferRateBounds(port, rn, streamSet), mBufferGraph);
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief verifyIOStructure
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::verifyIOStructure() const {


#if 0

    // verify that the buffer config is valid
    for (unsigned i = FirstStreamSet; i <= LastStreamSet; ++i) {

        const BufferNode & bn = G[i];
        const auto pe = in_edge(i, G);
        const auto producerVertex = source(pe, G);
        const Kernel * const producer = getKernel(producerVertex);
        const BufferRateData & producerRate = G[pe];
        const Binding & output = producerRate.Binding;




        // Type check stream set I/O types.
        Type * const baseType = output.getType();
        for (const auto e : make_iterator_range(out_edges(i, G))) {
            const BufferRateData & consumerRate = G[e];
            const Binding & input = consumerRate.Binding;
            if (LLVM_UNLIKELY(baseType != input.getType())) {
                SmallVector<char, 256> tmp;
                raw_svector_ostream msg(tmp);
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

        for (const auto ce : make_iterator_range(out_edges(i, G))) {
            const Binding & input = G[ce].Binding;
            if (LLVM_UNLIKELY(requiresLinearAccess(input))) {
                SmallVector<char, 256> tmp;
                raw_svector_ostream out(tmp);
                const auto consumer = target(ce, G);
                out << getKernel(consumer)->getName()
                    << '.' << input.getName()
                    << " requires that "
                    << producer->getName()
                    << '.' << output.getName()
                    << " is a Linear buffer.";
                report_fatal_error(out.str());
            }
        }


    }

#endif

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief identifyLinearBuffers
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::identifyLinearBuffers() {

    for (auto streamSet = FirstStreamSet; streamSet <= LastStreamSet; ++streamSet) {
        BufferNode & N = mBufferGraph[streamSet];
        N.NonLinear = true;
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

        const auto binding = in_edge(streamSet, mBufferGraph);
        const BufferRateData & producerRate = mBufferGraph[binding];
        const Binding & output = producerRate.Binding;

        auto requiresLinearAccess = [](const Binding & binding) {
            for (const Attribute & attr : binding.getAttributes()) {
                switch(attr.getKind()) {
                    case AttrId::Linear:
                    case AttrId::Deferred:
                        return true;
                    case AttrId::LookBehind:
                        if (LLVM_UNLIKELY(attr.amount() == 0)) {
                            return true;
                        }
                    default: break;
                }
            }
            return false;
        };

        const auto producer = source(binding, mBufferGraph);
        const auto partitionId = KernelPartitionId[producer];

        bool mustBeLinear = false;
        if (LLVM_UNLIKELY(requiresLinearAccess(output))) {
            mustBeLinear = true;
        } else {
            bool samePartition = true;

            for (const auto binding : make_iterator_range(out_edges(streamSet, mBufferGraph))) {
                const BufferRateData & consumerRate = mBufferGraph[binding];
                const Binding & input = consumerRate.Binding;
                if (LLVM_UNLIKELY(requiresLinearAccess(input))) {
                    mustBeLinear = true;
                    break;
                }
                const auto consumer = target(binding, mBufferGraph);
                samePartition &= (KernelPartitionId[consumer] == partitionId);
           }
           mustBeLinear |= samePartition;
        }
        if (LLVM_UNLIKELY(mustBeLinear)) {
            BufferNode & N = mBufferGraph[streamSet];
            N.NonLinear = false;
        }
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
