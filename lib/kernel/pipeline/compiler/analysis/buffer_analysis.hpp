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

    // fill in any known managed buffers
    for (auto kernel = FirstKernel; kernel <= LastKernel; ++kernel) {
        const Kernel * const kernelObj = getKernel(kernel);
        const auto internallySynchronized = kernelObj->hasAttribute(AttrId::InternallySynchronized);
        for (const auto e : make_iterator_range(out_edges(kernel, mBufferGraph))) {
            const BufferPort & rate = mBufferGraph[e];
            const Binding & output = rate.Binding;

            if (LLVM_UNLIKELY(internallySynchronized || rate.IsManaged)) {
                const auto streamSet = target(e, mBufferGraph);
                BufferNode & bn = mBufferGraph[streamSet];
                // Every managed buffer is considered linear to the pipeline
                bn.Buffer = new ExternalBuffer(b, output.getType(), true, 0);
                bn.Type |= BufferType::Unowned;
                if (rate.IsShared) {
                    bn.Type |= BufferType::Shared;
                }
            }
        }
    }

    // fill in any unmanaged pipeline input buffers
    for (const auto e : make_iterator_range(out_edges(PipelineInput, mBufferGraph))) {
        const auto streamSet = target(e, mBufferGraph);
        BufferNode & bn = mBufferGraph[streamSet];
        bn.Type |= BufferType::External;
        if (LLVM_LIKELY(bn.Buffer == nullptr)) {
            const BufferPort & rate = mBufferGraph[e];
            const Binding & input = rate.Binding;
            bn.Buffer = new ExternalBuffer(b, input.getType(), true, 0);
            bn.Type |= BufferType::Unowned;
        }
    }

    // and pipeline output buffers ...
    for (const auto e : make_iterator_range(in_edges(PipelineOutput, mBufferGraph))) {
        const auto streamSet = source(e, mBufferGraph);
        BufferNode & bn = mBufferGraph[streamSet];
        bn.Type |= BufferType::External;
        if (LLVM_LIKELY(bn.Buffer == nullptr)) {
            const BufferPort & rate = mBufferGraph[e];
            const Binding & output = rate.Binding;
            if (LLVM_UNLIKELY(rate.IsShared || rate.IsManaged)) {
                if (rate.IsShared) {
                    bn.Type |= BufferType::Shared;
                }
            } else {
                bn.Buffer = new ExternalBuffer(b, output.getType(), true, 0);
                bn.Type |= BufferType::Unowned;
            }
        }
    }

    mInternalBuffers.resize(LastStreamSet - FirstStreamSet + 1);

    // then construct the rest
    for (auto streamSet = FirstStreamSet; streamSet <= LastStreamSet; ++streamSet) {

        BufferNode & bn = mBufferGraph[streamSet];
        const auto producerOutput = in_edge(streamSet, mBufferGraph);

        if (LLVM_LIKELY(bn.Buffer == nullptr)) { // is internal buffer

            const BufferPort & producerRate = mBufferGraph[producerOutput];
            const Binding & output = producerRate.Binding;

            // A DynamicBuffer is necessary when we cannot bound the amount of unconsumed data a priori.
            StreamSetBuffer * buffer = nullptr;
            // if (bn.NonLocal && (bn.Locality != BufferLocality::ThreadLocal)) {
            // if (bn.Locality != BufferLocality::ThreadLocal) {
            if (bn.Locality == BufferLocality::GloballyShared) {
                // TODO: we can make some buffers static despite crossing a partition but only if we can guarantee
                // an upper bound to the buffer size for all potential inputs. Build a dataflow analysis to
                // determine this.
                const auto bufferSize = bn.RequiredCapacity * mNumOfThreads;
                assert (bufferSize > 0);
                buffer = new DynamicBuffer(b, output.getType(), bufferSize, bn.OverflowCapacity, bn.UnderflowCapacity, !bn.NonLinear, 0U);
            } else {
                auto bufferSize = bn.RequiredCapacity;
                if (bn.Locality == BufferLocality::PartitionLocal) {
                    bufferSize *= mNumOfThreads;
                }
                buffer = new StaticBuffer(b, output.getType(), bufferSize, bn.OverflowCapacity, bn.UnderflowCapacity, !bn.NonLinear, 0U);
            }
            bn.Buffer = buffer;

        }

        assert ("missing buffer?" && bn.Buffer);
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

    for (auto kernel = PipelineInput; kernel <= PipelineOutput; ++kernel) {

        const RelationshipNode & node = mStreamGraph[kernel];
        const Kernel * const kernelObj = node.Kernel; assert (kernelObj);

        auto makeBufferPort = [&](const RelationshipType port,
                                  const RelationshipNode & bindingNode,
                                  const unsigned streamSet) -> BufferPort {
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
                const BufferPort & producerBr = mBufferGraph[e];
                ub = std::max(lb, producerBr.Maximum);
            } else {
                const auto strideLength = kernelObj->getStride();
                if (LLVM_UNLIKELY(rate.isRelative())) {
                    const Binding & ref = getBinding(kernel, getReference(kernel, port));
                    const ProcessingRate & refRate = ref.getRate();
                    lb *= refRate.getLowerBound();
                    ub *= refRate.getUpperBound();
                }
                lb *= strideLength;
                ub *= strideLength;
            }

            BufferPort bp(port, binding, lb, ub);

            bool cannotBePlacedIntoThreadLocalMemory = false;
            bool mustBeLinear = false;

            if (LLVM_UNLIKELY(rate.getKind() == RateId::Unknown)) {
                bp.IsManaged = true;
                cannotBePlacedIntoThreadLocalMemory = true;
            }

            for (const Attribute & attr : binding.getAttributes()) {
                switch (attr.getKind()) {
                    case AttrId::Add:                        
                        bp.Add = std::max(bp.Add, attr.amount());
                        break;
                    case AttrId::Delayed:
                        cannotBePlacedIntoThreadLocalMemory = true;
                        bp.Delay = std::max(bp.Delay, attr.amount());
                        break;
                    case AttrId::LookAhead:
                        cannotBePlacedIntoThreadLocalMemory = true;
                        bp.LookAhead = std::max(bp.LookAhead, attr.amount());
                        break;
                    case AttrId::LookBehind:
                        cannotBePlacedIntoThreadLocalMemory = true;
                        bp.LookBehind = std::max(bp.LookBehind, attr.amount());
                        break;
                    case AttrId::Truncate:
                        bp.Truncate = std::max(bp.Truncate, attr.amount());
                        break;
                    case AttrId::Principal:
                        bp.IsPrincipal = true;
                        break;
                    case AttrId::Linear:
                        mustBeLinear = true;
                        break;
                    case AttrId::Deferred:
                        cannotBePlacedIntoThreadLocalMemory = true;
                        mustBeLinear = true;
                        bp.IsDeferred = true;
                        break;
                    case AttrId::SharedManagedBuffer:
                        cannotBePlacedIntoThreadLocalMemory = true;
                        bp.IsShared = true;
                        break;                        
                    case AttrId::ManagedBuffer:
                        bp.IsManaged = true;
                        break;
                    default: break;
                }
            }

            BufferNode & bn = mBufferGraph[streamSet];
            if (cannotBePlacedIntoThreadLocalMemory) {
                bn.Locality = BufferLocality::PartitionLocal;
            }
            if (mustBeLinear) {
                bn.NonLinear = false;
            }

            return bp;
        };

        // Evaluate the input/output ordering here and ensure that any reference port is stored first.
        const auto numOfInputs = in_degree(kernel, mStreamGraph);
        const auto numOfOutputs = out_degree(kernel, mStreamGraph);

        const auto numOfPorts = numOfInputs + numOfOutputs;

        if (LLVM_UNLIKELY(numOfPorts == 0)) {
            continue;
        }

        Graph E(numOfPorts);

        #ifndef NDEBUG
        RelationshipType prior_in{};
        #endif
        for (auto e : make_iterator_range(in_edges(kernel, mStreamGraph))) {
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
        for (auto e : make_iterator_range(out_edges(kernel, mStreamGraph))) {
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
                add_edge(streamSet, kernel, makeBufferPort(port, rn, streamSet), mBufferGraph);
            } else {
                const auto binding = target(e, mStreamGraph);
                const RelationshipNode & rn = mStreamGraph[binding];
                assert (rn.Type == RelationshipNode::IsBinding);
                const auto f = out_edge(binding, mStreamGraph);
                assert (mStreamGraph[f].Reason != ReasonType::Reference);
                const auto streamSet = target(f, mStreamGraph);
                assert (mStreamGraph[streamSet].Type == RelationshipNode::IsRelationship);
                assert (isa<StreamSet>(mStreamGraph[streamSet].Relationship));
                add_edge(kernel, streamSet, makeBufferPort(port, rn, streamSet), mBufferGraph);
            }
        }
    }


}



/** ------------------------------------------------------------------------------------------------------------- *
 * @brief markInterPartitionStreamSetsAsGloballyShared
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::markInterPartitionStreamSetsAsGloballyShared() {

    for (const auto input : make_iterator_range(out_edges(PipelineInput, mBufferGraph))) {
        const auto streamSet = target(input, mBufferGraph);
        BufferNode & bn = mBufferGraph[streamSet];
        bn.Locality = BufferLocality::GloballyShared;
    }

    for (auto streamSet = FirstStreamSet; streamSet <= LastStreamSet; ++streamSet) {
        const auto producer = parent(streamSet, mBufferGraph);
        const auto partitionId = KernelPartitionId[producer];
        assert (partitionId < PartitionCount);

        for (const auto input : make_iterator_range(out_edges(streamSet, mBufferGraph))) {
            const auto consumer = target(input, mBufferGraph);
            const auto consumerPartitionId = KernelPartitionId[consumer];
            assert (consumerPartitionId >= partitionId);
            assert (consumerPartitionId < PartitionCount);

            if (partitionId != consumerPartitionId) {
                BufferNode & bn = mBufferGraph[streamSet];
                bn.Locality = BufferLocality::GloballyShared;
                break;
            }
        }
    }

    for (const auto output : make_iterator_range(in_edges(PipelineOutput, mBufferGraph))) {
        const auto streamSet = source(output, mBufferGraph);
        BufferNode & bn = mBufferGraph[streamSet];
        bn.Locality = BufferLocality::GloballyShared;
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief identifyLinearBuffers
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::identifyOutputNodeIds() {

    if (mLengthAssertions.empty()) {

        for (auto streamSet = FirstStreamSet; streamSet <= LastStreamSet; ++streamSet) {
            BufferNode & bn = mBufferGraph[streamSet];
            bn.OutputItemCountId = streamSet;
        }

    } else {

        const auto n = LastStreamSet - FirstStreamSet + 1;

        flat_map<const StreamSet *, unsigned> StreamSetToNodeIdMap;
        StreamSetToNodeIdMap.reserve(n);

        for (auto streamSet = FirstStreamSet; streamSet <= LastStreamSet; ++streamSet) {
            assert (mStreamGraph[streamSet].Type == RelationshipNode::IsRelationship);
            const StreamSet * const ss = cast<StreamSet>(mStreamGraph[streamSet].Relationship);
            StreamSetToNodeIdMap.emplace(ss, streamSet - FirstStreamSet);
        }

        std::vector<unsigned> component(n);
        std::iota(component.begin(), component.end(), 0);

        std::function<unsigned(unsigned)> find = [&](unsigned x) {
            assert (x < n);
            if (component[x] != x) {
                component[x] = find(component[x]);
            }
            return component[x];
        };

        auto union_find = [&](unsigned x, unsigned y) {

            x = find(x);
            y = find(y);

            if (x < y) {
                component[y] = x;
            } else {
                component[x] = y;
            }

        };

        for (const auto & pair : mLengthAssertions) {
            unsigned id[2];
            for (unsigned i = 0; i < 2; ++i) {
                const auto f = StreamSetToNodeIdMap.find(pair[i]);
                if (f == StreamSetToNodeIdMap.end()) {
                    report_fatal_error("Length equality assertions contains an unknown streamset");
                }
                id[i] = f->second;
            }
            auto a = id[0], b = id[1];
            if (b > a) {
                std::swap(a, b);
            }
            union_find(a, b);
        }





        for (auto streamSet = FirstStreamSet; streamSet <= LastStreamSet; ++streamSet) {
            BufferNode & bn = mBufferGraph[streamSet];
            const auto id = FirstStreamSet + find(component[streamSet - FirstStreamSet]);
            assert (id >= FirstStreamSet && id <= streamSet);
            bn.OutputItemCountId = id;
        }

    }


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
        const BufferPort & br = mBufferGraph[output];
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
                const BufferPort & rateData = mBufferGraph[e];
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
        const BufferPort & producerRate = mBufferGraph[binding];
        const Binding & output = producerRate.Binding;


        const auto producer = source(binding, mBufferGraph);
        const auto partitionId = KernelPartitionId[producer];

        bool nonLinear = true;
        if (LLVM_UNLIKELY(isNonLinear(output))) {
            nonLinear = false;
        } else {
            bool samePartition = true;

            for (const auto binding : make_iterator_range(out_edges(streamSet, mBufferGraph))) {
                const BufferPort & consumerRate = mBufferGraph[binding];
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
