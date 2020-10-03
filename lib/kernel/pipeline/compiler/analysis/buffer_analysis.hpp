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

    const auto blockWidth = b->getBitBlockWidth();

    mInternalBuffers.resize(LastStreamSet - FirstStreamSet + 1);

    std::vector<unsigned> minStride(LastKernel + 1U);

    unsigned partitionRootId = 0U;
    unsigned currentPartitionId = -1U;

    std::vector<unsigned> minFactor(PipelineOutput + 1U);
    std::vector<unsigned> maxFactor(PipelineOutput + 1U);


    for (auto kernel = FirstKernel; kernel <= LastKernel; ++kernel) {
        const auto pId = KernelPartitionId[kernel];
        if (pId != currentPartitionId) {
            currentPartitionId = pId;
            partitionRootId = kernel;
            minFactor[kernel] = floor(MinimumNumOfStrides[kernel]);
            maxFactor[kernel] = ceiling(MaximumNumOfStrides[kernel]);
        } else {
            const auto m = MaximumNumOfStrides[kernel] / MaximumNumOfStrides[partitionRootId];
            minFactor[kernel] = floor(m * minFactor[partitionRootId]);
            maxFactor[kernel] = ceiling(m * maxFactor[partitionRootId]);
        }
    }
    minFactor[PipelineOutput] = 1;
    maxFactor[PipelineOutput] = 1;

    // then construct the rest
    for (auto streamSet = FirstStreamSet; streamSet <= LastStreamSet; ++streamSet) {

        BufferNode & bn = mBufferGraph[streamSet];
        const auto producerOutput = in_edge(streamSet, mBufferGraph);
        const BufferPort & producerRate = mBufferGraph[producerOutput];

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


            // If this buffer is externally used, we cannot analyze the dataflow rate of
            // external consumers. Default to dynamic for such buffers.

            // Similarly if any internal consumer has a deferred rate, we cannot analyze
            // any consumption rates.

            nonLocal |= bn.isExternal() || producerRate.IsDeferred;

            const Binding & output = producerRate.Binding;

            auto maxDelay = producerRate.Delay;
            auto maxLookAhead = producerRate.LookAhead;
            auto maxLookBehind = producerRate.LookBehind;

            auto bMin = floor(producerRate.Minimum) * minFactor[producer];
            auto bMax = ceiling(producerRate.Maximum) * maxFactor[producer];

            assert (producerRate.Maximum >= producerRate.Minimum);

            if (producerRate.Minimum < producerRate.Maximum) {
                nonLocal = true;
            }

            for (const auto e : make_iterator_range(out_edges(streamSet, mBufferGraph))) {

                const BufferPort & consumerRate = mBufferGraph[e];

                const auto consumer = target(e, mBufferGraph);

                const auto cMin = floor(consumerRate.Minimum) * minFactor[consumer];
                const auto cMax = ceiling(consumerRate.Maximum) * maxFactor[consumer];

                assert (cMax >= cMin);

                // Could we consume less data than we produce?
                if (consumerRate.Minimum < consumerRate.Maximum) {
                    nonLocal = true;
                // Or is the data consumption rate unpredictable despite its type?
                } else if (LLVM_UNLIKELY(consumerRate.IsShared || consumerRate.IsDeferred)) {
                    nonLocal = true;
                }

                assert (consumerRate.Maximum >= consumerRate.Minimum);

                bMin = std::min(bMin, cMin);
                bMax = std::max(bMax, cMax);

//                // Get output overflow size
                auto lookAhead = consumerRate.LookAhead;
                if (consumerRate.Maximum > consumerRate.Minimum) {
                    lookAhead += ceiling(consumerRate.Maximum - consumerRate.Minimum);
                }

                maxDelay = std::max(maxDelay, consumerRate.Delay);
                maxLookAhead = std::max(maxLookAhead, lookAhead);
                maxLookBehind = std::max(maxLookBehind, consumerRate.LookBehind);
            }

            // calculate overflow (copyback) and fascimile (copyforward) space
            bn.LookAhead = maxLookAhead;
            bn.LookBehind = maxLookBehind;

            const auto overflow0 = std::max(bn.MaxAdd, bn.LookAhead);
            const auto overflow1 = std::max(overflow0, bMax);
            const auto overflowSize = round_up_to(overflow1, blockWidth) / blockWidth;

            const auto underflow0 = std::max(bn.LookBehind, maxDelay);
            const auto underflowSize = round_up_to(underflow0, blockWidth) / blockWidth;
            const auto required = (bMax * 2) - bMin;

            const auto reqSize1 = round_up_to(required, blockWidth) / blockWidth;
            const auto reqSize2 = 2 * (overflowSize + underflowSize);
            auto requiredSize = std::max(reqSize1, reqSize2);

            // if this buffer is "stateful", we cannot make it *thread* local
            if (maxLookBehind || maxDelay || bn.CopyBack || maxLookAhead) {
                nonLocal = true;
            }

            Type * const baseType = output.getType();

            // A DynamicBuffer is necessary when we cannot bound the amount of unconsumed data a priori.
            StreamSetBuffer * buffer = nullptr;
            if (nonLocal) {
                // TODO: we can make some buffers static despite crossing a partition but only if we can guarantee
                // an upper bound to the buffer size for all potential inputs. Build a dataflow analysis to
                // determine this.
                const auto bufferSize = requiredSize * mNumOfThreads;
                buffer = new DynamicBuffer(b, baseType, bufferSize, overflowSize, underflowSize, !bn.NonLinear, 0U);
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
                    const Binding & ref = getBinding(i, getReference(i, port));
                    const ProcessingRate & refRate = ref.getRate();
                    lb *= refRate.getLowerBound();
                    ub *= refRate.getUpperBound();
                }
                lb *= strideLength;
                ub *= strideLength;
            }

            BufferPort bp(port, binding, lb, ub);

            if (LLVM_UNLIKELY(rate.getKind() == RateId::Unknown)) {
                bp.IsManaged = true;
            }

            for (const Attribute & attr : binding.getAttributes()) {
                switch (attr.getKind()) {
                    case AttrId::Add:                        
                        bp.Add = std::max(bp.Add, attr.amount());
                        break;
                    case AttrId::Delayed:
                        bp.Delay = std::max(bp.Delay, attr.amount());
                        break;
                    case AttrId::LookAhead:
                        bp.LookAhead = std::max(bp.LookAhead, attr.amount());
                        break;
                    case AttrId::LookBehind:
                        bp.LookBehind = std::max(bp.LookBehind, attr.amount());
                        break;
                    case AttrId::Truncate:
                        bp.Truncate = std::max(bp.Truncate, attr.amount());
                        break;
                    case AttrId::Principal:
                        bp.IsPrincipal = true;
                        break;
                    case AttrId::Deferred:
                        bp.IsDeferred = true;
                        break;
                    case AttrId::SharedManagedBuffer:
                        bp.IsShared = true;
                        break;                        
                    case AttrId::ManagedBuffer:
                        bp.IsManaged = true;
                        break;
                    default: break;
                }
            }
            return bp;
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


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief identifyNonLocalBuffers
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::identifyNonLocalBuffers() {



}

}

#endif
