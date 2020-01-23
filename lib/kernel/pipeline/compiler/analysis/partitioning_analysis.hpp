#ifndef PARTITIONING_ANALYSIS_HPP
#define PARTITIONING_ANALYSIS_HPP

#include "../pipeline_compiler.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief partitionIntoFixedRateRegionWithOrderingConstraints
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned PipelineCompiler::partitionIntoFixedRateRegionsWithOrderingConstraints(Relationships & G, std::vector<unsigned> & partitionIds) const {

    using BV = dynamic_bitset<>;

    using Graph = adjacency_list<vecS, vecS, bidirectionalS, BV>;
    using Vertex = Graph::vertex_descriptor;

    using BufferAttributeMap = flat_map<std::pair<Vertex, unsigned>, Vertex>;
    using PartialSumMap = flat_map<Vertex, Vertex>;

    using Partition = std::vector<unsigned>;
    using PartitionMap = std::map<BV, Partition>;

    // Convert G into a simpler representation of the graph that we can annotate

    unsigned kernels = 0;
    unsigned streamSets = 0;
    for (const auto u : make_iterator_range(vertices(G))) {
        const RelationshipNode & node = G[u];
        switch (node.Type) {
            case RelationshipNode::IsKernel:
                ++kernels;
                break;
            case RelationshipNode::IsRelationship:
                if (LLVM_LIKELY(isa<StreamSet>(G[u].Relationship))) {
                    ++streamSets;
                }
                break;
            default: break;
        }
    }

    Graph H(kernels + streamSets);

    flat_map<Relationships::vertex_descriptor, Vertex> M;

    unsigned nextKernel = 0;
    unsigned nextStreamSet = kernels;

    auto mapStreamSet = [&](const Relationships::vertex_descriptor u) -> Vertex {
        const auto f = M.find(u);
        if (f == M.end()) {
            const auto id = nextStreamSet++;
            assert (id < (streamSets + kernels));
            M.emplace(u, id);
            return id;
        }
        return f->second;
    };

    unsigned currentRateId = 0;

    auto addRateId = [](BV & bv, const unsigned rateId) {
       if (LLVM_UNLIKELY(rateId >= bv.capacity())) {
           bv.resize(round_up_to(rateId + 1, BV::bits_per_block));
       }
       bv.set(rateId);
    };

    BufferAttributeMap L;
    BufferAttributeMap D;

    auto makeAttributeVertex = [&](const Vertex streamSet, const unsigned amount, BufferAttributeMap & M) -> Vertex {
        const auto key = std::make_pair(streamSet, amount);
        const auto f = M.find(key);
        if (LLVM_LIKELY(f == M.end())) {
            const auto buf = add_vertex(H);
            addRateId(H[buf], currentRateId++);
            add_edge(streamSet, buf, H);
            M.emplace(key, buf);
            return buf;
        } else {
            return f->second;
        }
    };

    PartialSumMap P;

    auto checkForPartialSumEntry = [&](const Vertex streamSet) -> Vertex {
        const auto f = P.find(streamSet);
        if (LLVM_LIKELY(f == P.end())) {
            const auto partialSum = add_vertex(H);
            addRateId(H[partialSum], currentRateId++);
            P.emplace(streamSet, partialSum);
            return partialSum;
        } else {
            return f->second;
        }
    };

    std::vector<Vertex> O;
    O.reserve(num_vertices(G));
    if (LLVM_UNLIKELY(!lexical_ordering(G, O))) {
        report_fatal_error("Cannot lexically order the initial pipeline graph!");
    }

    std::vector<Relationships::vertex_descriptor> mappedKernel(kernels);

    // Begin by constructing a graph that represents the I/O relationships
    // and any partition boundaries.
    for (const auto u : O) {

        const RelationshipNode & node = G[u];

        if (node.Type == RelationshipNode::IsKernel) {

            const auto kernel = nextKernel++;
            assert (kernel < kernels);
            mappedKernel[kernel] = u;
            assert (H[kernel].empty());

            bool partitionRoot = false;

            // add in any inputs
            for (const auto e : make_iterator_range(in_edges(u, G))) {
                const auto binding = source(e, G);
                const RelationshipNode & rn = G[binding];
                if (rn.Type == RelationshipNode::IsBinding) {

                    const auto f = first_in_edge(binding, G);
                    assert (G[f].Reason != ReasonType::Reference);
                    const auto streamSet = source(f, G);
                    assert (G[streamSet].Type == RelationshipNode::IsRelationship);
                    assert (isa<StreamSet>(G[streamSet].Relationship));
                    auto buffer = mapStreamSet(streamSet);

                    const Binding & b = rn.Binding;
                    const ProcessingRate & rate = b.getRate();

                    switch (rate.getKind()) {
                        case RateId::PartialSum:
                            BEGIN_SCOPED_REGION
                            const auto partialSum = checkForPartialSumEntry(buffer);
                            add_edge(partialSum, buffer, H);
                            END_SCOPED_REGION
                            break;
                        case RateId::Greedy:
                            // A kernel with a greedy input cannot safely be included in its
                            // producers' partitions unless its producers are guaranteed to
                            // generate at least as much data as this kernel consumes.
                            BEGIN_SCOPED_REGION
                            const auto produced = parent(streamSet, G);
                            assert (G[produced].Type == RelationshipNode::IsBinding);
                            const Binding & prodBinding = G[produced].Binding;
                            const ProcessingRate & prodRate = prodBinding.getRate();
                            if (prodRate.getLowerBound() >= rate.getLowerBound()) {
                                break;
                            }
                            END_SCOPED_REGION
                        case RateId::Bounded:
                            // A bounded rate input always signifies a new partition.
                            partitionRoot = true;
                        default: break;
                    }

                    // If we have a lookahead/delay attribute on any stream, create
                    // a new buffer vertex (with a new rate id) to represent it each
                    // unique pairing.
                    for (const Attribute & attr : b.getAttributes()) {
                        switch (attr.getKind()) {
                            case AttrId::Delayed:
                                buffer = makeAttributeVertex(buffer, attr.amount(), D);
                                break;
                            case AttrId::LookAhead:
                                buffer = makeAttributeVertex(buffer, attr.amount(), L);
                                break;
                            default: break;
                        }
                    }

                    add_edge(buffer, kernel, H);
                }
            }

            assert (!partitionRoot || in_degree(kernel, H) != 0);

            // and any outputs
            for (const auto e : make_iterator_range(out_edges(u, G))) {

                const auto binding = target(e, G);
                const RelationshipNode & rn = G[binding];

                if (rn.Type == RelationshipNode::IsBinding) {

                    const auto f = out_edge(binding, G);
                    assert (G[f].Reason != ReasonType::Reference);
                    const auto streamSet = target(f, G);
                    assert (G[streamSet].Type == RelationshipNode::IsRelationship);
                    assert (isa<StreamSet>(G[streamSet].Relationship));
                    const auto buffer = mapStreamSet(streamSet);
                    const Binding & b = rn.Binding;
                    const ProcessingRate & rate = b.getRate();
                    add_edge(kernel, buffer, H);
                    switch (rate.getKind()) {
                        case RateId::PartialSum:
                            BEGIN_SCOPED_REGION
                            const auto partialSum = checkForPartialSumEntry(buffer);
                            add_edge(partialSum, buffer, H);
                            END_SCOPED_REGION
                            break;
                        case RateId::Bounded:
                            addRateId(H[buffer], currentRateId++);
                            break;
                        case RateId::Relative:
                            // Link a relative rate output stream to its reference stream
                            for (const auto f : make_iterator_range(in_edges(binding, G))) {
                                const RelationshipType & type = G[f];
                                if (type.Reason == ReasonType::Reference) {
                                    const auto ref = source(f, G);
                                    const auto refBuffer = mapStreamSet(ref);
                                    add_edge(refBuffer, buffer, H);
                                    goto found_ref;
                                }
                            }
                            llvm_unreachable("could not locate reference buffer?");
                            found_ref: break;
                        default: break;
                    }
                    // Check the attributes to see whether any impose a partition change
                    for (const Attribute & attr : b.getAttributes()) {
                        switch (attr.getKind()) {
                            case AttrId::Delayed:
                            case AttrId::Deferred:
                                // A deferred output rate is closer to an bounded rate than a
                                // countable rate but a deferred input rate simply means the
                                // buffer must be dynamic.
                                addRateId(H[buffer], currentRateId++);
                                break;
                            default: break;
                        }
                    }
                }
            }

            // If this kernel is the root of a new partition or a source kernel,
            // give it a new rate id.

            auto isSourceKernel = [&]() {
                if (LLVM_UNLIKELY(in_degree(kernel, H) == 0)) {
                    return out_degree(kernel, H) > 0;
                }
                return false;
            };

            if (partitionRoot || LLVM_UNLIKELY(isSourceKernel())) {
                BV & node = H[kernel];
                assert (node.none());
                addRateId(node, currentRateId++);
            }
        }
    }

    assert (nextKernel == kernels);
    assert (nextStreamSet == streamSets + kernels);

    // Combine all incoming rates sets

    BEGIN_SCOPED_REGION

    O.clear();
    O.reserve(num_vertices(H));
    if (LLVM_UNLIKELY(!lexical_ordering(H, O))) {
        report_fatal_error("Cannot lexically order the partition graph!");
    }

    for (const auto u : O) {
        auto & nodeRateSet = H[u];
        // boost dynamic_bitset will segfault when buffers are of differing lengths.
        nodeRateSet.resize(currentRateId);
        for (const auto e : make_iterator_range(in_edges(u, H))) {
            const auto v = source(e, H);
            const auto & inputRateSet = H[v];
            assert ("input rate set cannot be empty!" && inputRateSet.any());
            nodeRateSet |= inputRateSet;
        }
    }

    END_SCOPED_REGION

    // Now that we've tainted the kernels with any influencing rates,
    // cluster them into partitions.
    PartitionMap partitionSets;
    for (unsigned u = 0; u != kernels; ++u) {
        BV & node = H[u];
        assert ("kernel rate set cannot be empty!" && (node.none() ^ (in_degree(u, H) != 0 || out_degree(u, H) != 0)));
        if (LLVM_UNLIKELY(node.none())) {
            continue;
        }
        auto & partition = partitionSets[std::move(node)];
        partition.push_back(mappedKernel[u]);
    }

    partitionIds.resize(num_vertices(G), 0);

    const auto n = partitionSets.size();
    if (LLVM_UNLIKELY(n < 2)) {
        return n;
    }

    // Then provide each partition with a unique id
    unsigned partitionId = 0U;
    for (auto i = partitionSets.begin();;) {
        const Partition & A = i->second;
        for (const auto v : A) {
            partitionIds[v] = partitionId;
        }
        ++partitionId;
        if (++i == partitionSets.end()) {
            break;
        }
        // and add any constraints between each adjacent partition
        const Partition & B = i->second;
        for (const auto u : A) {
            for (const auto v : B) {
                assert (u != v);
                add_edge(u, v, RelationshipType{PortType::Input, 0, ReasonType::OrderingConstraint}, G);
            }
        }
    }

    assert (partitionId == n);
    return n;
}

#if 1

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generatePartitioningGraph
 *
 * The partitioning graph summarizes the buffer graph to indicate what dynamic buffers must be tested /
 * potentially expanded to satisfy the dataflow requirements of each partition.
 ** ------------------------------------------------------------------------------------------------------------- */
PartitioningGraph PipelineCompiler::generatePartitioningGraph() const {

    using BufferVertex = BufferGraph::vertex_descriptor;

    using Vertex = PartitioningGraph::vertex_descriptor;
    using Edge = graph_traits<PartitioningGraph>::edge_descriptor;

    using StreamSetMap = flat_map<BufferVertex, Vertex>;

    // To preserve acyclicity, each partition is represented by an input/output vertex pair.
    // This ensures that any internal dynamic buffer do not create a self-loop which would
    // complicate the pseudo-transitive closure step later.

    PartitioningGraph G(PartitionCount * 2);

    StreamSetMap S;

    auto getStreamSetVertex = [&](const BufferVertex streamSet) {
        const auto f = S.find(streamSet);
        if (LLVM_UNLIKELY(f == S.end())) {
            const auto v = add_vertex(G);
            G[v] = streamSet;
            S.emplace(streamSet, v);
            return v;
        }
        return f->second;
    };

    assert (KernelPartitionId[FirstKernel] == 0);
    assert ((KernelPartitionId[LastKernel] + 1) == PartitionCount);

    for (auto start = FirstKernel; start <= LastKernel; ) {
        // Determine which kernels are in this partition
        const auto partitionId = KernelPartitionId[start];
        assert (partitionId < PartitionCount);
        auto end = start + 1U;
        for (; end <= LastKernel; ++end) {
            if (KernelPartitionId[end] != partitionId) {
                break;
            }
        }

        #ifndef NDEBUG
        const Rational check{MaximumNumOfStrides[start], MinimumNumOfStrides[start]};
        for (auto kernel = start + 1; kernel < end; ++kernel) {
            const Rational check2{MaximumNumOfStrides[kernel], MinimumNumOfStrides[kernel]};
            assert ("non-synchronous dataflow in same partition?" && (check == check2));
        }
        #endif

        const auto partitionInput = partitionId * 2;
        const auto partitionOutput = partitionInput | 1;

        for (auto kernel = start; kernel < end; ++kernel) {

            G[partitionInput] = start;
            G[partitionOutput] = end - 1;

            const auto minStrides = MinimumNumOfStrides[kernel];

            for (const auto input : make_iterator_range(in_edges(kernel, mBufferGraph))) {
                const auto streamSet = source(input, mBufferGraph);
                const BufferNode & bn = mBufferGraph[streamSet];
                if (LLVM_UNLIKELY(isa<DynamicBuffer>(bn.Buffer))) {
                    const auto buffer = getStreamSetVertex(streamSet);
                    const BufferRateData & inputRate = mBufferGraph[input];
                    const auto maxRate = inputRate.Maximum * minStrides;
                    add_edge(buffer, partitionInput, maxRate, G);
                }
            }

            for (const auto output : make_iterator_range(out_edges(kernel, mBufferGraph))) {
                const auto streamSet = target(output, mBufferGraph);
                const BufferNode & bn = mBufferGraph[streamSet];
                if (LLVM_UNLIKELY(isa<DynamicBuffer>(bn.Buffer))) {
                    const auto buffer = getStreamSetVertex(streamSet);
                    const BufferRateData & outputRate = mBufferGraph[output];
                    const auto maxRate = outputRate.Maximum * minStrides;
                    add_edge(partitionOutput, buffer, maxRate, G);
                }
            }
        }
        start = end;
    }

    const auto n = num_vertices(G);
    BitVector encountered(n, false);
    std::vector<Rational> currentMaxRate(n);

    // Filter the graph to ensure each input to a partition only captures the largest rate
    // for each stream set
    for (auto partitionId = 0; partitionId < PartitionCount; ++partitionId) {

        const auto partitionInput = partitionId * 2;

        bool regenerate = false;
        for (const auto input : make_iterator_range(in_edges(partitionInput, G))) {
            const auto streamSet = source(input, G);
            const Rational maxRate = G[input];
            if (encountered.test(streamSet)) {
                currentMaxRate[streamSet] = std::max(currentMaxRate[streamSet], maxRate);
                regenerate = true;
            } else {
                encountered.set(streamSet);
                currentMaxRate[streamSet] = maxRate;
            }
        }

        if (LLVM_UNLIKELY(regenerate)) {
            clear_in_edges(partitionInput, G);
            for (const auto streamSet : encountered.set_bits()) {
                add_edge(streamSet, partitionInput, currentMaxRate[streamSet], G);
            }
        }

        encountered.reset();
    }

    // Compute something similar to a transitive reduction of G; the difference is we'll keep
    // a transitive edge if and only if it requires more data than its predecessors.

    std::vector<Vertex> ordering;
    ordering.reserve(num_vertices(G));

    topological_sort(G, std::back_inserter(ordering)); // reverse ordering

    // Simple transitive closure for DAGs
    for (unsigned u : ordering) {
        for (const auto e : make_iterator_range(in_edges(u, G))) {
            const auto s = source(e, G);
            for (const auto f : make_iterator_range(out_edges(u, G))) {
                const auto t = target(f, G);
                if (edge(s, t, G).second) continue;
                add_edge(s, t, G[e], G);
            }
        }
    }

    for (unsigned u : ordering) {
        for (auto e : make_iterator_range(in_edges(u, G))) {
            const auto s = source(e, G);
            assert (!encountered.test(s));
            currentMaxRate[s] = G[e];
            encountered.set(s);
        }
        for (auto e : make_iterator_range(out_edges(u, G))) {
            remove_in_edge_if(target(e, G), [&](const Edge f) {
                const auto s = source(f, G);
                if (encountered.test(s)) {
                    return G[f] <= currentMaxRate[s];
                }
                return false;
            }, G);
        }
        encountered.reset();
    }


    for (auto partitionId = 0; partitionId < PartitionCount; ++partitionId) {
        const auto partitionInput = partitionId * 2;
        const auto partitionOutput = partitionInput | 1;
        add_edge(partitionInput, partitionOutput, 0, G);
    }

    return G;
}

#endif

} // end of namespace kernel

#endif // PARTITIONING_ANALYSIS_HPP
