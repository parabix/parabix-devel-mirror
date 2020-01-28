#ifndef PARTITIONING_ANALYSIS_HPP
#define PARTITIONING_ANALYSIS_HPP

#include "../pipeline_compiler.hpp"
#include "../internal/partitionmetakernel.h"

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
    using PartitionMap = std::map<BV, unsigned>;

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

    unsigned nextRateId = 0;

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
            addRateId(H[buf], nextRateId++);
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
            addRateId(H[partialSum], nextRateId++);
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
            assert (H[kernel].none());

            bool partitionRoot = false;

            // Iterate through the inputs
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

            // Check whether any of the kernel attributes require this to be a partition root
            const Kernel * const kernelObj = node.Kernel;
            for (const Attribute & attr : kernelObj->getAttributes()) {
                switch (attr.getKind()) {
                    case AttrId::CanTerminateEarly:
                    case AttrId::MustExplicitlyTerminate:
                    case AttrId::MayFatallyTerminate:
                        partitionRoot = true;
                        break;
                    default:
                        break;
                }
            }

            // Assign a root of a partition a new id.
            if (LLVM_UNLIKELY(partitionRoot)) {
                addRateId(H[kernel], nextRateId++);
            }

            // Now iterate through the outputs
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
                            addRateId(H[buffer], nextRateId++);
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
                                addRateId(H[buffer], nextRateId++);
                                break;
                            default: break;
                        }
                    }
                }
            }

            // Each output of a source kernel is given a rate not shared by the kernel itself
            // to ensure its descendents are in a seperate partition. However, the descendents
            // of a source kernel with two or more outputs that have an equivalent rate ought
            // to be in the same partition (assuming they have no other inputs from differing
            // partitions.)
            if (LLVM_UNLIKELY(in_degree(kernel, H) == 0 && out_degree(kernel, H) != 0)) {
                // place the source in its own partition
                addRateId(H[kernel], nextRateId++);
                const auto sourceKernelRateId = nextRateId++;
                for (const auto output : make_iterator_range(out_edges(u, H))) {
                    const auto buffer = target(output, H);
                    addRateId(H[buffer], sourceKernelRateId);
                }
            }

        }
    }

    assert (nextKernel == kernels);
    assert (nextStreamSet == streamSets + kernels);

    // Combine all incoming rates sets

    O.clear();
    O.reserve(num_vertices(H));
    if (LLVM_UNLIKELY(!lexical_ordering(H, O))) {
        report_fatal_error("Cannot lexically order the partition graph!");
    }

    for (const auto u : O) {
        auto & nodeRateSet = H[u];
        // boost dynamic_bitset will segfault when buffers are of differing lengths.
        nodeRateSet.resize(nextRateId);
        for (const auto e : make_iterator_range(in_edges(u, H))) {
            const auto v = source(e, H);
            const auto & inputRateSet = H[v];
            assert ("input rate set cannot be empty!" && inputRateSet.any());
            nodeRateSet |= inputRateSet;
        }
    }

    std::vector<Partition> partitions;

    BEGIN_SCOPED_REGION

    PartitionMap partitionSets;
    // Now that we've tainted the kernels with any influencing rates,
    // cluster them into partitions.
    for (const auto u : O) {
        // We want to obtain all kernels but still maintain the lexical ordering of each partition root
        if (u < kernels) {
            BV & node = H[u];
            assert ("active kernel rate set cannot be empty!" && (node.none() ^ (in_degree(u, H) != 0 || out_degree(u, H) != 0)));
            if (LLVM_UNLIKELY(node.none())) continue;
            auto f = partitionSets.find(node);
            if (f == partitionSets.end()) {
                const auto i = partitions.size();
                partitions.emplace_back();
                f = partitionSets.emplace(std::move(node), i).first;
            }
            Partition & P = partitions[f->second];
            P.push_back(mappedKernel[u]);
        }
    }

    END_SCOPED_REGION

    partitionIds.resize(num_vertices(G), 0);

    const auto n = partitions.size();
    if (LLVM_UNLIKELY(n < 2)) {
        return n;
    }

    // Then provide each partition with a unique id
    unsigned partitionId = 0U;
    for (auto i = partitions.begin();;) {
        const Partition & A = *i;
        for (const auto v : A) {
            partitionIds[v] = partitionId;
        }
        ++partitionId;
        if (++i == partitions.end()) {
            break;
        }
        // and add any constraints between each adjacent partition
        const Partition & B = *i;
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

#if 0

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief extractPartitionSubgraphs
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::extractPartitionSubgraphs(Relationships & G,
                                                 std::vector<unsigned> & partitionIds,
                                                 const unsigned numOfPartitions) const {

    std::vector<unsigned> O;
    const auto s = lexical_ordering(G, O);
    assert (s);

    std::vector<<std::vector<unsigned>> group(numOfPartitions);
    for (const auto u : O) {
        const RelationshipNode & node = G[u];
        switch (node.Type) {
            case RelationshipNode::IsKernel:
                group[partitionIds[u]].push_back(u);
                break;
            default: break;
        }
    }

    for (unsigned i = 0; i < numOfPartitions; ++i) {

        const auto n = group[i].size();

        if (n > 1) {

            RelationshipGraph H(n);
            flat_map<unsigned, unsigned> M;

            auto map = [&](const unsigned u) -> unsigned {
                const auto f = M.find(u);
                if (f == M.end()) {
                    const auto v = add_vertex(H);
                    H[v] = G[u];
                    M.emplace(u, v);
                    return v;
                }
                return f->second;
            };




            for (const auto kernel : group) {


                for (const auto e : make_iterator_range(in_edges(kernel, G))) {
                    const auto bindingOrScalar = source(e, G);
                    const auto u = map(bindingOrScalar);
                    add_edge(u, kernel, G[e], H);
                    const RelationshipNode & node = G[bindingOrScalar];

                    if (node.Type == RelationshipNode::IsBinding) {

                        for (const auto f : make_iterator_range(in_edges(bindingOrScalar, G))) {

                            const auto streamSet = source(f, G);
                            const auto v = map(streamSet);
                            add_edge(v, u, G[f], H);
                            assert (G[streamSet].Type == RelationshipNode::IsRelationship);
                            const auto producer = parent(parent(streamSet, G), G);
                            assert (G[producer].Type == RelationshipNode::IsKernel);
                            const auto crossesPartition = (partitionIds[producer] != i);



                        }











                    } else if (node.Type == RelationshipNode::IsRelationship) {
                        const auto producer = parent(bindingOrScalar, G);
                        assert (G[producer].Type == RelationshipNode::IsKernel);
                        const auto crossesPartition = (partitionIds[producer] != i);
                    }
                }

                for (const auto e : make_iterator_range(out_edges(kernel, G))) {
                    const auto bindingOrScalar = target(e, G);
                    const auto u = map(bindingOrScalar);
                    add_edge(u, kernel, G[e], H);
                    const RelationshipNode & node = G[bindingOrScalar];

                    if (node.Type == RelationshipNode::IsBinding) {
                        const auto e1 = out_edge(bindingOrScalar, G);
                        const auto buffer = target(e1, G);
                        const auto v = map(buffer);
                        add_edge(v, u, G[e1], H);
                        assert (G[buffer].Type == RelationshipNode::IsRelationship);
                        const auto consumer = child(child(buffer, G), G);
                        assert (G[consumer].Type == RelationshipNode::IsKernel);
                        const auto crossesPartition = (partitionIds[consumer] != i);


                    } else if (node.Type == RelationshipNode::IsRelationship) {
                        const auto consumer = child(bindingOrScalar, G);
                        assert (G[consumer].Type == RelationshipNode::IsKernel);
                        const auto crossesPartition = (partitionIds[consumer] != i);
                    }
                }


            }





        }

    }




}

#endif

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

    PartitioningGraph G(PartitionCount);

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

    const auto firstKernel = out_degree(PipelineInput, mBufferGraph) == 0 ? FirstKernel : PipelineInput;
    const auto lastKernel = in_degree(PipelineOutput, mBufferGraph) == 0 ? LastKernel : PipelineOutput;

    assert (KernelPartitionId[firstKernel] == 0);
    assert ((KernelPartitionId[lastKernel] + 1) == PartitionCount);

    BitVector encountered(num_vertices(mBufferGraph), false);

    for (auto start = firstKernel; start <= lastKernel; ) {
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

        G[partitionId] = start;
        assert (degree(partitionId, G) == 0);

        for (auto kernel = start; kernel < end; ++kernel) {

            const auto minStrides = MinimumNumOfStrides[kernel];

            for (const auto input : make_iterator_range(in_edges(kernel, mBufferGraph))) {
                const auto streamSet = source(input, mBufferGraph);
                const BufferNode & bn = mBufferGraph[streamSet];
                if (LLVM_UNLIKELY(bn.NonLocal || isa<DynamicBuffer>(bn.Buffer))) {
                    const auto buffer = getStreamSetVertex(streamSet);
                    const BufferRateData & inputRate = mBufferGraph[input];
                    const Binding & binding = inputRate.Binding;
                    const ProcessingRate & rate = binding.getRate();
                    const auto rateId = rate.getKind();
                    unsigned reference = 0;
                    switch (rateId) {
                        case RateId::PartialSum:
                        case RateId::Relative:
                            reference = getReferenceBufferVertex(kernel, inputRate.Port);
                            break;
                        default: break;
                    }
                    const auto maxRate = inputRate.Maximum * minStrides;

                    // To preserve acyclicity, any internally used dynamic buffer is treated as an
                    // output buffer for the purpose of computing the partitioning graph.
                    unsigned s = buffer, t = partitionId;
                    if (LLVM_UNLIKELY(encountered.test(buffer))) {
                        s = partitionId; t = buffer;
                    }
                    add_edge(s, t, PartitionData(kernel, rateId, maxRate, reference, inputRate.Port), G);
                }
            }

            for (const auto output : make_iterator_range(out_edges(kernel, mBufferGraph))) {
                const auto streamSet = target(output, mBufferGraph);
                const BufferNode & bn = mBufferGraph[streamSet];
                if (LLVM_UNLIKELY(bn.NonLocal || isa<DynamicBuffer>(bn.Buffer))) {
                    const auto buffer = getStreamSetVertex(streamSet);
                    const BufferRateData & outputRate = mBufferGraph[output];
                    const Binding & binding = outputRate.Binding;
                    const ProcessingRate & rate = binding.getRate();
                    const auto rateId = rate.getKind();
                    unsigned reference = 0;
                    switch (rateId) {
                        case RateId::PartialSum:
                        case RateId::Relative:
                            reference = getReferenceBufferVertex(kernel, outputRate.Port);
                            break;
                        default: break;
                    }
                    const auto maxRate = outputRate.Maximum * minStrides;
                    encountered.set(buffer);
                    add_edge(partitionId, buffer, PartitionData(kernel, rateId, maxRate, reference, outputRate.Port), G);
                }
            }

        }

        encountered.reset();

        start = end;
    }

    const auto n = num_vertices(G);

    auto implies = [&](const PartitionData & A, const PartitionData & B) {
        if (LLVM_LIKELY(A.Type == B.Type)) {
            if (LLVM_LIKELY(A.Reference == B.Reference)) {
                return A.MaxRate <= B.MaxRate;
            }
        } else {
            switch (B.Type) {
                case RateId::Greedy:
                    return true;
                case RateId::Fixed:
                    switch (A.Type) {
                        case RateId::Bounded:
                        case RateId::Relative:
                        case RateId::PartialSum:
                            return A.MaxRate <= B.MaxRate;
                        default: break;
                    }
                default: break;
            }

        }
        return false;
    };

    auto filter = [&](std::vector<PartitionData> & data) {
        for (auto i = 0U; i < data.size(); ) {
            for (auto j = 0U; j < i; ++j) {
                if (implies(data[i], data[j])) {
                    data.erase(data.begin() + i);
                    goto next;
                }
            }

            for (auto j = i + 1; j < data.size(); ++j) {
                if (implies(data[i], data[j])) {
                    data.erase(data.begin() + i);
                    goto next;
                }
            }
            ++i;
            next: continue;
        }
        return data;
    };

    flat_map<unsigned, std::vector<PartitionData>> partitionData;

    // Filter the graph to ensure each input to/output from a partition captures only the
    // largest comparable rates for each stream set.
    for (auto partitionId = 0U; partitionId < PartitionCount; ++partitionId) {

        for (const auto input : make_iterator_range(in_edges(partitionId, G))) {
            const auto streamSet = source(input, G);
            partitionData[streamSet].emplace_back(G[input]);
        }
        clear_in_edges(partitionId, G);
        for (auto p : partitionData) {
            const auto streamSet = p.first;
            for (const PartitionData & data : filter(p.second)) {
                add_edge(streamSet, partitionId, data, G);
            }
        }
        partitionData.clear();


        for (const auto output : make_iterator_range(out_edges(partitionId, G))) {
            const auto streamSet = target(output, G);
            partitionData[streamSet].emplace_back(G[output]);
        }
        clear_out_edges(partitionId, G);
        for (auto p : partitionData) {
            const auto streamSet = p.first;
            for (const PartitionData & data : filter(p.second)) {
                add_edge(partitionId, streamSet, data, G);
            }
        }
        partitionData.clear();
    }

    // Compute something similar to a transitive reduction of G; the difference is we'll retain
    // a transitive edge if and only if it requires more data than its predecessors.

    std::vector<Vertex> ordering;
    ordering.reserve(n);

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
            partitionData[s].emplace_back(G[e]);
        }
        for (auto e : make_iterator_range(out_edges(u, G))) {
            remove_in_edge_if(target(e, G), [&](const Edge f) {
                const auto s = source(f, G);
                const PartitionData & A = G[f];
                for (const PartitionData & B : partitionData[s]) {
                    if (implies(A, B)) {
                        return true;
                    }
                }
                return false;
            }, G);
        }
        partitionData.clear();
    }

    return G;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief determinePartitionJumpIndices
 *
 * If a partition determines it has insufficient data to execute, identify which partition is the next one to test.
 * I.e., the one with input from some disjoint path. If none exists, we'll begin jump to "PartitionCount", which
 * marks the end of the processing loop.
 ** ------------------------------------------------------------------------------------------------------------- */
std::vector<unsigned> PipelineCompiler::determinePartitionJumpIndices() const {

    using BV = dynamic_bitset<>;
    using Graph = adjacency_list<vecS, vecS, bidirectionalS>;

    // Begin by cloning the partitioning graph
    const auto n = std::max<unsigned>(num_vertices(mPartitioningGraph), (PartitionCount + 1));

    Graph G(n);
    for (auto partitionId = 0U; partitionId < PartitionCount; ++partitionId) {
        for (const auto e : make_iterator_range(out_edges(partitionId, mPartitioningGraph))) {
            const auto v = target(e, mPartitioningGraph);
            add_edge(partitionId, v, G);
        }
        for (const auto e : make_iterator_range(in_edges(partitionId, mPartitioningGraph))) {
            const auto v = source(e, mPartitioningGraph);
            add_edge(v, partitionId, G);
        }
    }

    BEGIN_SCOPED_REGION

    // Now compute the transitive reduction of the partition relationships
    ReverseTopologicalOrdering ordering;
    ordering.reserve(n);
    topological_sort(G, std::back_inserter(ordering));
    transitive_closure_dag(ordering, G);
    for (auto u = PartitionCount; u < n; ++u) {
        clear_vertex(u, G);
    }
    transitive_reduction_dag(ordering, G);

    // Add a special sink node that marks the end of the processing loop.
    for (auto partitionId = 0U; partitionId < PartitionCount; ++partitionId) {
        if (LLVM_UNLIKELY(out_degree(partitionId, G) == 0)) {
            add_edge(partitionId, PartitionCount, G);
        }
    }

    END_SCOPED_REGION


    // Generate a post dominator tree of G. If we do not have enough data to execute
    // a partition along some branched path, it's possible a post dominator of the paths
    // was prevented from executing because of some other paths output. If that path was
    // successfully executed, it may have enough data to process now despite the fact we
    // had insufficient data on this path.

    BEGIN_SCOPED_REGION

    std::vector<BV> paths(PartitionCount + 1);
    for (unsigned u = 0, p = 0; u <= PartitionCount; ++u) { // forward topological ordering
        BV & P = paths[u];
        P.resize(PartitionCount + 1);
        for (const auto e : make_iterator_range(in_edges(u, G))) {
            const auto v = source(e, G);
            P |= paths[v];
        }

        for (const auto e : make_iterator_range(out_edges(u, G))) {
            const auto v = target(e, G);
            BV & O = paths[v];
            O = P;
            O.set(p++);
        }
    }

    // ensure that the common sink is a sentinal for the subsequent search process

    add_edge(PartitionCount, PartitionCount, G);

    BV M(PartitionCount + 1);

    // reverse topological ordering starting at (PartitionCount - 1)
    for (auto u = PartitionCount; u--; ) {

        auto v = u;

        if (LLVM_UNLIKELY(out_degree(u, G) > 1)) {

            assert (M.none());

            for (const auto output : make_iterator_range(out_edges(u, G))) {
                const auto v = target(output, G);
                const BV & O = paths[v];
                M |= O;
            }

            while (++v <= PartitionCount) {
                const BV & O = paths[v];
                // since each output of partition u is assigned a new rate id,
                // the only way that M ⊆ O is if v post dominates u.
                if (LLVM_UNLIKELY(M.is_subset_of(O))) {
                    break;
                }
            }

            M.reset();

        }

        assert ("an immediate post dominator is guaranteed!" && v <= PartitionCount);

        // v is the immediate post-dominator of u; however, since this graph indicates
        // that we could not execute u nor any of its branched paths, we search for the
        // first non-immediate post dominator with an in-degree > 1. We're guaranteed
        // to find one since the common sink must have an in-degree >= 2.

        // NOTE: the sole child of the common sink sentinal is itself.

        for (;;) {
            v = child(v, G);
            if (LLVM_UNLIKELY(in_degree(v, G) > 1)) {
                break;
            }            
        }

        clear_out_edges(u, G);
        add_edge(u, v, G);

    }

    END_SCOPED_REGION

    std::vector<unsigned> jumpIndices(PartitionCount);
    for (unsigned u = 0; u < PartitionCount; ++u) {
        jumpIndices[u] = child(u, G);
    }
    return jumpIndices;

}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief determinePartitionInputEvaluationOrder
 ** ------------------------------------------------------------------------------------------------------------- */
std::vector<PartitionInput> PipelineCompiler::determinePartitionInputEvaluationOrder(const unsigned partitionId) const {

    // Compute the order in which we need to test the partition inputs. Some may be dependent on
    // others (such as popcounts or relative rates).

    using Map = flat_map<unsigned, PartitioningGraph::vertex_descriptor>;

    constexpr auto ROOT = 0U;

    PartitioningGraph G(1);
    Map M;

    auto addOrFind =[&](const unsigned streamSet) {
        const auto f = M.find(streamSet);
        if (f == M.end()) {
            const auto v = add_vertex(G);
            G[v] = streamSet;
            M.emplace(streamSet, v);
            return v;
        }
        return f->second;
    };

    for (const auto e : make_iterator_range(in_edges(partitionId, mPartitioningGraph))) {
        const auto streamSet = mPartitioningGraph[source(e, mPartitioningGraph)];
        assert (streamSet >= FirstStreamSet && streamSet <= LastStreamSet);
        const PartitionData & pd = mPartitioningGraph[e];

        const auto buffer = addOrFind(streamSet);

        add_edge(ROOT, buffer, pd, G);

        switch (pd.Type) {
            case RateId::PartialSum:
            case RateId::Relative:
                add_edge(addOrFind(pd.Reference), buffer, G);
                break;
            default: break;
        }
    }

    std::vector<unsigned> O;
    const auto b = lexical_ordering(G, O);
    assert ("input data must be acyclic!" && b);

    std::vector<PartitionInput> eval;

    for (auto v : O) {
        const auto streamSet = G[v];
        for (const auto e : make_iterator_range(in_edges(v, G))) {
            if (LLVM_LIKELY(source(e, G) == ROOT)) {
                eval.emplace_back(G[e], streamSet);
            }
        }
    }

    return eval;

}

} // end of namespace kernel

#endif // PARTITIONING_ANALYSIS_HPP
