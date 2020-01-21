#ifndef PARTITIONING_ANALYSIS_HPP
#define PARTITIONING_ANALYSIS_HPP

#include "../pipeline_compiler.hpp"

namespace kernel {

#if 1

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief partitionIntoFixedRateRegionWithOrderingConstraints
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned PipelineCompiler::partitionIntoFixedRateRegionsWithOrderingConstraints(Relationships & G, std::vector<unsigned> & partitionIds) const {

    using BV = dynamic_bitset<>;

    struct PartitionNode {
        unsigned    Vertex;
        BV          RateSet;
    };

    using PartitioningGraph = adjacency_list<vecS, vecS, bidirectionalS, PartitionNode>;
    using Vertex = PartitioningGraph::vertex_descriptor;

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

    PartitioningGraph H(kernels + streamSets);

    flat_map<Relationships::vertex_descriptor, Vertex> M;

    unsigned nextKernel = 0;
    unsigned nextStreamSet = kernels;

    auto mapStreamSet = [&](const Relationships::vertex_descriptor u) -> Vertex {
        const auto f = M.find(u);
        if (f == M.end()) {
            const auto id = nextStreamSet++;
            PartitionNode & K = H[id];
            K.Vertex = u;
            M.emplace(u, id);
            return id;
        }
        return f->second;
    };

    unsigned currentRateId = 0;

    auto addRateId = [](PartitionNode & pn, const unsigned rateId) {
       auto & bv = pn.RateSet;
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
            PartitionNode & B = H[buf];
            addRateId(B, currentRateId++);
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
            PartitionNode & B = H[partialSum];
            addRateId(B, currentRateId++);
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

    // Begin by constructing a graph that represents the I/O relationships
    // and any partition boundaries.
    for (const auto u : O) {

        const RelationshipNode & node = G[u];

        if (node.Type == RelationshipNode::IsKernel) {

            const auto kernel = nextKernel++;
            PartitionNode & K = H[kernel];
            K.Vertex = u;

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
                            // A greedy input with a lower bound > 0 cannot safely be included
                            // in its producers's partition unless its producer is guaranteed
                            // to generate more data than it consumes.
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

            // If this kernel is the root of a new partition or a source kernel,
            // give it a new rate id.
            if (partitionRoot || LLVM_UNLIKELY(in_degree(kernel, H) == 0)) {
                addRateId(K, currentRateId++);
            }

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
                            // Link a relative rate output stream to its output reference stream
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
        PartitionNode & U = H[u];
        BV & nodeRateSet = U.RateSet;
        // boost dynamic_bitset will segfault when buffers are of differing lengths.
        nodeRateSet.resize(currentRateId);
        for (const auto e : make_iterator_range(in_edges(u, H))) {
            const auto v = source(e, H);
            const PartitionNode & V = H[v];
            const BV & inputRateSet = V.RateSet;
            nodeRateSet |= inputRateSet;
        }
    }

    END_SCOPED_REGION

    // Now that we've tainted the kernels with any influencing rate ids, sort them into partitions.
    PartitionMap partitionSets;
    for (unsigned u = 0; u != kernels; ++u) {
        const PartitionNode & node = H[u];
        auto & partition = partitionSets[std::move(node.RateSet)];
        partition.push_back(node.Vertex);
    }

    partitionIds.resize(num_vertices(G), 0);

    const auto n = partitionSets.size();
    if (LLVM_UNLIKELY(n < 2)) {
        return n;
    }

    unsigned partitionId = 1U;
    for (auto i = partitionSets.begin();;) {
        const Partition & A = i->second;
        for (const auto v : A) {
            partitionIds[v] = partitionId;
        }
        ++partitionId;
        if (++i == partitionSets.end()) {
            break;
        }
        const Partition & B = i->second;
        for (const auto u : A) {
            for (const auto v : B) {
                assert (u != v);
                add_edge(u, v, RelationshipType{PortType::Input, 0, ReasonType::OrderingConstraint}, G);
            }
        }
    }

    assert (partitionId == (n + 1U));

    printRelationshipGraph(G, errs(), "RESULT");

    return n;
}

#else

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief partitionIntoFixedRateRegionWithOrderingConstraints
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned PipelineCompiler::partitionIntoFixedRateRegionsWithOrderingConstraints(Relationships & G, std::vector<unsigned> & partitionIds) const {

    // LLVM BitVector does not have a built-in < comparator
    using BV = dynamic_bitset<>;
    using Partition = std::vector<unsigned>;
    using PartitionMap = std::map<BV, Partition>;
    using PartialSumKey = std::pair<BV, unsigned>;
    using PartialSumMap = flat_map<PartialSumKey, unsigned>;

    std::vector<unsigned> O;
    if (LLVM_UNLIKELY(!lexical_ordering(G, O))) {
        report_fatal_error("Pipeline contains a cycle");
    }

    std::vector<BV> rateSet(num_vertices(G));

    PartialSumMap P;

    unsigned currentRateId = 0;

    for (const auto u : O) {

        const RelationshipNode & node = G[u];

        auto mergeRateIds = [](BV & dst, BV & src) {
            // boost dynamic_bitset will segfault if both buffers are not of equivalent length.
            if (src.size() < dst.size()) {
                src.resize(dst.size());
            } else if (dst.size() < src.size()) {
                dst.resize(src.size());
            }
            dst |= src;
        };

        BV & nodeRateSet = rateSet[u];

        for (const auto e : make_iterator_range(in_edges(u, G))) {
            const auto input = source(e, G);
            // Combine all incoming rates sets
            BV & inputRateSet = rateSet[input];
            // assert (inputRateSet.any());
            mergeRateIds(nodeRateSet, inputRateSet);
        }

        if (node.Type == RelationshipNode::IsKernel) {

             auto addRateId = [](BV & bv, const unsigned rateId) {
                if (LLVM_UNLIKELY(rateId >= bv.capacity())) {
                    bv.resize(round_up_to(rateId + 1, BV::bits_per_block));
                }
                bv.set(rateId);
            };

            auto checkForPartialSum = [&](const ProcessingRate & rate, const unsigned bindingNode, BV & updateableRateSet) {
                if (LLVM_UNLIKELY(rate.isPartialSum())) {
                    bool found = false;
                    for (const auto f : make_iterator_range(in_edges(bindingNode, G))) {
                        const RelationshipType t = G[f];
                        if (t.Reason == ReasonType::Reference) {
                            const auto refBuffer = source(f, G);
                            BV & bindingRateSet = rateSet[bindingNode];
                            auto key = std::make_pair(bindingRateSet, refBuffer);
                            const auto p = P.find(key);
                            unsigned partialSumId;
                            if (p == P.end()) {
                                partialSumId = currentRateId++;
                                P.emplace(key, partialSumId);
                            } else {
                                partialSumId = p->second;
                            }
                            addRateId(updateableRateSet, partialSumId);
                            found = true;
                        }
                    }
                    assert (found);
                }
            };


            // Determine whether this kernel is a source kernel; note: it may have
            // input scalars so we cannot simply test whether its in-degree is 0.

            bool isSourceKernel = true;
            for (const auto e : make_iterator_range(in_edges(u, G))) {
                const auto input = source(e, G);
                const RelationshipNode & inputNode = G[input];
                if (LLVM_LIKELY(inputNode.Type == RelationshipNode::IsBinding)) {
                    isSourceKernel = false;
                    break;
                }
            }

            if (isSourceKernel) {
                bool noOutputStreamSet = true;
                for (const auto e : make_iterator_range(out_edges(u, G))) {
                    const auto output = target(e, G);
                    const RelationshipNode & outputNode = G[output];
                    if (LLVM_LIKELY(outputNode.Type == RelationshipNode::IsBinding)) {
                        noOutputStreamSet = false;
                        break;
                    }
                }
                if (LLVM_UNLIKELY(noOutputStreamSet)) {
                    continue;
                }
            }

            auto bindingRateRequiresNewPartition = [&](const Binding & binding, const unsigned bindingNode, const bool isInput) {
                const ProcessingRate & rate = binding.getRate();
                switch (rate.getKind()) {
                    // countable
                    case RateId::Fixed:
                        // A source kernel produces data without input thus its output(s)
                        // must begin a new partition even for Fixed rate kernels.
                        if (isSourceKernel) {
                            return true;
                        }
                        break;
                    case RateId::Greedy:
                        // A greedy input with a lower bound > 0 cannot safely be included
                        // in its producers's partition unless its producer is guaranteed
                        // to generate more data than it consumes.
                        return true;
                    case RateId::PartialSum:
                        break;
                    // non-countable
                    case RateId::Bounded:
                        return true;
                    default: break;
                }
                // Check the attributes to see whether any impose a partition change
                for (const Attribute & attr : binding.getAttributes()) {
                    switch (attr.getKind()) {
                        case AttrId::Deferred:
                            // A deferred output rate is closer to an unknown rate than a
                            // countable rate but a deferred input rate simply means the
                            // buffer must be dynamic.
                            if (isInput) {
                                break;
                            }
                        case AttrId::Delayed:
                        case AttrId::LookAhead:
                            return true;
                        default:
                            break;
                    }
                }
                return false;
            };



            // If this kernel has any bounded or unknown input rates, there is a
            // potential change of rate of dataflow through the kernel.
            auto startNewPartition = false;
            for (const auto e : make_iterator_range(in_edges(u, G))) {
                const auto input = source(e, G);
                const RelationshipNode & inputNode = G[input];
                if (LLVM_LIKELY(inputNode.Type == RelationshipNode::IsBinding)) {
                    const Binding & binding = inputNode.Binding;
                    checkForPartialSum(binding.getRate(), input, nodeRateSet);
                    if (LLVM_UNLIKELY(bindingRateRequiresNewPartition(binding, input, true))) {
                        startNewPartition = true;
                    }
                }
            }

            if (LLVM_UNLIKELY(startNewPartition)) {
                addRateId(nodeRateSet, currentRateId++);
            }

            for (const auto e : make_iterator_range(out_edges(u, G))) {
                const auto output = target(e, G);
                const RelationshipNode & outputNode = G[output];
                if (LLVM_LIKELY(outputNode.Type == RelationshipNode::IsBinding)) {
                    // inherit all of the kernel rates
                    BV & outgoingRateSet = rateSet[output];
                    outgoingRateSet = nodeRateSet;
                    const Binding & binding = outputNode.Binding;
                    checkForPartialSum(binding.getRate(), output, outgoingRateSet);
                    if (bindingRateRequiresNewPartition(binding, output, false)) {
                        addRateId(outgoingRateSet, currentRateId++);
                    }
                }
            }

            // Fill in any relative rates
            for (const auto e : make_iterator_range(out_edges(u, G))) {
                const auto output = target(e, G);
                const RelationshipNode & outputNode = G[output];
                if (LLVM_LIKELY(outputNode.Type == RelationshipNode::IsBinding)) {
                    const Binding & binding = outputNode.Binding;
                    const ProcessingRate & rate = binding.getRate();
                    if (LLVM_UNLIKELY(rate.isRelative())) {
                        bool found = false;
                        for (const auto f : make_iterator_range(in_edges(output, G))) {
                            const RelationshipType & type = G[f];
                            if (type.Reason == ReasonType::Reference) {
                                const auto ref = source(f, G);
                                mergeRateIds(rateSet[output], rateSet[ref]);
                                found = true;
                                break;
                            }
                        }
                        assert (found);
                    }
                }
            }
        }
    }

    // Now that we've tainted the kernels with any influencing rate ids, sort them into partitions.
    PartitionMap partitionSets;
    for (auto u : O) {
        const RelationshipNode & node = G[u];
        if (node.Type == RelationshipNode::IsKernel) {
            auto & partition = partitionSets[rateSet[u]];
            partition.push_back(u);
        }
    }

    partitionIds.resize(num_vertices(G), 0);

    const auto n = partitionSets.size();
    if (LLVM_UNLIKELY(n < 2)) {
        return n;
    }

    unsigned partitionId = 1U;
    for (auto i = partitionSets.begin();;) {
        const Partition & A = i->second;
        for (const auto v : A) {
            partitionIds[v] = partitionId;
        }
        ++partitionId;
        if (++i == partitionSets.end()) {
            break;
        }
        const Partition & B = i->second;
        for (const auto u : A) {
            for (const auto v : B) {
                add_edge(u, v, RelationshipType{PortType::Input, 0, ReasonType::OrderingConstraint}, G);
            }
        }
    }

    #ifndef NDEBUG
    // Report a "not a DAG" error if something has gone wrong
    BEGIN_SCOPED_REGION
    std::vector<unsigned> __tmp;
    topological_sort(G, std::back_inserter(__tmp));
    END_SCOPED_REGION
    #endif

    assert (partitionId == (n + 1U));

    return n;
}

#endif

}

#endif // PARTITIONING_ANALYSIS_HPP
