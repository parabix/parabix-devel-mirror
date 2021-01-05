#ifndef PARTITIONING_ANALYSIS_HPP
#define PARTITIONING_ANALYSIS_HPP

#include "pipeline_analysis.hpp"
#include <toolchain/toolchain.h>
#include <util/slab_allocator.h>

namespace kernel {


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief identifyKernelPartitions
 ** ------------------------------------------------------------------------------------------------------------- */
PartitionGraph PipelineAnalysis::identifyKernelPartitions() {

    using FixedRateReachability = adjacency_matrix<directedS>;

    using BitSet = dynamic_bitset<>;

    using Graph = adjacency_list<vecS, vecS, bidirectionalS, BitSet>;
    using Vertex = Graph::vertex_descriptor;

    using BufferAttributeMap = flat_map<std::pair<Vertex, unsigned>, Vertex>;
    using PartialSumMap = flat_map<std::pair<Vertex, Vertex>, Vertex>;

    using PartitionMap = std::map<BitSet, unsigned>;

    std::vector<ProgramGraph::vertex_descriptor> kernelSequence;

    std::vector<unsigned> ordering;
    ordering.reserve(num_vertices(Relationships));
    if (LLVM_UNLIKELY(!lexical_ordering(Relationships, ordering))) {
        report_fatal_error("Cannot lexically order the initial pipeline graph!");
    }

    // Convert G into a simpler representation of the graph that we can annotate

    flat_set<Vertex> streamSets;

    for (const auto u : ordering) {
        const RelationshipNode & node = Relationships[u];
        switch (node.Type) {
            case RelationshipNode::IsKernel:
                kernelSequence.push_back(u);
                break;
            case RelationshipNode::IsRelationship:
                if (LLVM_LIKELY(isa<StreamSet>(Relationships[u].Relationship))) {
                    streamSets.insert_unique(u);
                }
                break;
            default: break;
        }
    }

    const auto numOfKernels = kernelSequence.size();

    auto getKernel = [&](const Vertex u) {
        assert (Relationships[u].Type == RelationshipNode::IsKernel);
        const auto f = std::find(kernelSequence.begin(), kernelSequence.end(), u);
        assert (f != kernelSequence.end());
        return static_cast<unsigned>(std::distance(kernelSequence.begin(), f));
    };

    FixedRateReachability R(numOfKernels);

    for (const auto u : ordering) {
        const RelationshipNode & node = Relationships[u];

        if (node.Type == RelationshipNode::IsRelationship) {
            if (LLVM_LIKELY(isa<StreamSet>(node.Relationship))) {

                const auto f = first_in_edge(u, Relationships);
                assert (Relationships[f].Reason != ReasonType::Reference);
                const auto v = source(f, Relationships);
                assert (Relationships[v].Type == RelationshipNode::IsBinding);
                const RelationshipNode & output = Relationships[v];
                const Binding & binding = output.Binding;
                const ProcessingRate & outputRate = binding.getRate();
                if (LLVM_LIKELY(outputRate.getKind() == RateId::Fixed)) {

                    const auto producer = getKernel(parent(v, Relationships));

                    for (const auto e : make_iterator_range(out_edges(u, Relationships))) {
                        const auto w = target(e, Relationships);
                        const RelationshipNode & input = Relationships[w];
                        assert (input.Type == RelationshipNode::IsBinding);
                        const Binding & binding = output.Binding;
                        const ProcessingRate & inputRate = binding.getRate();
                        if (LLVM_LIKELY(inputRate.getKind() == RateId::Fixed)) {

                            const auto f = first_out_edge(w, Relationships);
                            assert (Relationships[f].Reason != ReasonType::Reference);
                            const auto consumer = getKernel(target(f, Relationships));

                            add_edge(producer, consumer, R);
                        }
                    }
                }
            }
        }
    }

    BEGIN_SCOPED_REGION
    reverse_traversal ordering{numOfKernels};
    assert (is_valid_topological_sorting(ordering, R));
    transitive_closure_dag(ordering, R);
    END_SCOPED_REGION


    auto noFixedRatePath = [&](const Vertex streamSet, const Vertex consumer) -> bool {
        assert (Relationships[streamSet].Type == RelationshipNode::IsRelationship);
        assert (isa<StreamSet>(Relationships[streamSet].Relationship));
        assert (consumer < numOfKernels);
        assert (Relationships[kernelSequence[consumer]].Type == RelationshipNode::IsKernel);



        const auto binding = parent(streamSet, Relationships);
        assert (Relationships[binding].Type == RelationshipNode::IsBinding);
        const auto e = first_in_edge(binding, Relationships);
        assert (Relationships[e].Reason != ReasonType::Reference);
        const auto producer = getKernel(source(e, Relationships));
        const auto hasPath = edge(producer, consumer, R).second;

        return !hasPath;
    };

    const auto numOfStreamSets = streamSets.size();


    Graph H(numOfKernels + numOfStreamSets);

    flat_map<ProgramGraph::vertex_descriptor, Vertex> M;

    auto mapStreamSet = [&](const ProgramGraph::vertex_descriptor u) -> Vertex {
        assert (Relationships[u].Type == RelationshipNode::IsRelationship);
        assert (isa<StreamSet>(Relationships[u].Relationship));
        const auto f = streamSets.find(u);
        assert (f != streamSets.end());
        return numOfKernels + static_cast<unsigned>(std::distance(streamSets.begin(), f));
    };

    unsigned nextRateId = 0;


    auto addRateId = [](BitSet & bv, const unsigned rateId) {
        if (LLVM_UNLIKELY(rateId >= bv.size())) {
            #ifndef NDEBUG
            const auto c = bv.count();
            #endif
            bv.resize(round_up_to(rateId + 1, BitSet::bits_per_block));
            assert (bv.count() == c);
        }
       bv.set(rateId);
    };

    auto linkNodes =[&](const Vertex u, const Vertex v) {

//        const auto & inputRateSet = H[v];
//        auto & nodeRateSet = H[u];
//        ensureRateCapacity(nodeRateSet, nextRateId);
//        nodeRateSet |= inputRateSet;

        add_edge(u, v, H);

    };

    BufferAttributeMap LookAhead;
    BufferAttributeMap Delay;

    auto addAttribute = [&](const Vertex streamSet, BufferAttributeMap & M, const unsigned amount) -> Vertex {
        const auto key = std::make_pair(streamSet, amount);
        const auto f = M.find(key);
        if (LLVM_LIKELY(f == M.end())) {
            const auto attr = add_vertex(H);
            addRateId(H[attr], nextRateId++);
            linkNodes(streamSet, attr);
            M.emplace(key, attr);
            return attr;
        } else {
            return f->second;
        }
    };

    PartialSumMap PartialSum;

    auto checkForPartialSumEntry = [&](const Vertex binding, const Vertex buffer) -> Vertex {

        ProgramGraph::in_edge_iterator ei, ei_end;
        std::tie(ei, ei_end) = in_edges(binding, Relationships);
        for (; ei != ei_end; ++ei) {
            auto r = Relationships[*ei];
            if (r.Reason == ReasonType::Reference) {
                const auto bindingRef = source(*ei, Relationships);
                assert (Relationships[bindingRef].Type == RelationshipNode::IsBinding);
                const auto rateStream = parent(bindingRef, Relationships);
                assert (Relationships[rateStream].Type == RelationshipNode::IsRelationship);

                const auto key = std::make_pair(rateStream, buffer);
                const auto f = PartialSum.find(key);
                if (LLVM_LIKELY(f == PartialSum.end())) {
                    const auto partialSum = add_vertex(H);
                    addRateId(H[partialSum], nextRateId++);
                    PartialSum.emplace(key, partialSum);
                    return partialSum;
                } else {
                    return f->second;
                }
            }
        }

        llvm_unreachable("could not find pop count reference");

    };

    auto getReference = [&](const ProgramGraph::vertex_descriptor v) {
        ProgramGraph::in_edge_iterator ei, ei_end;
        std::tie(ei, ei_end) = in_edges(v, Relationships);
        for (; ei != ei_end; ++ei) {
            auto r = Relationships[*ei];
            if (r.Reason == ReasonType::Reference) {
                return mapStreamSet(source(*ei, Relationships));
            }
        }
        llvm_unreachable("could not find reference");
    };

    // Begin by constructing a graph that represents the I/O relationships
    // and any partition boundaries.

    for (unsigned kernel = 0; kernel < numOfKernels; ++kernel) {
        const auto u = kernelSequence[kernel];
        const RelationshipNode & node = Relationships[u];
        assert (node.Type == RelationshipNode::IsKernel);

        const auto kernelObj = node.Kernel;

        bool isNewPartitionRoot = false;

        // Iterate through the inputs
        for (const auto e : make_iterator_range(in_edges(u, Relationships))) {
            const auto binding = source(e, Relationships);
            const RelationshipNode & rn = Relationships[binding];
            if (rn.Type == RelationshipNode::IsBinding) {

                const auto f = first_in_edge(binding, Relationships);
                assert (Relationships[f].Reason != ReasonType::Reference);
                const auto streamSet = source(f, Relationships);
                assert (Relationships[streamSet].Type == RelationshipNode::IsRelationship);
                assert (isa<StreamSet>(Relationships[streamSet].Relationship));

                auto buffer = mapStreamSet(streamSet);

                const Binding & b = rn.Binding;
                const ProcessingRate & rate = b.getRate();

                switch (rate.getKind()) {
                    case RateId::Fixed:
                        BEGIN_SCOPED_REGION
                        const auto prodBinding = parent(streamSet, Relationships);
                        const RelationshipNode & rn = Relationships[prodBinding];
                        const Binding & b = rn.Binding;
                        const ProcessingRate & prodRate = b.getRate();
                        if (prodRate.isFixed()) {
                            const Rational inputRate{rate.getLowerBound() * kernelObj->getStride()};
                            const auto producer = parent(prodBinding, Relationships);
                            const RelationshipNode & prod = Relationships[producer];
                            assert (node.Type == RelationshipNode::IsKernel);
                            const Rational outputRate{prodRate.getLowerBound() * prod.Kernel->getStride()};
                            const auto relativeRate = outputRate / inputRate;
                            if (relativeRate.denominator() != 1) {
                                isNewPartitionRoot = true;
                            }
                        } else {
                            isNewPartitionRoot = true;
                        }
                        END_SCOPED_REGION
                        break;
                    case RateId::PartialSum:
                        if (noFixedRatePath(streamSet, kernel)) {
                            const auto partialSum = checkForPartialSumEntry(binding, buffer);
                            linkNodes(buffer, partialSum);
                            buffer = partialSum;
                        }
                        break;
                    case RateId::Greedy:
                        // A kernel with a greedy input cannot safely be included in its
                        // producers' partitions unless its producers are guaranteed to
                        // generate at least as much data as this kernel consumes.
                        BEGIN_SCOPED_REGION
                        const auto produced = parent(streamSet, Relationships);
                        assert (Relationships[produced].Type == RelationshipNode::IsBinding);
                        const Binding & prodBinding = Relationships[produced].Binding;
                        const ProcessingRate & prodRate = prodBinding.getRate();
                        if (prodRate.getLowerBound() < rate.getLowerBound()) {
                            isNewPartitionRoot = true;
                        }
                        END_SCOPED_REGION
                        break;
                    case RateId::Bounded:
                        if (noFixedRatePath(streamSet, kernel)) {
                            // A bounded input rate always starts a new partition
                            isNewPartitionRoot = true;
                        }
                        break;

                    default: break;
                }

                // If we have a lookahead/delay attribute on any stream, create
                // a new buffer vertex (with a new rate id) to represent it each
                // unique pairing.
                for (const Attribute & attr : b.getAttributes()) {
                    switch (attr.getKind()) {
                        case AttrId::Delayed:
                            buffer = addAttribute(buffer, Delay, attr.amount());
                            break;
                        case AttrId::LookAhead:
                            buffer = addAttribute(buffer, LookAhead, attr.amount());
                            break;
                        default: break;
                    }
                }

                linkNodes(buffer, kernel);
            }
        }

        // TODO: currently any kernel K that can terminate early is the root of a new partition. However,
        // we could include it one of its sources' partition P if and only if there is no divergent path
        // of kernels S in P for which *all* consumers of the outputs of S are controlled by an output of
        // K.


        // Check whether this (internal) kernel could terminate early
        bool mayTerminateEarly = false;
        bool internallySynchronized = false;
        if (kernelObj != mPipelineKernel) {
            mayTerminateEarly = kernelObj->canSetTerminateSignal();
            internallySynchronized = kernelObj->hasAttribute(AttrId::InternallySynchronized);
        }

        // Assign a root of a partition a new id.
        if (LLVM_UNLIKELY(isNewPartitionRoot || mayTerminateEarly)) {
            addRateId(H[kernel], nextRateId++);
        }

        // TODO: an internally synchronzied kernel with fixed rate I/O can be contained within a partition
        // but cannot be the root of a non-isolated partition. To permit them to be roots, they'd need
        // some way of informing the pipeline as to how many strides they executed or the pipeline
        // would need to know to calculate it from its outputs. Rather than handling this complication,
        // for now we simply prevent this case.

        const auto demarcateOutputs = mayTerminateEarly || internallySynchronized || (in_degree(u, Relationships) == 0);
        unsigned demarcationId = 0;

        if (LLVM_UNLIKELY(demarcateOutputs)) {
            demarcationId = nextRateId++;
        }

        // Now iterate through the outputs
        for (const auto e : make_iterator_range(out_edges(u, Relationships))) {

            const auto binding = target(e, Relationships);
            const RelationshipNode & rn = Relationships[binding];

            if (rn.Type == RelationshipNode::IsBinding) {

                const auto f = out_edge(binding, Relationships);
                assert (Relationships[f].Reason != ReasonType::Reference);
                const auto streamSet = target(f, Relationships);
                assert (Relationships[streamSet].Type == RelationshipNode::IsRelationship);
                assert (isa<StreamSet>(Relationships[streamSet].Relationship));

                const auto buffer = mapStreamSet(streamSet);

                const Binding & b = rn.Binding;
                const ProcessingRate & rate = b.getRate();
                if (LLVM_UNLIKELY(demarcateOutputs)) {
                    addRateId(H[buffer], demarcationId);
                }

                switch (rate.getKind()) {
                    case RateId::Fixed:
                        break;
                    case RateId::PartialSum:
                        BEGIN_SCOPED_REGION
                        const auto partialSum = checkForPartialSumEntry(binding, buffer);
                        linkNodes(partialSum, buffer);
                        END_SCOPED_REGION
                        break;
                    case RateId::Bounded:
                        addRateId(H[buffer], nextRateId++);
                        break;
                    case RateId::Relative:
                        BEGIN_SCOPED_REGION
                        const auto refBuffer = getReference(binding);
                        linkNodes(refBuffer, buffer);
                        END_SCOPED_REGION
                        break;
                    default: break;
                }
                // Check the attributes to see whether any impose a partition change
                auto hasRateChangeAttribute = [](const Binding & b) {
                    for (const Attribute & attr : b.getAttributes()) {
                        switch (attr.getKind()) {
                            case AttrId::Delayed:
                            case AttrId::Deferred:
                            case AttrId::BlockSize:
                                // A deferred output rate is closer to an bounded rate than a
                                // countable rate but a deferred input rate simply means the
                                // buffer must be dynamic.
                                return true;
                            default: break;
                        }
                    }
                    return false;
                };

                linkNodes(kernel, buffer);

                if (LLVM_UNLIKELY(hasRateChangeAttribute(b))) {
                    addRateId(H[buffer], nextRateId++);
                }

            }
        }

        // Each output of a source kernel is given a rate not shared by the kernel itself
        // to ensure its descendents are in a seperate partition. However, the descendents
        // of a source kernel with two or more outputs that have an equivalent rate ought
        // to be in the same partition (assuming they have no other inputs from differing
        // partitions.)

        if (LLVM_UNLIKELY(in_degree(kernel, H) == 0 && out_degree(kernel, H) != 0)) {
            // place each input source kernel in its own partition
            addRateId(H[kernel], nextRateId++);
            const auto sourceKernelRateId = nextRateId++;
            for (const auto output : make_iterator_range(out_edges(kernel, H))) {
                const auto buffer = target(output, H);
                addRateId(H[buffer], sourceKernelRateId);
            }
        }
    }

    // Combine all incoming rates sets
    ordering.clear();
    ordering.reserve(num_vertices(H));
    if (LLVM_UNLIKELY(!lexical_ordering(H, ordering))) {
        report_fatal_error("Cannot lexically order the partition graph!");
    }

    for (const auto u : ordering) {
        auto & nodeRateSet = H[u];
        // boost dynamic_bitset will segfault when buffers are of differing lengths.
        nodeRateSet.resize(nextRateId);
        for (const auto e : make_iterator_range(in_edges(u, H))) {
            const auto v = source(e, H);
            const auto & inputRateSet = H[v];
            nodeRateSet |= inputRateSet;
        }
    }

    // NOTE: summarizing the partitions here into a graph then taking the topological
    // order of them to determine the partition ids is almost certainly overkill but
    // was done to future proof this portion of the algorithm. Counting them initially
    // in reverse (given that the kernels are in topological order) ought to account
    // for any scenarios were given a lexographic ordering of the original graph,
    // the ordering of one partition is bisected by kernels in another partition.

    using SummaryGraph = adjacency_list<vecS, vecS, bidirectionalS, Partition, unsigned>;

    SummaryGraph S(1);

    BEGIN_SCOPED_REGION

    flat_map<unsigned, unsigned> partitionMap;
    partitionMap.reserve(numOfKernels);

    BEGIN_SCOPED_REGION

    PartitionMap partitionSets;

    for (unsigned u = 0; u < numOfKernels; ++u) {
        const BitSet & bitSet = H[u];
        unsigned partitionId = 0;
        if (LLVM_LIKELY(bitSet.any())) {
            auto f = partitionSets.find(bitSet);
            if (f == partitionSets.end()) {
                partitionId = add_vertex(S);
                partitionSets.emplace(bitSet, partitionId);
            } else {
                partitionId = f->second;
            }
        }
        const auto v = kernelSequence[u];
        #ifndef NDEBUG
        const auto & N = Relationships[v];
        assert (N.Type == RelationshipNode::IsKernel);
        assert (bitSet.any() || N.Kernel == mPipelineKernel);
        #endif
        partitionMap.emplace(v, partitionId);
        S[partitionId].push_back(v);
    }

    END_SCOPED_REGION

    PartitionCount = num_vertices(S);

    for (unsigned i = 1; i < PartitionCount; ++i) {
        add_edge(0, i, S);
        for (const auto producer : S[i]) {
            for (const auto e : make_iterator_range(out_edges(producer, Relationships))) {
                const auto output = target(e, Relationships);
                const RelationshipNode & rn = Relationships[output];

                if (LLVM_LIKELY(rn.Type == RelationshipNode::IsBinding)) {

                    const auto f = first_out_edge(output, Relationships);
                    assert (Relationships[f].Reason != ReasonType::Reference);
                    const auto streamSet = target(f, Relationships);
                    assert (Relationships[streamSet].Type == RelationshipNode::IsRelationship);
                    assert (isa<StreamSet>(Relationships[streamSet].Relationship));

                    for (const auto g : make_iterator_range(out_edges(streamSet, Relationships))) {
                        const auto input = target(g, Relationships);
                        const RelationshipNode & rn = Relationships[input];
                        if (LLVM_LIKELY(rn.Type == RelationshipNode::IsBinding)) {

                            const auto h = first_out_edge(input, Relationships);
                            assert (Relationships[h].Reason != ReasonType::Reference);
                            const auto consumer = target(h, Relationships);
                            assert (Relationships[consumer].Type == RelationshipNode::IsKernel);

                            const auto p = partitionMap.find(consumer);
                            assert (p != partitionMap.end());
                            const auto j = p->second;
                            if (i != j) {
                                add_edge(i, j, streamSet, S);
                            }
                        }
                    }
                }

            }
        }
    }

    END_SCOPED_REGION

    ordering.clear();
    if (LLVM_UNLIKELY(!lexical_ordering(S, ordering))) {
        report_fatal_error("Cannot lexically order the partition summary graph!");
    }

    assert (ordering[0] == 0);

    PartitionGraph P(PartitionCount);

    std::vector<unsigned> reverseMapping(PartitionCount);

    for (unsigned i = 0; i < PartitionCount; ++i) {
        reverseMapping[ordering[i]] = i;
    }

    PartitionIds.reserve(numOfKernels);

    clear_out_edges(0, S);

//    printRelationshipGraph(Relationships, errs(), "R");

    for (unsigned i = 0; i < PartitionCount; ++i) {
        const auto j = ordering[i];
        PartitionData & N = P[i];
        N.Kernels = std::move(S[j]);

//        errs() << "PARTITION " << i << ":";
//        char joiner = ' ';
//        for (const auto k : N.Kernels) {
//            errs() << joiner << k;
//            joiner = ',';
//        }
//        errs() << "\n";


        for (const auto k : N.Kernels) {
            PartitionIds.emplace(k, i);
        }




        for (const auto e : make_iterator_range(out_edges(j, S))) {
            const auto k = reverseMapping[target(e, S)];
            const auto t = S[e];
            assert (t < num_vertices(Relationships));
            add_edge(i, k, t, P);
        }
    }

    assert (PartitionIds.size() == numOfKernels);

    return P;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief determinePartitionJumpIndices
 *
 * If a partition determines it has insufficient data to execute, identify which partition is the next one to test.
 * I.e., the one with input from some disjoint path. If none exists, we'll begin jump to "PartitionCount", which
 * marks the end of the processing loop.
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::determinePartitionJumpIndices() {

    using BV = dynamic_bitset<>;
    using Graph = adjacency_list<hash_setS, vecS, bidirectionalS>;

    // Summarize the partitioning graph to only represent the existance of a dataflow relationship
    // between the partitions.

    Graph G(PartitionCount);
    std::vector<BV> paths(PartitionCount);

    for (auto consumer = FirstKernel; consumer <= PipelineOutput; ++consumer) {
       const auto cid = KernelPartitionId[consumer];
       for (const auto e : make_iterator_range(in_edges(consumer, mBufferGraph))) {
           const auto buffer = source(e, mBufferGraph);
           const auto producer = parent(buffer, mBufferGraph);
           const auto pid = KernelPartitionId[producer];
           assert (pid <= cid);
           if (pid != cid) {
               add_edge(pid, cid, G);
           }
       }
    }

    const auto terminal = PartitionCount - 1U;

    for (auto partitionId = 0U; partitionId < terminal; ++partitionId) {
       if (out_degree(partitionId, G) == 0) {
           add_edge(partitionId, terminal, G);
       }
    }

    // Now compute the transitive reduction of the partition relationships
    BEGIN_SCOPED_REGION
    const reverse_traversal ordering(PartitionCount);
    assert (is_valid_topological_sorting(ordering, G));
    transitive_closure_dag(ordering, G);
    transitive_reduction_dag(ordering, G);
    END_SCOPED_REGION

    for (auto partitionId = 0U; partitionId < terminal; ++partitionId) {
        if (mTerminationCheck[partitionId] & TerminationCheckFlag::Soft) {
            add_edge(partitionId, partitionId + 1U, G);
        }
    }
    add_edge(terminal, terminal, G);

    // Generate a post dominator tree of G. If we do not have enough data to execute
    // a partition along some branched path, it's possible a post dominator of the paths
    // was prevented from executing because of some other paths output. If that path was
    // successfully executed, it may have enough data to process now despite the fact we
    // produced no new data along this path.

    BEGIN_SCOPED_REGION

    for (auto u = 0U; u < PartitionCount; ++u) {
        BV & P = paths[u];
        P.resize(PartitionCount);
        P.set(u);
    }

    BV M(PartitionCount);

    for (auto u = PartitionCount; u--; ) { // forward topological ordering
        assert (out_degree(u, G) > 0);
        M.set();
        assert (M.count() == PartitionCount);
        for (const auto e : make_iterator_range(out_edges(u, G))) {
            const auto v = target(e, G);
            assert (v < PartitionCount);
            M &= paths[v];
        }
        BV & P = paths[u];
        P |= M;
    }

    // reverse topological ordering starting at (PartitionCount - 1)
    for (auto u = terminal; u--; ) {

        const BV & P = paths[u];

        auto v = u;

        while (++v < terminal) {
            const BV & O = paths[v];
            if (LLVM_UNLIKELY(!O.is_subset_of(P))) {
                break;
            }
        }

        // v is the immediate post-dominator of u; however, since this graph indicates
        // that we could not execute u nor any of its branched paths, we search for the
        // first non-immediate post dominator with an in-degree > 1. We're guaranteed
        // to find one since the common sink must have an in-degree >= 2.

        clear_out_edges(u, G);
        assert (u != v);
        add_edge(u, v, G);
    }

    END_SCOPED_REGION

    mPartitionJumpIndex.resize(PartitionCount);
    for (unsigned i = 0; i < PartitionCount; ++i) {
        const auto j = child(i, G);
        assert ("jump target cannot preceed source" && i <= j);
        mPartitionJumpIndex[i] = j;
        #ifndef NDEBUG
        for (unsigned k = 0; k < i; ++k) {

            /* Recall that G is a tree where each node is numbered
            in order that partitions will be executed in.

            Suppose we have a tree:

                                 1   3
                                  \ /
                                   4   2
                                    \ /
                                     5

            If we jump from partition 2 to 5, we'll miss processing
            partition 3 and 4 and the pipeline will never able to
            progress further. In such a case, the chosen partition
            ordering is degenerate and in general could result in an
            infinite loop / backlog of unprocessed input.

            By simply verifying that we touch every child of the
            a node before looking at another node, we prove the tree
            has a valid jump structure. */

            assert ("degenerate jump tree structure!" && (mPartitionJumpIndex[k] <= j));
        }
        #endif
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makePartitionJumpGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::makePartitionJumpTree() {
    mPartitionJumpTree = PartitionJumpTree(PartitionCount);
    for (auto i = 0U; i < (PartitionCount - 1); ++i) {        
        add_edge(i, mPartitionJumpIndex[i], mPartitionJumpTree);
    }
//    for (auto i = 1U; i < (PartitionCount - 1); ++i) {
//        add_edge(i, (i + 1U), mPartitionJumpTree);
//    }
}

} // end of namespace kernel

#endif // PARTITIONING_ANALYSIS_HPP
