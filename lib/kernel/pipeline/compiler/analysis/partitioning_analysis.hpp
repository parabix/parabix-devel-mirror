#ifndef PARTITIONING_ANALYSIS_HPP
#define PARTITIONING_ANALYSIS_HPP

#include "pipeline_analysis.hpp"
#include <util/slab_allocator.h>

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief partitionIntoFixedRateRegionWithOrderingConstraints
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::partitionRelationshipGraphIntoSynchronousRegions() {
    std::vector<unsigned> orderingOfG;
    orderingOfG.reserve(num_vertices(mRelationships));
    if (LLVM_UNLIKELY(!lexical_ordering(mRelationships, orderingOfG))) {
        report_fatal_error("Cannot lexically order the initial pipeline graph!");
    }    
    identifyKernelPartitions(orderingOfG);
    addOrderingConstraintsToPartitionSubgraphs(orderingOfG);
}

template<typename Map>
typename Map::mapped_type & get(const typename Map::key_type & key, Map & M) {
    const auto f = M.find(key);
    assert (f != M.end());
    return f->second;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief identifyKernelPartitions
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::identifyKernelPartitions(const std::vector<unsigned> & orderingOfG) {

    using BitSet = dynamic_bitset<>;

    using Graph = adjacency_list<vecS, vecS, bidirectionalS, BitSet, unsigned>;
    using Vertex = Graph::vertex_descriptor;

    using BufferAttributeMap = flat_map<std::pair<Vertex, unsigned>, Vertex>;
    using PartialSumMap = flat_map<Vertex, Vertex>;

    using PartitionMap = std::map<BitSet, unsigned>;

    // Convert G into a simpler representation of the graph that we can annotate
    unsigned kernels = 0;
    unsigned streamSets = 0;
    for (const auto u : make_iterator_range(vertices(mRelationships))) {
        const RelationshipNode & node = mRelationships[u];
        switch (node.Type) {
            case RelationshipNode::IsKernel:
                ++kernels;
                break;
            case RelationshipNode::IsRelationship:
                if (LLVM_LIKELY(isa<StreamSet>(mRelationships[u].Relationship))) {
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
        assert (mRelationships[u].Type == RelationshipNode::IsRelationship);
        assert (isa<StreamSet>(mRelationships[u].Relationship));
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

    auto addRateId = [](BitSet & bv, const unsigned rateId) {
       if (LLVM_UNLIKELY(rateId >= bv.size())) {
           bv.resize(round_up_to(rateId + 1, BitSet::bits_per_block));
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
            add_edge(streamSet, buf, 0, H);
            M.emplace(key, buf);
            return buf;
        } else {
            return f->second;
        }
    };

    PartialSumMap PI;
    PartialSumMap PO;

    auto checkForPartialSumEntry = [&](const Vertex streamSet, PartialSumMap & P) -> Vertex {
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

    auto getReference = [&](const Relationships::vertex_descriptor v) {
        Relationships::in_edge_iterator ei, ei_end;
        std::tie(ei, ei_end) = in_edges(v, mRelationships);
        for (; ei != ei_end; ++ei) {
            auto r = mRelationships[*ei];
            if (r.Reason == ReasonType::Reference) {
                return mapStreamSet(source(*ei, mRelationships));
            }
        }
        llvm_unreachable("could not find reference");
    };

    SmallFlatSet<unsigned, 8> fixedRates;

    std::vector<Relationships::vertex_descriptor> mappedKernel(kernels);

    // Begin by constructing a graph that represents the I/O relationships
    // and any partition boundaries.
    for (const auto u : orderingOfG) {

        const RelationshipNode & node = mRelationships[u];

        if (node.Type == RelationshipNode::IsKernel) {

            const auto kernel = nextKernel++;
            assert (kernel < kernels);
            mappedKernel[kernel] = u;
            assert (H[kernel].none());

            const Kernel * const kernelObj = node.Kernel;

            if ((kernelObj == mPipelineKernel) && (out_degree(u, mRelationships) == 0)) {
                continue;
            }

            bool isNewPartitionRoot = false;

            // Iterate through the inputs
            for (const auto e : make_iterator_range(in_edges(u, mRelationships))) {
                const auto binding = source(e, mRelationships);
                const RelationshipNode & rn = mRelationships[binding];
                if (rn.Type == RelationshipNode::IsBinding) {

                    const auto f = first_in_edge(binding, mRelationships);
                    assert (mRelationships[f].Reason != ReasonType::Reference);
                    const auto streamSet = source(f, mRelationships);
                    assert (mRelationships[streamSet].Type == RelationshipNode::IsRelationship);
                    assert (isa<StreamSet>(mRelationships[streamSet].Relationship));
                    auto buffer = mapStreamSet(streamSet);

                    const Binding & b = rn.Binding;
                    const ProcessingRate & rate = b.getRate();

                    unsigned fixedRate = 0;
                    switch (rate.getKind()) {
                        case RateId::Fixed:
                            BEGIN_SCOPED_REGION
                            const auto stride = rate.getRate() * kernelObj->getStride();
                            assert (stride.denominator() == 1);
                            fixedRate = stride.numerator();
                            fixedRates.insert(fixedRate);
                            END_SCOPED_REGION
                            break;
                        case RateId::PartialSum:
                            BEGIN_SCOPED_REGION
                            const auto partialSum = checkForPartialSumEntry(buffer, PI);
                            add_edge(buffer, partialSum, 0, H);
                            buffer = partialSum;
                            END_SCOPED_REGION
                            break;
                        case RateId::Greedy:
                            // A kernel with a greedy input cannot safely be included in its
                            // producers' partitions unless its producers are guaranteed to
                            // generate at least as much data as this kernel consumes.
                            BEGIN_SCOPED_REGION
                            const auto produced = parent(streamSet, mRelationships);
                            assert (mRelationships[produced].Type == RelationshipNode::IsBinding);
                            const Binding & prodBinding = mRelationships[produced].Binding;
                            const ProcessingRate & prodRate = prodBinding.getRate();
                            if (prodRate.getLowerBound() < rate.getLowerBound()) {
                                isNewPartitionRoot = true;
                            }
                            END_SCOPED_REGION
                            break;
                        case RateId::Bounded:
                            // A bounded input rate always starts a new partition
                            isNewPartitionRoot = true;
                            break;

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

                    add_edge(buffer, kernel, fixedRate, H);
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

            const auto demarcateOutputs = mayTerminateEarly || internallySynchronized; // (isNewPartitionRoot && internallySynchronized);
            unsigned demarcationId = 0;

            if (LLVM_UNLIKELY(demarcateOutputs)) {
                demarcationId = nextRateId++;
            }

            // Now iterate through the outputs
            for (const auto e : make_iterator_range(out_edges(u, mRelationships))) {

                const auto binding = target(e, mRelationships);
                const RelationshipNode & rn = mRelationships[binding];

                if (rn.Type == RelationshipNode::IsBinding) {

                    const auto f = out_edge(binding, mRelationships);
                    assert (mRelationships[f].Reason != ReasonType::Reference);
                    const auto streamSet = target(f, mRelationships);
                    assert (mRelationships[streamSet].Type == RelationshipNode::IsRelationship);
                    assert (isa<StreamSet>(mRelationships[streamSet].Relationship));
                    const auto buffer = mapStreamSet(streamSet);
                    const Binding & b = rn.Binding;
                    const ProcessingRate & rate = b.getRate();                    
                    if (LLVM_UNLIKELY(demarcateOutputs)) {
                        addRateId(H[buffer], demarcationId);
                    }

                    unsigned fixedRate = 0;
                    switch (rate.getKind()) {
                        case RateId::Fixed:
                            BEGIN_SCOPED_REGION
                            const auto stride = rate.getRate() * kernelObj->getStride();
                            assert (stride.denominator() == 1);
                            fixedRate = stride.numerator();
                            fixedRates.insert(fixedRate);
                            END_SCOPED_REGION
                            break;
                        case RateId::PartialSum:
                            BEGIN_SCOPED_REGION
                            const auto partialSum = checkForPartialSumEntry(buffer, PO);
                            add_edge(partialSum, buffer, 0, H);
                            END_SCOPED_REGION
                            break;
                        case RateId::Bounded:
                            addRateId(H[buffer], nextRateId++);
                            break;
                        case RateId::Relative:
                            BEGIN_SCOPED_REGION
                            const auto refBuffer = getReference(binding);
                            add_edge(refBuffer, buffer, 0, H);
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

                    add_edge(kernel, buffer, fixedRate, H);

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
    }


    auto printGraph =[&](const Graph & G, raw_ostream & out, const StringRef name = "G") {

        out << "digraph \"" << name << "\" {\n";
        for (auto v : make_iterator_range(vertices(G))) {
            out << "v" << v << " [label=\"" << v << ": {";
            bool comma = false;
            const BitSet & bv = G[v];
            for (auto i = bv.find_first(); i != BitSet::npos; i = bv.find_next(i)) {
                if (comma) out << ',';
                out << i;
                comma = true;
            }
            out << "}\"];\n";
        }
        for (auto e : make_iterator_range(edges(G))) {
            const auto s = source(e, G);
            const auto t = target(e, G);
            out << "v" << s << " -> v" << t << " ";
            if (G[e]) {
                out << " [label=\"<" << G[e] << ">\"]";
            }
            out << ";\n";
        }

        out << "}\n\n";
        out.flush();
    };

    // Note: it's possible some stream sets are produced but never consumed
    assert (nextStreamSet <= (streamSets + kernels));
    assert (nextKernel == kernels);

    // Combine all incoming rates sets
    std::vector<Vertex> orderingOfH;
    orderingOfH.reserve(num_vertices(H));
    if (LLVM_UNLIKELY(!lexical_ordering(H, orderingOfH))) {
        report_fatal_error("Cannot lexically order the partition graph!");
    }


    const auto sink = orderingOfH.back();
    addRateId(H[sink], nextRateId++);    

    std::vector<Vertex> kernelOrdering;
    kernelOrdering.reserve(kernels);

    for (const auto u : orderingOfH) {
        auto & nodeRateSet = H[u];
        // boost dynamic_bitset will segfault when buffers are of differing lengths.
        nodeRateSet.resize(nextRateId);
        for (const auto e : make_iterator_range(in_edges(u, H))) {
            const auto v = source(e, H);
            const auto & inputRateSet = H[v];
            nodeRateSet |= inputRateSet;
        }
        if (u < kernels) {
            kernelOrdering.push_back(u);
        }
    }

    PartitionMap partitionSets;

    unsigned nextPartitionId = 1;

    #if 1

    KernelPartitionIds partitionIds;
    partitionIds.reserve(kernels);


    // Now that we've tainted the kernels with any influencing rates,
    // determine their initial partition placement.
    for (const auto u : kernelOrdering) {
        assert (u < kernels);
        BitSet & node = H[u];
        auto id = 0;
        if (LLVM_LIKELY(node.any())) {
            auto f = partitionSets.find(node);
            if (f == partitionSets.end()) {
                id = nextPartitionId++;
                partitionSets.emplace(node, id);
            } else {
                id = f->second;
            }
        }
        partitionIds.emplace(u, id);
    }
    assert (partitionIds.size() == kernels);

    const auto maxPartitionId = nextPartitionId;

    const auto numOfFixedRates = fixedRates.size();
    const auto patitionIdSize = numOfFixedRates + nextPartitionId;

    for (const auto u : make_iterator_range(vertices(H))) {
        auto & nodeRateSet = H[u];
        nodeRateSet.reset();
        nodeRateSet.resize(patitionIdSize);
    }

    std::queue<unsigned> Q;
    for (const auto u : kernelOrdering) {
        const auto partitionId = get(u, partitionIds);
        if (partitionId == 0) {
            continue;
        }

        assert (Q.empty());
        Q.push(u);
        // Remove any edges that cross a partition boundary to simplify
        // analysis later.
        for(;;) {
            const auto v = Q.front();
            Q.pop();
            remove_out_edge_if(v, [&](const Graph::edge_descriptor e) {
                const auto w = target(e, H);
                if (w < kernels) {
                    if (partitionId != get(w, partitionIds)) {
                        return true;
                    }
                } else {
                    Q.push(w);
                }
                return false;
            }, H);
            if (LLVM_UNLIKELY(Q.empty())) {
                break;
            }
        }

        auto & nodeRateSet = H[u];
        nodeRateSet.set(partitionId);

        // Potentially break a partition into into two whenever we go from a smaller
        // fixed rate to a larger one. This is both to improve performance and simplify
        // final segment processing since when we would have to hold back processing
        // data in the earlier kernels in order to ensure that the kernel with a larger
        // stride size can process all of the data passed into it from its root.
        // However, the final stride would have to always process all of it so
        // the kernels within the same partition could potentially have different
        // number of relative strides.

        auto addFixedRate = [&](const unsigned fixedRate) {
            if (fixedRate) {
                const auto entry = fixedRates.find(fixedRate);
                assert (entry != fixedRates.end());
                const auto begin = fixedRates.begin();
                for (auto i = begin; i != entry; ++i) {
                    assert (*i < fixedRate);
                    if ((fixedRate % *i) == 0) {
                        const auto k = maxPartitionId + std::distance(begin, i);
                        assert (k < patitionIdSize);
                        nodeRateSet.set(k);
                    }
                }
                const auto k = maxPartitionId + std::distance(begin, entry);
                assert (k < patitionIdSize);
                nodeRateSet.set(k);
            }
        };

        for (const auto e : make_iterator_range(in_edges(u, H))) {
            addFixedRate(H[e]);
        }

        for (const auto e : make_iterator_range(out_edges(u, H))) {
            addFixedRate(H[e]);
        }
    }

    for (const auto u : orderingOfH) {
        auto & nodeRateSet = H[u];
        for (const auto e : make_iterator_range(in_edges(u, H))) {
            const auto v = source(e, H);
            const auto & inputRateSet = H[v];
            nodeRateSet |= inputRateSet;
        }
    }

    #endif

    mPartitionIds.reserve(kernels);
    partitionSets.clear();
    nextPartitionId = 0;

    for (const auto u : kernelOrdering) {
        const BitSet & node = H[u];
        unsigned id = 0;
        if (LLVM_LIKELY(node.any())) {
            auto f = partitionSets.find(node);
            if (f == partitionSets.end()) {
                id = nextPartitionId++;
                partitionSets.emplace(node, id);
            } else {
                id = f->second;
            }
        }
        mPartitionIds.emplace(mappedKernel[u], id);
    }
    assert (mPartitionIds.size() == kernels);
    PartitionCount = nextPartitionId;

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief extractPartitionSubgraphs
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::addOrderingConstraintsToPartitionSubgraphs(const std::vector<unsigned> & orderingOfG) {

    using ConstraintGraph = adjacency_list<hash_setS, vecS, bidirectionalS, BitVector, no_property>;
    using BitSet = BitVector;
    using EdgeIterator = graph_traits<ConstraintGraph>::edge_iterator;

    std::vector<unsigned> kernels;
    for (const auto u : orderingOfG) {
        const RelationshipNode & node = mRelationships[u];
        switch (node.Type) {
            case RelationshipNode::IsKernel:
                BEGIN_SCOPED_REGION
                const auto f = mPartitionIds.find(u);
                assert (f != mPartitionIds.end());
                const auto id = f->second;
                if (LLVM_LIKELY(id != -1U)) {
                    kernels.push_back(u);
                }
                END_SCOPED_REGION
            default: break;
        }
    }

    const auto numOfKernels = kernels.size();

    flat_map<unsigned, unsigned> mapping;
    mapping.reserve(numOfKernels);

    auto from_original =[&](const unsigned u) {
        const auto f = mapping.find(u); assert (f != mapping.end());
        return f->second;
    };

    flat_map<unsigned, unsigned> reverse_mapping;
    reverse_mapping.reserve(numOfKernels);

    auto to_original =[&](const unsigned u) {
        const auto f = reverse_mapping.find(u); assert (f != reverse_mapping.end());
        return f->second;
    };

    unsigned i = 0;
    for (const auto u : kernels) {
        mapping.emplace(u, i);
        reverse_mapping.emplace(i, u);
        ++i;
    }

    const auto cfg = Z3_mk_config();
    Z3_set_param_value(cfg, "model", "true");
    Z3_set_param_value(cfg, "proof", "false");
    const auto ctx = Z3_mk_context(cfg);
    Z3_del_config(cfg);

    bool canRemake = true;

    auto print_constraint_graph = [&](ConstraintGraph & G, raw_ostream & out, const StringRef name = "G") {

        out << "digraph \"" << name << "\" {\n";
        for (auto v : make_iterator_range(vertices(G))) {
            out << "v" << v << " [label=\"" << v << " : {";
            const BitSet & V = G[v];
            bool comma = false;
            for (auto i : V.set_bits()) {
                if (comma) out << ',';
                out << i;
                comma = true;
            }
            out << "}\"];\n";
        }
        for (auto e : make_iterator_range(edges(G))) {
            const auto s = source(e, G);
            const auto t = target(e, G);
            out << "v" << s << " -> v" << t << ";\n";
        }

        out << "}\n\n";
        out.flush();

    };

remake_constraint_graphs:

    ConstraintGraph P(PartitionCount);
    ConstraintGraph C(numOfKernels);

    flat_set<unsigned> inputs;

    for (const auto kernel : kernels) {
        const auto partitionId = get(kernel, mPartitionIds);
        assert (partitionId != -1U);


        const RelationshipNode & K = mRelationships[kernel];
        assert (K.Type == RelationshipNode::IsKernel);
        assert (inputs.empty());


        // enumerate the input constraints
        for (const auto input : make_iterator_range(in_edges(kernel, mRelationships))) {
            const auto node = source(input, mRelationships);
            const RelationshipNode & rn = mRelationships[node];
            if (rn.Type == RelationshipNode::IsBinding) {
                const auto f = first_in_edge(node, mRelationships);
                assert (mRelationships[f].Reason != ReasonType::Reference);
                const auto streamSet = source(f, mRelationships);
                assert (mRelationships[streamSet].Type == RelationshipNode::IsRelationship);
                assert (isa<StreamSet>(mRelationships[streamSet].Relationship));
                const auto outputBinding = parent(streamSet, mRelationships);
                assert (mRelationships[outputBinding].Type == RelationshipNode::IsBinding);
                const auto output = first_in_edge(outputBinding, mRelationships);
                assert (mRelationships[output].Reason != ReasonType::Reference);
                const auto producer = source(output, mRelationships);
                assert (mRelationships[producer].Type == RelationshipNode::IsKernel);
                inputs.insert(producer);
            } else if (LLVM_UNLIKELY(rn.Type == RelationshipNode::IsKernel)) {
                assert (mRelationships[input].Reason == ReasonType::OrderingConstraint);
                const auto f = mPartitionIds.find(node);
                assert (f != mPartitionIds.end());
                const auto partitionId = f->second;
                if (LLVM_UNLIKELY(partitionId != -1U)) {
                    inputs.insert(node);
                }
            }
        }

        // then write them and any summarized partition constraints
        const auto u = from_original(kernel);
        for (const auto input : inputs) {
            const auto v = from_original(input);
            add_edge(v, u, C);
            const auto producerPartitionId = get(input, mPartitionIds);
            if (partitionId != producerPartitionId && producerPartitionId != -1U) {
                add_edge(producerPartitionId, partitionId, P);
            }
        }

        inputs.clear();

    }

    // Take the transitive reduction of the constraint graphs to reduce the number of
    // constraints we'll add to the formula.

    std::vector<unsigned> orderingOfP;
    orderingOfP.reserve(num_vertices(P));
    lexical_ordering(P, orderingOfP);

    // If the original ordering of the program wasn't a topological ordering, the
    // partition ids should be renumbered.

    for (unsigned i = 0; i < PartitionCount; ++i) {
        if (i != orderingOfP[i]) {
            for (auto & i : mPartitionIds) {
                i.second = orderingOfP[i.second];
            }
            goto remake_constraint_graphs;
        }
    }

    std::vector<unsigned> orderingOfC;
    orderingOfC.reserve(numOfKernels);
    lexical_ordering(C, orderingOfC);

    std::reverse(orderingOfP.begin(), orderingOfP.end());
    std::reverse(orderingOfC.begin(), orderingOfC.end());

    transitive_closure_dag(orderingOfP, P);
    transitive_closure_dag(orderingOfC, C);

    transitive_reduction_dag(orderingOfP, P);
    transitive_reduction_dag(orderingOfC, C);

    // Then contract any chains into a single vertex to both simplify the SMT formula
    // in a way that reflects an optimal answer and prevent any "false" breaks later
    // in the partition skipping logic.

    std::vector<unsigned> mappedPartitionId(numOfKernels);

    auto contract_constraint_graph = [&](ConstraintGraph & H, const std::vector<unsigned> & O, const bool checkPartition) {

        assert (num_vertices(H) == O.size());

        const auto m = O.size();

        for (unsigned i = 0; i < m; ++i) {
            assert (i == O[m - i - 1]);
            BitSet & B = H[i];
            B.resize(m);
            B.set(i);
        }

        unsigned contracted = 0;

        // NOTE: since the vertices are already lexically labelled, the label of any
        // descendent of a vertex must be greater than all prior vertices.

        for (const auto u : O) {
            if (out_degree(u, H) == 1) {
                const auto v = child(u, H);
                if (in_degree(v, H) == 1) {
                    if (checkPartition && (mappedPartitionId[u] != mappedPartitionId[v])) {
                        continue;
                    }
                    BitSet & U = H[u];
                    assert (U.count() == 1);
                    BitSet & V = H[v];

                    assert (U.find_first() < V.find_first());
                    U |= V;
                    V.reset();
                    for (const auto e : make_iterator_range(out_edges(v, H))) {
                        add_edge(u, target(e, H), H);
                    }
                    clear_vertex(v, H);
                    ++contracted;
                }
            }
        }

        return contracted;
    };

    const auto contractedPartitions = contract_constraint_graph(P, orderingOfP, false);

//    BEGIN_SCOPED_REGION
//    unsigned k = 0;
//    bool restart = false;
//    SmallVector<unsigned, 32> remappedPartitionId(PartitionCount);
//    for (unsigned i = 0; i < PartitionCount; ++i) {
//        const BitSet & nodes = P[i];
//        for (unsigned j : nodes.set_bits()) {
//            assert (k < PartitionCount);
//            restart |= (j != k);
//            remappedPartitionId[j] = k++;
//        }
//    }
//    assert (k == PartitionCount);
//    if (restart) {
//        for (auto & i : mPartitionIds) {
//            i.second = remappedPartitionId[i.second];
//        }
//        goto remake_constraint_graphs;
//    }
//    END_SCOPED_REGION

    // Build a partition mapping that reflects the *original* graph C

    std::vector<Partition> partitions(PartitionCount);
    for (const auto u : kernels) {
        const auto f = mPartitionIds.find(u);
        assert (f != mPartitionIds.end());
        const auto partitionId = f->second;
        assert (partitionId < PartitionCount);
        partitions[partitionId].push_back(u);
    }

    for (const auto p : make_iterator_range(vertices(P))) {
        for (const auto i : P[p].set_bits()) {
            for (const auto j : partitions[i]) {
                const auto k = from_original(j);
                mappedPartitionId[k] = i;
            }
        }
    }

    const auto contractedKernels = contract_constraint_graph(C, orderingOfC, true);

    // Modify the vertex mapping from G to C to reflect the contracted graph C'.
    for (auto i = mapping.begin(); i != mapping.end(); ) {
        auto & to = i->second;
        if (LLVM_UNLIKELY(C[to].none())) {
            for (const auto u : make_iterator_range(vertices(C))) {
                if (C[u].test(to)) {
                    to = u;
                    goto found;
                }
            }
            llvm_unreachable("failed to locate contracted vertex?");
        }
found:  ++i;
    }

    // Construct an "ordering" for the partitions that attempts to minimize the liveness
    // of the partition streams.

    const auto solver = Z3_mk_optimize(ctx);
    Z3_optimize_inc_ref(ctx, solver);

    assert (PartitionCount <= numOfKernels);

    std::vector<Z3_ast> vars(numOfKernels);

    const auto varType = Z3_mk_int_sort(ctx);

    Z3_optimize_push(ctx, solver);

    const auto numOfContractedPartitions = (PartitionCount - contractedPartitions);

    BEGIN_SCOPED_REGION

    SmallVector<Z3_ast, 32> contractedVars(numOfContractedPartitions);

    const auto lb = Z3_mk_int(ctx, 0, varType);
    const auto ub = Z3_mk_int(ctx, numOfContractedPartitions, varType);
    for (unsigned i = 0, j = 0; i < PartitionCount; ++i) {
        if (P[i].any()) {
            const auto var = Z3_mk_fresh_const(ctx, nullptr, varType);
            Z3_optimize_assert(ctx, solver, Z3_mk_ge(ctx, var, lb));
            Z3_optimize_assert(ctx, solver, Z3_mk_lt(ctx, var, ub));
            vars[i] = var;
            contractedVars[j++] = var;
        }
    }

    // ensure no partition is assigned the same position
    Z3_optimize_assert(ctx, solver, Z3_mk_distinct(ctx, numOfContractedPartitions, contractedVars.data()));

    // create the dependency constraints
    for (const auto e : make_iterator_range(edges(P))) {
        const auto i = vars[source(e, P)]; assert (i);
        const auto j = vars[target(e, P)]; assert (j);
        Z3_optimize_assert(ctx, solver, Z3_mk_lt(ctx, i, j));
        Z3_ast args[2] = { j, i };
        const auto r = Z3_mk_sub(ctx, 2, args);
        Z3_optimize_minimize(ctx, solver, r);
    }

    // TODO: is there any advantage in one ordering over another? If the the last partition of one cluster
    // shares the same kernels as the first partition of another and we can schedule one after the other,
    // this may improve I-Cache utilization.

    if (Z3_optimize_check(ctx, solver) != Z3_L_TRUE) {
        report_fatal_error("Z3 failed to find a partition ordering solution");
    }

    END_SCOPED_REGION

    SmallVector<ConstraintGraph::vertex_descriptor, 32> partition_order(numOfContractedPartitions, -1U);

    BEGIN_SCOPED_REGION
    const auto model = Z3_optimize_get_model(ctx, solver);
    Z3_model_inc_ref(ctx, model);
    for (unsigned i = 0; i < PartitionCount; ++i) {
        Z3_ast var = vars[i];
        if (LLVM_UNLIKELY(var == nullptr)) {
            continue;
        }
        Z3_ast value;
        if (LLVM_UNLIKELY(Z3_model_eval(ctx, model, var, Z3_L_TRUE, &value) != Z3_L_TRUE)) {
            report_fatal_error("Unexpected Z3 error when attempting to obtain value from model!");
        }
        __int64 num;
        if (LLVM_UNLIKELY(Z3_get_numeral_int64(ctx, value, &num) != Z3_L_TRUE)) {
            report_fatal_error("Unexpected Z3 error when attempting to convert model value to number!");
        }
        assert (num >= 0 && num < static_cast<__int64>(numOfContractedPartitions));
        assert (partition_order[num] == -1U);
        partition_order[num] = i;
    }
    Z3_model_dec_ref(ctx, model);
    END_SCOPED_REGION

    // discard the partition solution
    Z3_optimize_pop(ctx, solver);

    // Rewrite the partition set mapping to reflect the contracted graph C
    SmallVector<BitSet, 32> nodesInPartition(PartitionCount);

    std::fill(mappedPartitionId.begin(), mappedPartitionId.end(), -1U);

    bool restart = false;
    for (const auto p : partition_order) {
        const BitSet & K = P[p];
        assert (K.any());
        for (const auto i : K.set_bits()) {
            assert (i >= 0 && i < PartitionCount);
            BitSet & nodes = nodesInPartition[i];
            nodes.resize(numOfKernels);
            assert (nodes.none());
            for (const auto j : partitions[i]) {
                const auto k = from_original(j);
                // Since the partitions may be contracted, a contracted kernel
                // could cross a partition boundary. This kernel must occur
                // in the first such partition set.
               if (LLVM_LIKELY(mappedPartitionId[k] == -1U)) {
                    assert (k >= 0 && k < numOfKernels);
                    nodes.set(k);
                    mappedPartitionId[k] = p;
               }
            }
        }
    }

    // construct the kernel verticies
    std::fill(vars.begin(), vars.end(), nullptr);
    for (unsigned i = 0; i < numOfKernels; ++i) {
        if (mappedPartitionId[i] != -1U) {
            vars[i] = Z3_mk_fresh_const(ctx, nullptr, varType);
        }
    }

    BEGIN_SCOPED_REGION

    // iterate through the ordered partitions
    unsigned start = 0;
    for (const auto p : partition_order) {
        for (const auto i : P[p].set_bits()) {
            const BitSet & nodes = nodesInPartition[i];
            const auto numOfMappedKernels = nodes.count();
            if (numOfMappedKernels > 0) {
                const auto end = start + numOfMappedKernels;
                const auto lb = Z3_mk_int(ctx, start, varType);
                // and bound each (contracted) kernel to its given partition
                if (numOfMappedKernels == 1) {
                    const auto j = nodes.find_first();
                    assert (j != -1);
                    const auto var = vars[j]; assert (var);
                    Z3_optimize_assert(ctx, solver, Z3_mk_eq(ctx, var, lb));
                } else if (numOfMappedKernels > 1) {
                    SmallVector<Z3_ast, 32> kernelVars(numOfMappedKernels);
                    const auto ub = Z3_mk_int(ctx, end, varType);
                    unsigned k = 0;
                    for (const auto j : nodes.set_bits()) {
                        const auto var = vars[j]; assert (var);
                        Z3_optimize_assert(ctx, solver, Z3_mk_ge(ctx, var, lb));
                        Z3_optimize_assert(ctx, solver, Z3_mk_lt(ctx, var, ub));
                        kernelVars[k++] = var;
                    }
                    assert (k == numOfMappedKernels);
                    // whilst ensuring no kernel is assigned the same position
                    auto distinct = Z3_mk_distinct(ctx, numOfMappedKernels, kernelVars.data());
                    Z3_optimize_assert(ctx, solver, distinct);
                }
                start = end;
            }
        }
    }
    END_SCOPED_REGION

    // create the dependency constraints

    EdgeIterator ei, ei_end;
    std::tie(ei, ei_end) = edges(C);

    while (ei != ei_end) {
        const auto e = *ei++;
        const auto i = source(e, C);
        const auto j = target(e, C);

        assert (i != j);

        // We're already guaranteed that any cross partition dependency will be
        // satisfied.
        if (mappedPartitionId[i] == mappedPartitionId[j]) {
            const auto Vi = vars[i];
            const auto Vj = vars[j];
            Z3_optimize_assert(ctx, solver, Z3_mk_lt(ctx, Vi, Vj));

            // TODO: the following needs to be reevaluated. Despite being a minimization
            // goal, even non-contradictory goals can result in


//            // Add some optimization goals to try minimize the distance between
//            // producers and consumers. By limiting the goals to only the partition
//            // we get a substantive speed-up on complex problems but a potentially
//            // worse solution.
//            Z3_ast args1[2] = { Vj, Vi };
//            const auto r = Z3_mk_sub(ctx, 2, args1);
//            Z3_optimize_minimize(ctx, solver, r);
        }


    }

    const auto numOfContractedKernels = numOfKernels - contractedKernels;

     // Now search through all kernels and try to find any with a matching signature.
    // Try to minimize the distance between such kernels. However, since a kernel
    // cannot escape its partition and each partition will be executed according to
    // an already determined ordering, rather than adding minimization goals globally,
    // only encode the rules for the current and prior partition sets. This will
    // provide the same solution as a brute force attempt with a substantial reduction
    // in the number of minimization goals.

    transitive_closure_dag(orderingOfC, C);

    BEGIN_SCOPED_REGION

    using SigGraph = adjacency_list<vecS, vecS, directedS, no_property, unsigned>;

    using SigMap = flat_map<StringRef, SigGraph::vertex_descriptor>;

    SigGraph S(numOfContractedKernels);

    BEGIN_SCOPED_REGION

    std::map<StringRef, flat_set<unsigned>> tempSet;

    SigMap M;

    unsigned partitionNum = 0;

    for (const auto p : partition_order) {
        const auto & partition = partitions[p];
        // enumerate all of the kernels in the current partition
        assert (tempSet.empty());
        for (const auto i : partition) {
            const RelationshipNode & node = mRelationships[i];
            assert (node.Type == RelationshipNode::IsKernel);
            auto sig = node.Kernel->getSignature();
            tempSet[sig].insert(from_original(i));
        }
        // then record them into the graph
        for (const auto & k : tempSet) {
            auto f = M.find(k.first);
            if (f == M.end()) {
                f = M.emplace(k.first, add_vertex(S)).first;
            }
            const auto v = f->second;
            for (const auto u : k.second) {
                add_edge(v, u, partitionNum, S);
            }
        }
        tempSet.clear();
        ++partitionNum;
    }

    END_SCOPED_REGION

    // now iterate through the graph and check whether we have any "same kernel"
    // minimization goals to encode.

    const auto n = num_vertices(S);

    using SigSet = SmallVector<unsigned, 2>;

    SigSet priorSet;
    SigSet currentSet;

    for (auto sigSet = numOfContractedKernels; sigSet < n; ++sigSet) {

        unsigned partitionId = -1U;

        auto addSigConstraints = [&]() {

            const auto n = currentSet.size();
            for (unsigned i = 1; i < n; ++i) {
                const auto u = currentSet[i];
                const auto X = vars[u];
                for (unsigned j = 0; j != i; ++j) {
                    const auto v = currentSet[j];
                    const auto Y = vars[v];

                    // minimize the absolute diff between X and Y but take the
                    // graph structure into account to simplify the formula
                    if (edge(u, v, C).second) {
                        // u dominates v ⊢ X < Y
                        Z3_ast args1[2] = { Y, X };
                        const auto r = Z3_mk_sub(ctx, 2, args1);
                        Z3_optimize_minimize(ctx, solver, r);
                    } else if (edge(v, u, C).second) {
                        // v dominates u ⊢ Y < X
                        Z3_ast args2[2] = { X, Y };
                        const auto r = Z3_mk_sub(ctx, 2, args2);
                        Z3_optimize_minimize(ctx, solver, r);
                    } else { // neither x nor y dominate the other
                        Z3_ast args1[2] = { X, Y };
                        const auto XmY = Z3_mk_sub(ctx, 2, args1);
                        Z3_ast args2[2] = { Y, X };
                        const auto YmX = Z3_mk_sub(ctx, 2, args2);
                        const auto c = Z3_mk_gt(ctx, X, Y);
                        const auto r = Z3_mk_ite(ctx, c, XmY, YmX);
                        Z3_optimize_minimize(ctx, solver, r);
                    }
                }
            }
            const auto m = priorSet.size();
            for (unsigned j = 0; j < m; ++j) {
                const auto v = priorSet[j];
                const auto Y = vars[v];
                for (unsigned i = 0; i != n; ++i) {
                    const auto u = currentSet[i];
                    const auto X = vars[u];
                    // PARTITION(u) dominates PARTITION(v) ⊢ X < Y
                    Z3_ast args1[2] = { Y, X };
                    const auto r = Z3_mk_sub(ctx, 2, args1);
                    Z3_optimize_minimize(ctx, solver, r);
                }
            }

        };

        for (const auto e : make_iterator_range(out_edges(sigSet, S))) {
            const unsigned currentPartitionId = S[e];
            if (currentPartitionId != partitionId) {
                addSigConstraints();
                assert (currentPartitionId > partitionId || partitionId == -1U);
                if ((currentPartitionId - partitionId) > 1) {
                    priorSet.clear();
                } else {
                    priorSet.swap(currentSet);
                }
                currentSet.clear();
            }
            currentSet.push_back(target(e, S));
        }
        addSigConstraints();
        priorSet.clear();
        currentSet.clear();

    }

    END_SCOPED_REGION

    if (Z3_optimize_check(ctx, solver) == Z3_L_FALSE) {
        report_fatal_error("Z3 failed to find a kernel ordering solution");
    }

    std::vector<unsigned> ordering(numOfContractedKernels);
    #ifndef NDEBUG
    std::vector<__int64> test(numOfKernels);
    #endif

    #ifndef NDEBUG
    std::fill_n(ordering.begin(), numOfContractedKernels, -1U);
    #endif
    const auto model = Z3_optimize_get_model(ctx, solver);
    Z3_model_inc_ref(ctx, model);
    for (unsigned i = 0; i < numOfKernels; ++i) {
        Z3_ast var = vars[i];
        if (LLVM_UNLIKELY(var == nullptr)) {
            continue;
        }
        Z3_ast value;
        if (LLVM_UNLIKELY(Z3_model_eval(ctx, model, var, Z3_L_TRUE, &value) != Z3_L_TRUE)) {
            report_fatal_error("Unexpected Z3 error when attempting to obtain value from model!");
        }
        __int64 num;
        if (LLVM_UNLIKELY(Z3_get_numeral_int64(ctx, value, &num) != Z3_L_TRUE)) {
            report_fatal_error("Unexpected Z3 error when attempting to convert model value to number!");
        }
        assert (num >= 0 && num < static_cast<__int64>(numOfContractedKernels));
        assert (ordering[num] == -1U);
        ordering[num] = i;
        #ifndef NDEBUG
        test[i] = num;
        #endif
    }
    Z3_model_dec_ref(ctx, model);

    #ifndef NDEBUG
    for (const auto e : make_iterator_range(edges(C))) {
        const auto u = source(e, C);
        const auto v = target(e, C);
        assert (test[u] < test[v]);
    }
    #endif

    Z3_optimize_dec_ref(ctx, solver);
    Z3_del_context(ctx);
    Z3_reset_memory();

    // Add the final constraints to the graph
    const auto end = ordering.end();
    for (auto i = ordering.begin();; ) {

        assert (*i != -1U);

        const BitSet & A = C[*i];
        assert (A.any());

        auto a = A.set_bits_begin();
        const auto a_end = A.set_bits_end();

        auto u = to_original(*a);
        while (++a != a_end) {
            const auto v = to_original(*a);
            add_edge(u, v, RelationshipType{ReasonType::OrderingConstraint}, mRelationships);
            u = v;
        }

        if ((++i) == end) break;

        assert (*i != -1U);

        const BitSet & B = C[*i];
        assert (B.any());
        const auto b = B.set_bits_begin();
        const auto v = to_original(*b);
        add_edge(u, v, RelationshipType{ReasonType::OrderingConstraint}, mRelationships);
    }

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
    using Vertex = Graph::vertex_descriptor;

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
    ReverseTopologicalOrdering ordering(PartitionCount);
    std::iota(ordering.rbegin(), ordering.rend(), 0);
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
        M.set(0, PartitionCount, true);
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
