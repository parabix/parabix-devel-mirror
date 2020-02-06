#ifndef PARTITIONING_ANALYSIS_HPP
#define PARTITIONING_ANALYSIS_HPP

#include "../pipeline_compiler.hpp"
#include "../internal/partitionmetakernel.h"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief partitionIntoFixedRateRegionWithOrderingConstraints
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned PipelineCompiler::partitionIntoFixedRateRegionsWithOrderingConstraints(Relationships & G,
                                                                                std::vector<unsigned> & partitionIds,
                                                                                const PipelineKernel * const pipelineKernel) const {
    std::vector<unsigned> orderingOfG;
    orderingOfG.reserve(num_vertices(G));
    if (LLVM_UNLIKELY(!lexical_ordering(G, orderingOfG))) {
        report_fatal_error("Cannot lexically order the initial pipeline graph!");
    }


    const auto partitions = identifyKernelPartitions(G, orderingOfG, partitionIds, pipelineKernel);
    addOrderingConstraintsToPartitionSubgraphs(G, orderingOfG, partitionIds, partitions);
    return partitions.size();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief identifyKernelPartitions
 ** ------------------------------------------------------------------------------------------------------------- */
std::vector<Partition> PipelineCompiler::identifyKernelPartitions(const Relationships & G,
                                                                  const std::vector<unsigned> & orderingOfG,
                                                                  std::vector<unsigned> & partitionIds,
                                                                  const PipelineKernel * const pipelineKernel) const {

    using BitSet = dynamic_bitset<>;

    using PartitionTaintGraph = adjacency_list<vecS, vecS, bidirectionalS, BitSet>;
    using Vertex = PartitionTaintGraph::vertex_descriptor;

    using BufferAttributeMap = flat_map<std::pair<Vertex, unsigned>, Vertex>;
    using PartialSumMap = flat_map<Vertex, Vertex>;

    using Partition = std::vector<unsigned>;
    using PartitionMap = std::map<BitSet, unsigned>;

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

    PartitionTaintGraph H(kernels + streamSets);

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

    auto addRateId = [](BitSet & bv, const unsigned rateId) {
       if (LLVM_UNLIKELY(rateId >= bv.capacity())) {
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

    std::vector<Relationships::vertex_descriptor> mappedKernel(kernels);

    // Begin by constructing a graph that represents the I/O relationships
    // and any partition boundaries.
    for (const auto u : orderingOfG) {

        const RelationshipNode & node = G[u];

        if (node.Type == RelationshipNode::IsKernel) {

            const auto kernel = nextKernel++;
            assert (kernel < kernels);
            mappedKernel[kernel] = u;
            assert (H[kernel].none());

            const Kernel * const kernelObj = node.Kernel;
            if (kernelObj == pipelineKernel) {
                continue;
            }

            // place the pipeline source/sink into a new (isolated) partition
            bool isNewPartitionRoot = false;

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

                    add_edge(buffer, kernel, H);
                }
            }

            // Check whether any of the kernel attributes require this to be a partition root
            for (const Attribute & attr : kernelObj->getAttributes()) {
                switch (attr.getKind()) {
                    case AttrId::CanTerminateEarly:
                    case AttrId::MustExplicitlyTerminate:
                    case AttrId::MayFatallyTerminate:
                        isNewPartitionRoot = true;
                        break;
                    default:
                        break;
                }
            }

            // Assign a root of a partition a new id.
            if (LLVM_UNLIKELY(isNewPartitionRoot)) {
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
                // place each input source kernel in its own partition
                addRateId(H[kernel], nextRateId++);
                if (LLVM_LIKELY(out_degree(kernel, H) != 0)) {
                    const auto sourceKernelRateId = nextRateId++;
                    for (const auto output : make_iterator_range(out_edges(u, H))) {
                        const auto buffer = target(output, H);
                        addRateId(H[buffer], sourceKernelRateId);
                    }
                }
            }
        }
    }

    assert (nextKernel == kernels);
    assert (nextStreamSet == streamSets + kernels);

    // Combine all incoming rates sets
    std::vector<Vertex> orderingOfH;
    orderingOfH.reserve(num_vertices(H));
    if (LLVM_UNLIKELY(!lexical_ordering(H, orderingOfH))) {
        report_fatal_error("Cannot lexically order the partition graph!");
    }

    for (const auto u : orderingOfH) {
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

    partitionIds.resize(num_vertices(G), 0);

    BEGIN_SCOPED_REGION

    PartitionMap partitionSets;

    // Now that we've tainted the kernels with any influencing rates,
    // cluster them into partitions.
    for (const auto u : orderingOfH) {
        // We want to obtain all kernels but still maintain the lexical ordering of each partition root
        if (u < kernels) {
            BitSet & node = H[u];
            const auto k = mappedKernel[u];
            assert(node.none() ^ (G[k].Kernel != pipelineKernel));
            if (LLVM_UNLIKELY(node.none())) {
                partitionIds[k] = -1U;
            } else {
                auto f = partitionSets.find(node);
                if (f == partitionSets.end()) {
                    const auto i = partitions.size();
                    partitions.emplace_back();
                    f = partitionSets.emplace(std::move(node), i).first;
                }
                Partition & P = partitions[f->second];
                P.push_back(k);
            }
        }
    }

    END_SCOPED_REGION

    // Then provide each partition with a unique id
    unsigned partitionId = 0U;
    for (const Partition & A : partitions) {
        for (const auto v : A) {
            partitionIds[v] = partitionId;
        }
        ++partitionId;
    }
    assert (partitionId == partitions.size());
    return partitions;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief extractPartitionSubgraphs
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::addOrderingConstraintsToPartitionSubgraphs(Relationships & G,
                                                                  const std::vector<unsigned> & orderingOfG,
                                                                  const std::vector<unsigned> & partitionIds,
                                                                  const std::vector<Partition> & partitions) const {

    using BitSet = BitVector;

    using ConstraintGraph = adjacency_list<hash_setS, vecS, bidirectionalS, BitSet>;
    using EdgeIterator = graph_traits<ConstraintGraph>::edge_iterator;

    unsigned numOfKernels = 0;
    for (const Partition & P : partitions) {
        numOfKernels += P.size();
    }

    flat_map<unsigned, unsigned> mapping;
    mapping.reserve(numOfKernels);

    auto from_original =[&](const unsigned u) {
        const auto f = mapping.find(u); assert (f != mapping.end());
        return f->second;
    };

    flat_map<unsigned, unsigned> reverse_mapping;

    auto to_original =[&](const unsigned u) {
        const auto f = reverse_mapping.find(u); assert (f != reverse_mapping.end());
        return f->second;
    };

    BEGIN_SCOPED_REGION

    unsigned i = 0;
    for (const auto u : orderingOfG) {
        const RelationshipNode & node = G[u];
        switch (node.Type) {
            case RelationshipNode::IsKernel:
                if (LLVM_LIKELY(partitionIds[u] != -1U)) {
                    mapping.emplace(u, i++);
                }
            default: break;
        }
    }

    assert (i == numOfKernels);

    // pregenerate the reverse mapping for when we translate the
    // constraint graph back to the original graph G.
    reverse_mapping.reserve(numOfKernels);
    for (auto a : mapping) {
        reverse_mapping.emplace(a.second, a.first);
    }

    END_SCOPED_REGION

    const auto numOfPartitions = partitions.size();
    ConstraintGraph P(numOfPartitions);
    ConstraintGraph C(numOfKernels);

    flat_set<unsigned> inputs;

    for (const auto p : mapping) {
        const auto kernel = p.first;
        const auto partitionId = partitionIds[kernel];
        assert (partitionId != -1U);
        assert (inputs.empty());
        assert (G[kernel].Type == RelationshipNode::IsKernel);

        // enumerate the input constraints
        for (const auto input : make_iterator_range(in_edges(kernel, G))) {
            const auto node = source(input, G);
            const RelationshipNode & rn = G[node];
            if (rn.Type == RelationshipNode::IsBinding) {
                const auto f = first_in_edge(node, G);
                assert (G[f].Reason != ReasonType::Reference);
                const auto streamSet = source(f, G);
                assert (G[streamSet].Type == RelationshipNode::IsRelationship);
                assert (isa<StreamSet>(G[streamSet].Relationship));
                const auto outputBinding = parent(streamSet, G);
                assert (G[outputBinding].Type == RelationshipNode::IsBinding);
                const auto output = first_in_edge(outputBinding, G);
                assert (G[output].Reason != ReasonType::Reference);
                const auto producer = source(output, G);
                assert (G[producer].Type == RelationshipNode::IsKernel);
                inputs.insert(producer);
            } else if (LLVM_UNLIKELY(rn.Type == RelationshipNode::IsKernel)) {
                assert (G[input].Reason == ReasonType::OrderingConstraint);
                if (partitionIds[node] != -1U) {
                    inputs.insert(node);
                }
            }
        }

        // then write them and any summarized partition constraints
        const auto u = p.second;
        for (const auto input : inputs) {
            const auto v = from_original(input);
            add_edge(v, u, C);
            const auto producerPartitionId = partitionIds[input];
            assert (producerPartitionId != -1U);
            if (partitionId != producerPartitionId) {
                add_edge(producerPartitionId, partitionId, P);
            }
        }

        inputs.clear();

    }

    // Take the transitive reduction of the constraint graphs to reduce the number of
    // constraints we'll add to the formula.

    ReverseTopologicalOrdering orderingOfP;
    orderingOfP.reserve(numOfPartitions);
    ReverseTopologicalOrdering orderingOfC;
    orderingOfC.reserve(numOfKernels);
    topological_sort(P, std::back_inserter(orderingOfP));
    topological_sort(C, std::back_inserter(orderingOfC));

    transitive_closure_dag(orderingOfP, P);
    transitive_closure_dag(orderingOfC, C);

    transitive_reduction_dag(orderingOfP, P);
    transitive_reduction_dag(orderingOfC, C);

    // Then contract any chains into a single vertex to both simplify the SMT formula
    // in a way that reflects an optimal answer and prevent any "false" breaks later
    // in the partition skipping logic.

    std::vector<unsigned> mappedPartitionId(numOfKernels);

    auto contract_constraint_graph = [&mappedPartitionId](ConstraintGraph & H, const ReverseTopologicalOrdering & O, const bool checkPartition) {

        assert (num_vertices(H) == O.size());

        const auto m = O.size();

        for (unsigned i = 0; i < m; ++i) {
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

    // Build a partition mapping that reflects the *original* graph C

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

    const auto cfg = Z3_mk_config();
    Z3_set_param_value(cfg, "model", "true");
    Z3_set_param_value(cfg, "proof", "false");
    const auto ctx = Z3_mk_context(cfg);
    Z3_del_config(cfg);
    const auto solver = Z3_mk_optimize(ctx);
    Z3_optimize_inc_ref(ctx, solver);

    assert (numOfPartitions <= numOfKernels);

    std::vector<Z3_ast> vars(numOfKernels);

    const auto varType = Z3_mk_int_sort(ctx);

    Z3_optimize_push(ctx, solver);

    const auto numOfContractedPartitions = (numOfPartitions - contractedPartitions);

    BEGIN_SCOPED_REGION

    SmallVector<Z3_ast, 32> contractedVars(numOfContractedPartitions);

    const auto lb = Z3_mk_int(ctx, 0, varType);
    const auto ub = Z3_mk_int(ctx, numOfContractedPartitions, varType);
    for (unsigned i = 0, j = 0; i < numOfPartitions; ++i) {
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
    for (unsigned i = 0; i < numOfPartitions; ++i) {
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
    SmallVector<BitSet, 32> nodesInPartition(numOfPartitions);

    std::fill(mappedPartitionId.begin(), mappedPartitionId.end(), -1U);

    for (const auto p : partition_order) {
        const BitSet & K = P[p];
        assert (K.any());
        for (const auto i : K.set_bits()) {
            assert (i >= 0 && i < numOfPartitions);
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
    SmallVector<Z3_ast, 32> kernelVars(32);
    unsigned start = 0;

    for (const auto p : partition_order) {

        for (const auto i : P[p].set_bits()) {

            const BitSet & nodes = nodesInPartition[i];
            const auto numOfMappedKernels = nodes.count();
            if (numOfMappedKernels == 0) continue;

            const auto end = start + numOfMappedKernels;
            const auto lb = Z3_mk_int(ctx, start, varType);
            const auto ub = Z3_mk_int(ctx, end, varType);
            if (LLVM_UNLIKELY(kernelVars.size() < numOfMappedKernels)) {
                kernelVars.resize(numOfMappedKernels);
            }
            // and bound each (contracted) kernel to its given partition
            if (numOfMappedKernels == 1) {
                const auto j = nodes.find_first();
                assert (j != -1);
                const auto var = vars[j]; assert (var);
                Z3_optimize_assert(ctx, solver, Z3_mk_eq(ctx, var, lb));
            } else {
                unsigned k = 0;
                for (const auto j : nodes.set_bits()) {
                    const auto var = vars[j]; assert (var);
                    Z3_optimize_assert(ctx, solver, Z3_mk_ge(ctx, var, lb));
                    Z3_optimize_assert(ctx, solver, Z3_mk_lt(ctx, var, ub));
                    kernelVars[k++] = var;
                }
                assert (k == numOfMappedKernels);
                // whilst ensuring no kernel is assigned the same position
                Z3_optimize_assert(ctx, solver, Z3_mk_distinct(ctx, numOfMappedKernels, kernelVars.data()));
            }
            start = end;

        }
    }
    END_SCOPED_REGION

    // create the dependency constraints

    EdgeIterator ei, ei_end;
    std::tie(ei, ei_end) = edges(C);

    while (ei != ei_end) {
        const auto e = *ei++;
        const auto u = source(e, C);
        const auto v = target(e, C);

        const auto P = vars[u];
        const auto C = vars[v];

        const auto pu = mappedPartitionId[u];
        const auto pv = mappedPartitionId[v];
        // We're already guaranteed that any cross partition dependency will be
        // satisfied.
        if (pu == pv) {
            Z3_optimize_assert(ctx, solver, Z3_mk_lt(ctx, P, C));
        }

        // Add some optimization goals to try minimize the distance between
        // producers and consumers. By limiting the goals to only the partition
        // we get a substantive speed-up on complex problems but a potentially
        // worse solution.
        Z3_ast args[2] = { C, P };
        const auto r = Z3_mk_sub(ctx, 2, args);
        Z3_optimize_minimize(ctx, solver, r);

    }

    const auto numOfContractedKernels = numOfKernels - contractedKernels;

    BEGIN_SCOPED_REGION

    // Now search through all kernels and try to find any with a matching signature.
    // Try to minimize the distance between such kernels. However, since a kernel
    // cannot escape its partition and each partition will be executed according to
    // an already determined ordering, rather than adding minimization goals globally,
    // only encode the rules for the current and prior partition sets. This will
    // provide the same solution as a brute force attempt with a substantial reduction
    // in the number of minimization goals.

    transitive_closure_dag(orderingOfC, C);

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
            const RelationshipNode & node = G[i];
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

    if (Z3_optimize_check(ctx, solver) != Z3_L_TRUE) {
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

    Z3_finalize_memory();

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
            add_edge(u, v, RelationshipType{ReasonType::OrderingConstraint}, G);
            u = v;
        }

        if ((++i) == end) break;

        assert (*i != -1U);

        const BitSet & B = C[*i];
        assert (B.any());
        const auto b = B.set_bits_begin();
        const auto v = to_original(*b);
        add_edge(u, v, RelationshipType{ReasonType::OrderingConstraint}, G);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generatePartitioningGraph
 *
 * The partitioning graph summarizes the buffer graph to indicate what dynamic buffers must be tested /
 * potentially expanded to satisfy the dataflow requirements of each partition.
 ** ------------------------------------------------------------------------------------------------------------- */
PartitioningGraph PipelineCompiler::generatePartitioningGraph() const {

    using BufferVertex = BufferGraph::vertex_descriptor;
    using Vertex = PartitioningGraph::vertex_descriptor;
//    using Edge = graph_traits<PartitioningGraph>::edge_descriptor;
    using StreamSetMap = flat_map<BufferVertex, Vertex>;

    struct Node {
        enum TypeId {
            Partition = 0
            , Fixed
            , Bounded
            , Unknown
            , PartialSum
            , Relative
            , Greedy
        };

        TypeId Type = TypeId::Partition;
    };

    struct EdgeType {
        unsigned        Kernel = 0;
        StreamSetPort   Port;

        EdgeType() = default;
        EdgeType(unsigned kernel, StreamSetPort port) : Kernel(kernel), Port(port) { }
    };


    using Graph = adjacency_list<vecS, vecS, bidirectionalS, Node, EdgeType>;

    const auto firstKernel = out_degree(PipelineInput, mBufferGraph) == 0 ? FirstKernel : PipelineInput;
    const auto lastKernel = in_degree(PipelineOutput, mBufferGraph) == 0 ? LastKernel : PipelineOutput;

    assert (PartitionCount > 0);
    assert (KernelPartitionId[firstKernel] == 0U);
    assert ((KernelPartitionId[lastKernel] + 1) == PartitionCount);
    assert (PartitionCount < FirstStreamSet);

    Graph G(PartitionCount);

    StreamSetMap streamSetMap;

    flat_set<Vertex> fixedRateSet;

    flat_set<BufferVertex> visited;

    auto priorPartitionId = 0U;

    for (auto start = firstKernel; start <= lastKernel; ) {
        // Determine which kernels are in this partition
        const auto partitionId = KernelPartitionId[start];
        assert (priorPartitionId <= partitionId);
        assert (partitionId < PartitionCount);
        auto end = start + 1U;
        for (; end <= LastKernel; ++end) {
            if (KernelPartitionId[end] != partitionId) {
                break;
            }
        }

        errs() << "PARTITION: " << partitionId << "   [" << start << ',' << end << ")\n";

        priorPartitionId = partitionId;

        Node & N = G[partitionId];
        N.Type = Node::Partition;

        // Each fixed rate of a partition is guaranteed to be consumed/produced at some mutually relative rate.
        bool makeFixedOutput = true;
        Vertex fixedOutput = 0;

        for (auto kernel = start; kernel < end; ++kernel) {

            auto makeNode = [&](const Node::TypeId typeId) {
                const auto v = add_vertex(G);
                Node & N = G[v];
                N.Type = typeId;
                return v;
            };

            auto makeNodeWithReference = [&](const Node::TypeId typeId, const StreamSetPort port) {
//                const auto reference = getReferenceBufferVertex(kernel, port);
//                const auto f = streamSetMap.find(reference);
//                assert (f != streamSetMap.end());
//                const auto u = f->second;
                const auto v = makeNode(typeId);
//                 add_edge(u, v, G);
                return v;
            };

            for (const auto input : make_iterator_range(in_edges(kernel, mBufferGraph))) {
                const auto streamSet = source(input, mBufferGraph);
                const BufferNode & bn = mBufferGraph[streamSet];
                if (LLVM_UNLIKELY(bn.NonLocal)) {

                    const auto f = streamSetMap.find(streamSet);
                    assert (f != streamSetMap.end());
                    const auto src = f->second;

                    if (visited.insert(src).second) {
                        const BufferRateData & inputRate = mBufferGraph[input];
                        const Binding & binding = inputRate.Binding;
                        const ProcessingRate & rate = binding.getRate();
                        const auto rateId = rate.getKind();

                        assert (src != fixedOutput);

                        if (rateId == RateId::Fixed) {

                            if (fixedRateSet.insert(src).second) {
                                const auto fixedInput = makeNode(Node::Fixed);
                                add_edge(src, fixedInput, G);
                                add_edge(fixedInput, partitionId, EdgeType{kernel, inputRate.Port}, G);
                            }

                        } else {

                            Vertex input;
                            switch (rateId) {
                                case RateId::Bounded:
                                    input = makeNode(Node::Bounded);
                                    break;
                                case RateId::PartialSum:
                                    input = makeNodeWithReference(Node::PartialSum, inputRate.Port);
                                    break;
                                case RateId::Relative:
                                    input = makeNodeWithReference(Node::Relative, inputRate.Port);
                                    break;
                                case RateId::Greedy:
                                    input = makeNode(Node::Greedy);
                                    break;
                                default: llvm_unreachable("unhandled input rate type");
                            }

                            add_edge(src, input, G);
                            add_edge(input, partitionId, EdgeType{kernel, inputRate.Port}, G);
                        }
                    }
                }
            }

            for (const auto output : make_iterator_range(out_edges(kernel, mBufferGraph))) {
                const auto streamSet = target(output, mBufferGraph);
                const BufferNode & bn = mBufferGraph[streamSet];
                if (LLVM_UNLIKELY(bn.NonLocal)) {

                    const BufferRateData & outputRate = mBufferGraph[output];
                    const Binding & binding = outputRate.Binding;
                    const ProcessingRate & rate = binding.getRate();
                    const auto rateId = rate.getKind();

                    visited.insert(streamSet);

                    if (rateId == RateId::Fixed) {
                        if  (makeFixedOutput) {
                            fixedOutput = makeNode(Node::Fixed);
                            add_edge(partitionId, fixedOutput, EdgeType{}, G);
                            makeFixedOutput = false;
                        }
                        streamSetMap.emplace(streamSet, fixedOutput);

                    } else {
                        Vertex output;
                        switch (rateId) {
                            case RateId::Unknown:
                                output = makeNode(Node::Unknown);
                                break;
                            case RateId::Bounded:
                                output = makeNode(Node::Bounded);
                                break;
                            case RateId::PartialSum:
                                output = makeNodeWithReference(Node::PartialSum, outputRate.Port);
                                break;
                            case RateId::Relative:
                                output = makeNodeWithReference(Node::Relative, outputRate.Port);
                                break;
                            default: llvm_unreachable("unhandled output rate type");
                        }
                        streamSetMap.emplace(streamSet, output);
                        add_edge(partitionId, output, EdgeType{kernel, outputRate.Port}, G);
                    }
                }
            }
        }
        visited.clear();
        fixedRateSet.clear();
        start = end;
    }


    auto & out = errs();

    out << "digraph \"G\" {\n";
    for (auto v : make_iterator_range(vertices(G))) {
        out << "v" << v << " [label=\"" << v << ": ";
        const Node & N = G[v];

        switch (N.Type) {
            case Node::Partition:
                out << "Partition";
                break;
            case Node::Fixed:
                out << "Fixed";
                break;
            case Node::Bounded:
                out << "Bounded";
                break;
            case Node::Unknown:
                out << "Unknown";
                break;
            case Node::PartialSum:
                out << "PartialSum";
                break;
            case Node::Relative:
                out << "Relative";
                break;
            case Node::Greedy:
                out << "Greedy";
                break;
            default: llvm_unreachable("unhandled node type");
        }

        out << "\"];\n";
    }
    for (auto e : make_iterator_range(edges(G))) {
        const auto s = source(e, G);
        const auto t = target(e, G);
        out << "v" << s << " -> v" << t;
        if (s < PartitionCount || t < PartitionCount) {
            out << " [label=\"";
            const EdgeType & E = G[e];
            out << "[" << E.Kernel << "] #";
            if (E.Port.Type == PortType::Input) {
                out << 'I';
            } else {
                out << 'O';
            }
            out << ':' << E.Port.Number << "\"]";
        }
        out << ";\n";
    }

    out << "}\n\n";
    out.flush();




    PartitioningGraph H;

    return H;
}

#if 0

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

//    assert (KernelPartitionId[firstKernel] == 0);
//    assert ((KernelPartitionId[lastKernel] + 1) == PartitionCount);

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

        G[partitionId] = start;
        assert (degree(partitionId, G) == 0);

        for (auto kernel = start; kernel < end; ++kernel) {

            const auto minStrides = MinimumNumOfStrides[kernel];

            for (const auto input : make_iterator_range(in_edges(kernel, mBufferGraph))) {
                const auto streamSet = source(input, mBufferGraph);
                const BufferNode & bn = mBufferGraph[streamSet];
                if (LLVM_UNLIKELY(bn.NonLocal)) {
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
                if (LLVM_UNLIKELY(bn.NonLocal)) {
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

#endif

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief determinePartitionJumpIndices
 *
 * If a partition determines it has insufficient data to execute, identify which partition is the next one to test.
 * I.e., the one with input from some disjoint path. If none exists, we'll begin jump to "PartitionCount", which
 * marks the end of the processing loop.
 ** ------------------------------------------------------------------------------------------------------------- */
std::vector<unsigned> PipelineCompiler::determinePartitionJumpIndices() const {

#if 0

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
    // produced no new data along this path.

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

#endif
    std::vector<unsigned> jumpIndices(PartitionCount);
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
