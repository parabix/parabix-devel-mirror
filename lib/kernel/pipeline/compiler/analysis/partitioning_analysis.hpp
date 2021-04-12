#ifndef PARTITIONING_ANALYSIS_HPP
#define PARTITIONING_ANALYSIS_HPP

#include "pipeline_analysis.hpp"
#include <toolchain/toolchain.h>
#include <util/slab_allocator.h>

namespace kernel {

void PipelineAnalysis::addKernelRelationshipsInReferenceOrdering(const unsigned kernel, const RelationshipGraph & G,
                                                                 std::function<void(PortType, unsigned, unsigned)> insertionFunction) {

    using Graph = adjacency_list<hash_setS, vecS, bidirectionalS, RelationshipGraph::edge_descriptor>;
    using Vertex = graph_traits<Graph>::vertex_descriptor;

    const RelationshipNode & node = G[kernel];
    assert (node.Type == RelationshipNode::IsKernel);
    const Kernel * const kernelObj = node.Kernel; assert (kernelObj);

    unsigned maxInputPort = -1U;
    for (auto e : reverse(make_iterator_range(in_edges(kernel, G)))) {
        const auto binding = source(e, G);
        const RelationshipNode & rn = G[binding];
        if (LLVM_LIKELY(rn.Type == RelationshipNode::IsBinding)) {
            const RelationshipType & port = G[e];
            maxInputPort = port.Number;
            break;
        }
    }
    const auto numOfInputs = maxInputPort + 1U;

    unsigned maxOutputPort = -1U;
    for (auto e : reverse(make_iterator_range(out_edges(kernel, G)))) {
        const auto binding = target(e, G);
        const RelationshipNode & rn = G[binding];
        if (LLVM_LIKELY(rn.Type == RelationshipNode::IsBinding)) {
            const RelationshipType & port = G[e];
            maxOutputPort = port.Number;
            break;
        }
    }
    const auto numOfOutputs = maxOutputPort + 1U;

    // Evaluate the input/output ordering here and ensure that any reference port is stored first.
    const auto numOfPorts = numOfInputs + numOfOutputs;

    if (LLVM_UNLIKELY(numOfPorts == 0)) {
        return;
    }

    Graph E(numOfPorts);

    for (auto e : make_iterator_range(in_edges(kernel, G))) {
        const auto binding = source(e, G);
        const RelationshipNode & rn = G[binding];
        if (LLVM_LIKELY(rn.Type == RelationshipNode::IsBinding)) {
            const RelationshipType & port = G[e];
            assert (port.Number < numOfInputs);
            E[port.Number] = e;
            if (LLVM_UNLIKELY(in_degree(binding, G) != 1)) {
                for (const auto f : make_iterator_range(in_edges(binding, G))) {
                    const RelationshipType & ref = G[f];
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

    }

    for (auto e : make_iterator_range(out_edges(kernel, G))) {
        const auto binding = target(e, G);
        const RelationshipNode & rn = G[binding];
        if (LLVM_LIKELY(rn.Type == RelationshipNode::IsBinding)) {
            const RelationshipType & port = G[e];
            assert (port.Number < numOfOutputs);
            const auto portNum = port.Number + numOfInputs;
            E[portNum] = e;
            if (LLVM_UNLIKELY(in_degree(binding, G) != 1)) {
                for (const auto f : make_iterator_range(in_edges(binding, G))) {
                    const RelationshipType & ref = G[f];
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
    }

//    BitVector V(numOfPorts);
//    std::queue<Vertex> Q;

//    auto add_edge_if_no_induced_cycle = [&](const Vertex s, const Vertex t) {
//        // If s-t exists, skip adding this edge
//        if (LLVM_UNLIKELY(edge(s, t, E).second || s == t)) {
//            return;
//        }

//        // If G is a DAG and there is a t-s path, adding s-t will induce a cycle.
//        if (in_degree(s, E) > 0) {
//            // do a BFS to search for a t-s path
//            V.reset();
//            assert (Q.empty());
//            Q.push(t);
//            for (;;) {
//                const auto u = Q.front();
//                Q.pop();
//                for (auto e : make_iterator_range(out_edges(u, E))) {
//                    const auto v = target(e, E);
//                    if (LLVM_UNLIKELY(v == s)) {
//                        // we found a t-s path
//                        return;
//                    }
//                    if (LLVM_LIKELY(!V.test(v))) {
//                        V.set(v);
//                        Q.push(v);
//                    }
//                }
//                if (Q.empty()) {
//                    break;
//                }
//            }
//        }
//        add_edge(s, t, E);
//    };

//    for (unsigned j = 1; j < numOfPorts; ++j) {
//        add_edge_if_no_induced_cycle(j - 1, j);
//    }

    SmallVector<Graph::vertex_descriptor, 16> ordering;
    ordering.reserve(numOfPorts);
    lexical_ordering(E, ordering);
    assert (ordering.size() == numOfPorts);

    for (const auto k : ordering) {
        const auto e = E[k];
        const RelationshipType & port = G[e];
        if (port.Type == PortType::Input) {
            const auto binding = source(e, G);
            const RelationshipNode & rn = G[binding];
            assert(rn.Type == RelationshipNode::IsBinding);
            const auto f = first_in_edge(binding, G);
            assert (G[f].Reason != ReasonType::Reference);
            const auto streamSet = source(f, G);
            assert (G[streamSet].Type == RelationshipNode::IsRelationship);
            assert (isa<StreamSet>(G[streamSet].Relationship));
            insertionFunction(PortType::Input, binding, streamSet);
        } else {
            const auto binding = target(e, G);
            const RelationshipNode & rn = G[binding];
            assert(rn.Type == RelationshipNode::IsBinding);
            const auto f = out_edge(binding, G);
            assert (G[f].Reason != ReasonType::Reference);
            const auto streamSet = target(f, G);
            assert (G[streamSet].Type == RelationshipNode::IsRelationship);
            assert (isa<StreamSet>(G[streamSet].Relationship));
            insertionFunction(PortType::Output, binding, streamSet);
        }
    }

}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief identifyKernelPartitions
 ** ------------------------------------------------------------------------------------------------------------- */
PartitionGraph PipelineAnalysis::identifyKernelPartitions() {

    using BitSet = dynamic_bitset<>;

    using BindingVertex = RelationshipGraph::vertex_descriptor;

    using Graph = adjacency_list<vecS, vecS, bidirectionalS, BitSet, BindingVertex>;

    using PartitionMap = std::map<BitSet, unsigned>;

    const unsigned n = num_vertices(Relationships);

    std::vector<unsigned> sequence;
    sequence.reserve(n);

    std::vector<unsigned> mapping(n, -1U);

//    // We're not guaranteed that our equal-length streamsets are in
//    // the penultimate relationship graph.
//    flat_map<const StreamSet *, unsigned> lengthAssertedStreamSet;
//    for (const LengthAssertion & a : mLengthAssertions) {
//        lengthAssertedStreamSet.emplace(a[0], -1U);
//        lengthAssertedStreamSet.emplace(a[1], -1U);
//    }

    BEGIN_SCOPED_REGION

    std::vector<unsigned> ordering;
    if (LLVM_UNLIKELY(!lexical_ordering(Relationships, ordering))) {
        report_fatal_error("Failed to generate acyclic partition graph from kernel ordering");
    }

    // Convert the relationship graph into a simpler graph G that we can annotate

    for (unsigned u : ordering) {
        const RelationshipNode & node = Relationships[u];
        switch (node.Type) {
            case RelationshipNode::IsKernel:
                mapping[u] = sequence.size();
                sequence.push_back(u);
                break;
            case RelationshipNode::IsRelationship:
                BEGIN_SCOPED_REGION
                const Relationship * const ss = Relationships[u].Relationship;
                if (LLVM_LIKELY(isa<StreamSet>(ss))) {
//                    const auto f = lengthAssertedStreamSet.find(cast<StreamSet>(ss));
//                    if (LLVM_UNLIKELY(f != lengthAssertedStreamSet.end())) {
//                        f->second = sequence.size();
//                    }
                    mapping[u] = sequence.size();
                    sequence.push_back(u);
                }
                END_SCOPED_REGION
                break;
            default: break;
        }
    }

    END_SCOPED_REGION

    const auto m = sequence.size();

    Graph G(m);

    for (unsigned i = 0; i < m; ++i) {
        const auto u = sequence[i];
        const RelationshipNode & node = Relationships[u];
        if (node.Type == RelationshipNode::IsKernel) {
            addKernelRelationshipsInReferenceOrdering(u, Relationships,
                [&](const PortType type, const unsigned binding, const unsigned streamSet) {
                    const auto j = mapping[streamSet];
                    assert (j < m);
                    assert (sequence[j] == streamSet);
                    auto a = i, b = j;
                    if (type == PortType::Input) {
                        a = j; b = i;
                    }
                    assert (a < b);
                    add_edge(a, b, binding, G);
                }
            );
        }
    }

    // Stage 1: identify synchronous components

    // wcan through the graph and determine where every non-Fixed relationship exists
    // so that we can construct our initial set of partitions. The goal here is to act
    // as a first pass to simplify the problem before using Z3.

    for (unsigned i = 0; i < m; ++i) {
        BitSet & V = G[i];
        V.resize(n);
    }

    for (unsigned i = 0, nextRateId = 0; i < m; ++i) {

        BitSet & V = G[i];

        for (const auto e : make_iterator_range(in_edges(i, G))) {
            const BitSet & R = G[source(e, G)];
            V |= R;
        }

        const auto u = sequence[i];
        const RelationshipNode & node = Relationships[u];

        if (node.Type == RelationshipNode::IsKernel) {

            if (in_degree(i, G) == 0) {
                if (out_degree(i, G) > 0) {
                    V.set(nextRateId++);
                } else {
                    assert (node.Kernel == mPipelineKernel);
                }
            } else {
                for (const auto e : make_iterator_range(in_edges(i, G))) {

                    const RelationshipNode & rn = Relationships[G[e]];
                    assert (rn.Type == RelationshipNode::IsBinding);
                    const Binding & b = rn.Binding;
                    const ProcessingRate & rate = b.getRate();

                    // Check the attributes to see whether any impose a partition change
                    auto hasInputRateChangeAttribute = [](const Binding & b) {
                        for (const Attribute & attr : b.getAttributes()) {
                            switch (attr.getKind()) {
                                case AttrId::Delayed:
                                case AttrId::BlockSize:
                                case AttrId::LookAhead:
                                    return true;
                                default: break;
                            }
                        }
                        return false;
                    };
                    if (rate.getKind() != RateId::Fixed || hasInputRateChangeAttribute(b)) {
                        V.set(nextRateId++);
                        break;
                    }
                }
            }

            assert (V.any() || node.Kernel == mPipelineKernel);

            // Now iterate through the outputs
            for (const auto e : make_iterator_range(out_edges(i, G))) {

                const RelationshipNode & rn = Relationships[G[e]];
                assert (rn.Type == RelationshipNode::IsBinding);
                const Binding & b = rn.Binding;
                const ProcessingRate & rate = b.getRate();

                // Check the attributes to see whether any impose a partition change
                auto hasOutputRateChangeAttribute = [](const Binding & b) {
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

                if (LLVM_UNLIKELY(rate.getKind() != RateId::Fixed || hasOutputRateChangeAttribute(b))) {
                    BitSet & R = G[target(e, G)];
                    R.set(nextRateId++);
                }

            }
        }

        for (const auto e : make_iterator_range(out_edges(i, G))) {
            BitSet & R = G[target(e, G)];
            R |= V;
        }

    }

    std::vector<unsigned> partitionIds(m);

    auto writePartitionIds = [&]() {
        PartitionMap partitionSets;
        unsigned nextPartitionId = 1;
        for (unsigned i = 0; i < m; ++i) {
            const auto u = sequence[i];
            const RelationshipNode & node = Relationships[u];
            if (node.Type == RelationshipNode::IsKernel) {
                BitSet & V = G[i];
                unsigned partitionId = 0;
                if (LLVM_LIKELY(V.any())) {
                    auto f = partitionSets.find(V);
                    if (f == partitionSets.end()) {
                        partitionId = nextPartitionId++;
                        partitionSets.emplace(V, partitionId);
                    } else {
                        partitionId = f->second;
                    }
                } else {
                    assert (node.Kernel == mPipelineKernel);
                }
                partitionIds[i] = partitionId;
            }
        }
        return nextPartitionId;
    };

    auto synchronousPartitionCount = writePartitionIds();

    assert (synchronousPartitionCount > 0);

    // Stage 2: estimate synchronous inter-partition dataflow

    // From the first pass analysis, determine the synchronous dataflow of each partition
    // so that we can correctly identify how much data will pass through the inter-partition
    // channels.

    auto estimateSynchronousDataflow = [&]() {

        const auto cfg = Z3_mk_config();
        Z3_set_param_value(cfg, "model", "true");
        Z3_set_param_value(cfg, "proof", "false");

        const auto ctx = Z3_mk_context(cfg);
        Z3_del_config(cfg);

        const auto realType = Z3_mk_real_sort(ctx);

        const auto intType = Z3_mk_int_sort(ctx);

        auto constant_real = [&](const Rational value) {
            if (value.denominator() == 1) {
                return Z3_mk_int(ctx, value.numerator(), intType);
            } else {
                return Z3_mk_real(ctx, value.numerator(), value.denominator());
            }
        };

        auto multiply =[&](Z3_ast X, Z3_ast Y) {
            Z3_ast args[2] = { X, Y };
            return Z3_mk_mul(ctx, 2, args);
        };

        const auto ONE = constant_real(1);

        const auto solver = Z3_mk_solver(ctx);
        Z3_solver_inc_ref(ctx, solver);

        auto hard_assert = [&](Z3_ast c) {
            Z3_solver_assert(ctx, solver, c);
        };

        auto variable_real = [&]() {
            auto v = Z3_mk_fresh_const(ctx, nullptr, realType);
            hard_assert(Z3_mk_ge(ctx, v, ONE));
            return v;
        };

        std::vector<Z3_ast> repetitions(m);

        for (unsigned i = 0; i < m; ++i) {
            const auto u = sequence[i];
            const RelationshipNode & currentNode = Relationships[u];
            if (currentNode.Type == RelationshipNode::IsKernel) {
                repetitions[i] = variable_real();
            }
        }

        for (unsigned i = 0; i < m; ++i) {
            if (repetitions[i]) {
                continue;
            }

            const auto output = in_edge(i, G);
            const auto producer = source(output, G);
            const auto prodPartId = partitionIds[producer];

            const RelationshipNode & ouputNode = Relationships[G[output]];
            const Binding & outputBinding = ouputNode.Binding;
            const ProcessingRate & outputRate = outputBinding.getRate();

            if (LLVM_LIKELY(outputRate.isFixed())) {

                Z3_ast outRate = nullptr;

                for (const auto input : make_iterator_range(out_edges(i, G))) {
                    const auto consumer = target(output, G);
                    const auto consPartId = partitionIds[consumer];
                    // we're only concerned with intra-partition relationships here
                    if (prodPartId == consPartId) {
                        const RelationshipNode & inputNode = Relationships[G[input]];
                        const Binding & inputBinding = inputNode.Binding;
                        const ProcessingRate & inputRate = inputBinding.getRate();
                        if (LLVM_LIKELY(inputRate.isFixed())) {

                            auto makeFixedRateVar = [&](const ProcessingRate & rate, const unsigned kernel) {
                                assert (rate.isFixed());
                                const RelationshipNode & node = Relationships[sequence[kernel]];
                                assert (node.Type == RelationshipNode::IsKernel);
                                const auto fixed = rate.getLowerBound() * node.Kernel->getStride();
                                assert (repetitions[kernel]);
                                return multiply(repetitions[kernel], constant_real(fixed));
                            };

                            // if this is the first instance of an intra-partition relationship for
                            // this streamset, then determine the output rate.
                            if (outRate == nullptr) {
                                outRate = makeFixedRateVar(outputRate, producer);
                            }
                            const Z3_ast inRate = makeFixedRateVar(inputRate, consumer);
                            hard_assert(Z3_mk_eq(ctx, outRate, inRate));

                        }
                    }
                }
            }
        }

        if (LLVM_UNLIKELY(Z3_solver_check(ctx, solver) == Z3_L_FALSE)) {
            report_fatal_error("Z3 failed to find an intra-partition dataflow solution");
        }

        std::vector<Rational> numOfStridesMultiplier(m);

        const auto model = Z3_solver_get_model(ctx, solver);
        Z3_model_inc_ref(ctx, model);
        for (unsigned i = 0; i < m; ++i) {
            if (repetitions[i]) {
                Z3_ast value;
                if (LLVM_UNLIKELY(Z3_model_eval(ctx, model, repetitions[i], Z3_L_TRUE, &value) != Z3_L_TRUE)) {
                    report_fatal_error("Unexpected Z3 error when attempting to obtain value from model!");
                }
                __int64 num, denom;
                if (LLVM_UNLIKELY(Z3_get_numeral_rational_int64(ctx, value, &num, &denom) != Z3_L_TRUE)) {
                    report_fatal_error("Unexpected Z3 error when attempting to convert model value to number!");
                }
                assert (num > 0);
                numOfStridesMultiplier[i] = Rational{num, denom};
            }
        }

        Z3_model_dec_ref(ctx, model);
        Z3_solver_dec_ref(ctx, solver);
        Z3_del_context(ctx);

        return numOfStridesMultiplier;
    };


    // Stage 3: fuse stride-linked partitions

    // Analyze the partition I/O channel rates to determine whether any partitions
    // are synchronous and fuse them. Note: this algorithm assumes no program permits
    // the unbounded accumulation of data along any channel and explicitly ignores
    // detection of this case when attempting to fuse partitions.

    auto fuseStrideLinkedPartitions = [&]() {

        const auto numOfStridesMultiplier = estimateSynchronousDataflow();

        const auto cfg = Z3_mk_config();
        Z3_set_param_value(cfg, "model", "false");
        Z3_set_param_value(cfg, "proof", "false");

        const auto ctx = Z3_mk_context(cfg);
        Z3_del_config(cfg);

        const auto intType = Z3_mk_int_sort(ctx);

        auto constant_real = [&](const Rational value) {
            if (value.denominator() == 1) {
                return Z3_mk_int(ctx, value.numerator(), intType);
            } else {
                return Z3_mk_real(ctx, value.numerator(), value.denominator());
            }
        };

        auto multiply =[&](Z3_ast X, Z3_ast Y) {
            Z3_ast args[2] = { X, Y };
            return Z3_mk_mul(ctx, 2, args);
        };

        struct PartitionVariables {
            Z3_ast StrideVar;
            flat_set<Z3_app> Vars;
            void setRoot(Z3_context ctx, Z3_ast root) {
                StrideVar = root;
                addVar(ctx, root);
            }
            void addVar(Z3_context ctx, Z3_ast var) {
                assert (Z3_is_app(ctx, var));
                Vars.insert((Z3_app)var);
            }
        };

        enum FusionConstraintType { EQ = 0, GE = 1 };

        using FusionConstraint = std::tuple<FusionConstraintType, Z3_ast, Z3_ast>;

        using FusionConstaints = SmallVector<FusionConstraint, 8>;

        using PartitionFusionGraph = adjacency_list<hash_setS, vecS, bidirectionalS, PartitionVariables, FusionConstaints>;

        #warning fix transitive reduction to permit adjacency_matrix

        using DependencyGraph = adjacency_list<hash_setS, vecS, bidirectionalS>; // adjacency_matrix<directedS>;

        const auto tQE = Z3_mk_tactic(ctx, "qe");
        Z3_tactic_inc_ref (ctx, tQE);
        const auto tSMT = Z3_mk_tactic(ctx, "smt");
        Z3_tactic_inc_ref (ctx, tSMT);
        const auto tactics = Z3_tactic_and_then (ctx, tQE, tSMT);
        const auto solver = Z3_mk_solver_from_tactic(ctx, tactics);
        Z3_solver_inc_ref(ctx, solver);

        Z3_params params = Z3_mk_params(ctx);
        Z3_params_inc_ref(ctx, params);

        // Fusing two partitions is an optimization --- not a requirement. I have not seen a case
        // in which there is a non-trivial solution but many in which Z3 will run "forever" to
        // locate one. To handle this, we set the timeout low enough that Z3 ought to have ample
        // time to locate a trivial solution to the fusion constraint problem.
        Z3_symbol r = Z3_mk_string_symbol(ctx, ":timeout");
        Z3_params_set_uint(ctx, params, r, 500);
        Z3_solver_set_params(ctx, solver, params);
        Z3_params_dec_ref(ctx, params);

        auto hard_assert = [&](Z3_ast c) {
            Z3_solver_assert(ctx, solver, c);
        };

        struct IORateVar {
            Z3_ast Var;
            Z3_ast Expr;
            IORateVar(Z3_ast boundedVar, Z3_ast expression)
            : Var(boundedVar)
            , Expr(expression) {

            }
        };

        flat_map<unsigned, IORateVar> IORateVarMap;

        IORateVarMap.reserve(num_edges(G));

        const auto ONE = Z3_mk_int(ctx, 1, intType);

        DependencyGraph H(synchronousPartitionCount);

        for (unsigned i = 0; i < m; ++i) {
            const auto u = sequence[i];
            const RelationshipNode & node = Relationships[u];
            if (node.Type == RelationshipNode::IsRelationship) {
                assert (isa<StreamSet>(Relationships[u].Relationship));
                const auto input = in_edge(i, G);
                const auto producer = source(input, G);
                const auto prodPartId = partitionIds[producer];
                assert (prodPartId > 0);
                for (const auto output : make_iterator_range(out_edges(i, G))) {
                    const auto consumer = target(output, G);
                    const auto consPartId = partitionIds[consumer];
                    assert (consPartId > 0);
                    // we're only concerned with inter-partition relationships here
                    if (prodPartId == consPartId) {
                        continue;
                    }
                    add_edge(prodPartId, consPartId, H);
                }
            }
        }

        transitive_reduction_dag(H);

        PartitionFusionGraph P(synchronousPartitionCount);

        for (unsigned i = 0; i < synchronousPartitionCount; ++i) {
            for (const auto e : make_iterator_range(out_edges(i, H))) {
                add_edge(i, target(e, H), FusionConstaints{}, P);
            }
        }

//        out << "digraph \"P\" {\n";
//        for (unsigned partitionId = 0; partitionId < synchronousPartitionCount; ++partitionId) {
//            out << "v" << partitionId << " [label=\"P" << partitionId << "\n";
//            for (unsigned i = 0; i < m; ++i) {
//                if (partitionIds[i] == partitionId) {
//                    const auto u = sequence[i];
//                    const RelationshipNode & node = Relationships[u];
//                    if (node.Type == RelationshipNode::IsKernel) {
//                        out << u << ". " << node.Kernel->getName() << "\n";
//                    }
//                }
//            }
//            out << "\"];\n";
//        }
//        for (auto e : make_iterator_range(edges(P))) {
//            const auto s = source(e, P);
//            const auto t = target(e, P);
//            out << "v" << s << " -> v" << t << ";\n";
//        }

//        out << "}\n\n";
//        out.flush();


        for (unsigned i = 0; i < synchronousPartitionCount; ++i) {
            const auto var = Z3_mk_fresh_const(ctx, nullptr, intType);
            PartitionVariables & V = P[i];
            V.addVar(ctx, var);
            hard_assert(Z3_mk_ge(ctx, var, ONE));
            V.setRoot(ctx, var);
        }

        flat_map<unsigned, Z3_ast> symbolicPopCountRateVar;

        flat_map<const StreamSet *, Z3_ast> streamSetVar;

        for (unsigned i = 0; i < m; ++i) {
            const auto u = sequence[i];
            const RelationshipNode & node = Relationships[u];
            if (node.Type == RelationshipNode::IsRelationship) {
                assert (isa<StreamSet>(node.Relationship));

                const auto input = in_edge(i, G);
                const auto producer = source(input, G);
                const auto prodPartId = partitionIds[producer];

                assert (prodPartId < synchronousPartitionCount);

                Z3_ast outRateVar = nullptr;
                Z3_ast outRateExpr = nullptr;

                for (const auto output : make_iterator_range(out_edges(i, G))) {
                    const auto consumer = target(output, G);
                    const auto consPartId = partitionIds[consumer];

                    assert (consPartId < synchronousPartitionCount);

                    // we're only concerned with inter-partition relationships here
                    if (prodPartId == consPartId) {
                        continue;
                    }

                    const auto e = edge(prodPartId, consPartId, P);
                    if (!e.second) {
                        continue;
                    }

                    FusionConstaints & fusionConstraints = P[e.first];

                    auto makeBoundedVar = [&](PartitionVariables & V, const Rational & lb, const Rational & ub) {
                        const auto var = Z3_mk_fresh_const(ctx, nullptr, intType);
                        V.addVar(ctx, var);
                        hard_assert(Z3_mk_ge(ctx, var, Z3_mk_int(ctx, floor(lb), intType)));
                        hard_assert(Z3_mk_le(ctx, var, Z3_mk_int(ctx, ceiling(ub), intType)));
                        return var;
                    };

                    auto calculateBaseSymbolicRateVar = [&](const BindingVertex bindingVertex, const ProcessingRate & rate, const unsigned kernel, const unsigned partId)  {
                        const RelationshipNode & node = Relationships[sequence[kernel]];
                        assert (node.Type == RelationshipNode::IsKernel);

                        PartitionVariables & V = P[partId];

                        Z3_ast var = nullptr;
                        Z3_ast expr = nullptr;

                        auto getReferenceBinding = [&]() {
                            for (const auto e : make_iterator_range(in_edges(bindingVertex, Relationships))) {
                                if (Relationships[e].Reason == ReasonType::Reference) {
                                    const auto refBinding = source(e, Relationships);
                                    assert (Relationships[refBinding].Type == RelationshipNode::IsBinding);
                                    return refBinding;
                                }
                            }
                            llvm_unreachable("could not find reference?");
                        };

                        switch (rate.getKind()) {
                            case RateId::Fixed:
                                BEGIN_SCOPED_REGION
                                const auto strideSize = node.Kernel->getStride() * numOfStridesMultiplier[kernel];
                                const auto numOfItems = rate.getLowerBound() * strideSize;
                                var = V.StrideVar;
                                expr = multiply(V.StrideVar, constant_real(numOfItems));
                                END_SCOPED_REGION
                                break;
                            case RateId::Greedy:
                                BEGIN_SCOPED_REGION
                                assert ("greedy rate cannot be an output rate" && producer != kernel);
                                assert (outRateVar && outRateExpr);
                                if (rate.getLowerBound() > Rational{0}) {
                                    fusionConstraints.emplace_back(std::make_tuple(GE, outRateExpr, constant_real(rate.getLowerBound())));
                                }
                                var = outRateVar;
                                expr = outRateExpr;
                                END_SCOPED_REGION
                                break;
                            case RateId::Bounded:
                                BEGIN_SCOPED_REGION
                                const auto strideSize = node.Kernel->getStride() * numOfStridesMultiplier[kernel];
                                const auto lb = rate.getLowerBound() * strideSize;
                                const auto ub = rate.getUpperBound() * strideSize;
                                var = makeBoundedVar(V, lb, ub);
                                expr = var;
                                END_SCOPED_REGION
                                break;
                            case RateId::PartialSum:
                                BEGIN_SCOPED_REGION
                                const auto refBinding = getReferenceBinding();
                                const auto refStreamSet = parent(refBinding, Relationships);
                                assert (Relationships[refStreamSet].Type == RelationshipNode::IsRelationship);
                                const auto f = symbolicPopCountRateVar.find(refStreamSet);
                                if (f == symbolicPopCountRateVar.end()) {
                                    var = Z3_mk_fresh_const(ctx, nullptr, intType);
                                    symbolicPopCountRateVar.emplace(refStreamSet, var);
                                } else {
                                    var = f->second;
                                }
                                V.addVar(ctx, var);

                                Z3_ast args[3] = { var, V.StrideVar, constant_real(numOfStridesMultiplier[kernel]) };

                                expr = Z3_mk_mul(ctx, 3, args);
                                END_SCOPED_REGION
                                break;
                            case RateId::Relative:
                                BEGIN_SCOPED_REGION
                                const auto refBinding = getReferenceBinding();
                                const auto f = IORateVarMap.find(refBinding);
                                assert ("relative rate occured before ref" && f != IORateVarMap.end());
                                const IORateVar & E = f->second;
                                var = E.Var;
                                expr = E.Expr;
                                assert ("failed to locate symbolic rate for relative rate?" && expr);
                                if (rate.getLowerBound() != Rational{1}) {
                                    expr = multiply(expr, constant_real(rate.getLowerBound()) );
                                }
                                END_SCOPED_REGION
                                break;
                            default:
                                llvm_unreachable("unknown rate type?");
                        }
                        assert (var && expr);
                        IORateVarMap.emplace(bindingVertex, IORateVar{var, expr});
                        return std::make_pair(var, expr);
                    };


                    // if this is the first instance of an inter-partition relationship for
                    // this streamset, then determine the output rate.
                    if (outRateExpr == nullptr) {

                        const BindingVertex bindingVertex = G[input];
                        const RelationshipNode & rn = Relationships[bindingVertex];
                        const Binding & binding = rn.Binding;
                        const ProcessingRate & rate = binding.getRate();

                        std::tie(outRateVar, outRateExpr) = calculateBaseSymbolicRateVar(bindingVertex, rate, producer, prodPartId);

                        for (const Attribute & attr : binding.getAttributes()) {
                            switch (attr.getKind()) {
                                case AttrId::Delayed:
                                    BEGIN_SCOPED_REGION
                                    Z3_ast args[2] = { outRateExpr, Z3_mk_int(ctx, attr.amount(), intType) };
                                    outRateExpr = Z3_mk_sub(ctx, 2, args);
                                    END_SCOPED_REGION
                                    break;
                                case AttrId::Deferred:
                                    // A deferred output rate is closer to an bounded rate than a
                                    // countable rate but a deferred input rate simply means the
                                    // buffer must be dynamic.
                                    BEGIN_SCOPED_REGION
                                    PartitionVariables & V = P[prodPartId];
                                    const auto deferredRateVar = Z3_mk_fresh_const(ctx, nullptr, intType);
                                    V.addVar(ctx, deferredRateVar);
                                    hard_assert(Z3_mk_ge(ctx, deferredRateVar, Z3_mk_int(ctx, 0, intType)));
                                    hard_assert(Z3_mk_le(ctx, deferredRateVar, outRateExpr));
                                    outRateExpr = deferredRateVar;
                                    END_SCOPED_REGION
                                    break;
                                case AttrId::BlockSize:
                                    BEGIN_SCOPED_REGION
                                    outRateExpr = Z3_mk_mod(ctx, outRateExpr, Z3_mk_int(ctx, attr.amount(), intType) );
                                    END_SCOPED_REGION
                                    break;
                                default: break;
                            }
                        }

                        streamSetVar.emplace(cast<StreamSet>(node.Relationship), outRateExpr);
                    }

                    const BindingVertex bindingVertex = G[output];
                    const RelationshipNode & rn = Relationships[bindingVertex];
                    const Binding & binding = rn.Binding;
                    const ProcessingRate & rate = binding.getRate();

                    Z3_ast inRateExpr = calculateBaseSymbolicRateVar(bindingVertex, rate, consumer, consPartId).second;

                    for (const Attribute & attr : binding.getAttributes()) {
                        switch (attr.getKind()) {
                            case AttrId::LookAhead:
                            case AttrId::Delayed:
                                BEGIN_SCOPED_REGION
                                Z3_ast args[2] = { inRateExpr, Z3_mk_int(ctx, attr.amount(), intType) };
                                inRateExpr = Z3_mk_sub(ctx, 2, args);
                                END_SCOPED_REGION
                                break;
                            case AttrId::BlockSize:
                                BEGIN_SCOPED_REGION
                                inRateExpr = Z3_mk_mod(ctx, inRateExpr, Z3_mk_int(ctx, attr.amount(), intType));
                                END_SCOPED_REGION
                                break;
                            default: break;
                        }
                    }
                    fusionConstraints.emplace_back(std::make_tuple(EQ, outRateExpr, inRateExpr));
                }
            }
        }

        for (const LengthAssertion & a : mLengthAssertions)  {
            Z3_ast args[2];
            bool necessary = true;
            for (unsigned i = 0; i < 2; ++i) {
                const auto f = streamSetVar.find(cast<StreamSet>(a[i]));
                if (LLVM_UNLIKELY(f == streamSetVar.end())) {
                    necessary = false;
                    break;
                } else {
                    args[i] = f->second;
                }
            }
            if (necessary) {
                // if for some reason adding this constraint makes the formula unsatisfiable,
                // discard it.
                Z3_solver_push(ctx, solver);
                hard_assert(Z3_mk_eq(ctx, args[0], args[1]));
                if (LLVM_UNLIKELY(Z3_solver_check(ctx, solver) != Z3_L_TRUE)) {
                    Z3_solver_pop(ctx, solver, 1);
                }
            }
        }

        std::vector<unsigned> fusedPartitionIds(synchronousPartitionCount);
        std::iota(fusedPartitionIds.begin(), fusedPartitionIds.end(), 0);
        std::vector<unsigned> rank(synchronousPartitionCount, 0);

        adjacency_list<hash_setS, vecS, undirectedS> L(synchronousPartitionCount);

        std::function<unsigned(unsigned)> find = [&](unsigned x) {
            if (fusedPartitionIds[x] != x) {
                fusedPartitionIds[x] = find(fusedPartitionIds[x]);
            }
            return fusedPartitionIds[x];
        };

        auto union_find = [&](unsigned x, unsigned y) {

            add_edge(x, y, L);

            x = find(x);
            y = find(y);


            if (rank[x] > rank[y]) {
                fusedPartitionIds[y] = x;
            } else {
                fusedPartitionIds[x] = y;
                if (rank[x] == rank[y]) {
                    rank[y]++;
                }
            }

        };

        auto in_same_connected_component = [&](unsigned x, unsigned y) {
            return find(x) == find(y);
        };

//        errs() << "------------------------------\n";
//        errs() << "CLAUSES:\n";

//        errs() << Z3_solver_to_string(ctx, solver) << "\n";

//        errs() << "\n";

        std::vector<Z3_ast> list;

        flat_set<Z3_app> vars;

        bool fusedAny = false;

        for (const auto e : make_iterator_range(edges(P))) {

            const auto s = source(e, P);
            const auto t = target(e, P);

            if (in_same_connected_component(s, t)) {
                continue;
            }

            const auto & A = P[s];
            const auto & B = P[t];

            Z3_solver_push(ctx, solver);
            assert (list.empty());

            for (const FusionConstraint & c : P[e]) {
                Z3_ast e;
                if (std::get<0>(c) == EQ) {
                    e = Z3_mk_eq(ctx, std::get<1>(c), std::get<2>(c));
                } else {
                    e = Z3_mk_ge(ctx, std::get<1>(c), std::get<2>(c));
                }
                list.push_back(e);
            }
            assert (list.size() > 0);
            const auto constraints = Z3_mk_and(ctx, list.size(), list.data());
            list.clear();

            assert (vars.empty());
            vars.insert(B.Vars.begin(), B.Vars.end());
            const auto exists = Z3_mk_exists_const(ctx, 0, vars.size(), &*vars.begin(), 0, nullptr, constraints);
            vars.clear();

            const auto & Pr = A.Vars;
            const auto forall = Z3_mk_forall_const(ctx, 0, Pr.size(), &*Pr.begin(), 0, nullptr, exists);

            hard_assert(forall);

//            errs() << "------------------------------\n";
//            errs() << "PRED:\n";

//            errs() << Z3_solver_to_string(ctx, solver) << "\n";

            // errs() << Z3_ast_to_string(ctx, forall) << "\n";

            const auto r = Z3_solver_check(ctx, solver);

//            errs() << " r=" << r << "\n";

            if (LLVM_UNLIKELY(r == Z3_L_TRUE)) {
                union_find(s, t);
                fusedAny = true;
            } else {
                Z3_solver_pop(ctx, solver, 1);
            }


        }



        for (unsigned i = 0; i < synchronousPartitionCount; ++i) {

            if (out_degree(i, P) > 1) {
                graph_traits<PartitionFusionGraph>::out_edge_iterator begin, end;
                std::tie(begin, end) = out_edges(i, P);

                auto & Pr = P[i].Vars;

                for (auto ei = begin; ei != end; ++ei) {
                    for (auto ej = ei; ++ej != end; ) {

                        const auto a = target(*ei, P);
                        const auto b = target(*ej, P);

                        if (in_same_connected_component(a, b)) {
                            continue;
                        }

                        Z3_solver_push(ctx, solver);

//                        const auto ratio = Z3_mk_fresh_const(ctx, nullptr, realType);
//                        hard_assert(Z3_mk_ge(ctx, ratio, rONE));

                        const auto & A = P[a];
                        const auto & B = P[b];
                        assert (vars.empty());
                        vars.insert(A.Vars.begin(), A.Vars.end());
                        vars.insert(B.Vars.begin(), B.Vars.end());
//                        vars.insert((Z3_app)ratio);

                        assert (list.empty());

                        auto add_constraints = [&](const PartitionFusionGraph::edge_descriptor & e) {
                            assert (P[e].size() > 0);
                            for (const FusionConstraint & c : P[e]) {
                                list.push_back(Z3_mk_ge(ctx, std::get<1>(c), std::get<2>(c)));
                            }
                        };

                        add_constraints(*ei);
                        add_constraints(*ej);
                        list.push_back(Z3_mk_eq(ctx, A.StrideVar, B.StrideVar));
                        const auto constraints = Z3_mk_and(ctx, list.size(), list.data());
                        list.clear();
                        const auto exists = Z3_mk_exists_const(ctx, 0, vars.size(), &*vars.begin(), 0, nullptr, constraints);
                        const auto forall = Z3_mk_forall_const(ctx, 0, Pr.size(), &*Pr.begin(), 0, nullptr, exists);

                        vars.clear();

                        hard_assert(forall);

//                        errs() << "------------------------------\n";
//                        errs() << "SIBLINGS:\n";

//                        errs() << Z3_solver_to_string(ctx, solver) << "\n";

                        // errs() << Z3_ast_to_string(ctx, forall) << "\n";

                        const auto r = Z3_solver_check(ctx, solver);

//                        errs() << " r=" << r << "\n";

                        if (LLVM_UNLIKELY(r == Z3_L_TRUE)) {
                            union_find(a, b);
                            fusedAny = true;
                        } else {
                            Z3_solver_pop(ctx, solver, 1);
                        }


                    }
                }
            }
        }

        Z3_tactic_dec_ref (ctx, tQE);
        Z3_tactic_dec_ref (ctx, tSMT);
        Z3_solver_dec_ref(ctx, solver);
        Z3_del_context(ctx);

        // Z3_finalize_memory();

//        out << "graph \"L\" {\n";
//        for (unsigned partitionId = 0; partitionId < synchronousPartitionCount; ++partitionId) {
//            out << "v" << partitionId << " [label=\"P" << partitionId << "\n";
//            for (unsigned i = 0; i < m; ++i) {
//                if (partitionIds[i] == partitionId) {
//                    const auto u = sequence[i];
//                    const RelationshipNode & node = Relationships[u];
//                    if (node.Type == RelationshipNode::IsKernel) {
//                        out << u << ". " << node.Kernel->getName() << "\n";
//                    }
//                }
//            }
//            out << "\"];\n";
//        }
//        for (auto e : make_iterator_range(edges(L))) {
//            const auto s = source(e, L);
//            const auto t = target(e, L);
//            out << "v" << s << " -- v" << t << ";\n";
//        }

//        out << "}\n\n";
//        out.flush();



        if (fusedAny) {

            flat_set<unsigned> partitions;
            partitions.reserve(synchronousPartitionCount);
            for (unsigned i = 0; i < synchronousPartitionCount; ++i) {
                partitions.insert(fusedPartitionIds[i]);
            }
            synchronousPartitionCount = partitions.size();
            for (unsigned i = 0; i < m; ++i) {
                auto & p = partitionIds[i];
                const auto k = fusedPartitionIds[p];
                const auto f = partitions.find(k);
                assert (f != partitions.end());
                p = std::distance(partitions.begin(), f);
            }

        }

        return fusedAny;
    };

    while (fuseStrideLinkedPartitions());

    // Stage 4: finalize the partition structure

    // Based on the (fused) partition ids, repeat the process used to generate the initial
    // synchronous components but also factor in early termination and any other kernel
    // attributes that could interrupt the flow of data through the program.

    flat_map<std::pair<unsigned, unsigned>, unsigned> interPartitionMap;

    assert (num_vertices(G) == m);

    for (unsigned i = 0; i < m; ++i) {
        BitSet & V = G[i];
        V.reset();
    }

    for (unsigned i = 0, nextRateId = 0; i < m; ++i) {

        BitSet & V = G[i];

        for (const auto e : make_iterator_range(in_edges(i, G))) {
            const BitSet & R = G[source(e, G)];
            V |= R;
        }

        const auto u = sequence[i];
        const RelationshipNode & node = Relationships[u];

        if (node.Type == RelationshipNode::IsKernel) {

            bool demarcateOutputs = false;

            if (in_degree(i, G) == 0) {
                if (out_degree(i, G) > 0) {
                    V.set(nextRateId++);
                } else {
                    assert (node.Kernel == mPipelineKernel);
                }
                demarcateOutputs = true;
            } else {
                // Check whether this (internal) kernel could terminate early
                const auto kernelObj = node.Kernel;
                if (kernelObj != mPipelineKernel) {
                    demarcateOutputs |= kernelObj->canSetTerminateSignal();
                    // TODO: an internally synchronzied kernel with fixed rate I/O can be contained within a partition
                    // but cannot be the root of a non-isolated partition. To permit them to be roots, they'd need
                    // some way of informing the pipeline as to how many strides they executed or the pipeline
                    // would need to know to calculate it from its outputs. Rather than handling this complication,
                    // for now we simply prevent this case.
                    demarcateOutputs |= kernelObj->hasAttribute(AttrId::InternallySynchronized);
                }
            }

            assert (V.any() || node.Kernel == mPipelineKernel);

            if (LLVM_UNLIKELY(demarcateOutputs)) {
                const auto demarcationId = nextRateId++;
                for (const auto e : make_iterator_range(out_edges(i, G))) {
                    BitSet & R = G[target(e, G)];
                    R.set(demarcationId);
                }
            }

        } else {

            const auto j = parent(i, G);
            const auto producer = sequence[j];
            assert (Relationships[producer].Type == RelationshipNode::IsKernel);
            const auto prodPartId = partitionIds[j];
            for (const auto e : make_iterator_range(out_edges(i, G))) {
                const auto k = target(e, G);
                const auto consumer = sequence[k];
                assert (Relationships[consumer].Type == RelationshipNode::IsKernel);
                const auto consPartId = partitionIds[k];
                if (prodPartId != consPartId) {
                    unsigned rateId = 0;
                    auto f = interPartitionMap.find(std::make_pair(prodPartId, consPartId));
                    if (f == interPartitionMap.end()) {
                        rateId = nextRateId++;
                        interPartitionMap.emplace(std::make_pair(prodPartId, consPartId), rateId);
                    } else {
                        rateId = f->second;
                    }
                    BitSet & R = G[k];
                    R.set(rateId);
                }
            }
        }

        for (const auto e : make_iterator_range(out_edges(i, G))) {
            BitSet & R = G[target(e, G)];
            R |= V;
        }

    }

    // TODO: now test every Fixed-rate partition to determine whether its worthwhile keeping
    // them in the same partition. For example, suppose kernel A has a Fixed(7) output rate
    // that is consumed by kernel B with a Fixed(8) input rate. To keep A and B in the same
    // partition, they would have to execute a multiple of 56 strides per segment. If these
    // items represent single bytes, this would be beneficial but could slow down the
    // pipeline if they represent 1K chunks.


    const auto partitionCount = writePartitionIds();

    using RenumberingGraph = adjacency_list<vecS, vecS, bidirectionalS, no_property, unsigned>;

    // Stage 5: renumber the partition ids

    // To simplify processing later, renumber the partitions such that the partition id
    // of any predecessor of a kernel K is <= the partition id of K.

    RenumberingGraph T(partitionCount);

    for (unsigned i = 1; i < partitionCount; ++i) {
        add_edge(0, i, 0, T);
    }

    for (unsigned i = 0; i < m; ++i) {
        const auto u = sequence[i];
        const RelationshipNode & node = Relationships[u];
        if (node.Type == RelationshipNode::IsRelationship) {
            const auto j = parent(i, G);
            const auto producer = sequence[j];
            assert (Relationships[producer].Type == RelationshipNode::IsKernel);
            const auto prodPartId = partitionIds[j];
            for (const auto e : make_iterator_range(out_edges(i, G))) {
                const auto k = target(e, G);
                const auto consumer = sequence[k];
                assert (Relationships[consumer].Type == RelationshipNode::IsKernel);
                const auto consPartId = partitionIds[k];
                if (prodPartId != consPartId) {
                    add_edge(prodPartId, consPartId, u, T);
                }
            }
        }
    }

    std::vector<unsigned> renumberingSeq;
    renumberingSeq.reserve(partitionCount);

    if (LLVM_UNLIKELY(!lexical_ordering(T, renumberingSeq))) {
        report_fatal_error("Internal error: failed to generate acyclic partition graph");
    }

    assert (renumberingSeq[0] == 0);

    std::vector<unsigned> renumbered(partitionCount);

    for (unsigned i = 0; i < partitionCount; ++i) {
        renumbered[renumberingSeq[i]] = i;
    }

    assert (renumbered[0] == 0);

    PartitionGraph P(partitionCount);

    for (unsigned i = 0; i < m; ++i) {
        const auto u = sequence[i];
        const RelationshipNode & node = Relationships[u];
        if (node.Type == RelationshipNode::IsKernel) {
            const auto j = renumbered[partitionIds[i]];
            P[j].Kernels.push_back(u);
            PartitionIds.emplace(u, j);
        }
    }

    #ifndef NDEBUG
    for (const auto u : P[0].Kernels) {
        assert (Relationships[u].Type == RelationshipNode::IsKernel);
        assert (Relationships[u].Kernel == mPipelineKernel);
    }
    #endif

    for (unsigned i = 1; i < partitionCount; ++i) {
        assert (P[i].Kernels.size() > 0);
        const auto j = renumbered[i];
        assert (j > 0);
        for (const auto e : make_iterator_range(out_edges(i, T))) {
            const auto k = renumbered[target(e, T)];
            assert (k > j);
            const auto streamSet = T[e];
            assert (streamSet < num_vertices(Relationships));
            add_edge(j, k, streamSet, P);
        }
    }

    PartitionCount = partitionCount;

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
