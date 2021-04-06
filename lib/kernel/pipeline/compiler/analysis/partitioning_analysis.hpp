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

    using RelVert = RelationshipGraph::vertex_descriptor;

    struct RateData {
        BitSet RateIds;
        const RelVert Vertex;
        RateData(unsigned w, RelVert v) : RateIds(w), Vertex(v) { }
    };

    // Convert G into a simpler representation of the graph that we can annotate

    using Graph = adjacency_list<vecS, vecS, bidirectionalS, no_property, RateData>;

    using LinkedPartitionGraph = adjacency_list<vecS, vecS, undirectedS>;

    using PartitionMap = std::map<BitSet, unsigned>;

#if 1
    const unsigned n = num_vertices(Relationships);

    std::vector<unsigned> sequence;
    sequence.reserve(n);

    std::vector<unsigned> mapping(n, -1U);

    flat_map<const StreamSet *, unsigned> lengthAssertedStreamSet;
    for (const LengthAssertion & a : mLengthAssertions) {
        lengthAssertedStreamSet.emplace(a[0], -1U);
        lengthAssertedStreamSet.emplace(a[1], -1U);
    }

    BEGIN_SCOPED_REGION

    std::vector<unsigned> ordering;
    ordering.reserve(n);
    topological_sort(Relationships, std::back_inserter(ordering));
    assert (ordering.size() == n);

    for (unsigned u : reverse(ordering)) {
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
                    const auto f = lengthAssertedStreamSet.find(cast<StreamSet>(ss));
                    if (LLVM_UNLIKELY(f != lengthAssertedStreamSet.end())) {
                        f->second = sequence.size();
                    }
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
                    add_edge(a, b, RateData{n, binding}, G);
                }
            );
        }
    }

    // Now scan through the graph and determine where every non-Fixed relationship exists
    // so that we can construct our initial set of partitions. The goal here is to act as
    // a first pass to further simplify the problem before using Z3.

    BitSet nodeRateSet(n);

    for (unsigned i = 0, nextRateId = 0; i < m; ++i) {

        nodeRateSet.reset();
        for (const auto e : make_iterator_range(in_edges(i, G))) {
            const RateData & R = G[e];
            nodeRateSet |= R.RateIds;
        }
        for (const auto e : make_iterator_range(out_edges(i, G))) {
            RateData & R = G[e];
            R.RateIds = nodeRateSet;
        }

        const auto u = sequence[i];
        const RelationshipNode & node = Relationships[u];

        if (node.Type == RelationshipNode::IsKernel) {

            bool demarcateOutputs = false;

            for (const auto e : make_iterator_range(in_edges(i, G))) {

                const RateData & R = G[e];
                const auto binding = R.Vertex;
                const RelationshipNode & rn = Relationships[binding];
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
                    demarcateOutputs = true;
                }

            }

            unsigned demarcationId = 0;
            if (LLVM_UNLIKELY(demarcateOutputs)) {
                demarcationId = nextRateId++;
            }

            // Now iterate through the outputs
            for (const auto e : make_iterator_range(out_edges(i, G))) {

                RateData & R = G[e];
                const auto binding = R.Vertex;
                const RelationshipNode & rn = Relationships[binding];
                const Binding & b = rn.Binding;
                const ProcessingRate & rate = b.getRate();

                if (LLVM_UNLIKELY(demarcateOutputs)) {
                    R.RateIds.set(demarcationId);
                }

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
                    R.RateIds.set(nextRateId++);
                }

            }
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
                nodeRateSet.reset();
                for (const auto e : make_iterator_range(in_edges(i, G))) {
                    const RateData & R = G[e];
                    nodeRateSet |= R.RateIds;
                }
                unsigned partitionId = 0;
                if (LLVM_LIKELY(nodeRateSet.any())) {
                    auto f = partitionSets.find(nodeRateSet);
                    if (f == partitionSets.end()) {
                        partitionId = nextPartitionId++;
                        partitionSets.emplace(nodeRateSet, partitionId);
                    } else {
                        partitionId = f->second;
                    }
                }
                partitionIds[i] = partitionId;
            }
        }
        return nextPartitionId;
    };

    const auto l = writePartitionIds();

    assert (l > 0);

    // From the first pass analysis, determine the synchronous dataflow of each partition
    // so that we can correctly identify how much data will pass through the channels.

    std::vector<Rational> numOfStridesMultiplier(m);

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

    BEGIN_SCOPED_REGION

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

        const auto input = in_edge(i, G);
        const auto producer = source(input, G);
        const auto prodPartId = partitionIds[producer];

        Z3_ast outRate = nullptr;

        for (const auto output : make_iterator_range(out_edges(i, G))) {
            const auto consumer = target(output, G);
            const auto consPartId = partitionIds[consumer];
            // we're only concerned with intra-partition relationships here
            if (prodPartId != consPartId) {
                continue;
            }

            auto makeFixedRateVar = [&](const RateData & R, const unsigned kernel) {
                const RelationshipNode & rn = Relationships[R.Vertex];
                const Binding & binding = rn.Binding;
                const ProcessingRate & rate = binding.getRate();
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
                outRate = makeFixedRateVar(G[input], producer);
            }
            const Z3_ast inRate = makeFixedRateVar(G[output], consumer);
            hard_assert(Z3_mk_eq(ctx, outRate, inRate));
        }
    }

    if (LLVM_UNLIKELY(Z3_solver_check(ctx, solver) == Z3_L_FALSE)) {
        report_fatal_error("Z3 failed to find an intra-partition dataflow solution");
    }

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

    END_SCOPED_REGION

    LinkedPartitionGraph L(l);

    BEGIN_SCOPED_REGION

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

    using FusionConstaints = SmallVector<Z3_ast, 8>;

    using PartitionFusionGraph = adjacency_list<hash_setS, vecS, bidirectionalS, PartitionVariables, FusionConstaints>;

    const auto tQE = Z3_mk_tactic(ctx, "qe");
    Z3_tactic_inc_ref (ctx, tQE);
    const auto tSMT = Z3_mk_tactic(ctx, "smt");
    Z3_tactic_inc_ref (ctx, tSMT);
    const auto tactics = Z3_tactic_and_then (ctx, tQE, tSMT);

    const auto solver = Z3_mk_solver_from_tactic(ctx, tactics);
    Z3_solver_inc_ref(ctx, solver);

    Z3_params params = Z3_mk_params(ctx);
    Z3_params_inc_ref(ctx, params);
    Z3_symbol r = Z3_mk_string_symbol(ctx, ":timeout");
    Z3_params_set_uint(ctx, params, r, 1000);
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

    PartitionFusionGraph P(l);

    for (unsigned i = 0; i < l; ++i) {
        const auto var = Z3_mk_fresh_const(ctx, nullptr, intType);
        PartitionVariables & V = P[i];
        V.addVar(ctx, var);
        hard_assert(Z3_mk_ge(ctx, var, ONE));
        V.setRoot(ctx, var);
    }

    flat_map<unsigned, Z3_ast> symbolicPopCountRateVar;

    for (unsigned i = 0; i < m; ++i) {
        const auto u = sequence[i];
        const RelationshipNode & node = Relationships[u];
        if (node.Type == RelationshipNode::IsRelationship) {
            assert (isa<StreamSet>(Relationships[u].Relationship));
            const auto input = in_edge(i, G);
            const auto producer = source(input, G);
            const auto prodPartId = partitionIds[producer];
            for (const auto output : make_iterator_range(out_edges(i, G))) {
                const auto consumer = target(output, G);
                const auto consPartId = partitionIds[consumer];
                // we're only concerned with inter-partition relationships here
                if (prodPartId == consPartId) {
                    continue;
                }
                if (!edge(prodPartId, consPartId, P).second) {
                    add_edge(prodPartId, consPartId, FusionConstaints{}, P);
                }
            }
        }
    }

    for (unsigned i = 0; i < m; ++i) {
        const auto u = sequence[i];
        const RelationshipNode & node = Relationships[u];
        if (node.Type == RelationshipNode::IsRelationship) {
            assert (isa<StreamSet>(Relationships[u].Relationship));
            const auto input = in_edge(i, G);
            const auto producer = source(input, G);
            const auto prodPartId = partitionIds[producer];

            Z3_ast outRateVar = nullptr;
            Z3_ast outRateExpr = nullptr;


            for (const auto output : make_iterator_range(out_edges(i, G))) {
                const auto consumer = target(output, G);
                const auto consPartId = partitionIds[consumer];
                // we're only concerned with inter-partition relationships here
                if (prodPartId == consPartId) {
                    continue;
                }

                const auto e = edge(prodPartId, consPartId, P);
                assert (e.second);
                FusionConstaints & fusionConstraints = P[e.first];

                auto makeBoundedVar = [&](PartitionVariables & V, const Rational & lb, const Rational & ub) {
                    const auto var = Z3_mk_fresh_const(ctx, nullptr, intType);
                    V.addVar(ctx, var);
                    hard_assert(Z3_mk_ge(ctx, var, Z3_mk_int(ctx, floor(lb), intType)));
                    hard_assert(Z3_mk_le(ctx, var, Z3_mk_int(ctx, ceiling(ub), intType)));
                    return var;
                };

                auto calculateBaseSymbolicRateVar = [&](const RateData & R, const ProcessingRate & rate, const unsigned kernel, const unsigned partId)  {
                    const RelationshipNode & node = Relationships[sequence[kernel]];
                    assert (node.Type == RelationshipNode::IsKernel);

                    PartitionVariables & V = P[partId];

                    Z3_ast var = nullptr;
                    Z3_ast expr = nullptr;

                    auto getReferenceBinding = [&]() {
                        for (const auto e : make_iterator_range(in_edges(R.Vertex, Relationships))) {
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
                            assert (outRateVar);
                            if (rate.getLowerBound() > Rational{0}) {
                                fusionConstraints.push_back(Z3_mk_ge(ctx, outRateExpr, constant_real(rate.getLowerBound())));
                            }
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
                                V.addVar(ctx, var);
                                symbolicPopCountRateVar.emplace(refStreamSet, var);
                            } else {
                                var = f->second;
                            }

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

                    IORateVarMap.emplace(R.Vertex, IORateVar{var, expr});
                    return std::make_pair(var, expr);
                };


                // if this is the first instance of an inter-partition relationship for
                // this streamset, then determine the output rate.
                if (outRateExpr == nullptr) {

                    const RateData & R = G[input];
                    const RelationshipNode & rn = Relationships[R.Vertex];
                    const Binding & binding = rn.Binding;
                    const ProcessingRate & rate = binding.getRate();

                    std::tie(outRateVar, outRateExpr) = calculateBaseSymbolicRateVar(R, rate, producer, prodPartId);

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

                }

                const RateData & R = G[output];
                const RelationshipNode & rn = Relationships[R.Vertex];
                const Binding & binding = rn.Binding;
                const ProcessingRate & rate = binding.getRate();

                Z3_ast inRateExpr = calculateBaseSymbolicRateVar(R, rate, consumer, consPartId).second;

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
                            inRateExpr = Z3_mk_mod(ctx, inRateExpr, Z3_mk_int(ctx, attr.amount(), intType) );
                            END_SCOPED_REGION
                            break;
                        default: break;
                    }
                }
                fusionConstraints.push_back(Z3_mk_eq(ctx, outRateExpr, inRateExpr));
            }
        }
    }

    for (const LengthAssertion & a : mLengthAssertions)  {
        Z3_ast args[2];
        bool necessary = true;
        for (unsigned i = 0; i < 2; ++i) {
            const auto f = lengthAssertedStreamSet.find(cast<StreamSet>(a[i]));
            if (LLVM_UNLIKELY(f == lengthAssertedStreamSet.end())) {
                necessary = false;
                break;
            } else {
                const auto streamSet = f->second;
                const auto g = first_in_edge(streamSet, Relationships);
                assert (Relationships[g].Reason != ReasonType::Reference);
                const auto binding = source(g, Relationships);
                assert (Relationships[binding].Type == RelationshipNode::IsBinding);
                const auto h = IORateVarMap.find(binding);
                assert (h != IORateVarMap.end());
                const IORateVar & prodRate = h->second;
                args[i] = prodRate.Expr;
            }
        }
        if (necessary) {
            // if for some reason adding this constraint makes the formula unsat,
            // discard it.
            Z3_solver_push(ctx, solver);
            hard_assert(Z3_mk_eq(ctx, args[0], args[1]));
            if (LLVM_UNLIKELY(Z3_solver_check(ctx, solver) != Z3_L_TRUE)) {
                Z3_solver_pop(ctx, solver, 1);
            }
        }
    }

    for (const auto e : make_iterator_range(edges(P))) {

        Z3_solver_push(ctx, solver);

        const auto & fusionConstraints = P[e];

        Z3_ast body = Z3_mk_and(ctx, fusionConstraints.size(), fusionConstraints.data());

        const auto t = target(e, P);
        auto & Co = P[t].Vars;
        Z3_ast exists = Z3_mk_exists_const(ctx, 0, Co.size(), &*Co.begin(), 0, nullptr, body);

        const auto s = source(e, P);
        auto & Pr = P[s].Vars;
        Z3_ast forall = Z3_mk_forall_const(ctx, 0, Pr.size(), &*Pr.begin(), 0, nullptr, exists);

        hard_assert(forall);

        const auto r = Z3_solver_check(ctx, solver);

        if (LLVM_UNLIKELY(r == Z3_L_TRUE)) {
            add_edge(s, t, L);
        }

        Z3_solver_pop(ctx, solver, 1);

    }

    Z3_tactic_dec_ref (ctx, tQE);
    Z3_tactic_dec_ref (ctx, tSMT);
    Z3_solver_dec_ref(ctx, solver);

    END_SCOPED_REGION

    printGraph(L, errs(), "L");

    std::vector<unsigned> linkedPartition(l);

    connected_components(L, linkedPartition.data());

#error here!

//    Z3_model_dec_ref(ctx, model);
//    Z3_solver_dec_ref(ctx, solver);


    Z3_del_context(ctx);
    Z3_reset_memory();

    for (unsigned i = 0, nextRateId = 0; i < m; ++i) {

        nodeRateSet.reset();
        for (const auto e : make_iterator_range(in_edges(i, G))) {
            const RateData & R = G[e];
            nodeRateSet |= R.RateIds;
        }
        for (const auto e : make_iterator_range(out_edges(i, G))) {
            RateData & R = G[e];
            R.RateIds = nodeRateSet;
        }

        const auto u = sequence[i];
        const RelationshipNode & node = Relationships[u];

        if (node.Type == RelationshipNode::IsKernel) {

            bool demarcateOutputs = in_degree(u, G) == 0;

            // Check whether this (internal) kernel could terminate early
            const auto kernelObj = node.Kernel;
            if (kernelObj != mPipelineKernel) {
                demarcateOutputs |= kernelObj->canSetTerminateSignal();
                demarcateOutputs |= kernelObj->hasAttribute(AttrId::InternallySynchronized);
                // TODO: an internally synchronzied kernel with fixed rate I/O can be contained within a partition
                // but cannot be the root of a non-isolated partition. To permit them to be roots, they'd need
                // some way of informing the pipeline as to how many strides they executed or the pipeline
                // would need to know to calculate it from its outputs. Rather than handling this complication,
                // for now we simply prevent this case.
            }

            if (LLVM_UNLIKELY(demarcateOutputs)) {
                const auto demarcationId = nextRateId++;
                for (const auto e : make_iterator_range(out_edges(i, G))) {
                    RateData & R = G[e];
                    R.RateIds.set(demarcationId);
                }
            }

        } else {



        }
    }

    // Now test every Fixed-rate partition to determine whether its worthwhile keeping them
    // in the same partition since every


    exit(-1);
#endif
    return PartitionGraph{};
}



#if 0

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief identifyKernelPartitions
 ** ------------------------------------------------------------------------------------------------------------- */
PartitionGraph PipelineAnalysis::identifyKernelPartitions() {

    using FixedRateReachability = adjacency_matrix<directedS>;

    using BitSet = dynamic_bitset<>;

    using Graph = adjacency_list<vecS, vecS, bidirectionalS, std::array<BitSet, 2>>;
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

    enum Stage {
        ConsiderEarlyTermination = 0
        , IgnoreEarlyTermination = 1
    };

    #warning TODO: if this works, revise the algorithm to append the termination flags afterwards


    flat_map<unsigned, std::array<unsigned, 2>> partitionMap;
    partitionMap.reserve(numOfKernels);


    std::array<unsigned, 2> KernelPartitionIdCount;


    Graph H(numOfKernels + numOfStreamSets);

    for (unsigned stage = ConsiderEarlyTermination; stage <= IgnoreEarlyTermination; ++stage) {


        flat_map<ProgramGraph::vertex_descriptor, Vertex> M;

        auto mapStreamSet = [&](const ProgramGraph::vertex_descriptor u) -> Vertex {
            assert (Relationships[u].Type == RelationshipNode::IsRelationship);
            assert (isa<StreamSet>(Relationships[u].Relationship));
            const auto f = streamSets.find(u);
            assert (f != streamSets.end());
            return numOfKernels + static_cast<unsigned>(std::distance(streamSets.begin(), f));
        };

        unsigned nextRateId = 0;

        auto addRateId = [&stage](std::array<BitSet, 2> & bitSets, const unsigned rateId) {
            auto & bv = bitSets[stage];
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
                if (stage == ConsiderEarlyTermination) {
                    mayTerminateEarly = kernelObj->canSetTerminateSignal();
                }
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
        std::vector<unsigned> rateOrdering;
        rateOrdering.reserve(num_vertices(H));
        if (LLVM_UNLIKELY(!lexical_ordering(H, rateOrdering))) {
            report_fatal_error("Cannot lexically order the partition graph!");
        }

        for (const auto u : rateOrdering) {
            auto & nodeRateSet = H[u][stage];
            // boost dynamic_bitset will segfault when buffers are of differing lengths.
            nodeRateSet.resize(nextRateId);
            for (const auto e : make_iterator_range(in_edges(u, H))) {
                const auto v = source(e, H);
                const auto & inputRateSet = H[v][stage];
                nodeRateSet |= inputRateSet;
            }
        }

        PartitionMap partitionSets;

        unsigned nextPartitionId = 1;
        for (unsigned i = 0; i < numOfKernels; ++i) {
            const BitSet & bitSet = H[i][stage];
            unsigned partitionId = 0;
            if (LLVM_LIKELY(bitSet.any())) {
                auto f = partitionSets.find(bitSet);
                if (f == partitionSets.end()) {
                    partitionId = nextPartitionId++;
                    partitionSets.emplace(bitSet, partitionId);
                } else {
                    partitionId = f->second;
                }
            }
            const auto u = kernelSequence[i];
            auto f = partitionMap.emplace(u, std::array<unsigned, 2>{});
            f.first->second[stage] = partitionId;

        }

        KernelPartitionIdCount[stage] = nextPartitionId;
    }


    // NOTE: summarizing the partitions here into a graph then taking the topological
    // order of them to determine the partition ids is almost certainly overkill but
    // was done to future proof this portion of the algorithm. Counting them initially
    // in reverse (given that the kernels are in topological order) ought to account
    // for any scenarios were given a lexographic ordering of the original graph,
    // the ordering of one partition is bisected by kernels in another partition.

    using SummaryGraph = adjacency_list<vecS, vecS, bidirectionalS, Partition, unsigned>;

    const auto n = KernelPartitionIdCount[ConsiderEarlyTermination];

    SummaryGraph S(n);

    for (unsigned i = 1; i < n; ++i) {
        add_edge(0, i, 0, S);
    }

    LinkedPartitionGraph C(n);

    for (const auto streamSet : streamSets) {

        assert (Relationships[streamSet].Type == RelationshipNode::IsRelationship);
        assert (isa<StreamSet>(Relationships[streamSet].Relationship));

        const auto o1 = first_in_edge(streamSet, Relationships);
        assert (Relationships[o1].Reason != ReasonType::Reference);
        const auto output = source(o1, Relationships);
        assert (Relationships[output].Type == RelationshipNode::IsBinding);
        const auto o2 = first_in_edge(output, Relationships);
        assert (Relationships[o2].Reason != ReasonType::Reference);

        const auto producer = source(o2, Relationships);
        assert (Relationships[producer].Type == RelationshipNode::IsKernel);

        const auto f = partitionMap.find(producer);
        assert (f != partitionMap.end());
        const auto producer_pid = f->second;

        for (const auto i1 : make_iterator_range(out_edges(streamSet, Relationships))) {

            assert (Relationships[i1].Reason != ReasonType::Reference);
            const auto input = target(i1, Relationships);
            assert (Relationships[input].Type == RelationshipNode::IsBinding);
            const auto i2 = first_out_edge(input, Relationships);
            assert (Relationships[i2].Reason != ReasonType::Reference);
            const auto consumer = target(i2, Relationships);
            assert (Relationships[consumer].Type == RelationshipNode::IsKernel);


            const auto f = partitionMap.find(consumer);
            assert (f != partitionMap.end());
            const auto consumer_pid = f->second;

            const auto a = producer_pid[ConsiderEarlyTermination];
            const auto b = consumer_pid[ConsiderEarlyTermination];

            if (a == b) {
                assert (producer_pid[IgnoreEarlyTermination] == consumer_pid[IgnoreEarlyTermination]);
            } else {
                add_edge(a, b, streamSet, S);
                if (producer_pid[IgnoreEarlyTermination]  != consumer_pid[IgnoreEarlyTermination]) {
                    add_edge(a, b, C);
                }
            }

        }
    }


    ordering.clear();
    if (LLVM_UNLIKELY(!lexical_ordering(S, ordering))) {
        report_fatal_error("Cannot lexically order the partition summary graph!");
    }

    assert (ordering[0] == 0);

    LinkedPartitionGraph L(n);

    for (const auto e : make_iterator_range(edges(S))) {
        const auto a = source(e, S);
        const auto b = target(e, S);
        if (!edge(a, b, C).second) {
            add_edge(ordering[a], ordering[b], L);
            add_edge(a, b, C);
        }
    }

    PartitionGraph P(n, std::move(L));

    PartitionIds.reserve(numOfKernels);

    for (unsigned i = 0; i < numOfKernels; ++i) {
        const auto v = kernelSequence[i];

        const auto f = partitionMap.find(v);
        assert (f != partitionMap.end());
        const auto partitionId = f->second[ConsiderEarlyTermination];

        const auto j = ordering[partitionId];

        P[j].Kernels.push_back(v);
        PartitionIds.emplace(v, j);
    }

    std::vector<unsigned> reverseMapping(n);
    for (unsigned i = 1; i < n; ++i) {
        reverseMapping[ordering[i]] = i;
    }

    for (unsigned i = 1; i < n; ++i) {
        const auto j = ordering[i];
        for (const auto e : make_iterator_range(out_edges(j, S))) {
            const auto k = reverseMapping[target(e, S)];
            const auto t = S[e];
            assert (t < num_vertices(Relationships));
            add_edge(i, k, t, P);
        }
    }

    PartitionCount = n;

    return P;
}

#endif

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
