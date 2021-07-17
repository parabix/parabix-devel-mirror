#ifndef PARTITIONING_ANALYSIS_HPP
#define PARTITIONING_ANALYSIS_HPP

#include "pipeline_analysis.hpp"
#include <toolchain/toolchain.h>
#include <util/slab_allocator.h>

namespace kernel {

// #define PRINT_PARTITION_COMBINATIONS

#define USE_SIBLING_PARTITION_TEST

// #define PRINT_PARTITION_STATS

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

    #ifndef NDEBUG
    unsigned numOfKernelsInGraph = 2;
    #endif

    BEGIN_SCOPED_REGION

    std::vector<unsigned> ordering;
    ordering.reserve(n);
    if (LLVM_UNLIKELY(!lexical_ordering(Relationships, ordering))) {
        report_fatal_error("Failed to generate acyclic partition graph from kernel ordering");
    }

    // Convert the relationship graph into a simpler graph G that we can annotate.
    // For simplicity, force the pipeline input to be the first and the pipeline output
    // to be the last one.

    // For some reason, the Mac C++ compiler cannot link the constexpr PipelineInput value?
    // Hardcoding 0 here as a temporary workaround.
    mapping[0] = 0;
    sequence.push_back(0);

    for (unsigned u : ordering) {
        const RelationshipNode & node = Relationships[u];
        switch (node.Type) {
            case RelationshipNode::IsKernel:
                BEGIN_SCOPED_REGION
                #ifndef NDEBUG
                const auto & R = Relationships[u];
                #endif
                if (u == PipelineInput || u == PipelineOutput) {
                    assert (R.Kernel == mPipelineKernel);
                } else {
                    assert (R.Kernel != mPipelineKernel);
                    mapping[u] = sequence.size();
                    sequence.push_back(u);
                    #ifndef NDEBUG
                    ++numOfKernelsInGraph;
                    #endif
                }
                END_SCOPED_REGION
                break;
            case RelationshipNode::IsRelationship:
                BEGIN_SCOPED_REGION
                const Relationship * const ss = Relationships[u].Relationship;
                if (LLVM_LIKELY(isa<StreamSet>(ss))) {
                    mapping[u] = sequence.size();
                    sequence.push_back(u);
                }
                END_SCOPED_REGION
                break;
            default: break;
        }
    }

    mapping[PipelineOutput] = sequence.size();
    sequence.push_back(PipelineOutput);

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

                    // Check the attributes to see whether any impose a partition change
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

                // Check the attributes to see whether any impose a partition change
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

    std::vector<unsigned> partitionIds(m + 1);

    auto convertUniqueNodeBitSetsToUniquePartitionIds = [&]() {
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
                    assert (partitionId > 0);
                } else {
                    assert (node.Kernel == mPipelineKernel);
                }
                partitionIds[i] = partitionId;
            }
        }
        return nextPartitionId;
    };

#ifdef PRINT_PARTITION_STATS

    auto countStats = [&](const bool inputOnly, std::vector<unsigned> & counts) {

        unsigned kernelCount = 0;
        unsigned streamSetCount = 0;
        unsigned fixedRateCount = 0;
        unsigned boundedRateCount = 0;
        unsigned popCountRateCount = 0;
        unsigned greedyRateCount = 0;
        unsigned lookAheadCount = 0;


        for (unsigned i = 0; i < n; ++i) {
            const RelationshipNode & node = Relationships[i];
            switch (node.Type) {
                case RelationshipNode::IsKernel:
                    kernelCount++;
                    break;
                case RelationshipNode::IsRelationship:
                    BEGIN_SCOPED_REGION
                    const Relationship * const ss = Relationships[i].Relationship;
                    if (LLVM_LIKELY(isa<StreamSet>(ss))) {
                        streamSetCount++;
                    }
                    END_SCOPED_REGION
                    break;
                case RelationshipNode::IsBinding:
                    BEGIN_SCOPED_REGION
                    const auto input = first_out_edge(i, Relationships);
                    assert (Relationships[input].Reason != ReasonType::Reference);
                    const auto consumer = target(input, Relationships);

                    if (inputOnly && (Relationships[consumer].Type != RelationshipNode::IsKernel)) {
                        break;
                    }

                    const RelationshipNode & bind = Relationships[i];
                    const Binding & binding = bind.Binding;
                    const ProcessingRate & rate = binding.getRate();
                    switch (rate.getKind()) {
                        case RateId::Fixed:
                            fixedRateCount++;
                            break;
                        case RateId::Bounded:
                            boundedRateCount++;
                            break;
                        case RateId::PartialSum:
                            popCountRateCount++;
                            break;
                        case RateId::Greedy:
                            greedyRateCount++;
                            break;
                        default:
                            llvm_unreachable("");
                    }

                    if (binding.hasLookahead()) {
                        lookAheadCount++;
                    }
                    END_SCOPED_REGION

                default: break;
            }
        }

        counts.resize(7);
        counts[0] = kernelCount-2;
        counts[1] = streamSetCount;
        counts[2] = fixedRateCount;
        counts[3] = boundedRateCount;
        counts[4] = popCountRateCount;
        counts[5] = greedyRateCount;
        counts[6] = lookAheadCount;

    };

    std::vector<unsigned> inputOnly(7);

    countStats(true, inputOnly);

    std::vector<unsigned> all(7);

    countStats(false, all);

    assert (inputOnly[0] == all[0]);
    assert (inputOnly[1] == all[1]);

    errs() << "&" << inputOnly[0]
           << "&" << inputOnly[1];

    for (unsigned i = 2; i < 7; ++i) {
        if (inputOnly[i] == all[i]) {
            errs() << "&\\multicolumn{2}{r";
            if (i == 6) {
                errs() << "|";
            }
            errs() << "}{" << inputOnly[i] << "}";
        } else {
            errs() << "&" << inputOnly[i] << "&(" << all[i] << ")";
        }
    }

    errs() << "\\\\\n";


#endif

    auto synchronousPartitionCount = convertUniqueNodeBitSetsToUniquePartitionIds();

#ifndef PRINT_PARTITION_STATS

    assert (synchronousPartitionCount > 0);

    std::vector<Rational> partitionRepetitionVector(m);

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

        std::vector<Z3_ast> repetitions(m);

        for (unsigned i = 0; i < m; ++i) {
            const auto u = sequence[i];
            const RelationshipNode & currentNode = Relationships[u];
            Z3_ast var = nullptr;
            if (currentNode.Type == RelationshipNode::IsKernel) {
                var = Z3_mk_fresh_const(ctx, nullptr, intType);
                hard_assert(Z3_mk_ge(ctx, var, ONE));
            }
            repetitions[i] = var;
        }

        for (unsigned streamSet = 0; streamSet < m; ++streamSet) {
            if (repetitions[streamSet]) {
                continue;
            }

            const auto output = in_edge(streamSet, G);
            const auto producer = source(output, G);
            const auto prodPartId = partitionIds[producer];

            const RelationshipNode & ouputNode = Relationships[G[output]];
            const Binding & outputBinding = ouputNode.Binding;
            const ProcessingRate & outputRate = outputBinding.getRate();

            if (LLVM_LIKELY(outputRate.isFixed())) {

                Z3_ast outRate = nullptr;

                for (const auto input : make_iterator_range(out_edges(streamSet, G))) {
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

        const auto model = Z3_solver_get_model(ctx, solver);
        Z3_model_inc_ref(ctx, model);
        for (unsigned i = 0; i < m; ++i) {
            if (repetitions[i]) {
                Z3_ast value;
                if (LLVM_UNLIKELY(Z3_model_eval(ctx, model, repetitions[i], Z3_L_TRUE, &value) != Z3_L_TRUE)) {
                    report_fatal_error("Unexpected Z3 error when attempting to obtain value from model!");
                }
                Z3_int64 num, denom;
                if (LLVM_UNLIKELY(Z3_get_numeral_rational_int64(ctx, value, &num, &denom) != Z3_L_TRUE)) {
                    report_fatal_error("Unexpected Z3 error when attempting to convert model value to number!");
                }
                assert (num > 0 && denom == 1);
                partitionRepetitionVector[i] = Rational{num, denom};

            }
        }

        Z3_model_dec_ref(ctx, model);
        Z3_solver_dec_ref(ctx, solver);
        Z3_del_context(ctx);

    };

    // Stage 3: fuse stride-linked partitions

    // Analyze the partition I/O channel rates to determine whether any partitions
    // are synchronous and fuse them. Note: this algorithm assumes no program permits
    // the unbounded accumulation of data along any channel and explicitly ignores
    // detection of this case when attempting to fuse partitions.

    auto fuseStrideLinkedPartitions = [&]() {

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
            void setRoot(Z3_ast root) {
                StrideVar = root;
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

        const auto ONE = Z3_mk_int(ctx, 1, intType);
        const auto TWO = Z3_mk_int(ctx, 2, intType);

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

        PartitionFusionGraph P(synchronousPartitionCount);

        for (unsigned i = 0; i < synchronousPartitionCount; ++i) {
            for (const auto e : make_iterator_range(out_edges(i, H))) {
                add_edge(i, target(e, H), FusionConstaints{}, P);
            }
        }

        #ifdef PRINT_PARTITION_COMBINATIONS
        BEGIN_SCOPED_REGION
        auto & out = errs();
        out << "digraph \"P\" {\n";
        for (unsigned partitionId = 0; partitionId < synchronousPartitionCount; ++partitionId) {
            out << "v" << partitionId << " [label=\"P" << partitionId << "\n";
            for (unsigned i = 0; i < m; ++i) {
                if (partitionIds[i] == partitionId) {
                    const auto u = sequence[i];
                    const RelationshipNode & node = Relationships[u];
                    if (node.Type == RelationshipNode::IsKernel) {
                        out << u << ". " << node.Kernel->getName() << "\n";
                    }
                }
            }
            out << "\"];\n";
        }
        for (auto e : make_iterator_range(edges(P))) {
            const auto s = source(e, P);
            const auto t = target(e, P);
            out << "v" << s << " -> v" << t << ";\n";
        }
        out << "}\n\n";
        out.flush();
        END_SCOPED_REGION
        #endif

        transitive_reduction_dag(H);

        for (unsigned i = 0; i < synchronousPartitionCount; ++i) {
            const auto var = Z3_mk_fresh_const(ctx, "partition", intType);
            hard_assert(Z3_mk_ge(ctx, var, ONE));
            hard_assert(Z3_mk_le(ctx, var, TWO));
            PartitionVariables & V = P[i];
            V.setRoot(var);
            V.addVar(ctx, var);
        }

        flat_map<unsigned, Z3_ast> symbolicPopCountRateVar;

        flat_map<unsigned, IORateVar> IORateVarMap;

        IORateVarMap.reserve(num_edges(G));

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
                    assert ("no matching edge in fusion constraint graph?" && e.second);

                    FusionConstaints & fusionConstraints = P[e.first];

                    auto calculateBaseSymbolicRateVar = [&](const BindingVertex bindingVertex,
                                                            const PortType portType, const ProcessingRate & rate,
                                                            const unsigned kernel, const unsigned partId)  {
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

                        assert (partitionRepetitionVector[kernel] > 0);

                        const auto strideSize = node.Kernel->getStride() * partitionRepetitionVector[kernel];

                        switch (rate.getKind()) {
                            case RateId::Fixed:
                                BEGIN_SCOPED_REGION
                                const auto numOfItems = rate.getRate() * strideSize;
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
                                const auto lb = rate.getLowerBound() * strideSize;
                                const auto ub = rate.getUpperBound() * strideSize;
                                var = Z3_mk_fresh_const(ctx, "bounded", intType);
                                V.addVar(ctx, var);
                                hard_assert(Z3_mk_ge(ctx, var, Z3_mk_int(ctx, floor(lb), intType)));
                                hard_assert(Z3_mk_le(ctx, var, Z3_mk_int(ctx, ceiling(ub), intType)));
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
                                    var = Z3_mk_fresh_const(ctx, "partialSumVar", intType);
                                    hard_assert(Z3_mk_ge(ctx, var, Z3_mk_int(ctx, 0, intType)));
                                    symbolicPopCountRateVar.emplace(refStreamSet, var);
                                } else {
                                    var = f->second;
                                }
                                V.addVar(ctx, var);
                                Z3_ast args[3] = { var, V.StrideVar, constant_real(strideSize) };
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
                            case RateId::Unknown:
                                BEGIN_SCOPED_REGION
                                var = Z3_mk_fresh_const(ctx, "unknown", intType);
                                V.addVar(ctx, var);
                                hard_assert(Z3_mk_ge(ctx, var, Z3_mk_int(ctx, floor(rate.getRate()), intType)));
                                expr = var;
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

                        std::tie(outRateVar, outRateExpr) = calculateBaseSymbolicRateVar(bindingVertex, PortType::Output, rate, producer, prodPartId);

                        for (const Attribute & attr : binding.getAttributes()) {
                            switch (attr.getKind()) {
                                case AttrId::Delayed:
                                    BEGIN_SCOPED_REGION
                                    Z3_ast args[2] = { outRateExpr, Z3_mk_int(ctx, attr.amount(), intType) };
                                    outRateExpr = Z3_mk_sub(ctx, 2, args);

//                                    PartitionVariables & V = P[consPartId];
//                                    const auto delayedRateVar = Z3_mk_fresh_const(ctx, nullptr, intType);
//                                    V.addVar(ctx, delayedRateVar);
//                                    hard_assert(Z3_mk_ge(ctx, delayedRateVar, min));
//                                    hard_assert(Z3_mk_le(ctx, delayedRateVar, outRateExpr));
//                                    outRateExpr = delayedRateVar;

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

                    Z3_ast inRateExpr = calculateBaseSymbolicRateVar(bindingVertex, PortType::Input, rate, consumer, consPartId).second;

                    for (const Attribute & attr : binding.getAttributes()) {
                        switch (attr.getKind()) {
                            case AttrId::LookAhead:
                            case AttrId::Delayed:
                                BEGIN_SCOPED_REGION
                                Z3_ast args[2] = { inRateExpr, Z3_mk_int(ctx, attr.amount(), intType) };
                                inRateExpr = Z3_mk_sub(ctx, 2, args);

//                                PartitionVariables & V = P[consPartId];
//                                const auto delayedRateVar = Z3_mk_fresh_const(ctx, nullptr, intType);
//                                V.addVar(ctx, delayedRateVar);
//                                hard_assert(Z3_mk_ge(ctx, delayedRateVar, min));
//                                hard_assert(Z3_mk_le(ctx, delayedRateVar, inRateExpr));
//                                inRateExpr = delayedRateVar;

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
//        std::vector<unsigned> rank(synchronousPartitionCount, 0);

        #ifdef PRINT_PARTITION_COMBINATIONS
        adjacency_list<hash_setS, vecS, undirectedS> L(synchronousPartitionCount);
        #endif

        std::function<unsigned(unsigned)> find = [&](unsigned x) {
            if (fusedPartitionIds[x] != x) {
                fusedPartitionIds[x] = find(fusedPartitionIds[x]);
            }
            return fusedPartitionIds[x];
        };

        auto union_find = [&](unsigned x, unsigned y) {

            #ifdef PRINT_PARTITION_COMBINATIONS
            add_edge(x, y, L);
            #endif

            x = find(x);
            y = find(y);

            if (fusedPartitionIds[x] > fusedPartitionIds[y]) {
                fusedPartitionIds[y] = x;
            } else {
                fusedPartitionIds[x] = y;
//                if (rank[x] == rank[y]) {
//                    rank[y]++;
//                }
            }

        };

        std::vector<Z3_ast> list;

        std::vector<Z3_app> vars;
        std::vector<Z3_app> tmp;

        BitVector universalSet(synchronousPartitionCount);

        bool fusedAny = false;

        auto make_universal_clause = [&](const Z3_ast constraint) {
            assert (vars.empty() && tmp.empty());
            for (const auto i : universalSet.set_bits()) {
                const auto & V = P[i].Vars;
                std::merge(vars.begin(), vars.end(), V.begin(), V.end(), std::back_inserter(tmp));
                vars.swap(tmp);
                tmp.clear();
            }
            assert (std::is_sorted(vars.begin(), vars.end()));
            universalSet.reset();
            const auto end = std::unique(vars.begin(), vars.end());
            const auto k = std::distance(vars.begin(), end);
            assert (k > 0);
            const auto forall = Z3_mk_forall_const(ctx, 0, k, vars.data(), 0, nullptr, constraint);
            vars.clear();
            return forall;
        };

        for (const auto e : make_iterator_range(edges(H))) {

            const auto s = source(e, H);
            const auto t = target(e, H);

            assert ("transitive reduction H contains an edge not in P?" && edge(s, t, P).second);

            const auto x = find(s);
            const auto y = find(t);

            if (x == y) {
                continue;
            }

            const auto & A = P[s];
            const auto & B = P[t];

            Z3_solver_push(ctx, solver);


            assert (vars.empty());
            assert (list.empty());
            assert (universalSet.none());

            for (const auto f : make_iterator_range(edges(P))) {
                if (find(target(f, P)) == y && find(source(f, P)) == x) {
                    universalSet.set(source(f, P));
                    for (const FusionConstraint & c : P[f]) {
                        Z3_ast e;
                        if (std::get<0>(c) == EQ) {
                            e = Z3_mk_eq(ctx, std::get<1>(c), std::get<2>(c));
                        } else {
                            e = Z3_mk_ge(ctx, std::get<1>(c), std::get<2>(c));
                        }
                        list.push_back(e);
                    }
                }
            }
            assert (universalSet.test(s));
            assert (list.size() > 0);
            const auto constraints = Z3_mk_and(ctx, list.size(), list.data());
            list.clear();

            const auto ratio = Z3_mk_fresh_const(ctx, nullptr, intType);
            hard_assert(Z3_mk_ge(ctx, ratio, ONE));
            hard_assert(Z3_mk_eq(ctx, A.StrideVar, multiply(B.StrideVar, ratio)));

//            const auto & Co = B.Vars;
//            const auto exists = Z3_mk_exists_const(ctx, 0, 1, (Z3_app*)&ratio, 0, nullptr, constraints);

            const auto & Co = B.Vars;

            assert (vars.empty());

            vars.insert(vars.end(), Co.begin(), Co.end());

            const auto f = std::lower_bound(vars.begin(), vars.end(), (Z3_app)B.StrideVar);
            assert (f != vars.end());
            vars.erase(f);

            auto clause = constraints;
            if (!vars.empty()) {
                clause = Z3_mk_exists_const(ctx, 0, vars.size(), vars.data(), 0, nullptr, constraints);
                vars.clear();
            }

            hard_assert(make_universal_clause(clause));

            #ifdef PRINT_PARTITION_COMBINATIONS
            errs() << "-----------------------------------\n";
            errs() << "PRED: " << s << " -> " << t << "\n";
            errs() << Z3_solver_to_string(ctx, solver) << "\n";
            #endif

            const auto r = Z3_solver_check(ctx, solver);

            #ifdef PRINT_PARTITION_COMBINATIONS
            errs() << "r=" << r << "\n";
            #endif

            if (LLVM_UNLIKELY(r == Z3_L_TRUE)) {
                union_find(s, t);
                fusedAny = true;
            } else {
                Z3_solver_pop(ctx, solver, 1);
            }

        }

        #ifdef USE_SIBLING_PARTITION_TEST

        if (!fusedAny) {

            BitVector sourceSet(synchronousPartitionCount);

            for (unsigned i = 0; i < synchronousPartitionCount; ++i) {

                if (out_degree(i, H) > 1) {
                    graph_traits<DependencyGraph>::out_edge_iterator begin, end;
                    std::tie(begin, end) = out_edges(i, H);

                    const auto root = find(i);

                    for (auto ei = begin; ei != end; ++ei) {
                        for (auto ej = ei; ++ej != end; ) {

                            const auto a = target(*ei, H);
                            const auto b = target(*ej, H);

                            const auto x = find(a);
                            const auto y = find(b);

                            if (x == y || x == root || y == root) {
                                continue;
                            }

                            assert (sourceSet.none());

                            for (const auto f : make_iterator_range(in_edges(a, P))) {
                                sourceSet.set(find(source(f, P)));
                            }
                            for (const auto f : make_iterator_range(in_edges(b, P))) {
                                sourceSet.set(find(source(f, P)));
                            }

                            assert (sourceSet.test(find(i)));

                            Z3_solver_push(ctx, solver);

                            assert (vars.empty());
                            assert (list.empty());
                            assert (universalSet.none());
                            for (const auto f : make_iterator_range(edges(P))) {
                                const auto t = target(f, P);
                                if ((t == a || t == b) && sourceSet.test(find(source(f, P)))) {
                                    universalSet.set(source(f, P));
                                    for (const FusionConstraint & c : P[f]) {
                                        list.push_back(Z3_mk_ge(ctx, std::get<1>(c), std::get<2>(c)));
                                    }
                                }
                            }
                            sourceSet.reset();

                            assert (universalSet.test(i));
                            assert (list.size() > 0);
                            const auto constraints = Z3_mk_and(ctx, list.size(), list.data());
                            list.clear();

                            const auto & A = P[a];
                            const auto & B = P[b];

                            // Since these are siblings in a transitive reduction of the program graph,
                            // they by definition have no relationships in which we can ensure they can
                            // be stride linked. The following ensures some form of relationship exists.
                            hard_assert(Z3_mk_eq(ctx, A.StrideVar, B.StrideVar));

                            assert (vars.empty());
                            std::merge(A.Vars.begin(), A.Vars.end(), B.Vars.begin(), B.Vars.end(), std::back_inserter(vars));
                            const auto end = std::unique(vars.begin(), vars.end());
                            const auto k = std::distance(vars.begin(), end);
                            assert (k > 0);
                            assert (std::find(vars.begin(), end, (Z3_app)A.StrideVar) != end);
                            assert (std::find(vars.begin(), end, (Z3_app)B.StrideVar) != end);
                            const auto exists = Z3_mk_exists_const(ctx, 0, k, vars.data(), 0, nullptr, constraints);
                            vars.clear();

                            hard_assert(make_universal_clause(exists));

                            #ifdef PRINT_PARTITION_COMBINATIONS
                            errs() << "-----------------------------------\n";
                            errs() << "SIBS: " << i << " -> " << a << ", "
                                               << i << " -> " << b << "\n";
                            errs() << Z3_solver_to_string(ctx, solver) << "\n";
                            #endif

                            const auto r = Z3_solver_check(ctx, solver);

                            #ifdef PRINT_PARTITION_COMBINATIONS
                            errs() << "r=" << r << "\n";
                            #endif

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

        }

        #endif

        Z3_tactic_dec_ref (ctx, tQE);
        Z3_tactic_dec_ref (ctx, tSMT);
        Z3_solver_dec_ref(ctx, solver);
        Z3_del_context(ctx);

        // Z3_finalize_memory();

        #ifdef PRINT_PARTITION_COMBINATIONS
        auto & out = errs();
        out << "graph \"L\" {\n";
        for (unsigned partitionId = 0; partitionId < synchronousPartitionCount; ++partitionId) {
            out << "v" << partitionId << " [label=\"P" << partitionId << "\n";
            for (unsigned i = 0; i < m; ++i) {
                if (partitionIds[i] == partitionId) {
                    const auto u = sequence[i];
                    const RelationshipNode & node = Relationships[u];
                    if (node.Type == RelationshipNode::IsKernel) {
                        out << u << ". " << node.Kernel->getName() << "\n";
                    }
                }
            }
            out << "\"];\n";
        }
        for (auto e : make_iterator_range(edges(L))) {
            const auto s = source(e, L);
            const auto t = target(e, L);
            out << "v" << s << " -- v" << t << ";\n";
        }
        out << "}\n\n";
        out.flush();
        #endif


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


    do {
        estimateSynchronousDataflow();
    } while (fuseStrideLinkedPartitions()); // fuse until fix-point


    // TODO: when only lookahead relationships are preventing us from fusing two partitions,
    // we could "slow down" the one with the smaller lookahead (where a partition without
    // lookahead can be considered to have one of 0). Note: we can only do this if the
    // input with the lookahead is an input to the partition and we ensure that a kernel
    // with the greatest lookahead parameter is a partition root.

    // TODO: if the only consumers of all outputs of a kernel are in the same partition,
    // and the partition is not the one the kernel resides in, can we prove whether its
    // safe to move that kernel? we would need to develop a cost model for the I/O transfer.
    // There are numerous cases in which the PopCount kernels could be moved to reduce
    // the total dynamic memory cost. The difficulty, however, is if PopCount kernel becomes
    // the new partition root it may actually be ahead of the data stream for the PartialSum
    // rate refers to, which would break any stride rate equivalence within the partition.
    // We ought to be able to use it for PartialSum output rates.

    // TODO: test every Fixed-rate partition to determine whether its worthwhile keeping
    // them in the same partition. For example, suppose kernel A has a Fixed(7) output rate
    // that is consumed by kernel B with a Fixed(8) input rate. To keep A and B in the same
    // partition, they would have to execute a multiple of 56 strides per segment. If these
    // items represent single bytes, this would be beneficial but could slow down the
    // pipeline if they represent 1K chunks.

    Z3_finalize_memory();

#endif

    // Stage 5: finalize the partition structure

    // Based on the (fused) partition ids, repeat the process used to generate the initial
    // synchronous components but also factor in early termination and any other kernel
    // attributes that could interrupt the flow of data through the program.

    flat_map<std::pair<unsigned, unsigned>, unsigned> interPartitionMap;

    assert (num_vertices(G) == m);

    for (unsigned i = 0; i < m; ++i) {
        BitSet & V = G[i];
        V.reset();
    }

    auto nextRateId = synchronousPartitionCount;

    for (unsigned i = 0; i < m; ++i) {

        BitSet & V = G[i];

        for (const auto e : make_iterator_range(in_edges(i, G))) {
            const BitSet & R = G[source(e, G)];
            V |= R;
        }

        const auto u = sequence[i];
        const RelationshipNode & node = Relationships[u];

        if (node.Type == RelationshipNode::IsKernel) {

            const auto kernelObj = node.Kernel;

            if (in_degree(i, G) == 0) {
                V.set(nextRateId++);
            }

            const auto p = partitionIds[i];
            if (p > 0) {
                V.set(p - 1);
            }

            // Check whether this (internal) kernel could terminate early
            bool demarcateOutputs = (kernelObj == mPipelineKernel);
            if (kernelObj != mPipelineKernel) {
                demarcateOutputs = kernelObj->canSetTerminateSignal();
                // TODO: an internally synchronzied kernel with fixed rate I/O can be contained within a partition
                // but cannot be the root of a non-isolated partition. To permit them to be roots, they'd need
                // some way of informing the pipeline as to how many strides they executed or the pipeline
                // would need to know to calculate it from its outputs. Rather than handling this complication,
                // for now we simply prevent this case.
                if (kernelObj->hasAttribute(AttrId::InternallySynchronized)) {
                    V.set(nextRateId++);
                    demarcateOutputs = true;
                }
            }

            if (LLVM_UNLIKELY(demarcateOutputs)) {
                const auto demarcationId = nextRateId++;
                for (const auto e : make_iterator_range(out_edges(i, G))) {
                    BitSet & R = G[target(e, G)];
                    R.set(demarcationId);
                }
            }
        }

        for (const auto e : make_iterator_range(out_edges(i, G))) {
            BitSet & R = G[target(e, G)];
            R |= V;
        }
    }

    assert (Relationships[sequence[0]].Kernel == mPipelineKernel);
    assert (Relationships[sequence[m - 1]].Kernel == mPipelineKernel);

    G[0].reset();
    G[m - 1].set(nextRateId);

    const auto demarcatedPartitionCount = convertUniqueNodeBitSetsToUniquePartitionIds();

    assert (demarcatedPartitionCount > 1);

    // Stage 6: split disconnected components within a partition into separate partitions

    using DCGraph = adjacency_list<vecS, vecS, bidirectionalS>;

    DCGraph D(m);

    auto findIndex = [&](const unsigned vertex) {
        const auto s = std::find(sequence.begin(), sequence.end(), vertex);
        assert (s != sequence.end());
        return std::distance(sequence.begin(), s);
    };

    for (unsigned i = 0; i < m; ++i) {

        const auto producer = sequence[i];
        const RelationshipNode & node = Relationships[producer];

        if (node.Type == RelationshipNode::IsKernel) {
            const auto prodPartId = partitionIds[i];

            for (const auto e : make_iterator_range(out_edges(producer, Relationships))) {
                const auto output = target(e, Relationships);
                if (Relationships[output].Type == RelationshipNode::IsBinding) {
                    const auto f = first_out_edge(output, Relationships);
                    assert (Relationships[f].Reason != ReasonType::Reference);
                    const auto streamSet = target(f, Relationships);
                    assert (Relationships[streamSet].Type == RelationshipNode::IsRelationship);
                    assert (isa<StreamSet>(Relationships[streamSet].Relationship));
                    const auto j = findIndex(streamSet);
                    add_edge(i, j, D);
                    for (const auto g : make_iterator_range(out_edges(streamSet, Relationships))) {
                        assert (Relationships[g].Reason != ReasonType::Reference);
                        const auto input = target(g, Relationships);
                        assert (Relationships[input].Type == RelationshipNode::IsBinding);
                        const auto h = first_out_edge(input, Relationships);
                        assert (Relationships[h].Reason != ReasonType::Reference);
                        const auto consumer = target(h, Relationships);
                        assert (Relationships[consumer].Type == RelationshipNode::IsKernel);
                        const auto k = findIndex(consumer);
                        const auto consPartId = partitionIds[k];
                        assert (consPartId > 0);
                        if (prodPartId == consPartId) {
                            add_edge(j, k, D);
                        }
                    }
                }
            }

            if (prodPartId == 0) {
                assert (node.Kernel == mPipelineKernel);
                if (i > 0) {
                    add_edge(0, i, D);
                    assert (i == (m - 1));
                }
            }
        }
    }

    const auto partitionCount = connected_components(D, &partitionIds[0]);

    assert (partitionCount > 1);

    partitionIds[0] = 0;

    using RenumberingGraph = adjacency_list<vecS, vecS, bidirectionalS, no_property, unsigned>;

    // Stage 7: renumber the partition ids

    // To simplify processing later, renumber the partitions such that the partition id
    // of any predecessor of a kernel K is <= the partition id of K.

    RenumberingGraph T(partitionCount);

    for (unsigned i = 1; i < partitionCount; ++i) {
        add_edge(0, i, 0, T);
    }

    for (unsigned i = 1; i < m; ++i) {
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
                    assert (consPartId > 0);
                    add_edge(prodPartId, consPartId, u, T);
                }
            }
        }
    }

    for (unsigned i = 1; i < (partitionCount - 1); ++i) {
        if (out_degree(i, T) == 0) {
            add_edge(i, partitionCount - 1, 0, T);
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
        const auto j = renumberingSeq[i];
        assert (j < partitionCount);
        renumbered[j] = i;
    }

    assert (renumbered[0] == 0);

    PartitionGraph P(partitionCount);

    for (unsigned i = 0; i < m; ++i) {
        const auto u = sequence[i];
        const RelationshipNode & node = Relationships[u];
        if (node.Type == RelationshipNode::IsKernel) {

            assert (partitionIds[i] < partitionCount);
            const auto j = renumbered[partitionIds[i]];
            assert (j < partitionCount);

            assert ((j > 0 && (j + 1) < partitionCount) ^ (node.Kernel == mPipelineKernel));

            P[j].Kernels.push_back(u);
            PartitionIds.emplace(u, j);
        }
    }

    #ifndef NDEBUG
    BEGIN_SCOPED_REGION
    flat_set<unsigned> included;
    included.reserve(numOfKernelsInGraph);
    for (const auto u : P[0].Kernels) {
        assert ("kernel is in multiple partitions?" && included.insert(u).second);
        const auto & R = Relationships[u];
        assert (R.Type == RelationshipNode::IsKernel);
        assert (R.Kernel == mPipelineKernel);
    }
    auto numOfPartitionedKernels = P[0].Kernels.size();
    for (unsigned i = 1; i < partitionCount; ++i) {
        numOfPartitionedKernels += P[i].Kernels.size();
        for (const auto u : P[i].Kernels) {
            assert ("kernel is in multiple partitions?" && included.insert(u).second);
            const auto & R = Relationships[u];
            assert (R.Type == RelationshipNode::IsKernel);
        }
    }
    assert (numOfPartitionedKernels == numOfKernelsInGraph);
    END_SCOPED_REGION
    #endif

    flat_set<std::pair<unsigned, unsigned>> duplicateFilter;

    for (unsigned i = 0; i < partitionCount; ++i) {
        assert (P[i].Kernels.size() > 0);
        const auto j = renumbered[i];
        assert (duplicateFilter.empty());
        for (const auto e : make_iterator_range(out_edges(i, T))) {
            const auto k = renumbered[target(e, T)];
            assert (k > j);
            const auto streamSet = T[e];
            if (LLVM_UNLIKELY(streamSet == 0)) continue;
            assert (streamSet < num_vertices(Relationships));
            assert (Relationships[streamSet].Type == RelationshipNode::IsRelationship);
            if (duplicateFilter.emplace(k, streamSet).second) {
                add_edge(j, k, streamSet, P);
            }
        }
        duplicateFilter.clear();
    }

    assert (partitionCount > 2);

    for (unsigned i = 1; i < (partitionCount - 1); ++i) {
        if (in_degree(i, P) == 0) {
            add_edge(0, i, 0, P);
        }
        if (out_degree(i, P) == 0) {
            add_edge(i, partitionCount - 1, 0, P);
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
     mPartitionJumpIndex.resize(PartitionCount);
#ifdef DISABLE_PARTITION_JUMPING
    for (unsigned i = 0; i < (PartitionCount - 1); ++i) {
        mPartitionJumpIndex[i] = i + 1;
    }
    mPartitionJumpIndex[(PartitionCount - 1)] = (PartitionCount - 1);
#else

    using BV = dynamic_bitset<>;
    using Graph = adjacency_list<hash_setS, vecS, bidirectionalS>;

    // Summarize the partitioning graph to only represent the existance of a dataflow relationship
    // between the partitions.

    Graph G(PartitionCount);

    for (auto producer = PipelineInput; producer < PipelineOutput; ++producer) {
        bool anyNonFixedOutput = false;
        for (const auto e : make_iterator_range(out_edges(producer, mBufferGraph))) {
            const BufferPort & port = mBufferGraph[e];
            const Binding & binding = port.Binding;
            if (!binding.getRate().isFixed()) {
                anyNonFixedOutput = true;
                break;
            }
        }

        const auto pid = KernelPartitionId[producer];
        assert (pid < PartitionCount);

        if (anyNonFixedOutput) {
            for (const auto e : make_iterator_range(out_edges(producer, mBufferGraph))) {
                const auto streamSet = target(e, mBufferGraph);
                for (const auto f : make_iterator_range(out_edges(streamSet, mBufferGraph))) {
                    const auto consumer = target(f, mBufferGraph);
                    const auto cid = KernelPartitionId[consumer];
                    assert (pid <= cid && cid < PartitionCount);
                    if (pid != cid) {
                        add_edge(pid, cid, G);
                    }
                }
            }
        } else {
            add_edge(pid, pid + 1U, G);
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

    #ifndef NDEBUG
    for (unsigned i = 0; i < PartitionCount; ++i) {
        mPartitionJumpIndex[i] = -1U;
    }
    #endif

    std::vector<unsigned> rank(PartitionCount);
    for (unsigned i = 0; i < PartitionCount; ++i) { // forward topological ordering
        unsigned newRank = 0;
        for (const auto e : make_iterator_range(in_edges(i, G))) {
            newRank = std::max(newRank, rank[source(e, G)]);
        }
        rank[i] = newRank + 1;
    }

    std::vector<unsigned> occurences(PartitionCount);
    std::vector<unsigned> singleton(PartitionCount);

    std::vector<std::bitset<2>> ancestors(PartitionCount);

    std::vector<unsigned> reverseLCA(PartitionCount);
    for (unsigned i = 0; i < terminal; ++i) {
        reverseLCA[i] = i + 1;
    }

    for (unsigned i = 0; i < PartitionCount; ++i) {  // forward topological ordering
        const auto d = in_degree(i, G);

        if (d > 1) {

            Graph::in_edge_iterator begin, end;
            std::tie(begin, end) = in_edges(i, G);

            auto lca = i;
            for (auto ei = begin; (++ei) != end; ) {
                const auto x = source(*ei, G);
                for (auto ej = begin; ej != ei; ++ej) {
                    const auto y = source(*ej, G);
                    assert (x != y);

                    // Determine the common ancestors of each input to node_i
                    for (unsigned j = 0; j < lca; ++j) {
                        ancestors[j].reset();
                    }
                    ancestors[x].set(0);
                    ancestors[y].set(1);

                    std::fill_n(occurences.begin(), rank[i] - 1, 0);
                    for (auto j = i; j--; ) { // reverse topological ordering
                        for (const auto e : make_iterator_range(out_edges(j, G))) {
                            const auto v = target(e, G);
                            ancestors[j] |= ancestors[v];
                        }
                        if (ancestors[j].all()) {
                            const auto k = rank[j];
                            occurences[k]++;
                            singleton[k] = j;
                        }
                    }
                    // Now scan again through them to determine the single ancestor
                    // to the pair of inputs that is of highest rank.

                    for (auto j = rank[i] - 1; j--; ) {
                        if (occurences[j] == 1) {
                            lca = singleton[j];
                            break;
                        }
                    }
                }
            }
            assert (lca <= i);
            auto & val = reverseLCA[lca];
            val = std::max(val, i);
        }
    }
    reverseLCA[terminal] = terminal;

    for (auto partitionId = 1U; partitionId < terminal; ++partitionId) {
       add_edge(partitionId, partitionId + 1, G);
    }
    add_edge(terminal, terminal, G);

#if 0

    auto & out = errs();

    out << "digraph \"" << "J1" << "\" {\n";
    for (auto v : make_iterator_range(vertices(G))) {
        out << "v" << v << " [label=\"" << v << " : {";
        out << reverseLCA[v];
        out << "}\"];\n";
    }
    for (auto e : make_iterator_range(edges(G))) {
        const auto s = source(e, G);
        const auto t = target(e, G);
        out << "v" << s << " -> v" << t << ";\n";
    }

    out << "}\n\n";
    out.flush();

#endif

    for (unsigned i = 0; i < PartitionCount; ++i) {
        auto n = reverseLCA[i];
        assert (n < PartitionCount);
        while (in_degree(n, G) < 2) {
            const auto m = reverseLCA[n];
            assert (n != m);
            n = m;
            assert (n < PartitionCount);
        }
        assert (n > i || (i == (PartitionCount - 1)));
        mPartitionJumpIndex[i] = n;
    }

#endif
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makePartitionJumpGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::makePartitionJumpTree() {
    mPartitionJumpTree = PartitionJumpTree(PartitionCount);
    for (auto i = 0U; i < (PartitionCount - 1); ++i) {
        assert (mPartitionJumpIndex[i] >= i && mPartitionJumpIndex[i] < PartitionCount);
        add_edge(i, mPartitionJumpIndex[i], mPartitionJumpTree);
    }

#if 0

    auto & out = errs();

    out << "digraph \"" << "J2" << "\" {\n";
    for (auto v : make_iterator_range(vertices(mPartitionJumpTree))) {
        out << "v" << v << " [label=\"" << v << "\"];\n";
    }
    for (auto e : make_iterator_range(edges(mPartitionJumpTree))) {
        const auto s = source(e, mPartitionJumpTree);
        const auto t = target(e, mPartitionJumpTree);
        out << "v" << s << " -> v" << t << ";\n";
    }

    out << "}\n\n";
    out.flush();

#endif
}

} // end of namespace kernel

#endif // PARTITIONING_ANALYSIS_HPP
