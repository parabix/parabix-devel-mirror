#ifndef DATAFLOW_ANALYSIS_HPP
#define DATAFLOW_ANALYSIS_HPP

#include "pipeline_analysis.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeMinimumExpectedDataflow
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::computeMinimumExpectedDataflow(PartitionGraph & P) {

    const auto cfg = Z3_mk_config();
    Z3_set_param_value(cfg, "model", "true");
    Z3_set_param_value(cfg, "proof", "false");

    const auto ctx = Z3_mk_context(cfg);
    Z3_del_config(cfg);
    const auto solver = Z3_mk_optimize(ctx);
    Z3_optimize_inc_ref(ctx, solver);

    auto hard_assert = [&](Z3_ast c) {
        Z3_optimize_assert(ctx, solver, c);
    };

    auto soft_assert = [&](Z3_ast c) {
        Z3_optimize_assert_soft(ctx, solver, c, "1", nullptr);
    };

    auto check = [&]() -> Z3_lbool {
        #if Z3_VERSION_INTEGER >= LLVM_VERSION_CODE(4, 5, 0)
        return Z3_optimize_check(ctx, solver, 0, nullptr);
        #else
        return Z3_optimize_check(ctx, solver);
        #endif
    };

    const auto intType = Z3_mk_int_sort(ctx);

    auto constant_real = [&](const Rational value) {
        assert (value.numerator() > 0);
        return Z3_mk_real(ctx, value.numerator(), value.denominator());
    };

    auto multiply =[&](Z3_ast X, Z3_ast Y) {
        assert (X);
        assert (Y);
        Z3_ast args[2] = { X, Y };
        return Z3_mk_mul(ctx, 2, args);
    };

    const auto ONE = Z3_mk_int(ctx, 1, intType);

    const auto numOfPartitions = num_vertices(P);

    const auto m = num_vertices(Relationships);

    std::vector<Z3_ast> VarList(m);

    for (unsigned partition = 0; partition < numOfPartitions; ++partition) {
        const PartitionData & N = P[partition];
        for (const auto u : N.Kernels) {
            auto repVar = Z3_mk_fresh_const(ctx, nullptr, intType);
            hard_assert(Z3_mk_ge(ctx, repVar, ONE));
            VarList[u] = repVar; // multiply(rootVar, repVar);
        }
    }

    #warning Verify min dataflow considers multiple inputs of differing rates

    for (unsigned producerPartitionId = 1; producerPartitionId < numOfPartitions; ++producerPartitionId) {
        PartitionData & N = P[producerPartitionId];
        const auto & K = N.Kernels;

        for (const auto producer : K) {

            const RelationshipNode & producerNode = Relationships[producer];
            assert (producerNode.Type == RelationshipNode::IsKernel);

            const auto strideSize = producerNode.Kernel->getStride();

            assert (Relationships[producer].Type == RelationshipNode::IsKernel);
            for (const auto e : make_iterator_range(out_edges(producer, Relationships))) {
                const auto binding = target(e, Relationships);
                if (Relationships[binding].Type == RelationshipNode::IsBinding) {
                    const auto f = first_out_edge(binding, Relationships);
                    assert (Relationships[f].Reason != ReasonType::Reference);
                    const auto streamSet = target(f, Relationships);
                    assert (Relationships[streamSet].Type == RelationshipNode::IsRelationship);
                    assert (isa<StreamSet>(Relationships[streamSet].Relationship));

                    const RelationshipNode & output = Relationships[binding];
                    assert (output.Type == RelationshipNode::IsBinding);

                    const Binding & outputBinding = output.Binding;
                    const ProcessingRate & rate = outputBinding.getRate();
                    // ignore unknown output rates; we cannot reason about them here.
                    if (LLVM_LIKELY(rate.isUnknown())) {
                        continue;
                    }

                    const auto sum = rate.getLowerBound() + rate.getUpperBound();
                    const auto expectedOutput = sum * Rational{strideSize, 2};

                    assert (VarList[producer]);

                    const Z3_ast expOutRate = multiply(VarList[producer], constant_real(expectedOutput));

                    for (const auto e : make_iterator_range(out_edges(streamSet, Relationships))) {
                        const auto binding = target(e, Relationships);
                        const RelationshipNode & input = Relationships[binding];
                        if (LLVM_LIKELY(input.Type == RelationshipNode::IsBinding)) {
                            const auto f = first_out_edge(binding, Relationships);
                            assert (Relationships[f].Reason != ReasonType::Reference);
                            const unsigned consumer = target(f, Relationships);

                            const Binding & inputBinding = input.Binding;
                            const ProcessingRate & rate = inputBinding.getRate();

                            const auto c = PartitionIds.find(consumer);
                            assert (c != PartitionIds.end());
                            const auto consumerPartitionId = c->second;
                            assert (producerPartitionId <= consumerPartitionId);

                            const RelationshipNode & node = Relationships[consumer];
                            assert (node.Type == RelationshipNode::IsKernel);

                            if (LLVM_LIKELY(!rate.isGreedy())) {

                                const auto strideSize = node.Kernel->getStride();
                                const auto sum = rate.getLowerBound() + rate.getUpperBound();
                                const auto expectedInput = sum * Rational{strideSize, 2};
                                assert (VarList[consumer]);
                                const Z3_ast expInRate = multiply(VarList[consumer], constant_real(expectedInput));

                                if (producerPartitionId == consumerPartitionId && rate.isFixed()) {
                                    hard_assert(Z3_mk_eq(ctx, expOutRate, expInRate));
                                } else {
                                    hard_assert(Z3_mk_ge(ctx, expOutRate, expInRate));
                                    soft_assert(Z3_mk_eq(ctx, expOutRate, expInRate));
//                                    Z3_ast args[2] = { expOutRate, expInRate };
//                                    const auto diff = Z3_mk_sub(ctx, 2, args);
//                                    Z3_optimize_minimize(ctx, solver, diff);
                                }
                            }
                        }
                    }

                }
            }
        }
    }

    #warning ADD LENGTH EQUALITY ASSERTIONS HERE

    if (LLVM_UNLIKELY(check() == Z3_L_FALSE)) {
        assert (false);
        report_fatal_error("Z3 failed to find a solution to minimum expected dataflow problem");
    }

    const auto model = Z3_optimize_get_model(ctx, solver);
    Z3_model_inc_ref(ctx, model);
    for (unsigned partition = 0; partition < numOfPartitions; ++partition) {
        PartitionData & N = P[partition];
        const auto & K = N.Kernels;
        const auto n = K.size();
        N.Repetitions.resize(n);
        for (unsigned i = 0; i < n; ++i) {
            Z3_ast value;
            if (LLVM_UNLIKELY(Z3_model_eval(ctx, model, VarList[K[i]], Z3_L_TRUE, &value) != Z3_L_TRUE)) {
                report_fatal_error("Unexpected Z3 error when attempting to obtain value from model!");
            }

            Z3_int64 num, denom;
            if (LLVM_UNLIKELY(Z3_get_numeral_rational_int64(ctx, value, &num, &denom) != Z3_L_TRUE)) {
                report_fatal_error("Unexpected Z3 error when attempting to convert model value to number!");
            }
            assert (num > 0 && denom == 1);
            N.Repetitions[i] = num;
        }


    }
    Z3_model_dec_ref(ctx, model);

    Z3_optimize_dec_ref(ctx, solver);
    Z3_del_context(ctx);

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeMaximumExpectedDataflow
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::computeMaximumExpectedDataflow() {

    const auto cfg = Z3_mk_config();
    Z3_set_param_value(cfg, "model", "true");
    Z3_set_param_value(cfg, "proof", "false");
    const auto ctx = Z3_mk_context(cfg);
    Z3_del_config(cfg);
    const auto solver = Z3_mk_optimize(ctx);
    Z3_optimize_inc_ref(ctx, solver);

    auto hard_assert = [&](Z3_ast c) {
        Z3_optimize_assert(ctx, solver, c);
    };

    auto soft_assert = [&](Z3_ast c) {
        Z3_optimize_assert_soft(ctx, solver, c, "1", nullptr);
    };

    auto check = [&]() -> Z3_lbool {
        #if Z3_VERSION_INTEGER >= LLVM_VERSION_CODE(4, 5, 0)
        return Z3_optimize_check(ctx, solver, 0, nullptr);
        #else
        return Z3_optimize_check(ctx, solver);
        #endif
    };

    const auto intType = Z3_mk_int_sort(ctx);

    const auto realType = Z3_mk_real_sort(ctx);

    auto constant_real = [&](const Rational & value) {
        return Z3_mk_real(ctx, value.numerator(), value.denominator());
    };

    auto maximum = [&](const BufferPort & rate) {
        return constant_real(rate.Maximum);
    };

    auto minimum = [&](const BufferPort & rate) {
        return constant_real(rate.Minimum);
    };

    auto multiply =[&](Z3_ast X, Z3_ast Y) {
        Z3_ast args[2] = { X, Y };
        return Z3_mk_mul(ctx, 2, args);
    };

    const auto firstKernel = out_degree(PipelineInput, mBufferGraph) == 0 ? FirstKernel : PipelineInput;
    const auto lastKernel = in_degree(PipelineOutput, mBufferGraph) == 0 ? LastKernel : PipelineOutput;

    std::vector<Z3_ast> VarList(LastStreamSet + 1);

    bool useIntNumbers = true;

retry:

    Z3_optimize_push(ctx, solver);

    const auto ONE = constant_real(1);
    const auto TWO = constant_real(2);

    BEGIN_SCOPED_REGION

    auto make_partition_vars = [&](const unsigned first, const unsigned last) {
        auto rootVar = Z3_mk_fresh_const(ctx, nullptr, useIntNumbers ? intType : realType);
        hard_assert(Z3_mk_ge(ctx, rootVar, ONE));
        hard_assert(Z3_mk_le(ctx, rootVar, TWO));
        for (auto kernel = first; kernel <= last; ++kernel) {
            VarList[kernel] = multiply(rootVar, constant_real(MinimumNumOfStrides[kernel]));
        }
    };

    auto currentPartitionId = KernelPartitionId[firstKernel];
    auto firstKernelInPartition = firstKernel;
    for (auto kernel = (firstKernel + 1U); kernel <= lastKernel; ++kernel) {
        const auto partitionId = KernelPartitionId[kernel];
        if (partitionId != currentPartitionId) {
            make_partition_vars(firstKernelInPartition, kernel - 1U);
            // set the first kernel for the next partition
            firstKernelInPartition = kernel;
            currentPartitionId = partitionId;
        }
    }
    if (firstKernelInPartition <= lastKernel) {
        make_partition_vars(firstKernelInPartition, lastKernel);
    }

    END_SCOPED_REGION

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {

        const auto partitionId = KernelPartitionId[kernel];

        const auto stridesPerSegmentVar = VarList[kernel];
        assert (stridesPerSegmentVar);

        unsigned numOfGreedyRates = 0;

        for (const auto input : make_iterator_range(in_edges(kernel, mBufferGraph))) {
            const auto streamSet = source(input, mBufferGraph);
            const auto producer = parent(streamSet, mBufferGraph);

            // we're only interested in inter-partition dataflow here
            if (KernelPartitionId[producer] == partitionId) {
                continue;
            }

            const BufferPort & inputRate = mBufferGraph[input];
            const Binding & binding = inputRate.Binding;
            const ProcessingRate & rate = binding.getRate();

            const auto producedRate = VarList[streamSet]; assert (producedRate);

            if (LLVM_UNLIKELY(rate.isGreedy())) {
                ++numOfGreedyRates;
                // ideally we want to always have enough data to execute
                soft_assert(Z3_mk_ge(ctx, producedRate, minimum(inputRate)));
            } else { // Fixed, Bounded or Partial Sum
                const auto consumedRate = multiply(stridesPerSegmentVar, minimum(inputRate));
                hard_assert(Z3_mk_ge(ctx, producedRate, consumedRate));
                Z3_ast args[2] = { producedRate, consumedRate };
                const auto diff = Z3_mk_sub(ctx, 2, args);
                Z3_optimize_minimize(ctx, solver, diff);
            }
        }

        // Any kernel with all greedy rates must exhaust its input in a single iteration.
        if (LLVM_UNLIKELY(numOfGreedyRates > 0 && numOfGreedyRates == in_degree(kernel, mBufferGraph))) {
            hard_assert(Z3_mk_eq(ctx, stridesPerSegmentVar, ONE));
        }

        for (const auto output : make_iterator_range(out_edges(kernel, mBufferGraph))) {
            const BufferPort & outputRate = mBufferGraph[output];
            const Binding & binding = outputRate.Binding;
            const ProcessingRate & rate = binding.getRate();
            const auto streamSet = target(output, mBufferGraph);

            assert (VarList[streamSet] == nullptr);

            if (LLVM_UNLIKELY(rate.isUnknown())) {
                // TODO: is there a better way to handle unknown outputs? This
                // free variable represents the ideal amount of data to transfer
                // to subsequent kernels but that isn't very meaningful here.
                auto v = Z3_mk_fresh_const(ctx, nullptr, intType);
                hard_assert(Z3_mk_ge(ctx, v, minimum(outputRate)));
                VarList[streamSet] = v;
            } else { // Fixed, Bounded or Partial Sum
                VarList[streamSet] = multiply(stridesPerSegmentVar, maximum(outputRate));
            }
        }
    }

    // Include any length equality assertions
    if (!mLengthAssertions.empty()) {
        flat_map<const StreamSet *, unsigned> M;

        for (auto streamSet = FirstStreamSet; streamSet <= LastStreamSet; ++streamSet) {
            const auto output = in_edge(streamSet, mBufferGraph);
            const BufferPort & br = mBufferGraph[output];
            const Binding & binding = br.Binding;
            M.emplace(cast<StreamSet>(binding.getRelationship()), streamSet);
        }

        auto offset = [&](const StreamSet * streamSet) {
            const auto f = M.find(streamSet);
            assert (f != M.end());
            return f->second;
        };

        for (const LengthAssertion & la : mLengthAssertions) {
            const auto A = VarList[offset(la[0])];
            const auto B = VarList[offset(la[1])];
            soft_assert(Z3_mk_eq(ctx, A, B));
        }
    }

    if (LLVM_UNLIKELY(check() == Z3_L_FALSE)) {
        if (useIntNumbers) {
            // Earlier versions of Z3 seem to have an issue working out a solution to some problems
            // when using int-type variables. However, using real numbers generates an "infinite"
            // number of potential solutions and takes considerably longer to finish. Thus we only
            // fall back to using rational variables if this test fails.
            Z3_optimize_pop(ctx, solver);
            useIntNumbers = false;
            goto retry;
        }
        report_fatal_error("Z3 failed to find a solution to the maximum permitted dataflow problem");
    }

    MaximumNumOfStrides.resize(PipelineOutput + 1);

    const auto model = Z3_optimize_get_model(ctx, solver);
    Z3_model_inc_ref(ctx, model);
    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {

        Z3_ast const stridesPerSegmentVar = VarList[kernel];
        Z3_ast value;
        if (LLVM_UNLIKELY(Z3_model_eval(ctx, model, stridesPerSegmentVar, Z3_L_TRUE, &value) != Z3_L_TRUE)) {
            report_fatal_error("Unexpected Z3 error when attempting to obtain value from model!");
        }

        Z3_int64 num, denom;
        if (LLVM_LIKELY(useIntNumbers)) {
            if (LLVM_UNLIKELY(Z3_get_numeral_int64(ctx, value, &num) != Z3_L_TRUE)) {
                report_fatal_error("Unexpected Z3 error when attempting to convert model value to number!");
            }
            MaximumNumOfStrides[kernel] = num;
        } else {
            if (LLVM_UNLIKELY(Z3_get_numeral_rational_int64(ctx, value, &num, &denom) != Z3_L_TRUE)) {
                report_fatal_error("Unexpected Z3 error when attempting to convert model value to number!");
            }

            const auto x = MinimumNumOfStrides[kernel];
            const auto y = ceiling(Rational{num, denom * x});
            MaximumNumOfStrides[kernel]  = y * x;
        }


    }
    Z3_model_dec_ref(ctx, model);

    Z3_optimize_dec_ref(ctx, solver);
    Z3_del_context(ctx);
    Z3_reset_memory();

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief identifyInterPartitionSymbolicRates
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::identifyInterPartitionSymbolicRates() {

#ifdef COMPUTE_SYMBOLIC_RATE_IDS

    const auto cfg = Z3_mk_config();
    Z3_set_param_value(cfg, "model", "true");
    Z3_set_param_value(cfg, "proof", "false");
    const auto ctx = Z3_mk_context(cfg);
    Z3_del_config(cfg);

    const auto tQE = Z3_mk_tactic(ctx, "qe");
    Z3_tactic_inc_ref (ctx, tQE);
    const auto tSMT = Z3_mk_tactic(ctx, "smt");
    Z3_tactic_inc_ref (ctx, tSMT);
    const auto tactics = Z3_tactic_and_then (ctx, tQE, tSMT);
    const auto solver = Z3_mk_solver_from_tactic(ctx, tactics);
    Z3_solver_inc_ref(ctx, solver);
    Z3_tactic_dec_ref (ctx, tQE);
    Z3_tactic_dec_ref (ctx, tSMT);

    Z3_params params = Z3_mk_params(ctx);
    Z3_params_inc_ref(ctx, params);

    Z3_symbol r = Z3_mk_string_symbol(ctx, ":timeout");
    Z3_params_set_uint(ctx, params, r, 5000);
    Z3_solver_set_params(ctx, solver, params);
    Z3_params_dec_ref(ctx, params);


    const auto intType = Z3_mk_int_sort(ctx);

    auto constant_real = [&](const Rational & value) {
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

    auto hard_assert = [&](Z3_ast c) {
        Z3_solver_assert(ctx, solver, c);
    };

    std::vector<Z3_ast> VarList(LastStreamSet + 1);

    std::vector<Z3_ast> PartitionVarList(PartitionCount);

    std::vector<Z3_app> vars;

    const auto ONE = Z3_mk_int(ctx, 1, intType);

    const auto firstKernel = out_degree(PipelineInput, mBufferGraph) == 0 ? FirstKernel : PipelineInput;
    const auto lastKernel = in_degree(PipelineOutput, mBufferGraph) == 0 ? LastKernel : PipelineOutput;

    BEGIN_SCOPED_REGION

    auto make_partition_vars = [&](const unsigned first, const unsigned last) {
        auto rootVar = Z3_mk_fresh_const(ctx, nullptr, intType);
        for (auto kernel = first; kernel <= last; ++kernel) {
            auto kernelVar = rootVar;
            const auto m = MinimumNumOfStrides[kernel];
            if (LLVM_UNLIKELY(m != 1)) {
                kernelVar = multiply(kernelVar, constant_real(m));
            }
            VarList[kernel] = kernelVar;
        }
        return rootVar;

    };

    auto currentPartitionId = KernelPartitionId[firstKernel];
    auto firstKernelInPartition = firstKernel;
    for (auto kernel = (firstKernel + 1U); kernel <= lastKernel; ++kernel) {
        const auto partitionId = KernelPartitionId[kernel];
        if (partitionId != currentPartitionId) {
            PartitionVarList[currentPartitionId] =
                make_partition_vars(firstKernelInPartition, kernel - 1U);
            // set the first kernel for the next partition
            firstKernelInPartition = kernel;
            currentPartitionId = partitionId;
        }
    }
    if (LLVM_LIKELY(firstKernelInPartition <= lastKernel)) {
        PartitionVarList[currentPartitionId] =
            make_partition_vars(firstKernelInPartition, lastKernel);
    }
    END_SCOPED_REGION



    flat_map<unsigned, Z3_ast> symbolicPopCountRateVar;

    flat_map<unsigned, Z3_ast> IORateVarMap;

    flat_map<const StreamSet *, Z3_ast> streamSetVar;

    std::vector<std::vector<Z3_ast>> minConstraints(PartitionCount);

    struct EdgeData {
        RefWrapper<BufferPort> Port;
        Z3_ast Expr = nullptr;

        EdgeData() = default;
        EdgeData(BufferPort & port, Z3_ast expr) : Port(port), Expr(expr) { }
    };

    using Graph = adjacency_list<hash_setS, vecS, bidirectionalS, no_property, EdgeData>;

    Graph G(PartitionCount);

    for (auto streamSet = FirstStreamSet; streamSet <= LastStreamSet; ++streamSet) {
        const auto output = in_edge(streamSet, mBufferGraph);
        const auto producer = source(output, mBufferGraph);
        const auto prodPartId = KernelPartitionId[producer];


        Z3_ast outRateExpr = nullptr;
        unsigned streamSetVertex = 0;

        for (const auto input : make_iterator_range(out_edges(streamSet, mBufferGraph))) {
            const auto consumer = target(input, mBufferGraph);
            const auto consPartId = KernelPartitionId[consumer];

            // we're only interested in inter-partition relationships
            if (prodPartId == consPartId) {
                continue;
            }

            auto calculateBaseSymbolicRateVar = [&](const unsigned kernel, const BufferPort & port)  {

                const Z3_ast strideVar = VarList[kernel];
                const Binding & binding = port.Binding;
                const ProcessingRate & rate = binding.getRate();

                Z3_ast expr = nullptr;

                switch (rate.getKind()) {
                    case RateId::Fixed:
                        BEGIN_SCOPED_REGION
                        expr = multiply(strideVar, constant_real(port.Minimum));
                        END_SCOPED_REGION
                        break;
                    case RateId::Greedy:
                        BEGIN_SCOPED_REGION
                        assert ("greedy rate cannot be an output rate" && producer != kernel);
                        assert (outRateExpr);
                        const auto lb = constant_real(port.Minimum);
                        const auto comp = Z3_mk_ge(ctx, outRateExpr, lb);
                        expr = Z3_mk_ite(ctx, comp, outRateExpr, constant_real(0));
                        END_SCOPED_REGION
                        break;
                    case RateId::Bounded:
                        BEGIN_SCOPED_REGION
                        assert (port.Minimum < port.Maximum);
                        auto v = Z3_mk_fresh_const(ctx, nullptr, intType);
                        vars.push_back((Z3_app)v);
                        const auto lb = constant_real(port.Minimum);
                        hard_assert(Z3_mk_ge(ctx, v, lb));
                        const auto ub = constant_real(port.Maximum);
                        hard_assert(Z3_mk_le(ctx, v, ub));
                        expr = multiply(strideVar, v);
                        END_SCOPED_REGION
                        break;
                    case RateId::Unknown:
                        BEGIN_SCOPED_REGION
                        auto v = Z3_mk_fresh_const(ctx, nullptr, intType);
                        vars.push_back((Z3_app)v);
                        const auto lb = constant_real(port.Minimum);
                        hard_assert(Z3_mk_ge(ctx, v, lb));
                        expr = multiply(strideVar, v);
                        END_SCOPED_REGION
                        break;
                    case RateId::PartialSum:
                        BEGIN_SCOPED_REGION
                        const auto refStreamSet = getReferenceBufferVertex(kernel, port.Port);
                        const auto f = symbolicPopCountRateVar.find(refStreamSet);
                        Z3_ast symbolicRateVar;
                        if (f == symbolicPopCountRateVar.end()) {
                            symbolicRateVar = Z3_mk_fresh_const(ctx, nullptr, intType);
                            const auto ZERO = Z3_mk_int(ctx, 0, intType);
                            Z3_solver_assert(ctx, solver, Z3_mk_ge(ctx, symbolicRateVar, ZERO));
                            vars.push_back((Z3_app)symbolicRateVar);
                            symbolicPopCountRateVar.emplace(refStreamSet, symbolicRateVar);
                        } else {
                            symbolicRateVar = f->second;
                        }
                        Z3_ast args[2] = { symbolicRateVar, strideVar };
                        expr = Z3_mk_mul(ctx, 2, args);
                        END_SCOPED_REGION
                        break;
                    case RateId::Relative:
                        BEGIN_SCOPED_REGION
                        const auto refBinding = getReferenceBufferVertex(kernel, port.Port);
                        const auto f = IORateVarMap.find(refBinding);
                        assert ("relative rate occured before ref" && f != IORateVarMap.end());
                        expr = f->second;
                        assert ("failed to locate symbolic rate for relative rate?" && expr);
                        if (rate.getLowerBound() != Rational{1}) {
                            expr = multiply(expr, constant_real(rate.getLowerBound()) );
                        }
                        END_SCOPED_REGION
                        break;
                    default:
                        llvm_unreachable("unknown rate type?");
                }
                assert (expr);

                for (const Attribute & attr : binding.getAttributes()) {
                    switch (attr.getKind()) {
                        case AttrId::LookAhead:
                            if (port.Port.Type == PortType::Output) {
                                break;
                            }
                        case AttrId::Delayed:
                            BEGIN_SCOPED_REGION
                            Z3_ast args[2] = { expr, Z3_mk_int(ctx, attr.amount(), intType) };
                            expr = Z3_mk_sub(ctx, 2, args);
                            END_SCOPED_REGION
                            break;
                        case AttrId::Deferred:
                            // A deferred output rate is closer to an bounded rate than a
                            // countable rate but a deferred input rate simply means the
                            // buffer must be dynamic.
                            if (port.Port.Type == PortType::Output) {
                                const auto deferred = Z3_mk_fresh_const(ctx, nullptr, intType);
                                vars.push_back((Z3_app)deferred);
                                hard_assert(Z3_mk_ge(ctx, deferred, Z3_mk_int(ctx, 0, intType)));
                                hard_assert(Z3_mk_le(ctx, deferred, expr));
                                expr = deferred;
                            }
                            break;
                        case AttrId::BlockSize:
                            BEGIN_SCOPED_REGION
                            expr = Z3_mk_mod(ctx, expr, Z3_mk_int(ctx, attr.amount(), intType) );
                            END_SCOPED_REGION
                            break;
                        default: break;
                    }
                }

                return expr;
            };



            // construct our output variable
            if (outRateExpr == nullptr) {
                auto & port = mBufferGraph[output];
                outRateExpr = calculateBaseSymbolicRateVar(producer, port);
                streamSetVertex = add_vertex(G);
                add_edge(prodPartId, streamSetVertex, EdgeData{port, outRateExpr}, G);
                IORateVarMap.emplace(streamSet, outRateExpr);
            }

            auto & port = mBufferGraph[input];

            auto inRateExpr = calculateBaseSymbolicRateVar(consumer, port);
            hard_assert(Z3_mk_ge(ctx, outRateExpr, inRateExpr));


            add_edge(streamSetVertex, consPartId, EdgeData{port, inRateExpr}, G);



        }

    }





    if (!mLengthAssertions.empty()) {
        flat_map<const StreamSet *, unsigned> M;

        for (auto streamSet = FirstStreamSet; streamSet <= LastStreamSet; ++streamSet) {
            const auto output = in_edge(streamSet, mBufferGraph);
            const BufferPort & br = mBufferGraph[output];
            const Binding & binding = br.Binding;
            M.emplace(cast<StreamSet>(binding.getRelationship()), streamSet);
        }

        auto offset = [&](const StreamSet * streamSet) {
            const auto f = M.find(streamSet);
            assert (f != M.end());
            return f->second;
        };

        for (const LengthAssertion & la : mLengthAssertions) {
            const auto A = VarList[offset(la[0])];
            const auto B = VarList[offset(la[1])];
            hard_assert(Z3_mk_eq(ctx, A, B));
        }
    }

    if (LLVM_UNLIKELY(Z3_solver_check(ctx, solver) != Z3_L_TRUE)) {
        report_fatal_error("Z3 failed to find a solution to the symbolic rate graph");
    }




    const auto forall = Z3_mk_forall_const(ctx, 0, vars.size(), vars.data(), 0, nullptr, c);




    for (unsigned i = 1; ++ei, i < m; ++i) {
        EdgeData & A = G[*ei];
        auto ej = begin;

        for (unsigned j = 0; j < i; ++j, ++ej) {
            EdgeData & B = G[*ej];

            BEGIN_SCOPED_REGION
            Z3_solver_push(ctx, solver);
            const auto c = Z3_mk_eq(ctx, A.Expr, B.Expr);
            hard_assert(Z3_mk_forall_const(ctx, 0, vars.size(), vars.data(), 0, nullptr, c));
            const auto r = Z3_solver_check(ctx, solver);
            errs() << Z3_ast_to_string(ctx, c) << "\nr=" << r << "\n";
            if (r == Z3_L_TRUE) {
                add_edge(i, j, L);
                add_edge(j, i, L);
                continue;
            } else {
                Z3_solver_pop(ctx, solver, 1);
            }
            END_SCOPED_REGION

            BEGIN_SCOPED_REGION
            Z3_solver_push(ctx, solver);
            const auto c = Z3_mk_gt(ctx, A.Expr, B.Expr);
            hard_assert(Z3_mk_forall_const(ctx, 0, vars.size(), vars.data(), 0, nullptr, c));
            const auto r = Z3_solver_check(ctx, solver);
            errs() << Z3_ast_to_string(ctx, c) << "\nr=" << r << "\n";
            if (r == Z3_L_TRUE) {
                // add_edge(i, j, L);
                add_edge(j, i, L);
                continue;
            } else {
                Z3_solver_pop(ctx, solver, 1);
            }
            END_SCOPED_REGION

            BEGIN_SCOPED_REGION
            Z3_solver_push(ctx, solver);
            const auto c = Z3_mk_lt(ctx, A.Expr, B.Expr);
            hard_assert(Z3_mk_forall_const(ctx, 0, vars.size(), vars.data(), 0, nullptr, c));
            const auto r = Z3_solver_check(ctx, solver);
            errs() << Z3_ast_to_string(ctx, c) << "\nr=" << r << "\n";
            if (r == Z3_L_TRUE) {
                add_edge(i, j, L);
                // add_edge(j, i, L);
                continue;
            } else {
                Z3_solver_pop(ctx, solver, 1);
            }
            END_SCOPED_REGION

        }
    }

    printGraph(L, errs(), "L");

    Z3_solver_dec_ref(ctx, solver);
    Z3_del_context(ctx);
    Z3_reset_memory();

#endif

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeMinimumStrideLengthForConsistentDataflow
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::computeMinimumStrideLengthForConsistentDataflow() {

    // TODO: we already do this when scheduling. Organize the logic better to only do it once if this
    // ends up being necessary for performance.

    const auto firstKernel = out_degree(PipelineInput, mBufferGraph) == 0 ? FirstKernel : PipelineInput;
    const auto lastKernel = in_degree(PipelineOutput, mBufferGraph) == 0 ? LastKernel : PipelineOutput;

    StrideStepLength.resize(PipelineOutput + 1);

    auto make_partition_vars = [&](const unsigned first, const unsigned last) {
        auto gcd = MinimumNumOfStrides[first];
        for (auto i = first + 1; i <= last; ++i) {
            gcd = boost::gcd(gcd, MinimumNumOfStrides[i]);
        }
        for (auto i = first; i <= last; ++i) {
            StrideStepLength[i] = (MinimumNumOfStrides[i] / gcd);
        }
    };

    auto currentPartitionId = KernelPartitionId[firstKernel];
    auto firstKernelInPartition = firstKernel;
    for (auto kernel = (firstKernel + 1U); kernel <= lastKernel; ++kernel) {
        const auto partitionId = KernelPartitionId[kernel];
        if (partitionId != currentPartitionId) {
            make_partition_vars(firstKernelInPartition, kernel - 1U);
            // set the first kernel for the next partition
            firstKernelInPartition = kernel;
            currentPartitionId = partitionId;
        }
    }
    if (firstKernelInPartition <= lastKernel) {
        make_partition_vars(firstKernelInPartition, lastKernel);
    }

}

} // end of kernel namespace

#endif // DATAFLOW_ANALYSIS_HPP
