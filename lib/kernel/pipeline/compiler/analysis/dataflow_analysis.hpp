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

    for (unsigned partition = 1; partition < numOfPartitions; ++partition) {
        PartitionData & N = P[partition];
        const auto & K = N.Kernels;
//        auto rootVar = Z3_mk_fresh_const(ctx, nullptr, intType);
//        hard_assert(Z3_mk_ge(ctx, rootVar, ONE));
//        if (in_degree(partition, P) == 0) {
//            assert (K.size() == 1);
//            for (const auto u : K) {
//                VarList[u] = rootVar;
//            }
//        } else {
            for (const auto u : K) {
                auto repVar = Z3_mk_fresh_const(ctx, nullptr, intType);
                hard_assert(Z3_mk_ge(ctx, repVar, ONE));
                VarList[u] = repVar; // multiply(rootVar, repVar);
            }
//        }
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
    for (unsigned partition = 1; partition < numOfPartitions; ++partition) {
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

    auto constant_real = [&](const Rational & value) {
        if (value.denominator() == 1) {
            return Z3_mk_int(ctx, value.numerator(), intType);
        } else {
            return Z3_mk_real(ctx, value.numerator(), value.denominator());
        }
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

    const auto ONE = Z3_mk_int(ctx, 1, intType);
    const auto TWO = Z3_mk_int(ctx, 2, intType);

    BEGIN_SCOPED_REGION

    auto make_partition_vars = [&](const unsigned first, const unsigned last) {
        auto rootVar = Z3_mk_fresh_const(ctx, nullptr, intType);
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
        if (LLVM_UNLIKELY(Z3_get_numeral_rational_int64(ctx, value, &num, &denom) != Z3_L_TRUE)) {
            report_fatal_error("Unexpected Z3 error when attempting to convert model value to number!");
        }

        assert (num > 0 && denom == 1);
        MaximumNumOfStrides[kernel]  = num;
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

#if 0

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

        const auto & expectedBase = ExpectedNumOfStrides[first];
        assert (expectedBase.numerator() >= 1);
        auto rootVar = Z3_mk_fresh_const(ctx, nullptr, intType);
        VarList[first] = rootVar;

        Z3_solver_assert(ctx, solver, Z3_mk_ge(ctx, rootVar, ONE));
        assert (MaximumNumOfStrides.size() > first);
        const auto max = constant_real(MaximumNumOfStrides[first]);
        Z3_solver_assert(ctx, solver, Z3_mk_le(ctx, rootVar, max));

        if (in_degree(first, mBufferGraph) == 0) {
            vars.push_back((Z3_app)rootVar);
        }

        for (auto kernel = first + 1U; kernel <= last; ++kernel) {
            const auto r = ExpectedNumOfStrides[kernel] / expectedBase;
            auto kernelVar = rootVar;
            if (LLVM_UNLIKELY(r != Rational{1})) {
                kernelVar = multiply(kernelVar, constant_real(r));
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
                        expr = outRateExpr;
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
            hard_assert(Z3_mk_eq(ctx, outRateExpr, inRateExpr));
//            hard_assert(Z3_mk_ge(ctx, inRateExpr, ONE));

            add_edge(streamSetVertex, consPartId, EdgeData{port, inRateExpr}, G);

            auto & P = minConstraints[consPartId];
            P.push_back(Z3_mk_div(ctx, outRateExpr, inRateExpr));

        }

    }





    if (!mLengthAssertions.empty()) {
        flat_map<const StreamSet *, unsigned> M;

<<<<<<< HEAD
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
=======
        Z3_int64 z3_num, z3_denom;
        if (LLVM_UNLIKELY(Z3_get_numeral_rational_int64(ctx, value, &z3_num, &z3_denom) != Z3_L_TRUE)) {
            report_fatal_error("Unexpected Z3 error when attempting to convert model value to number!");
        }
        assert (z3_num > 0);

        const auto r = Rational{z3_num, z3_denom};
        for (unsigned bound = LowerBound; bound <= UpperBound; ++bound) {
            current[kernel][bound] = r;
>>>>>>> origin/master
        }
    }

    if (LLVM_UNLIKELY(Z3_solver_check(ctx, solver) != Z3_L_TRUE)) {

        errs() << "Z3 failed to find a solution to the core symbolic rate graph\n";

        report_fatal_error("Z3 failed to find a solution to the core symbolic rate graph");
    }

//    for (unsigned i = 0; i < PartitionCount; ++i) {
//        const auto & P = minConstraints[i];
//        const auto n = P.size();
//        if (n == 1) {
//            hard_assert(Z3_mk_eq(ctx, PartitionVarList[i], P[0]));
//        } else if (n > 1) {
//            Z3_ast a = P[0];
//            for (unsigned i = 1; i < n; ++i) {
//                const auto b = P[i];
//                const auto c = Z3_mk_lt(ctx, a, b);
//                a = Z3_mk_ite(ctx, c, a, b);
//            }
//            hard_assert(Z3_mk_eq(ctx, PartitionVarList[i], a));
//        }
//    }

//    if (LLVM_UNLIKELY(Z3_solver_check(ctx, solver)  != Z3_L_TRUE)) {

//        errs() << "Z3 failed to find a solution to the connected symbolic rate graph\n";

//        report_fatal_error("Z3 failed to find a solution to the connected symbolic rate graph");
//    }

    const auto m = num_edges(G);

    adjacency_list<hash_setS, vecS, directedS> L;

    Graph::edge_iterator begin, end;
    std::tie(begin, end) = edges(G);

<<<<<<< HEAD
    auto ei = begin;
=======
                    Z3_int64 z3_num, z3_denom;
                    if (LLVM_UNLIKELY(Z3_get_numeral_rational_int64(ctx, value, &z3_num, &z3_denom) != Z3_L_TRUE)) {
                        report_fatal_error("Unexpected Z3 error when attempting to convert model value to number!");
                    }
                    assert (z3_num > 0);
                    current[kernel][bound] = Rational{z3_num, z3_denom};
                }
                Z3_model_dec_ref(ctx, model);
>>>>>>> origin/master

    enum ConstraintType {
        EQ,
        GT,
        LT
    };

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
 * @brief identifyLinkedIOPorts
 *
 * Some I/O ports will always have an identical item count (relative to some known constant) throughout the
 * lifetime of the program. E.g., any partition local edge must be between two fixed rate I/O ports, which
 * entails that by knowing the initial value of one we can compute the latter.
 *
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::identifyLocalPortIds() {

    using BitSet = dynamic_bitset<>;

    struct Edge {
        RefWrapper<BufferPort> Port;
        BitSet RateSet;
        Edge() = default;
        Edge(BufferPort & br)
        : Port(br) { }
    };

    using Graph = adjacency_list<vecS, vecS, bidirectionalS, no_property, Edge>;

    const auto firstKernel = out_degree(PipelineInput, mBufferGraph) == 0 ? FirstKernel : PipelineInput;
    const auto lastKernel = in_degree(PipelineOutput, mBufferGraph) == 0 ? LastKernel : PipelineOutput;

    flat_set<Rational> fixedRate;

//    unsigned nextRateId = 0;

//    flat_map<Vertex, unsigned> partialSumRefId;
//    flat_map<unsigned, unsigned> addId;
//    flat_map<unsigned, unsigned> lookAheadId;
//    flat_map<unsigned, unsigned> lookBehindId;
//    flat_map<StreamSetPort, RefWrapper<BitSet>> relativeRefId;

    Graph H(LastStreamSet + 1);

    auto maxRateId = PartitionCount;
    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
        maxRateId += in_degree(kernel, mBufferGraph);
        maxRateId += out_degree(kernel, mBufferGraph);
    }

    const auto n = fixedRate.size();
    maxRateId += n;
    auto nextRateId = n;

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

        auto addRateId = [&](BitSet & bv, const unsigned rateId) {
           bv.resize(maxRateId);
           assert (rateId < maxRateId);
           bv.set(rateId);
        };

        const auto partRateId = nextRateId++;

        for (auto kernel = start; kernel < end; ++kernel) {

            auto updateBitSet = [&](const BufferPort & data, BitSet & S) {
                addRateId(S, partRateId);
                const auto anyFlags =
                    data.IsZeroExtended || data.IsDeferred ||
                    data.Add || data.Truncate ||
                    data.Delay || data.LookAhead || data.LookBehind;
                if (anyFlags || data.Minimum != data.Maximum) {
                    addRateId(S, nextRateId++);
                } else {
                    const auto f = fixedRate.find(data.Minimum);
                    const auto k = std::distance(fixedRate.begin(), f);
                    addRateId(S, k);
                }
            };

            for (const auto input : make_iterator_range(in_edges(kernel, mBufferGraph))) {
                const auto streamSet = source(input, mBufferGraph);
                BufferPort & data = mBufferGraph[input];
                const auto e = add_edge(streamSet, kernel, data, H).first;
                BitSet & S = H[e].RateSet;
                updateBitSet(data, S);
            }

            for (const auto output : make_iterator_range(out_edges(kernel, mBufferGraph))) {
                const auto streamSet = target(output, mBufferGraph);
                BufferPort & data = mBufferGraph[output];
                const auto e = add_edge(kernel, streamSet, data, H).first;
                BitSet & S = H[e].RateSet;
                updateBitSet(data, S);
            }

        }
        start = end;
    }

    #if 0

    auto & out = errs();
    out << "digraph \"H\" {\n";
    for (auto e : make_iterator_range(edges(H))) {
        const auto s = source(e, H);
        const auto t = target(e, H);
        out << "v" << s << " -> v" << t << " [label=\"{";
        const BitSet & S = H[e].RateSet;
        bool comma = false;
        for (auto i = S.find_first(); i != BitSet::npos; i = S.find_next(i)) {
            if (comma) out << ',';
            out << i;
            comma = true;
        }
        out << "}\"];\n";
    }

    out << "}\n\n";
    out.flush();

    #endif

    using GlobalPortIds = std::map<BitSet, unsigned>;

    GlobalPortIds globalPortIds;
    unsigned nextGlobalPortId = 0;

    auto getGlobalPortId = [&](const BitSet & B) {
        const auto f = globalPortIds.find(B);
        if (f == globalPortIds.end()) {
            const auto id = nextGlobalPortId++;
            globalPortIds.emplace(std::move(B), id);
            return id;
        }
        return f->second;
    };

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {

        for (const auto input : make_iterator_range(in_edges(kernel, H))) {
            auto & data = H[input];
            BufferPort & port = data.Port;
            port.GlobalPortId = getGlobalPortId(data.RateSet);
        }

        for (const auto output : make_iterator_range(out_edges(kernel, H))) {
            auto & data = H[output];
            BufferPort & port = data.Port.get();
            port.GlobalPortId = getGlobalPortId(data.RateSet);
        }

    }

    using LocalPortIdSet = SmallFlatSet<unsigned, 4>;
    using LocalPortIds = flat_map<std::pair<LocalPortIdSet, Rational>, unsigned>;


    LocalPortIds localPortIds;
    LocalPortIdSet ids;
    unsigned nextLocalPortId = 0;

    auto getLocalPortId = [&](const BufferNode & node, const BufferPort & rateData) {

//        Rational fillRate{};
//        if (node.NonLinear) {
//            const StreamSetBuffer * const buffer = node.Buffer;
//            assert (buffer);
//            if (isa<DynamicBuffer>(buffer)) {
//                return nextLocalPortId++;
//            } else if (isa<StaticBuffer>(buffer)) {
//                const Binding & binding = rateData.Binding;
//                const ProcessingRate & rate = binding.getRate();
//                if (LLVM_LIKELY(rate.isFixed())) {
//                    const StaticBuffer * const sbuffer = cast<StaticBuffer>(buffer);
//                    const Rational capacity{sbuffer->getCapacity()};
//                    fillRate = capacity / rateData.Minimum;
//                } else {
//                    return nextLocalPortId++;
//                }
//            }
//        }

        // auto key = std::make_pair(ids, fillRate);
        auto key = std::make_pair(ids, Rational{0});
        const auto f = localPortIds.find(key);
        if (f == localPortIds.end()) {
            const auto id = nextLocalPortId++;
            localPortIds.emplace(std::move(key), id);
            return id;
        } else {
            return f->second;
        }
    };

    auto resetLocalPortIds = [&]() {
        localPortIds.clear();
        nextLocalPortId = 0;
    };

//    auto calculateFillRatio = [&](const BufferNode & node, const BufferRateData & rateData) {
//        const StreamSetBuffer * const buffer = node.Buffer;
//        if (isa<StaticBuffer>(buffer)) {
//            const Binding & binding = rateData.Binding;
//            const ProcessingRate & rate = binding.getRate();
//            if (LLVM_LIKELY(rate.isFixed())) {
//                const StaticBuffer * const sbuffer = cast<StaticBuffer>(buffer);
//                const Rational capacity{sbuffer->getCapacity()};
//                return capacity / rateData.Minimum;
//            }
//        }
//        return Rational{};
//    };

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {

        assert (localPortIds.empty());

        for (const auto input : make_iterator_range(in_edges(kernel, mBufferGraph))) {
            const auto streamSet = source(input, mBufferGraph);
            const auto output = in_edge(streamSet, mBufferGraph);

            assert (streamSet >= FirstStreamSet && streamSet <= LastStreamSet);

            assert (ids.empty());

            const BufferNode & node = mBufferGraph[streamSet];

            BufferPort & I = mBufferGraph[input];
            if (node.isNonThreadLocal()) {
                I.LocalPortId = nextLocalPortId++;
            } else {
                ids.insert(I.GlobalPortId);
                const BufferPort & O = mBufferGraph[output];
                ids.insert(O.GlobalPortId);
                I.LocalPortId = getLocalPortId(node, I);
                ids.clear();
            }

        }

        assert (nextLocalPortId <= in_degree(kernel, mBufferGraph));

        MaxNumOfLocalInputPortIds = std::max<unsigned>(MaxNumOfLocalInputPortIds, nextLocalPortId);

        resetLocalPortIds();

        for (const auto output : make_iterator_range(out_edges(kernel, mBufferGraph))) {
            assert (ids.empty());

            BufferPort & O = mBufferGraph[output];
            ids.insert(O.GlobalPortId);

            const auto streamSet = target(output, mBufferGraph);

            assert (streamSet >= FirstStreamSet && streamSet <= LastStreamSet);

            for (const auto input : make_iterator_range(out_edges(streamSet, mBufferGraph))) {
                BufferPort & I = mBufferGraph[input];
                ids.insert(I.GlobalPortId);
            }

            const BufferNode & node = mBufferGraph[streamSet];
            O.LocalPortId = getLocalPortId(node, O);
            ids.clear();
        }

        assert (nextLocalPortId <= out_degree(kernel, mBufferGraph));

        MaxNumOfLocalOutputPortIds = std::max<unsigned>(MaxNumOfLocalOutputPortIds, nextLocalPortId);

        resetLocalPortIds();
    }

}


#if 0

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief identifyLinkedIOPorts
 *
 * Some I/O ports will always have an identical item count (relative to some known constant) throughout the
 * lifetime of the program. E.g., any partition local edge must be between two fixed rate I/O ports, which
 * entails that by knowing the initial value of one we can compute the latter.
 *
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::identifyLocalPortIds() {

    using BitSet = dynamic_bitset<>;
    using Vertex = BufferGraph::vertex_descriptor;
    using Graph = adjacency_list<vecS, vecS, bidirectionalS, no_property, BitSet>;


    const auto firstKernel = out_degree(PipelineInput, mBufferGraph) == 0 ? FirstKernel : PipelineInput;
    const auto lastKernel = in_degree(PipelineOutput, mBufferGraph) == 0 ? LastKernel : PipelineOutput;

    unsigned nextRateId = 0;

    flat_map<Vertex, unsigned> partialSumRefId;
    flat_map<unsigned, unsigned> addId;
    flat_map<unsigned, unsigned> lookAheadId;
    flat_map<unsigned, unsigned> lookBehindId;
    flat_map<StreamSetPort, RefWrapper<BitSet>> relativeRefId;

    Graph H(LastStreamSet + 1);

    auto addRateId = [](BitSet & bv, const unsigned rateId) {
       if (rateId >= bv.capacity()) {
           bv.resize(round_up_to(rateId + 1, BitSet::bits_per_block));
       }
       bv.set(rateId);
    };

    auto combine = [](BitSet & dst, BitSet & src) {
       if (LLVM_UNLIKELY(dst.size() < src.size())) {
           dst.resize(src.size());
       } else if (LLVM_UNLIKELY(src.size() < dst.size())) {
           src.resize(dst.size());
       }
       dst |= src;
    };

    for (auto start = firstKernel; start <= lastKernel; ) {
        // Determine which kernels are in this partition
        const auto partitionId = KernelPartitionId[start];
        auto end = start + 1U;
        for (; end <= LastKernel; ++end) {
            if (KernelPartitionId[end] != partitionId) {
                break;
            }
        }

        const auto partRateId = nextRateId++;
        for (auto kernel = start; kernel < end; ++kernel) {

            BitSet K;
            addRateId(K, partRateId);
            BitSet F = K;

            auto getPartialSumRefId = [&] (const StreamSetPort port) {
                const auto refPort = getReference(kernel, port);
                assert (refPort.Type == PortType::Input);
                const auto ref = getInputBufferVertex(kernel, refPort);
                const auto f = partialSumRefId.find(ref);
                if (f == partialSumRefId.end()) {
                    const auto id = nextRateId++;
                    partialSumRefId.emplace(ref, id);
                    return id;
                }
                return f->second;
            };

            auto getRelativeRefSet = [&] (const StreamSetPort port) -> BitSet & {
                const auto refPort = getReference(kernel, port);
                const auto f = relativeRefId.find(refPort);
                assert (f != relativeRefId.end());
                return f->second.get();
            };

            auto insert = [&] (BitSet & S, const int k, flat_map<unsigned, unsigned> & M) {
                if (k) {
                    unsigned id;
                    const auto f = M.find(k);
                    if (f == M.end()) {
                        id = nextRateId++;
                        M.emplace(k, id);
                    } else {
                        id = f->second;
                    }
                    addRateId(S, id);
                }
            };

            auto reset = [&]() {
                addId.clear();
                lookAheadId.clear();
                lookBehindId.clear();
            };

            reset();

            bool needsZeroExtendId = true;
            unsigned zeroExtendId = 0;
            bool hasLookBehind = false;
            bool hasInputFixedRate = false;
            for (const auto input : make_iterator_range(in_edges(kernel, mBufferGraph))) {
                const BufferRateData & data = mBufferGraph[input];
                const Binding & binding = data.Binding;
                const ProcessingRate & rate = binding.getRate();
                const auto streamSet = source(input, mBufferGraph);
                const auto output = in_edge(streamSet, H);
                const auto e = add_edge(streamSet, kernel, H).first;
                BitSet & S = H[e];
                S = H[output];
                addRateId(S, partRateId);
                if (rate.getKind() == RateId::Fixed) {
                    combine(F, S);
                    hasInputFixedRate = true;
                } else {
                    switch (rate.getKind()) {
                        case RateId::Bounded:
                        case RateId::Greedy:
                            addRateId(S, nextRateId++);
                            break;
                        case RateId::PartialSum:
                            addRateId(S, getPartialSumRefId(data.Port));
                            break;
                        case RateId::Relative:
                            combine(S, getRelativeRefSet(data.Port));
                        default: break;
                    }
                    relativeRefId.emplace(data.Port, S);
                }                
                if (data.IsZeroExtended) {
                    if (needsZeroExtendId) {
                        zeroExtendId = nextRateId++;
                        needsZeroExtendId = false;
                    }
                    addRateId(S, zeroExtendId);
                }
                insert(S, data.TransitiveAdd, addId);
                insert(S, data.LookAhead, lookAheadId);
                insert(S, data.LookBehind, lookBehindId);
                combine(K, S);
            }

            reset();

            for (const auto output : make_iterator_range(out_edges(kernel, mBufferGraph))) {
                const BufferRateData & data = mBufferGraph[output];
                const Binding & binding = data.Binding;
                const ProcessingRate & rate = binding.getRate();
                const auto streamSet = target(output, mBufferGraph);
                const auto e = add_edge(kernel, streamSet, H).first;
                BitSet & S = H[e];
                if (rate.getKind() == RateId::Fixed && hasInputFixedRate) {
                    S = F;
                } else {
                    S = K;
                    switch (rate.getKind()) {
                        case RateId::Bounded:
                        case RateId::Unknown:
                            addRateId(S, nextRateId++);
                            break;
                        case RateId::PartialSum:
                            addRateId(S, getPartialSumRefId(data.Port));
                            break;
                        case RateId::Relative:
                            combine(S, getRelativeRefSet(data.Port));
                        default: break;
                    }
                    relativeRefId.emplace(data.Port, S);
                }
                insert(S, data.TransitiveAdd, addId);
                insert(S, data.LookAhead, lookAheadId);
                insert(S, data.LookBehind, lookBehindId);
            }
            relativeRefId.clear();
        }
        start = end;
    }






    using GInIter = graph_traits<BufferGraph>::in_edge_iterator;
    using GOutIter = graph_traits<BufferGraph>::out_edge_iterator;

    using HInIter = graph_traits<Graph>::in_edge_iterator;
    using HOutIter = graph_traits<Graph>::out_edge_iterator;

    using GlobalPortIds = std::map<BitSet, unsigned>;

    GlobalPortIds globalPortIds;
    unsigned nextGlobalPortId = 0;

    auto getGlobalPortId = [&](const BitSet & B) {
        const auto f = globalPortIds.find(B);
        if (f == globalPortIds.end()) {
            const auto id = nextGlobalPortId++;
            globalPortIds.emplace(std::move(B), id);
            return id;
        }
        return f->second;
    };

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {

        GInIter ei_begin, ei_end;
        std::tie(ei_begin, ei_end) = in_edges(kernel, mBufferGraph);
        HInIter fi_begin, fi_end;
        std::tie(fi_begin, fi_end) = in_edges(kernel, H);

        assert (std::distance(ei_begin, ei_end) == std::distance(fi_begin, fi_end));

        auto ei = ei_begin;
        auto fi = fi_begin;
        for (; ei != ei_end; ++ei, ++fi) {
            assert (fi != fi_end);
            BufferRateData & br = mBufferGraph[*ei];
            BitSet & rateSet = H[*fi];
            if (nextRateId >= rateSet.capacity()) {
                rateSet.resize(nextRateId);
            }
            br.GlobalPortId = getGlobalPortId(rateSet);
        }

        GOutIter ej_begin, ej_end;
        std::tie(ej_begin, ej_end) = out_edges(kernel, mBufferGraph);
        HOutIter fj_begin, fj_end;
        std::tie(fj_begin, fj_end) = out_edges(kernel, H);

        assert (std::distance(ej_begin, ej_end) == std::distance(fj_begin, fj_end));

        auto ej = ej_begin;
        auto fj = fj_begin;
        for (; ej != ej_end; ++ej, ++fj) {
            assert (fj != fj_end);
            BufferRateData & br = mBufferGraph[*ej];
            BitSet & rateSet = H[*fj];
            if (nextRateId >= rateSet.capacity()) {
                rateSet.resize(nextRateId);
            }
            br.GlobalPortId = getGlobalPortId(rateSet);
        }

    }

    using LocalPortIdSet = SmallFlatSet<unsigned, 4>;
    using LocalPortIds = flat_map<std::pair<LocalPortIdSet, Rational>, unsigned>;


    LocalPortIds localPortIds;
    LocalPortIdSet ids;
    unsigned nextLocalPortId = 0;

    auto getLocalPortId = [&](const BufferNode & node, const BufferRateData & rateData) {

        Rational fillRate{};
        if (node.NonLinear) {
            const StreamSetBuffer * const buffer = node.Buffer;
            assert (buffer);
            if (isa<DynamicBuffer>(buffer)) {
                return nextLocalPortId++;
            } else if (isa<StaticBuffer>(buffer)) {
                const Binding & binding = rateData.Binding;
                const ProcessingRate & rate = binding.getRate();
                if (LLVM_LIKELY(rate.isFixed())) {
                    const StaticBuffer * const sbuffer = cast<StaticBuffer>(buffer);
                    const Rational capacity{sbuffer->getCapacity()};
                    fillRate = capacity / rateData.Minimum;
                } else {
                    return nextLocalPortId++;
                }
            }
        }

        auto key = std::make_pair(ids, fillRate);
        const auto f = localPortIds.find(key);
        if (f == localPortIds.end()) {
            const auto id = nextLocalPortId++;
            localPortIds.emplace(std::move(key), id);
            return id;
        } else {
            return f->second;
        }
    };

    auto resetLocalPortIds = [&]() {
        localPortIds.clear();
        nextLocalPortId = 0;
    };

    auto calculateFillRatio = [&](const BufferNode & node, const BufferRateData & rateData) {
        const StreamSetBuffer * const buffer = node.Buffer;
        if (isa<StaticBuffer>(buffer)) {
            const Binding & binding = rateData.Binding;
            const ProcessingRate & rate = binding.getRate();
            if (LLVM_LIKELY(rate.isFixed())) {
                const StaticBuffer * const sbuffer = cast<StaticBuffer>(buffer);
                const Rational capacity{sbuffer->getCapacity()};
                return capacity / rateData.Minimum;
            }
        }
        return Rational{};
    };

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {

        assert (localPortIds.empty());

        for (const auto input : make_iterator_range(in_edges(kernel, mBufferGraph))) {
            const auto streamSet = source(input, mBufferGraph);
            const auto output = in_edge(streamSet, mBufferGraph);

            assert (streamSet >= FirstStreamSet && streamSet <= LastStreamSet);

            assert (ids.empty());

            const BufferNode & node = mBufferGraph[streamSet];

            BufferRateData & I = mBufferGraph[input];
            if (node.NonLocal) {
                I.LocalPortId = nextLocalPortId++;
            } else {
                ids.insert(I.GlobalPortId);
                const BufferRateData & O = mBufferGraph[output];
                ids.insert(O.GlobalPortId);
                I.LocalPortId = getLocalPortId(node, I);
                ids.clear();
            }

        }

        assert (nextLocalPortId <= in_degree(kernel, mBufferGraph));

        MaxNumOfLocalInputPortIds = std::max<unsigned>(MaxNumOfLocalInputPortIds, nextLocalPortId);

        resetLocalPortIds();

        for (const auto output : make_iterator_range(out_edges(kernel, mBufferGraph))) {
            assert (ids.empty());

            BufferRateData & O = mBufferGraph[output];
            ids.insert(O.GlobalPortId);

            const auto streamSet = target(output, mBufferGraph);

            assert (streamSet >= FirstStreamSet && streamSet <= LastStreamSet);

            for (const auto input : make_iterator_range(out_edges(streamSet, mBufferGraph))) {
                BufferRateData & I = mBufferGraph[input];
                ids.insert(I.GlobalPortId);
            }

            const BufferNode & node = mBufferGraph[streamSet];
            O.LocalPortId = getLocalPortId(node, O);
            ids.clear();
        }

        assert (nextLocalPortId <= out_degree(kernel, mBufferGraph));

        MaxNumOfLocalOutputPortIds = std::max<unsigned>(MaxNumOfLocalOutputPortIds, nextLocalPortId);

        resetLocalPortIds();
    }

}

#endif

#if 0

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief identifyLinearBuffers
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::identifyPortsToCheck() {

    for (auto kernel = FirstKernel; kernel <= LastKernel; ++kernel) {

        bool hasDynamic = false;
        Rational fixedRateLCM{1};

        auto updateLCM = [&](const BufferNode & node, const BufferRateData & br) {
            const Binding & binding = br.Binding;
            const ProcessingRate & rate = binding.getRate();
            if (rate.isFixed()) {
                const StreamSetBuffer * const buffer = node.Buffer;
                if (const StaticBuffer * sbuffer = dyn_cast<StaticBuffer>(buffer)) {
                    const Rational capacity{sbuffer->getCapacity()};
                    const auto ratio = capacity / br.Minimum;
                    fixedRateLCM = lcm(fixedRateLCM, ratio);
                }
            }
        };

        for (const auto input : make_iterator_range(in_edges(kernel, mBufferGraph))) {
            const auto streamSet = source(input, mBufferGraph);
            BufferNode & node = mBufferGraph[streamSet];
            node.CheckRequired = node.NonLinear || node.NonLocal;
            updateLCM(node, mBufferGraph[input]);
        }

        for (const auto output : make_iterator_range(out_edges(kernel, mBufferGraph))) {
            const auto streamSet = target(output, mBufferGraph);
            BufferNode & node = mBufferGraph[streamSet];
            node.CheckRequired = node.NonLinear || node.NonLocal;
            updateLCM(node, mBufferGraph[output]);
        }

    }
}

#endif

} // end of kernel namespace

#endif // DATAFLOW_ANALYSIS_HPP
