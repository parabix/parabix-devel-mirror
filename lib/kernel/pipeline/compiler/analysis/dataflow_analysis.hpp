#ifndef DATAFLOW_ANALYSIS_HPP
#define DATAFLOW_ANALYSIS_HPP

#include "pipeline_analysis.hpp"

namespace kernel {


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeExpectedDataFlowRates
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::computeExpectedDataFlowRates(PartitionGraph & P) {

    const auto numOfPartitions = num_vertices(P);

    Partition kernels;

    for (unsigned partition = 0; partition < numOfPartitions; ++partition) {
        PartitionData & N = P[partition];
        const auto & K = N.Kernels;
        kernels.insert(kernels.end(), K.begin(), K.end());
        N.Repetitions.resize(K.size());
    }

    const auto numOfKernels = kernels.size();

    std::sort(kernels.begin(), kernels.end());

    Partition streamSets;

    for (const auto u : kernels) {
        const RelationshipNode & node = Relationships[u];
        assert (node.Type == RelationshipNode::IsKernel);
        for (const auto e : make_iterator_range(out_edges(u, Relationships))) {
            const auto binding = target(e, Relationships);
            if (Relationships[binding].Type == RelationshipNode::IsBinding) {
                const auto f = first_out_edge(binding, Relationships);
                assert (Relationships[f].Reason != ReasonType::Reference);
                const auto streamSet = target(f, Relationships);
                assert (Relationships[streamSet].Type == RelationshipNode::IsRelationship);
                assert (isa<StreamSet>(Relationships[streamSet].Relationship));
                streamSets.push_back(streamSet);
            }
        }
    }

    const auto numOfStreamSets = streamSets.size();

    std::sort(streamSets.begin(), streamSets.end());

    std::vector<Z3_ast> expReps(numOfKernels);

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
        #if Z3_VERSION_INTEGER > 40500
        return Z3_optimize_check(ctx, solver, 0, nullptr);
        #else
        return Z3_optimize_check(ctx, solver);
        #endif
    };

    const auto realType = Z3_mk_int_sort(ctx);

    auto constant_real = [&](const Rational value) {
        return Z3_mk_real(ctx, value.numerator(), value.denominator());
    };

    auto multiply =[&](Z3_ast X, Z3_ast Y) {
        assert (X);
        assert (Y);
        Z3_ast args[2] = { X, Y };
        return Z3_mk_mul(ctx, 2, args);
    };

    auto abs_sub =[&](Z3_ast X, Z3_ast Y) {
        Z3_ast args_1[2] = { X, Y };
        const auto a = Z3_mk_sub(ctx, 2, args_1);
        Z3_ast args_2[2] = { Y, X };
        const auto b = Z3_mk_sub(ctx, 2, args_2);
        const auto c = Z3_mk_le(ctx, X, Y);
        return Z3_mk_ite(ctx, c, b, a);
    };

    const auto ONE = Z3_mk_int(ctx, 1, realType);

    for (unsigned i = 0; i < numOfKernels; ++i) {
        auto v = Z3_mk_fresh_const(ctx, nullptr, realType);
        hard_assert(Z3_mk_ge(ctx, v, ONE));
        expReps[i] = v;
    }

    for (unsigned i = 0; i < numOfStreamSets; ++i) {

        const auto streamSet = streamSets[i];

        const auto f = first_in_edge(streamSet, Relationships);
        assert (Relationships[f].Reason != ReasonType::Reference);

        const auto binding = source(f, Relationships);
        const RelationshipNode & output = Relationships[binding];
        assert (output.Type == RelationshipNode::IsBinding);

        const Binding & outputBinding = output.Binding;
        const ProcessingRate & rate = outputBinding.getRate();
        // ignore unknown output rates; we cannot reason about them here.
        if (LLVM_LIKELY(rate.isUnknown())) {
            continue;
        }

        const auto g = first_in_edge(binding, Relationships);
        assert (Relationships[g].Reason != ReasonType::Reference);
        const unsigned producer = source(g, Relationships);

        const auto producerPartitionId = PartitionIds.find(producer)->second;
        assert (producerPartitionId > 0);

        const RelationshipNode & node = Relationships[producer];
        assert (node.Type == RelationshipNode::IsKernel);
        const auto h = std::lower_bound(kernels.begin(), kernels.end(), producer);
        assert (h != kernels.end() && *h == producer);
        const auto producerId = std::distance(kernels.begin(), h);

        const auto strideSize = node.Kernel->getStride();
        const auto sum = rate.getLowerBound() + rate.getUpperBound();
        const auto expectedOutput = sum * Rational{strideSize, 2};

        const Z3_ast expOutRate = multiply(expReps[producerId], constant_real(expectedOutput));

        for (const auto e : make_iterator_range(out_edges(streamSet, Relationships))) {
            const auto binding = target(e, Relationships);
            const RelationshipNode & input = Relationships[binding];
            if (LLVM_LIKELY(input.Type == RelationshipNode::IsBinding)) {
                const auto f = first_out_edge(binding, Relationships);
                assert (Relationships[f].Reason != ReasonType::Reference);
                const unsigned consumer = target(f, Relationships);

                const Binding & inputBinding = input.Binding;
                const ProcessingRate & rate = inputBinding.getRate();

                const auto consumerPartitionId = PartitionIds.find(consumer)->second;
                assert (producerPartitionId <= consumerPartitionId);

                const RelationshipNode & node = Relationships[consumer];
                assert (node.Type == RelationshipNode::IsKernel);

                const auto h = std::lower_bound(kernels.begin(), kernels.end(), consumer);
                assert (h != kernels.end() && *h == consumer);
                const auto consumerId = std::distance(kernels.begin(), h);

                if (LLVM_UNLIKELY(rate.isGreedy())) {
                    soft_assert(Z3_mk_eq(ctx, expReps[consumerId], ONE));
                } else {

                    const auto strideSize = node.Kernel->getStride();
                    const auto sum = rate.getLowerBound() + rate.getUpperBound();
                    const auto expectedInput = sum * Rational{strideSize, 2};

                    const Z3_ast expInRate = multiply(expReps[consumerId], constant_real(expectedInput));

                    if (producerPartitionId == consumerPartitionId) {
                        hard_assert(Z3_mk_eq(ctx, expOutRate, expInRate));
                    } else {
                        hard_assert(Z3_mk_ge(ctx, expOutRate, expInRate));
                        soft_assert(Z3_mk_eq(ctx, expOutRate, expInRate));
                        //Z3_optimize_minimize(ctx, solver, abs_sub(expOutRate, expInRate));
                    }
                }

            }
        }
    }

    if (LLVM_UNLIKELY(check() == Z3_L_FALSE)) {
        report_fatal_error("Z3 failed to find a solution to partition scheduling solution");
    }

    const auto model = Z3_optimize_get_model(ctx, solver);
    Z3_model_inc_ref(ctx, model);
    for (unsigned i = 0; i < numOfKernels; ++i) {

        Z3_ast value;
        if (LLVM_UNLIKELY(Z3_model_eval(ctx, model, expReps[i], Z3_L_TRUE, &value) != Z3_L_TRUE)) {
            report_fatal_error("Unexpected Z3 error when attempting to obtain value from model!");
        }

        __int64 num, denom;
        if (LLVM_UNLIKELY(Z3_get_numeral_rational_int64(ctx, value, &num, &denom) != Z3_L_TRUE)) {
            report_fatal_error("Unexpected Z3 error when attempting to convert model value to number!");
        }
        assert (num > 0);

        const auto kernelId = kernels[i];
        const auto f = PartitionIds.find(kernelId);
        assert (f != PartitionIds.end());
        const auto partitionId = f->second;
        assert (partitionId < numOfPartitions);

        auto & Pi = P[partitionId];

        const auto & K = Pi.Kernels;
        assert (Pi.Repetitions.size() == K.size());

        const auto h = std::find(K.begin(), K.end(), kernelId);
        assert (h != K.end() && *h == kernelId);
        const auto index = std::distance(K.begin(), h);

        Pi.Repetitions[index] = Rational{num, denom};
    }
    Z3_model_dec_ref(ctx, model);

    Z3_optimize_dec_ref(ctx, solver);
    Z3_del_context(ctx);
    Z3_reset_memory();

}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief identifyHardPartitionConstraints
 ** ------------------------------------------------------------------------------------------------------------- */
LengthConstraintGraph PipelineAnalysis::identifyLengthEqualityAssertions(BufferGraph & G) const {

    LengthConstraintGraph H(LastStreamSet + 1);

    if (mLengthAssertions.empty()) {
        return H;
    }

    flat_map<const StreamSet *, unsigned> M;

    for (auto streamSet = FirstStreamSet; streamSet <= LastStreamSet; ++streamSet) {
        const auto output = in_edge(streamSet, G);
        const BufferPort & br = G[output];
        const Binding & binding = br.Binding;
        M.emplace(cast<StreamSet>(binding.getRelationship()), streamSet);
    }

    auto offset = [&](const StreamSet * streamSet) {
        const auto f = M.find(streamSet);
        assert (f != M.end());
        return f->second;
    };

    for (const LengthAssertion & la : mLengthAssertions) {
        add_edge(offset(la[0]), offset(la[1]), H);
    }

    return H;
}

#ifdef USE_Z3_MINMAX_OPTIMIZE_FOR_DATAFLOW_ANALYSIS

// The advantage of using max-sat formulas instead of actual Z3 soft constraints is
// that we can play with the timeout to perform a pseudo-hill-climbing approach w.r.t.
// the number of soft constraints we satisfy. The disadvantage, however, is we
// dramatically increase the number of clauses in the formula Z3 considers.

#define USE_MAX_SAT_FOR_DATAFLOW_ANALYSIS

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeParitionDataFlowRates
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::computeDataFlowRates() {

    using Bound = std::array<Rational, 2>;

    errs() << "starting dataflow\n";

    const auto dataflow_start = std::chrono::high_resolution_clock::now();

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

    #ifdef USE_MAX_SAT_FOR_DATAFLOW_ANALYSIS
    std::vector<Z3_ast> assumptions;
    #endif
    auto soft_assert = [&](Z3_ast c) {
        #ifdef USE_MAX_SAT_FOR_DATAFLOW_ANALYSIS
        assumptions.push_back(c);
        #else
        Z3_optimize_assert_soft(ctx, solver, c, "1", nullptr);
        #endif
    };

    auto constant_real = [&](const Rational & value) {
        return Z3_mk_real(ctx, value.numerator(), value.denominator());
    };

    const auto ONE = constant_real(1);

    const auto TWO = constant_real(2);

    auto multiply =[&](Z3_ast X, Z3_ast Y) {
        Z3_ast args[2] = { X, Y };
        return Z3_mk_mul(ctx, 2, args);
    };

    auto sub =[&](Z3_ast X, Z3_ast Y) {
        Z3_ast args[2] = { X, Y };
        return Z3_mk_sub(ctx, 2, args);
    };

    auto add =[&](Z3_ast X, Z3_ast Y) {
        Z3_ast args[2] = { X, Y };
        return Z3_mk_add(ctx, 2, args);
    };

    auto mk_upperbound = [&](Z3_ast strides, const Rational & rate) {
        return multiply(strides, constant_real(rate));
    };

    auto mk_lowerbound = [&](Z3_ast strides, const Rational & rate) {
        if (rate.numerator() == 0) {
            return ONE;
        } else {
            return mk_upperbound(strides, rate);
        }
    };

    auto min_one_constant_real = [&](const Rational & rate) {
        if (rate.numerator() == 0) {
            return ONE;
        } else {
            return constant_real(rate);
        }
    };

    enum {
        LowerBound = 0,
        UpperBound = 1
    };

    const auto firstKernel = out_degree(PipelineInput, mBufferGraph) == 0 ? FirstKernel : PipelineInput;
    const auto lastKernel = in_degree(PipelineOutput, mBufferGraph) == 0 ? LastKernel : PipelineOutput;

    std::vector<std::array<Z3_ast, 2>> variables(LastStreamSet + 1);

    std::vector<Bound> bounds(PipelineOutput + 1);

    BEGIN_SCOPED_REGION

    const auto intType = Z3_mk_int_sort(ctx);

//    auto constant_int = [&](const Rational & value) {
//        assert (value.denominator() == 1);
//        return Z3_mk_int(ctx, value.numerator(), intType);
//    };

    auto make_partition_vars = [&](const unsigned first, const unsigned last) {

        const auto partitionId = KernelPartitionId[first];

        errs() << "Partition " << partitionId << " [" << first << "," << last << ")\n";

        Z3_optimize_push(ctx, solver);

        const auto & expectedBase = ExpectedNumOfStrides[first];
        assert (expectedBase.numerator() >= 1);
        const auto expected = constant_real(expectedBase);

        auto root_lb = expected;
        auto root_ub = expected;

        if (in_degree(first, mBufferGraph) > 0) {

            root_lb = Z3_mk_fresh_const(ctx, "lb", intType);
            root_ub = Z3_mk_fresh_const(ctx, "ub", intType);

            Z3_optimize_minimize(ctx, solver, root_lb);
            Z3_optimize_maximize(ctx, solver, root_ub);

            hard_assert(Z3_mk_ge(ctx, root_lb, ONE));

            hard_assert(Z3_mk_le(ctx, root_lb, root_ub));

//            hard_assert(Z3_mk_le(ctx, root_lb, expected));
//            hard_assert(Z3_mk_ge(ctx, root_ub, expected));

            const auto maximum = constant_real(expectedBase * Rational{2});
            hard_assert(Z3_mk_le(ctx, root_ub, maximum));
        }

        auto & root = variables[first];
        root[LowerBound] = root_lb;
        root[UpperBound] = root_ub;

        for (auto kernel = first + 1U; kernel <= last; ++kernel) {
            const auto r = ExpectedNumOfStrides[kernel] / expectedBase;
            auto lb = root_lb;
            auto ub = root_ub;
            if (LLVM_UNLIKELY(r != Rational{1})) {
                const auto ratio = constant_real(r);
                lb = multiply(lb, ratio);
                ub = multiply(ub, ratio);
            }
            auto & v = variables[kernel];
            v[LowerBound] = lb;
            v[UpperBound] = ub;
        }

        bool hasConstraints = false;

        for (auto kernel = first; kernel <= last; ++kernel) {

            const auto & stridesPerSegmentVar = variables[kernel];

            for (const auto input : make_iterator_range(in_edges(kernel, mBufferGraph))) {
                const auto streamSet = source(input, mBufferGraph);
                const auto producer = parent(streamSet, mBufferGraph);
                const auto producerPartitionId = KernelPartitionId[producer];

                if (producerPartitionId != partitionId) {
                    const BufferPort & inputRate = mBufferGraph[input];
                    const Binding & binding = inputRate.Binding;
                    const ProcessingRate & rate = binding.getRate();

                    const auto & outputVar = variables[streamSet];

                    assert (producerPartitionId < partitionId);
                    assert (outputVar[LowerBound]);
                    assert (outputVar[UpperBound]);

                    Z3_ast minInputRateVar = nullptr;
                    Z3_ast maxInputRateVar = nullptr;

                    if (LLVM_LIKELY(rate.isFixed())) {
                        const auto fixedRateVal = constant_real(inputRate.Minimum);
                        minInputRateVar = multiply(stridesPerSegmentVar[LowerBound], fixedRateVal);
                        maxInputRateVar = multiply(stridesPerSegmentVar[UpperBound], fixedRateVal);
                    } else if (LLVM_UNLIKELY(rate.isGreedy())) {
                        minInputRateVar = outputVar[LowerBound];
                        maxInputRateVar = outputVar[UpperBound];
                    } else {
                        minInputRateVar = mk_lowerbound(stridesPerSegmentVar[LowerBound], inputRate.Minimum);
                        maxInputRateVar = mk_upperbound(stridesPerSegmentVar[UpperBound], inputRate.Maximum);
                    }

                    auto minOutputRateVar = outputVar[LowerBound];
                    auto maxOutputRateVar = outputVar[UpperBound];

                    soft_assert(Z3_mk_le(ctx, minInputRateVar, minOutputRateVar));

                    const auto x2_diff = multiply(sub(minOutputRateVar, minInputRateVar), TWO);

                    soft_assert(Z3_mk_le(ctx, maxInputRateVar, add(maxOutputRateVar, x2_diff)));



//                    soft_assert(Z3_mk_le(ctx, minInputRateVar, maxOutputRateVar));


//                    const auto lb = Z3_mk_div(ctx, minOutputRateVar, maxInputRateVar);
//                    const auto ub = Z3_mk_div(ctx, maxOutputRateVar, minInputRateVar);


////                    soft_assert(Z3_mk_eq(ctx, lb, ONE));

////                    soft_assert(Z3_mk_eq(ctx, ub, ONE));

//                    Z3_optimize_maximize(ctx, solver, lb);
//                    Z3_optimize_minimize(ctx, solver, ub);

//                    soft_assert(Z3_mk_le(ctx, lb, ub));

//                    soft_assert(Z3_mk_le(ctx, ub, ONE));

//                    soft_assert(Z3_mk_eq(ctx, stridesPerSegmentVar[LowerBound], lb));
//                    soft_assert(Z3_mk_eq(ctx, stridesPerSegmentVar[UpperBound], ub));

                    hasConstraints = true;
                }
            }
        }

        errs() << Z3_optimize_to_string(ctx, solver) << "\n";

        #ifdef USE_MAX_SAT_FOR_DATAFLOW_ANALYSIS
        const auto m = Z3_maxsat(ctx, solver, assumptions);
        errs() << " m=" << m << " of " << assumptions.size() << "\n";
        #else

        if (LLVM_UNLIKELY(Z3_optimize_check(ctx, solver, 0, nullptr) == Z3_L_FALSE)) {
            errs() << "Z3 dataflow error: " << "failed to generate dataflow solution" << "\n";
            exit(-1);
        }

        #endif

        const auto model = Z3_optimize_get_model(ctx, solver);
        Z3_model_inc_ref(ctx, model);

        for (auto kernel = first; kernel <= last; ++kernel) {

            const auto & stridesPerSegmentVar = variables[kernel];

            for (unsigned bound = LowerBound; bound <= UpperBound; ++bound) {

                Z3_ast value;

                if (LLVM_UNLIKELY(Z3_model_eval(ctx, model, stridesPerSegmentVar[bound], Z3_L_TRUE, &value) != Z3_L_TRUE)) {
                    report_fatal_error("Unexpected Z3 error when attempting to obtain value from model!");
                }

                __int64 num, denom;
                if (LLVM_UNLIKELY(Z3_get_numeral_rational_int64(ctx, value, &num, &denom) != Z3_L_TRUE)) {
                    report_fatal_error("Unexpected Z3 error when attempting to convert model value to number!");
                }
               // assert (num > 0);
                bounds[kernel][bound] = Rational{num, denom};
            }
        }

        Z3_model_dec_ref(ctx, model);

        Z3_optimize_pop(ctx, solver);

        for (auto kernel = first; kernel <= last; ++kernel) {
            // const auto & stridesPerSegmentVar = variables[kernel];

            const auto & bound = bounds[kernel];

            auto print_rational =[&] (const Rational & r) {
                errs() << r.numerator() << "/" << r.denominator();
            };

            errs() << "  Kernel " << kernel << " [";
            print_rational(bound[LowerBound]);
            errs() << ",";
            print_rational(bound[UpperBound]);
            errs() << "]\n";

            for (const auto output : make_iterator_range(out_edges(kernel, mBufferGraph))) {
                const BufferPort & outputRate = mBufferGraph[output];
                const Binding & binding = outputRate.Binding;
                const ProcessingRate & rate = binding.getRate();
                const auto streamSet = target(output, mBufferGraph);

                Z3_ast minOutputRateVar = nullptr;
                Z3_ast maxOutputRateVar = nullptr;

                if (LLVM_LIKELY(rate.isFixed())) {
                    minOutputRateVar = constant_real(bound[LowerBound] * outputRate.Minimum);
                    maxOutputRateVar = constant_real(bound[UpperBound] * outputRate.Minimum);
                } else {
                    minOutputRateVar = min_one_constant_real(bound[LowerBound] * outputRate.Minimum);
                    assert (minOutputRateVar == ONE);
                    if (LLVM_UNLIKELY(rate.isUnknown())) {
                        // TODO: is there a better way to handle unknown outputs? This
                        // free variable represents the ideal amount of data to transfer
                        // to subsequent kernels but that isn't very meaningful here.
                        maxOutputRateVar = Z3_mk_fresh_const(ctx, nullptr, intType);
                        hard_assert(Z3_mk_le(ctx, minOutputRateVar, maxOutputRateVar));
                    } else {
                        maxOutputRateVar = constant_real(bound[UpperBound] * outputRate.Maximum);
                    }
                }

                auto & outputVar = variables[streamSet];
                outputVar[LowerBound] = minOutputRateVar;
                outputVar[UpperBound] = maxOutputRateVar;
            }

        }

//        for (auto kernel = first; kernel <= last; ++kernel) {
//            const auto & stridesPerSegmentVar = variables[kernel];

//            const auto & bound = bounds[kernel];

//            auto print_rational =[&] (const Rational & r) {
//                errs() << r.numerator() << "/" << r.denominator();
//            };

//            errs() << "  Kernel " << kernel << " [";
//            print_rational(bound[LowerBound]);
//            errs() << ",";
//            print_rational(bound[UpperBound]);
//            errs() << "]\n";

//            const auto lb = constant_real(bounds[kernel][LowerBound]);
//            hard_assert(Z3_mk_eq(ctx, stridesPerSegmentVar[LowerBound], lb));

//            const auto ub = constant_real(bounds[kernel][UpperBound]);
//            hard_assert(Z3_mk_eq(ctx, stridesPerSegmentVar[UpperBound], ub));

//            for (const auto output : make_iterator_range(out_edges(kernel, mBufferGraph))) {
//                const BufferPort & outputRate = mBufferGraph[output];
//                const Binding & binding = outputRate.Binding;
//                const ProcessingRate & rate = binding.getRate();
//                const auto streamSet = target(output, mBufferGraph);

//                Z3_ast minOutputRateVar = nullptr;
//                Z3_ast maxOutputRateVar = nullptr;

//                if (LLVM_LIKELY(rate.isFixed())) {
//                    const auto fixedRateVal = constant_real(outputRate.Minimum);
//                    minOutputRateVar = multiply(stridesPerSegmentVar[LowerBound], fixedRateVal);
//                    maxOutputRateVar = multiply(stridesPerSegmentVar[UpperBound], fixedRateVal);
//                } else {
//                    minOutputRateVar = mk_lowerbound(stridesPerSegmentVar[LowerBound], outputRate.Minimum);
//                    if (LLVM_UNLIKELY(rate.isUnknown())) {
//                        // TODO: is there a better way to handle unknown outputs? This
//                        // free variable represents the ideal amount of data to transfer
//                        // to subsequent kernels but that isn't very meaningful here.
//                        maxOutputRateVar = Z3_mk_fresh_const(ctx, nullptr, realType);
//                        hard_assert(Z3_mk_le(ctx, minOutputRateVar, maxOutputRateVar));
//                    } else {
//                        maxOutputRateVar = mk_upperbound(stridesPerSegmentVar[UpperBound], outputRate.Maximum);
//                    }
//                }

//                auto & outputVar = variables[streamSet];
//                outputVar[LowerBound] = minOutputRateVar;
//                outputVar[UpperBound] = maxOutputRateVar;
//            }

//        }

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




//    const auto E = identifyLengthEqualityAssertions(mBufferGraph);
//    for (const auto e : make_iterator_range(edges(E))) {
//        const auto & A = variables[source(e, mBufferGraph)];
//        const auto & B = variables[target(e, mBufferGraph)];

//        soft_assert(Z3_mk_eq(ctx, A[LowerBound], B[LowerBound]));
//        soft_assert(Z3_mk_eq(ctx, A[UpperBound], B[UpperBound]));
//    }

//    #ifdef USE_MAX_SAT_FOR_DATAFLOW_ANALYSIS
//    const auto m = Z3_maxsat(ctx, solver, assumptions);
//    errs() << " m=" << m << " of " << assumptions.size() << "\n";
//    #else

//    try {
//        if (LLVM_UNLIKELY(Z3_optimize_check(ctx, solver, 0, nullptr) == Z3_L_FALSE)) {
//            errs() << "Z3 dataflow error: " << "failed to generate dataflow solution" << "\n";
//            exit(-1);
//        }
//    } catch (const std::runtime_error & e) {
//        errs() << "Z3 dataflow error: " << e.what() << "\n";
//        exit(-1);
//    }

//    #endif
//    std::vector<Bound> bounds(PipelineOutput + 1);

//    const auto model = Z3_optimize_get_model(ctx, solver);
//    Z3_model_inc_ref(ctx, model);

//    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {

//        const auto & stridesPerSegmentVar = variables[kernel];

//        for (unsigned bound = LowerBound; bound <= UpperBound; ++bound) {

//            Z3_ast value;

//            if (LLVM_UNLIKELY(Z3_model_eval(ctx, model, stridesPerSegmentVar[bound], Z3_L_TRUE, &value) != Z3_L_TRUE)) {
//                report_fatal_error("Unexpected Z3 error when attempting to obtain value from model!");
//            }

//            __int64 num, denom;
//            if (LLVM_UNLIKELY(Z3_get_numeral_rational_int64(ctx, value, &num, &denom) != Z3_L_TRUE)) {
//                report_fatal_error("Unexpected Z3 error when attempting to convert model value to number!");
//            }
//            if (LLVM_UNLIKELY(num == 0)) {
//                std::string tmp;
//                raw_string_ostream out(tmp);
//                if (bound == LowerBound) {
//                    out << "Lower";
//                } else {
//                    out << "Upper";
//                }
//                out << "-bound for kernel " << kernel << " is zero?";
//                report_fatal_error(out.str());
//            }

//            bounds[kernel][bound] = Rational{num, denom};
//        }
//    }

//    Z3_model_dec_ref(ctx, model);

    Z3_optimize_dec_ref(ctx, solver);
    Z3_del_context(ctx);
    Z3_reset_memory();


    const auto & b = bounds[firstKernel];
    auto g = gcd(b[LowerBound], b[UpperBound]);
    for (auto kernel = firstKernel + 1; kernel <= lastKernel; ++kernel) {
        const auto & b = bounds[kernel];
        g = gcd(g, b[LowerBound]);
        g = gcd(g, b[UpperBound]);
    }
    if (LLVM_UNLIKELY(g > Rational{1})) {
        for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
            auto & r = bounds[kernel];
            r[LowerBound] /= g;
            r[UpperBound] /= g;
        }
    }

    // Then write them out
    MinimumNumOfStrides.resize(PipelineOutput + 1);
    MaximumNumOfStrides.resize(PipelineOutput + 1);

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
        auto & b = bounds[kernel];
        auto & lb = b[LowerBound];
        auto & ub = b[UpperBound];
        if (LLVM_UNLIKELY(lb.denominator() != 1)) {
            const auto r = Rational{lb.numerator() % lb.denominator(), lb.denominator()};
            lb -= r;
            ub += r;
        }
        if (LLVM_UNLIKELY(ub.denominator() != 1)) {
            const auto r = Rational{ub.denominator() - ub.numerator() % ub.denominator(), ub.denominator()};
            ub += r;
        }
        assert(lb.denominator() == 1);
        assert(ub.denominator() == 1);
        MinimumNumOfStrides[kernel] = lb;
        MaximumNumOfStrides[kernel] = ub;
    }

    const auto dataflow_end = std::chrono::high_resolution_clock::now();
    const auto total = (dataflow_end - dataflow_start).count();

    errs() << "finished compute dataflow: " << total << "\n";

}

#else

#if 0

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeParitionDataFlowRates
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::computeDataFlowRates() {

    struct KernelRange {
        unsigned first;
        unsigned last;
    };

    using Graph = adjacency_list<vecS, vecS, bidirectionalS, KernelRange>;

    const auto firstKernel = out_degree(PipelineInput, mBufferGraph) == 0 ? FirstKernel : PipelineInput;
    const auto lastKernel = in_degree(PipelineOutput, mBufferGraph) == 0 ? LastKernel : PipelineOutput;

    Graph G(PartitionCount);

    BEGIN_SCOPED_REGION

    BitVector targets(PartitionCount);

    for (auto first = firstKernel; first < lastKernel; ++first) {

        auto last = first + 1U;

        const auto partitionId = KernelPartitionId[first];
        for (; last <= lastKernel; ++last) {
            if (partitionId != KernelPartitionId[last]) {
                break;
            }
        }

        KernelRange & range = G[partitionId];

        range.first = first;
        range.last = last - 1;

        targets.reset();
        for (auto kernel = first; kernel < last; ++kernel) {
            for (const auto output : make_iterator_range(out_edges(kernel, mBufferGraph))) {
                const auto streamSet = target(output, mBufferGraph);
                for (const auto input : make_iterator_range(out_edges(streamSet, mBufferGraph))) {
                    targets.set(KernelPartitionId[target(input, mBufferGraph)]);
                }
            }
        }
        targets.reset(partitionId);
        for (const auto consumer : targets.set_bits()) {
            add_edge(partitionId, consumer, G);
        }
    }


    const reverse_traversal ordering{PartitionCount};
    assert (is_valid_topological_sorting(ordering, G));
    transitive_closure_dag(ordering, G);
    transitive_reduction_dag(ordering, G);

    END_SCOPED_REGION


    using Bound = std::array<Rational, 2>;

    const auto cfg = Z3_mk_config();
    Z3_set_param_value(cfg, "model", "true");
    Z3_set_param_value(cfg, "proof", "false");
    const auto ctx = Z3_mk_context(cfg);
    Z3_del_config(cfg);
    const auto solver = Z3_mk_solver(ctx);
    Z3_solver_inc_ref(ctx, solver);

    const auto realType = Z3_mk_real_sort(ctx);

    const auto intType = Z3_mk_int_sort(ctx);

    auto constant = [&](const Rational value) {
        return Z3_mk_real(ctx, value.numerator(), value.denominator());
    };

    const auto ONE = constant(1);

    auto average = [&](const BufferPort & rate) {
        return constant((rate.Minimum + rate.Maximum) * Rational{1, 2});
    };

    auto maximum = [&](const BufferPort & rate) {
        return constant(rate.Maximum);
    };

    auto minimum = [&](const BufferPort & rate) {
        return constant(rate.Minimum);
    };

    auto hard_assert = [&](Z3_ast c) {
        Z3_solver_assert(ctx, solver, c);
    };

    auto free_variable = [&]() {
        return Z3_mk_fresh_const(ctx, nullptr, realType);;
    };

    auto multiply =[&](Z3_ast X, Z3_ast Y) {
        Z3_ast args[2] = { X, Y };
        return Z3_mk_mul(ctx, 2, args);
    };

    std::vector<Z3_ast> VarList(LastStreamSet + 1);

    std::vector<unsigned> ordering;
    ordering.reserve(PartitionCount);
    lexical_ordering(G, ordering);


    BEGIN_SCOPED_REGION



    auto make_partition_vars = [&](const unsigned partitionId, const unsigned first, const unsigned last) {

        const auto & expectedBase = ExpectedNumOfStrides[first];
        assert (expectedBase.numerator() >= 1);

        // create our partition variables

        auto root = Z3_mk_fresh_const(ctx, nullptr, intType);

        VarList[first] = root;

        hard_assert(Z3_mk_ge(ctx, root, ONE));
        const auto maximum = constant(expectedBase * Rational{2});
        hard_assert(Z3_mk_le(ctx, root, maximum));

        for (auto kernel = first + 1U; kernel <= last; ++kernel) {
            const auto r = ExpectedNumOfStrides[kernel] / expectedBase;
            auto rate = root;
            if (LLVM_UNLIKELY(r != Rational{1})) {
                rate = multiply(rate, constant(r));
            }
            VarList[kernel] = rate;
        }

        // Z3_mk_forall()

        for (auto kernel = first; kernel <= last; ++kernel) {
            const auto stridesPerSegmentVar = VarList[kernel];

            for (const auto input : make_iterator_range(in_edges(kernel, mBufferGraph))) {
                const auto streamSet = source(input, mBufferGraph);
                const auto producer = parent(streamSet, mBufferGraph);
                const auto producerPartitionId = KernelPartitionId[producer];

                if (producerPartitionId != partitionId) {
                    const BufferPort & inputRate = mBufferGraph[input];
                    const Binding & binding = inputRate.Binding;
                    const ProcessingRate & rate = binding.getRate();

                    auto outputRateVar = VarList[streamSet];

                    if (inputRate.LookAhead) {




                    }

                    assert (producerPartitionId < partitionId);
                    assert (outputRateVar);

                    Z3_ast inputRateVar = nullptr;

                    if (LLVM_LIKELY(rate.isFixed())) {
                        inputRateVar = multiply(stridesPerSegmentVar, constant(inputRate.Minimum));
                    } else if (LLVM_UNLIKELY(rate.isGreedy())) {
                        inputRateVar = outputRateVar;
                    } else {
                        inputRateVar = Z3_mk_fresh_const(ctx, nullptr, intType);
                        const auto min = multiply(stridesPerSegmentVar, constant(inputRate.Minimum));
                        hard_assert(Z3_mk_ge(ctx, outputRateVar, min));
                        if (LLVM_LIKELY(rate.isBounded() || rate.isPartialSum())) {
                            // TODO: we need to still link popcount rates but can only do that after
                            // proving that the source item stream rates are equal
                            const auto max = multiply(stridesPerSegmentVar, constant(inputRate.Maximum));
                            hard_assert(Z3_mk_ge(ctx, outputRateVar, max));
                        }
                    }




                }
            }
        }

        for (auto kernel = first; kernel <= last; ++kernel) {
            const auto stridesPerSegmentVar = VarList[kernel];

            for (const auto output : make_iterator_range(out_edges(kernel, mBufferGraph))) {
                const BufferPort & outputRate = mBufferGraph[output];
                const Binding & binding = outputRate.Binding;
                const ProcessingRate & rate = binding.getRate();
                const auto streamSet = target(output, mBufferGraph);

                Z3_ast outputRateVar = nullptr;
                if (LLVM_LIKELY(rate.isFixed())) {
                    outputRateVar = multiply(stridesPerSegmentVar, constant(outputRate.Minimum));
                } else {
                    outputRateVar = Z3_mk_fresh_const(ctx, nullptr, intType);
                    const auto min = multiply(stridesPerSegmentVar, constant(outputRate.Minimum));
                    hard_assert(Z3_mk_ge(ctx, outputRateVar, min));
                    if (LLVM_LIKELY(rate.isBounded() || rate.isPartialSum())) {
                        const auto max = multiply(stridesPerSegmentVar, constant(outputRate.Maximum));
                        hard_assert(Z3_mk_ge(ctx, outputRateVar, max));
                    }
                }



                VarList[streamSet] = outputRateVar;
            }

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



//    enum {
//        LowerBound = 0,
//        UpperBound = 1,
//        Common = 2
//    };

//    std::array<std::vector<Z3_ast>, 3> assumptions;

//    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
//        const auto var = free_variable();



//        const auto expected = constant(ExpectedNumOfStrides[kernel]);
//        assumptions[LowerBound].push_back(Z3_mk_le(ctx, var, expected));


//        assumptions[UpperBound].push_back(Z3_mk_ge(ctx, var, expected));
//        const auto expected_x_2 = constant(ExpectedNumOfStrides[kernel] * Rational{2});
//        assumptions[UpperBound].push_back(Z3_mk_le(ctx, var, expected_x_2));
//        VarList[kernel] = var;
//    }

//    const auto H = identifyHardPartitionConstraints();

//    auto crosses_partition_boundary =[&H](const unsigned p1, const unsigned p2) {
//        return edge(p1, p2, H).second;
//    };

//    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {

//        const auto stridesPerSegmentVar = VarList[kernel];
//        const auto partitionId = KernelPartitionId[kernel];

//        unsigned numOfGreedyRates = 0;

//        for (const auto input : make_iterator_range(in_edges(kernel, mBufferGraph))) {
//            const auto buffer = source(input, mBufferGraph);

//            const BufferPort & inputRate = mBufferGraph[input];
//            const Binding & binding = inputRate.Binding;
//            const ProcessingRate & rate = binding.getRate();

//            const auto producedRate = VarList[buffer]; assert (producedRate);

//            if (LLVM_LIKELY(rate.isFixed())) {

//                const auto fixedRateVal = minimum(inputRate);
//                const auto consumedRate = multiply(stridesPerSegmentVar, fixedRateVal);
//                // To consume any data, we want to guarantee at least one full stride of work.
//                const auto atLeastOneStride = Z3_mk_ge(ctx, consumedRate, fixedRateVal);
//                assumptions[Common].push_back(atLeastOneStride);
//                // A partition-local stream must *always* consume everything but streams that
//                // cross a partition simply need to produce enough to satisfy its consumer(s)
//                const auto consumeEverything = Z3_mk_eq(ctx, producedRate, consumedRate);
//                const auto producer = parent(buffer, mBufferGraph);

//                if (crosses_partition_boundary(KernelPartitionId[producer], partitionId)) {
//                    const auto produceEnough = Z3_mk_ge(ctx, producedRate, consumedRate);
//                    Z3_solver_assert(ctx, solver, produceEnough);
//                    // Assume we can consume all of the data
//                    assumptions[Common].push_back(consumeEverything);
//                } else {
//                    Z3_solver_assert(ctx, solver, consumeEverything);
//                }
//            } else if (LLVM_UNLIKELY(rate.isGreedy())) {
//                ++numOfGreedyRates;
//            } else {

//                const auto inputRateVar = bounded_variable(inputRate);
//                const auto consumedRate = multiply(stridesPerSegmentVar, inputRateVar);
//                const auto produceEnough = Z3_mk_ge(ctx, producedRate, consumedRate);
//                Z3_solver_assert(ctx, solver, produceEnough);

//                // To consume any data, we want to guarantee at least one full stride of work;
//                // however we can only state that only Bounded rates will block on less than
//                // this amount.
//                const auto atLeastOneStride = Z3_mk_ge(ctx, consumedRate, maximum(inputRate));
//                if (rate.isBounded()) {
//                    Z3_solver_assert(ctx, solver, atLeastOneStride);
//                } else {
//                    assumptions[Common].push_back(atLeastOneStride);
//                }

//                // Since we are trying to determine the lower and upper bound on the number
//                // of strides per segment, to determine the lower bound we assume that a
//                // kernel consumes the maximum amount of data but produces the minimum.
//                // The upper bound is similar except we consume the minimum and produce
//                // the maximum.

//                const auto avgInputRate = average(inputRate);

//                assumptions[LowerBound].push_back(Z3_mk_ge(ctx, inputRateVar, avgInputRate));
//                assumptions[LowerBound].push_back(Z3_mk_eq(ctx, inputRateVar, maximum(inputRate)));

//                assumptions[UpperBound].push_back(Z3_mk_le(ctx, inputRateVar, avgInputRate));
//                assumptions[UpperBound].push_back(Z3_mk_eq(ctx, consumedRate, minimum(inputRate)));
//            }
//        }

//        // Any kernel with all greedy rates must exhaust its input in a single iteration.
//        if (LLVM_UNLIKELY(numOfGreedyRates == in_degree(kernel, mBufferGraph))) {
//            const auto constraint = Z3_mk_eq(ctx, stridesPerSegmentVar, ONE);
//            Z3_solver_assert(ctx, solver, constraint);
//        }

//        for (const auto output : make_iterator_range(out_edges(kernel, mBufferGraph))) {
//            const BufferPort & outputRate = mBufferGraph[output];
//            const Binding & binding = outputRate.Binding;
//            const ProcessingRate & rate = binding.getRate();
//            const auto buffer = target(output, mBufferGraph);
//            if (LLVM_LIKELY(rate.isFixed())) {
//                const auto outputRateVar = maximum(outputRate);
//                const auto producedRate = multiply(stridesPerSegmentVar, outputRateVar);
//                VarList[buffer] = producedRate;
//            } else if (LLVM_UNLIKELY(rate.isUnknown())) {
//                // TODO: is there a better way to handle unknown outputs? This
//                // free variable represents the ideal amount of data to transfer
//                // to subsequent kernels but that isn't very meaningful here.
//                VarList[buffer] = lower_bounded_variable(outputRate);
//            } else {
//                const auto outputRateVar = bounded_variable(outputRate);
//                const auto producedRate = multiply(stridesPerSegmentVar, outputRateVar);
//                VarList[buffer] = producedRate;
//                // Like above, when calculating the lower bound of the number of kernel strides,
//                // we assume we've produced the minimum amount of data and for the upper bound,
//                // the maximum.

//                const auto avgOutputRate = average(outputRate);
//                assumptions[LowerBound].push_back(Z3_mk_le(ctx, outputRateVar, avgOutputRate));
//                assumptions[LowerBound].push_back(Z3_mk_eq(ctx, producedRate, minimum(outputRate)));

//                assumptions[UpperBound].push_back(Z3_mk_ge(ctx, outputRateVar, avgOutputRate));
//                assumptions[UpperBound].push_back(Z3_mk_eq(ctx, outputRateVar, maximum(outputRate)));
//            }
//        }
//    }

//    const auto E = identifyLengthEqualityAssertions(mBufferGraph);
//    for (const auto e : make_iterator_range(edges(E))) {
//        const auto A = VarList[source(e, mBufferGraph)];
//        const auto B = VarList[target(e, mBufferGraph)];

//        Z3_solver_assert(ctx, solver, Z3_mk_eq(ctx, A, B));
//    }

//    if (LLVM_UNLIKELY(Z3_solver_check(ctx, solver) == Z3_L_FALSE)) {
//        report_fatal_error("Z3 failed to find a solution to the core dataflow graph");
//    }
//    Z3_solver_push(ctx, solver);

//    const auto m = Z3_maxsat(ctx, solver, assumptions[Common]);

//    if (LLVM_UNLIKELY(m == 0)) {
//        Z3_solver_pop(ctx, solver, 1);
//        Z3_solver_check(ctx, solver);
//    }

//    // Use the initial solution as a default for both the upper and lower bounds incase
//    // there are no upper or lower bound assumptions.

//    std::vector<Bound> bounds(PipelineOutput + 1);

//    const auto model = Z3_solver_get_model(ctx, solver);
//    Z3_model_inc_ref(ctx, model);

//    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {

//        Z3_ast const stridesPerSegmentVar = VarList[kernel];
//        Z3_ast value;
//        if (LLVM_UNLIKELY(Z3_model_eval(ctx, model, stridesPerSegmentVar, Z3_L_TRUE, &value) != Z3_L_TRUE)) {
//            report_fatal_error("Unexpected Z3 error when attempting to obtain value from model!");
//        }

//        __int64 num, denom;
//        if (LLVM_UNLIKELY(Z3_get_numeral_rational_int64(ctx, value, &num, &denom) != Z3_L_TRUE)) {
//            report_fatal_error("Unexpected Z3 error when attempting to convert model value to number!");
//        }
//        assert (num > 0);

//        const auto r = Rational{num, denom};
//        for (unsigned bound = LowerBound; bound <= UpperBound; ++bound) {
//            bounds[kernel][bound] = r;
//        }
//    }

//    Z3_model_dec_ref(ctx, model);

//    // Now check whether any of the upper/lower bound assumptions alter the results

//    for (;;) {

//        for (unsigned bound = LowerBound; bound <= UpperBound; ++bound) {

//            const auto & A = assumptions[bound];

//            if (LLVM_UNLIKELY(A.empty())) continue;

//            Z3_solver_push(ctx, solver);

//            const auto m = Z3_maxsat(ctx, solver, A);

//            if (LLVM_LIKELY(m != 0)) {

//                const auto model = Z3_solver_get_model(ctx, solver);
//                Z3_model_inc_ref(ctx, model);
//                for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {

//                    Z3_ast const stridesPerSegmentVar = VarList[kernel];
//                    Z3_ast value;
//                    if (LLVM_UNLIKELY(Z3_model_eval(ctx, model, stridesPerSegmentVar, Z3_L_TRUE, &value) != Z3_L_TRUE)) {
//                        report_fatal_error("Unexpected Z3 error when attempting to obtain value from model!");
//                    }

//                    __int64 num, denom;
//                    if (LLVM_UNLIKELY(Z3_get_numeral_rational_int64(ctx, value, &num, &denom) != Z3_L_TRUE)) {
//                        report_fatal_error("Unexpected Z3 error when attempting to convert model value to number!");
//                    }
//                    assert (num > 0);
//                    bounds[kernel][bound] = Rational{num, denom};
//                }
//                Z3_model_dec_ref(ctx, model);

//            }

//            Z3_solver_pop(ctx, solver, 1);
//        }


//        // If we find a solution but discover that min > max for any bound, we're going to have to
//        // recompute the solution. This unfortunately does discard the learned clauses from the prior
//        // lower/upper bound checks but appears that the amount of duplicate work is relatively small.

//        // TODO: is there a way to have "divergent" solvers from the common assumption checks?

//        bool done = true;
//        for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
//            const Bound & bound = bounds[kernel];
//            if (LLVM_UNLIKELY(bound[LowerBound] > bound[UpperBound])) {
//                const auto stridesPerSegmentVar = VarList[kernel];
//                assumptions[LowerBound].push_back(Z3_mk_le(ctx, stridesPerSegmentVar, constant(bound[UpperBound])));
//                assumptions[UpperBound].push_back(Z3_mk_ge(ctx, stridesPerSegmentVar, constant(bound[LowerBound])));
//                done = false;
//            }
//        }

//        if (LLVM_LIKELY(done)) {
//            break;
//        }

//    }

//    Z3_solver_dec_ref(ctx, solver);
//    Z3_del_context(ctx);
//    Z3_reset_memory();

//    const auto & b = bounds[firstKernel];
//    auto g = gcd(b[LowerBound], b[UpperBound]);
//    for (auto kernel = firstKernel + 1; kernel <= lastKernel; ++kernel) {
//        const auto & b = bounds[kernel];
//        g = gcd(g, b[LowerBound]);
//        g = gcd(g, b[UpperBound]);
//    }
//    if (LLVM_UNLIKELY(g > Rational{1})) {
//        for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
//            auto & r = bounds[kernel];
//            r[LowerBound] /= g;
//            r[UpperBound] /= g;
//        }
//    }


//    // Then write them out
//    MinimumNumOfStrides.resize(PipelineOutput + 1);
//    MaximumNumOfStrides.resize(PipelineOutput + 1);

//    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
//        auto & b = bounds[kernel];
//        auto & lb = b[LowerBound];
//        auto & ub = b[UpperBound];
//        if (LLVM_UNLIKELY(lb.denominator() != 1)) {
//            const auto r = Rational{lb.numerator() % lb.denominator(), lb.denominator()};
//            lb -= r;
//            ub += r;
//        }
//        if (LLVM_UNLIKELY(ub.denominator() != 1)) {
//            const auto r = Rational{ub.denominator() - ub.numerator() % ub.denominator(), ub.denominator()};
//            ub += r;
//        }
//        assert(lb.denominator() == 1);
//        assert(ub.denominator() == 1);
//        MinimumNumOfStrides[kernel] = lb;
//        MaximumNumOfStrides[kernel] = ub;
//    }

}


#else

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief identifyHardPartitionConstraints
 ** ------------------------------------------------------------------------------------------------------------- */
PartitionConstraintGraph PipelineAnalysis::identifyHardPartitionConstraints() const {

    PartitionConstraintGraph H(PartitionCount);

    for (auto streamSet = FirstStreamSet; streamSet <= LastStreamSet; ++streamSet) {
        const BufferNode & bn = mBufferGraph[streamSet];
        if (bn.Locality == BufferLocality::GloballyShared) {
            const auto producer = parent(streamSet, mBufferGraph);
            const auto producerPartitionId = KernelPartitionId[producer];
            for (const auto data : make_iterator_range(out_edges(streamSet, mBufferGraph))) {
                const auto consumer = target(data, mBufferGraph);
                const auto consumerPartitionId = KernelPartitionId[consumer];
                assert (producerPartitionId <= consumerPartitionId);
                add_edge(producerPartitionId, consumerPartitionId, H);

            }
        }
    }

    return H;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeParitionDataFlowRates
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::computeDataFlowRates() {

    using Bound = std::array<Rational, 2>;

    const auto cfg = Z3_mk_config();
    Z3_set_param_value(cfg, "model", "true");
    Z3_set_param_value(cfg, "proof", "false");
    const auto ctx = Z3_mk_context(cfg);
    Z3_del_config(cfg);
    const auto solver = Z3_mk_solver(ctx);
    Z3_solver_inc_ref(ctx, solver);

    const auto varType = Z3_mk_real_sort(ctx);

    auto constant = [&](const Rational value) {
        return Z3_mk_real(ctx, value.numerator(), value.denominator());
    };

    const auto ONE = constant(1);

    auto average = [&](const BufferPort & rate) {
        return constant((rate.Minimum + rate.Maximum) * Rational{1, 2});
    };

    auto maximum = [&](const BufferPort & rate) {
        return constant(rate.Maximum);
    };

    auto minimum = [&](const BufferPort & rate) {
        return constant(rate.Minimum);
    };

    auto free_variable = [&]() {
        auto v = Z3_mk_fresh_const(ctx, nullptr, varType);
        auto c1 = Z3_mk_ge(ctx, v, ONE);
        Z3_solver_assert(ctx, solver, c1);
        return v;
    };

    auto lower_bounded_variable = [&](const BufferPort & rate) {
        auto v = Z3_mk_fresh_const(ctx, nullptr, varType);
        auto c1 = Z3_mk_ge(ctx, v, minimum(rate));
        Z3_solver_assert(ctx, solver, c1);
        return v;
    };

    auto bounded_variable = [&](const BufferPort & rate) {
        assert (rate.Minimum < rate.Maximum);
        auto v = Z3_mk_fresh_const(ctx, nullptr, varType);
        auto c1 = Z3_mk_ge(ctx, v, minimum(rate));
        Z3_solver_assert(ctx, solver, c1);
        auto c2 = Z3_mk_le(ctx, v, maximum(rate));
        Z3_solver_assert(ctx, solver, c2);
        return v;
    };

    auto multiply =[&](Z3_ast X, Z3_ast Y) {
        Z3_ast args[2] = { X, Y };
        return Z3_mk_mul(ctx, 2, args);
    };

//    #warning This does not calculate the initial lookahead/delay implications.

    // Instead of trying to reason about lookahead/delay, we could have internal buffers
    // "reset" at the start of each partition.

    const auto firstKernel = out_degree(PipelineInput, mBufferGraph) == 0 ? FirstKernel : PipelineInput;
    const auto lastKernel = in_degree(PipelineOutput, mBufferGraph) == 0 ? LastKernel : PipelineOutput;

    std::vector<Z3_ast> VarList(LastStreamSet + 1);

    enum {
        LowerBound = 0,
        UpperBound = 1,
        Common = 2
    };

    std::array<std::vector<Z3_ast>, 3> assumptions;

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
        const auto var = free_variable();



        const auto expected = constant(ExpectedNumOfStrides[kernel]);
        assumptions[LowerBound].push_back(Z3_mk_le(ctx, var, expected));


        assumptions[UpperBound].push_back(Z3_mk_ge(ctx, var, expected));
        const auto expected_x_2 = constant(ExpectedNumOfStrides[kernel] * Rational{2});
        assumptions[UpperBound].push_back(Z3_mk_le(ctx, var, expected_x_2));
        VarList[kernel] = var;
    }

    const auto H = identifyHardPartitionConstraints();

    auto crosses_partition_boundary =[&H](const unsigned p1, const unsigned p2) {
        return edge(p1, p2, H).second;
    };

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {

        const auto stridesPerSegmentVar = VarList[kernel];
        const auto partitionId = KernelPartitionId[kernel];

        unsigned numOfGreedyRates = 0;

        for (const auto input : make_iterator_range(in_edges(kernel, mBufferGraph))) {
            const auto buffer = source(input, mBufferGraph);

            const BufferPort & inputRate = mBufferGraph[input];
            const Binding & binding = inputRate.Binding;
            const ProcessingRate & rate = binding.getRate();

            const auto producedRate = VarList[buffer]; assert (producedRate);

            if (LLVM_LIKELY(rate.isFixed())) {

                const auto fixedRateVal = minimum(inputRate);
                const auto consumedRate = multiply(stridesPerSegmentVar, fixedRateVal);
                // To consume any data, we want to guarantee at least one full stride of work.
                const auto atLeastOneStride = Z3_mk_ge(ctx, consumedRate, fixedRateVal);
                assumptions[Common].push_back(atLeastOneStride);
                // A partition-local stream must *always* consume everything but streams that
                // cross a partition simply need to produce enough to satisfy its consumer(s)
                const auto consumeEverything = Z3_mk_eq(ctx, producedRate, consumedRate);
                const auto producer = parent(buffer, mBufferGraph);

                if (crosses_partition_boundary(KernelPartitionId[producer], partitionId)) {
                    const auto produceEnough = Z3_mk_ge(ctx, producedRate, consumedRate);
                    Z3_solver_assert(ctx, solver, produceEnough);
                    // Assume we can consume all of the data
                    assumptions[Common].push_back(consumeEverything);
                } else {
                    Z3_solver_assert(ctx, solver, consumeEverything);
                }
            } else if (LLVM_UNLIKELY(rate.isGreedy())) {
                ++numOfGreedyRates;
            } else {

                const auto inputRateVar = bounded_variable(inputRate);
                const auto consumedRate = multiply(stridesPerSegmentVar, inputRateVar);
                const auto produceEnough = Z3_mk_ge(ctx, producedRate, consumedRate);
                Z3_solver_assert(ctx, solver, produceEnough);

                // To consume any data, we want to guarantee at least one full stride of work;
                // however we can only state that only Bounded rates will block on less than
                // this amount.
                const auto atLeastOneStride = Z3_mk_ge(ctx, consumedRate, maximum(inputRate));
                if (rate.isBounded()) {
                    Z3_solver_assert(ctx, solver, atLeastOneStride);
                } else {
                    assumptions[Common].push_back(atLeastOneStride);
                }

                // Since we are trying to determine the lower and upper bound on the number
                // of strides per segment, to determine the lower bound we assume that a
                // kernel consumes the maximum amount of data but produces the minimum.
                // The upper bound is similar except we consume the minimum and produce
                // the maximum.

                const auto avgInputRate = average(inputRate);

                assumptions[LowerBound].push_back(Z3_mk_ge(ctx, inputRateVar, avgInputRate));
                assumptions[LowerBound].push_back(Z3_mk_eq(ctx, inputRateVar, maximum(inputRate)));

                assumptions[UpperBound].push_back(Z3_mk_le(ctx, inputRateVar, avgInputRate));
                assumptions[UpperBound].push_back(Z3_mk_eq(ctx, consumedRate, minimum(inputRate)));
            }
        }

        // Any kernel with all greedy rates must exhaust its input in a single iteration.
        if (LLVM_UNLIKELY(numOfGreedyRates == in_degree(kernel, mBufferGraph))) {
            const auto constraint = Z3_mk_eq(ctx, stridesPerSegmentVar, ONE);
            Z3_solver_assert(ctx, solver, constraint);
        }

        for (const auto output : make_iterator_range(out_edges(kernel, mBufferGraph))) {
            const BufferPort & outputRate = mBufferGraph[output];
            const Binding & binding = outputRate.Binding;
            const ProcessingRate & rate = binding.getRate();
            const auto buffer = target(output, mBufferGraph);
            if (LLVM_LIKELY(rate.isFixed())) {
                const auto outputRateVar = maximum(outputRate);
                const auto producedRate = multiply(stridesPerSegmentVar, outputRateVar);
                VarList[buffer] = producedRate;
            } else if (LLVM_UNLIKELY(rate.isUnknown())) {
                // TODO: is there a better way to handle unknown outputs? This
                // free variable represents the ideal amount of data to transfer
                // to subsequent kernels but that isn't very meaningful here.
                VarList[buffer] = lower_bounded_variable(outputRate);
            } else {
                const auto outputRateVar = bounded_variable(outputRate);
                const auto producedRate = multiply(stridesPerSegmentVar, outputRateVar);
                VarList[buffer] = producedRate;
                // Like above, when calculating the lower bound of the number of kernel strides,
                // we assume we've produced the minimum amount of data and for the upper bound,
                // the maximum.

                const auto avgOutputRate = average(outputRate);
                assumptions[LowerBound].push_back(Z3_mk_le(ctx, outputRateVar, avgOutputRate));
                assumptions[LowerBound].push_back(Z3_mk_eq(ctx, producedRate, minimum(outputRate)));

                assumptions[UpperBound].push_back(Z3_mk_ge(ctx, outputRateVar, avgOutputRate));
                assumptions[UpperBound].push_back(Z3_mk_eq(ctx, outputRateVar, maximum(outputRate)));
            }
        }
    }

    const auto E = identifyLengthEqualityAssertions(mBufferGraph);
    for (const auto e : make_iterator_range(edges(E))) {
        const auto A = VarList[source(e, mBufferGraph)];
        const auto B = VarList[target(e, mBufferGraph)];

        Z3_solver_assert(ctx, solver, Z3_mk_eq(ctx, A, B));
    }

    if (LLVM_UNLIKELY(Z3_solver_check(ctx, solver) == Z3_L_FALSE)) {
        report_fatal_error("Z3 failed to find a solution to the core dataflow graph");
    }
    Z3_solver_push(ctx, solver);

    const auto m = Z3_maxsat(ctx, solver, assumptions[Common]);

    if (LLVM_UNLIKELY(m == 0)) {
        Z3_solver_pop(ctx, solver, 1);
        Z3_solver_check(ctx, solver);
    }

    // Use the initial solution as a default for both the upper and lower bounds incase
    // there are no upper or lower bound assumptions.

    std::vector<Bound> bounds(PipelineOutput + 1);

    const auto model = Z3_solver_get_model(ctx, solver);
    Z3_model_inc_ref(ctx, model);

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {

        Z3_ast const stridesPerSegmentVar = VarList[kernel];
        Z3_ast value;
        if (LLVM_UNLIKELY(Z3_model_eval(ctx, model, stridesPerSegmentVar, Z3_L_TRUE, &value) != Z3_L_TRUE)) {
            report_fatal_error("Unexpected Z3 error when attempting to obtain value from model!");
        }

        __int64 num, denom;
        if (LLVM_UNLIKELY(Z3_get_numeral_rational_int64(ctx, value, &num, &denom) != Z3_L_TRUE)) {
            report_fatal_error("Unexpected Z3 error when attempting to convert model value to number!");
        }
        assert (num > 0);

        const auto r = Rational{num, denom};
        for (unsigned bound = LowerBound; bound <= UpperBound; ++bound) {
            bounds[kernel][bound] = r;
        }
    }

    Z3_model_dec_ref(ctx, model);

    // Now check whether any of the upper/lower bound assumptions alter the results

    for (;;) {

        for (unsigned bound = LowerBound; bound <= UpperBound; ++bound) {

            const auto & A = assumptions[bound];

            if (LLVM_UNLIKELY(A.empty())) continue;

            Z3_solver_push(ctx, solver);

            const auto m = Z3_maxsat(ctx, solver, A);

            if (LLVM_LIKELY(m != 0)) {

                const auto model = Z3_solver_get_model(ctx, solver);
                Z3_model_inc_ref(ctx, model);
                for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {

                    Z3_ast const stridesPerSegmentVar = VarList[kernel];
                    Z3_ast value;
                    if (LLVM_UNLIKELY(Z3_model_eval(ctx, model, stridesPerSegmentVar, Z3_L_TRUE, &value) != Z3_L_TRUE)) {
                        report_fatal_error("Unexpected Z3 error when attempting to obtain value from model!");
                    }

                    __int64 num, denom;
                    if (LLVM_UNLIKELY(Z3_get_numeral_rational_int64(ctx, value, &num, &denom) != Z3_L_TRUE)) {
                        report_fatal_error("Unexpected Z3 error when attempting to convert model value to number!");
                    }
                    assert (num > 0);
                    bounds[kernel][bound] = Rational{num, denom};
                }
                Z3_model_dec_ref(ctx, model);

            }

            Z3_solver_pop(ctx, solver, 1);
        }


        // If we find a solution but discover that min > max for any bound, we're going to have to
        // recompute the solution. This unfortunately does discard the learned clauses from the prior
        // lower/upper bound checks but appears that the amount of duplicate work is relatively small.

        // TODO: is there a way to have "divergent" solvers from the common assumption checks?

        bool done = true;
        for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
            const Bound & bound = bounds[kernel];
            if (LLVM_UNLIKELY(bound[LowerBound] > bound[UpperBound])) {
                const auto stridesPerSegmentVar = VarList[kernel];
                assumptions[LowerBound].push_back(Z3_mk_le(ctx, stridesPerSegmentVar, constant(bound[UpperBound])));
                assumptions[UpperBound].push_back(Z3_mk_ge(ctx, stridesPerSegmentVar, constant(bound[LowerBound])));
                done = false;
            }
        }

        if (LLVM_LIKELY(done)) {
            break;
        }

    }

    Z3_solver_dec_ref(ctx, solver);
    Z3_del_context(ctx);
    Z3_reset_memory();

    const auto & b = bounds[firstKernel];
    auto g = gcd(b[LowerBound], b[UpperBound]);
    for (auto kernel = firstKernel + 1; kernel <= lastKernel; ++kernel) {
        const auto & b = bounds[kernel];
        g = gcd(g, b[LowerBound]);
        g = gcd(g, b[UpperBound]);
    }
    if (LLVM_UNLIKELY(g > Rational{1})) {
        for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
            auto & r = bounds[kernel];
            r[LowerBound] /= g;
            r[UpperBound] /= g;
        }
    }


    // Then write them out
    MinimumNumOfStrides.resize(PipelineOutput + 1);
    MaximumNumOfStrides.resize(PipelineOutput + 1);

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
        auto & b = bounds[kernel];
        auto & lb = b[LowerBound];
        auto & ub = b[UpperBound];
        if (LLVM_UNLIKELY(lb.denominator() != 1)) {
            const auto r = Rational{lb.numerator() % lb.denominator(), lb.denominator()};
            lb -= r;
            ub += r;
        }
        if (LLVM_UNLIKELY(ub.denominator() != 1)) {
            const auto r = Rational{ub.denominator() - ub.numerator() % ub.denominator(), ub.denominator()};
            ub += r;
        }
        assert(lb.denominator() == 1);
        assert(ub.denominator() == 1);
        MinimumNumOfStrides[kernel] = lb;
        MaximumNumOfStrides[kernel] = ub;
    }

}

#endif

#endif

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
