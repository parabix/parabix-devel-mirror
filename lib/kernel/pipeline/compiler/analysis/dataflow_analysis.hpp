#ifndef DATAFLOW_ANALYSIS_HPP
#define DATAFLOW_ANALYSIS_HPP

#include "pipeline_analysis.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief identifyHardPartitionConstraints
 ** ------------------------------------------------------------------------------------------------------------- */
PartitionConstraintGraph PipelineAnalysis::identifyHardPartitionConstraints(BufferGraph & G) const {

    PartitionConstraintGraph H(PartitionCount);

    const auto firstKernel = out_degree(PipelineInput, G) == 0 ? FirstKernel : PipelineInput;
    const auto lastKernel = in_degree(PipelineOutput, G) == 0 ? LastKernel : PipelineOutput;

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {

        const auto partitionId = KernelPartitionId[kernel];
        assert (partitionId < PartitionCount);

        for (const auto input : make_iterator_range(in_edges(kernel, G))) {
            const auto buffer = source(input, G);
            const auto producer = parent(buffer, G);
            const auto producerPartitionId = KernelPartitionId[producer];
            assert (producerPartitionId <= partitionId);
            if (producerPartitionId != partitionId) {
                const BufferPort & inputRate = G[input];
                const Binding & binding = inputRate.Binding;
                const ProcessingRate & rate = binding.getRate();
                if (LLVM_UNLIKELY(!rate.isFixed())) {
                    add_edge(producerPartitionId, partitionId, H);
                }
            }
        }

        for (const auto output : make_iterator_range(out_edges(kernel, G))) {
            const auto buffer = target(output, G);
            for (const auto data : make_iterator_range(out_edges(buffer, G))) {
                const auto consumer = target(data, G);
                const auto consumerPartitionId = KernelPartitionId[consumer];
                assert (consumerPartitionId >= partitionId);
                assert (consumerPartitionId < PartitionCount);
                if (consumerPartitionId != partitionId) {
                    const BufferPort & outputRate = G[output];
                    const Binding & binding = outputRate.Binding;
                    const ProcessingRate & rate = binding.getRate();
                    if (LLVM_UNLIKELY(!rate.isFixed())) {
                        add_edge(partitionId, consumerPartitionId, H);
                    }
                }
            }
        }
    }

    return H;
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

#if 0

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

    auto average = [&](const BufferRateData & rate) {
        return constant((rate.Minimum + rate.Maximum) * Rational{1, 2});
    };

    auto maximum = [&](const BufferRateData & rate) {
        return constant(rate.Maximum);
    };

    auto minimum = [&](const BufferRateData & rate) {
        return constant(rate.Minimum);
    };

    auto free_variable = [&]() {
        auto v = Z3_mk_fresh_const(ctx, nullptr, varType);
        auto c1 = Z3_mk_ge(ctx, v, ONE);
        Z3_solver_assert(ctx, solver, c1);
        return v;
    };

    auto lower_bounded_variable = [&](const BufferRateData & rate) {
        auto v = Z3_mk_fresh_const(ctx, nullptr, varType);
        auto c1 = Z3_mk_ge(ctx, v, minimum(rate));
        Z3_solver_assert(ctx, solver, c1);
        return v;
    };

    auto bounded_variable = [&](const BufferRateData & rate) {
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

    #warning This does not calculate the initial lookahead/delay implications.

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

    // Begin by computing the synchronous dataflow rates of each partition independently and fill in the
    // VarList with the appropriate symbolic rates.

    for (auto start = firstKernel; start <= lastKernel; ) {
        // Determine which kernels are in this partition
        const auto partitionId = KernelPartitionId[start];
        auto end = start + 1U;
        for (; end <= LastKernel; ++end) {
            if (KernelPartitionId[end] != partitionId) {
                break;
            }
        }

        // Source kernels always perform exactly one iteration
        if (LLVM_UNLIKELY(in_degree(start, mBufferGraph) == 0)) {
            VarList[start] = ONE;
        } else {
            VarList[start] = free_variable();
        }

        if (LLVM_LIKELY((start + 1U) < end)) {

            SmallVector<Rational, 32> repetitionFactor(end - start + 1);

            Z3_solver_push(ctx, solver);

            for (auto kernel = start + 1; kernel < end; ++kernel) {
                VarList[kernel] = free_variable();
            }

            for (auto kernel = start; kernel < end; ++kernel) {
                const auto stridesPerSegmentVar = VarList[kernel];
                for (const auto input : make_iterator_range(in_edges(kernel, mBufferGraph))) {
                    const auto streamSet = source(input, mBufferGraph);
                    const auto producer = parent(streamSet, mBufferGraph);
                    // Is the producer of the stream in the current partition?
                    if (producer >= start) {
                        const BufferRateData & inputRate = mBufferGraph[input];
                        const Binding & binding = inputRate.Binding;
                        assert (binding.getRate().isFixed());
                        const auto fixedRateVal = minimum(inputRate);
                        const auto consumedRate = multiply(stridesPerSegmentVar, fixedRateVal);
                        const auto producedRate = VarList[streamSet]; assert (producedRate);
                        const auto consumeEverything = Z3_mk_eq(ctx, producedRate, consumedRate);
                        Z3_solver_assert(ctx, solver, consumeEverything);
                    }
                }

                for (const auto output : make_iterator_range(out_edges(kernel, mBufferGraph))) {
                    const auto streamSet = target(output, mBufferGraph);
                    for (const auto input : make_iterator_range(out_edges(streamSet, mBufferGraph))) {
                        const auto consumer = target(input, mBufferGraph);
                        // Is at least one consumer of the stream in the current partition?
                        if (consumer < end) {
                            const BufferRateData & outputRate = mBufferGraph[output];
                            const Binding & binding = outputRate.Binding;
                            assert (binding.getRate().isFixed());
                            const auto outputRateVar = minimum(outputRate);
                            const auto producedRate = multiply(stridesPerSegmentVar, outputRateVar);
                            VarList[streamSet] = producedRate;
                            break;
                        }
                    }
                }
            }

            if (LLVM_UNLIKELY(Z3_solver_check(ctx, solver) == Z3_L_FALSE)) {
                report_fatal_error("Z3 failed to find a solution to synchronous dataflow graph");
            }

            const auto model = Z3_solver_get_model(ctx, solver);
            Z3_model_inc_ref(ctx, model);
            for (auto kernel = start; kernel < end; ++kernel) {

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

                repetitionFactor[kernel - start] = Rational{num, denom};
            }
            Z3_model_dec_ref(ctx, model);
            Z3_solver_pop(ctx, solver, 1);



            const auto stridesPerSegmentVar = VarList[start];

            for (auto kernel = start + 1; kernel < end; ++kernel) {
                const auto factor = repetitionFactor[kernel - start] / repetitionFactor[0];
                Z3_ast repetitions = stridesPerSegmentVar;
                if (factor.numerator() != 1 || factor.denominator() != 1) {
                    repetitions = multiply(stridesPerSegmentVar, constant(factor));
                }
                VarList[kernel] = repetitions;
            }
        }

        start = end;
    }

    #ifndef NDEBUG
    std::fill_n(&VarList[FirstStreamSet], LastStreamSet - FirstStreamSet, nullptr);
    #endif

    const auto H = identifyHardPartitionConstraints(mBufferGraph);

    auto crossesVariableRatePartitionBoundary =[&H](const unsigned p1, const unsigned p2) {
        return edge(p1, p2, H).second;
    };

    std::array<std::vector<Z3_ast>, 3> assumptions;

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {

        const auto stridesPerSegmentVar = VarList[kernel];
        const auto partitionId = KernelPartitionId[kernel];

        unsigned numOfGreedyRates = 0;

        for (const auto input : make_iterator_range(in_edges(kernel, mBufferGraph))) {
            const auto buffer = source(input, mBufferGraph);

            const BufferRateData & inputRate = mBufferGraph[input];
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

                if (crossesVariableRatePartitionBoundary(KernelPartitionId[producer], partitionId)) {
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
            const BufferRateData & outputRate = mBufferGraph[output];
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
        const auto A = VarList[source(e, E)]; assert (A);
        const auto B = VarList[target(e, E)]; assert (B);

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

    std::vector<Bound> current(PipelineOutput + 1);

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
            current[kernel][bound] = r;
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
                    current[kernel][bound] = Rational{num, denom};
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
            const Bound & bound = current[kernel];
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

    const auto & b = current[firstKernel];
    auto g = gcd(b[LowerBound], b[UpperBound]);
    for (auto kernel = firstKernel + 1; kernel <= lastKernel; ++kernel) {
        const auto & b = current[kernel];
        g = gcd(g, b[LowerBound]);
        g = gcd(g, b[UpperBound]);
    }
    if (LLVM_UNLIKELY(g > Rational{1})) {
        for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
            auto & r = current[kernel];
            r[LowerBound] /= g;
            r[UpperBound] /= g;
        }
    }

    // Then write them out
    MinimumNumOfStrides.resize(PipelineOutput + 1);
    MaximumNumOfStrides.resize(PipelineOutput + 1);

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
        const auto & r = current[kernel];
        MinimumNumOfStrides[kernel] = r[LowerBound];
        MaximumNumOfStrides[kernel] = r[UpperBound];
    }

}

#else

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

    #warning This does not calculate the initial lookahead/delay implications.

    // Instead of trying to reason about lookahead/delay, we could have internal buffers
    // "reset" at the start of each partition.

    const auto firstKernel = out_degree(PipelineInput, mBufferGraph) == 0 ? FirstKernel : PipelineInput;
    const auto lastKernel = in_degree(PipelineOutput, mBufferGraph) == 0 ? LastKernel : PipelineOutput;

    std::vector<Z3_ast> VarList(LastStreamSet + 1);

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
        // Source kernels always perform exactly one iteration
        if (in_degree(kernel, mBufferGraph) == 0) {
            VarList[kernel] = ONE;
        } else {
            VarList[kernel] = free_variable();
        }
    }

    enum {
        LowerBound = 0,
        UpperBound = 1,
        Common = 2
    };

    const auto H = identifyHardPartitionConstraints(mBufferGraph);

    auto crossesVariableRatePartitionBoundary =[&H](const unsigned p1, const unsigned p2) {
        return edge(p1, p2, H).second;
    };

    std::array<std::vector<Z3_ast>, 3> assumptions;

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

                if (crossesVariableRatePartitionBoundary(KernelPartitionId[producer], partitionId)) {
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

   // errs() << "DATAFLOW Z3\n";

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

    std::vector<Bound> current(PipelineOutput + 1);

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
            current[kernel][bound] = r;
        }
    }

    Z3_model_dec_ref(ctx, model);

    // Now check whether any of the upper/lower bound assumptions alter the results

    for (;;) {

        for (unsigned bound = LowerBound; bound <= UpperBound; ++bound) {

            const auto & A = assumptions[bound];

            if (LLVM_UNLIKELY(A.empty())) continue;

           // errs() << "DATAFLOW Z3 " << bound << "\n";

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
                    current[kernel][bound] = Rational{num, denom};
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
            const Bound & bound = current[kernel];
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

    const auto & b = current[firstKernel];
    auto g = gcd(b[LowerBound], b[UpperBound]);
    for (auto kernel = firstKernel + 1; kernel <= lastKernel; ++kernel) {
        const auto & b = current[kernel];
        g = gcd(g, b[LowerBound]);
        g = gcd(g, b[UpperBound]);
    }
    if (LLVM_UNLIKELY(g > Rational{1})) {
        for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
            auto & r = current[kernel];
            r[LowerBound] /= g;
            r[UpperBound] /= g;
        }
    }


    // Then write them out
    MinimumNumOfStrides.resize(PipelineOutput + 1);
    MaximumNumOfStrides.resize(PipelineOutput + 1);

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
        const auto & r = current[kernel];
        MinimumNumOfStrides[kernel] = r[LowerBound];
        MaximumNumOfStrides[kernel] = r[UpperBound];
    }

    #ifndef NDEBUG
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
        if (end > (start + 1)) {
            const auto check = MaximumNumOfStrides[start] / MinimumNumOfStrides[start];
            for (auto kernel = start + 1; kernel < end; ++kernel) {
                const auto check2 = MaximumNumOfStrides[kernel] / MinimumNumOfStrides[kernel];
                if (LLVM_UNLIKELY(check != check2)) {
                    report_fatal_error("Kernel " + std::to_string(kernel) + " non-synchronous dataflow in partition " + std::to_string(partitionId) );
                }
            }
        }
        start = end;
    }
    #endif

}
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
            if (node.NonLocal) {
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
