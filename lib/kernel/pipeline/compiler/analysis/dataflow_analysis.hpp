#ifndef DATAFLOW_ANALYSIS_HPP
#define DATAFLOW_ANALYSIS_HPP

#include "../pipeline_compiler.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeParitionDataFlowRates
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::computeDataFlowRates(BufferGraph & G) {

    const auto lb = calculateNumOfStridesPerSegment(DataflowCalculationType::LowerBound, G);
    const auto ub = calculateNumOfStridesPerSegment(DataflowCalculationType::UpperBound, G);

    unsigned denom_lcm = 1;
    for (auto kernel = PipelineInput; kernel <= PipelineOutput; ++kernel) {
        const auto a = lb[kernel];
        if (a.denominator() != 1) {
            denom_lcm = boost::lcm(denom_lcm, a.denominator());
        }
        const auto b = ub[kernel];
        if (b.denominator() != 1) {
            denom_lcm = boost::lcm(denom_lcm, b.denominator());
        }
    }

    MinimumNumOfStrides.resize(PipelineOutput + 1);
    MaximumNumOfStrides.resize(PipelineOutput + 1);

    for (auto kernel = PipelineInput; kernel <= PipelineOutput; ++kernel) {
        const auto a = lb[kernel] * denom_lcm;
        assert (a.denominator() == 1);
        MinimumNumOfStrides[kernel] = a.numerator();
        const auto b = ub[kernel] * denom_lcm;
        assert (b.denominator() == 1);
        MaximumNumOfStrides[kernel] = b.numerator();
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateExpectedNumOfStridesPerSegment
 ** ------------------------------------------------------------------------------------------------------------- */
std::vector<KernelCompiler::Rational>
PipelineCompiler::calculateNumOfStridesPerSegment(const DataflowCalculationType type,
                                                  const BufferGraph & G) const {
    const auto cfg = Z3_mk_config();
    Z3_set_param_value(cfg, "MODEL", "true");
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
        return constant(std::max(rate.Minimum, Rational{1}));
    };

    auto free_variable = [&]() {
        auto v = Z3_mk_fresh_const(ctx, nullptr, varType);
        auto c1 = Z3_mk_ge(ctx, v, ONE);
        Z3_solver_assert(ctx, solver, c1);
        return v;
    };

    auto lower_bounded_variable = [&](const BufferRateData & rate) {
        auto v = Z3_mk_fresh_const(ctx, nullptr, varType);
        auto c1 = Z3_mk_ge(ctx, v, constant(rate.Minimum));
        Z3_solver_assert(ctx, solver, c1);
        return v;
    };

    auto bounded_variable = [&](const BufferRateData & rate) {
        auto v = Z3_mk_fresh_const(ctx, nullptr, varType);
        auto c1 = Z3_mk_ge(ctx, v, constant(rate.Minimum));
        Z3_solver_assert(ctx, solver, c1);
        auto c2 = Z3_mk_le(ctx, v, constant(rate.Maximum));
        Z3_solver_assert(ctx, solver, c2);
        return v;
    };

    auto multiply =[&](Z3_ast X, Z3_ast Y) {
        Z3_ast args[2] = { X, Y };
        return Z3_mk_mul(ctx, 2, args);
    };

    const auto firstKernel = out_degree(PipelineInput, G) == 0 ? FirstKernel : PipelineInput;
    const auto lastKernel = in_degree(PipelineOutput, G) == 0 ? LastKernel : PipelineOutput;

    std::vector<Z3_ast> VarList(LastStreamSet + 1);

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
        // Source kernels always perform exactly one iteration
        if (in_degree(kernel, G) == 0) {
            VarList[kernel] = ONE;
        } else {
            VarList[kernel] = free_variable();
        }
    }
#if 0
    const auto NO_ROOT = std::numeric_limits<unsigned>::max();
    std::vector<unsigned> partitionRoots(PartitionCount, NO_ROOT);
#endif
    std::vector<Z3_ast> assumptions;

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {

        const auto stridesPerSegmentVar = VarList[kernel];
        const auto partitionId = KernelPartitionId[kernel];

        unsigned numOfGreedyRates = 0;
     //   bool hasNonSynchronousInput = false;

        for (const auto input : make_iterator_range(in_edges(kernel, G))) {
            const auto buffer = source(input, G);

            const BufferRateData & inputRate = G[input];
            const Binding & binding = inputRate.Binding;
            const ProcessingRate & rate = binding.getRate();

            const auto producedRate = VarList[buffer]; assert (producedRate);

            if (LLVM_LIKELY(rate.isFixed())) {

                const auto fixedRateVal = maximum(inputRate);
                const auto consumedRate = multiply(stridesPerSegmentVar, fixedRateVal);
                // To consume any data, we must guarantee at least one full stride of work
                const auto atLeastOneStride = Z3_mk_ge(ctx, consumedRate, fixedRateVal);
                Z3_solver_assert(ctx, solver, atLeastOneStride);
                // A partition-local stream must always consume everything
                Z3_ast constraint = nullptr;
                const auto producer = parent(buffer, G);
                if (KernelPartitionId[producer] == partitionId) {
                    constraint = Z3_mk_eq(ctx, producedRate, consumedRate);
                } else {
                    constraint = Z3_mk_ge(ctx, producedRate, consumedRate);
                    // Assume we can consume all of the data
                    assumptions.push_back(Z3_mk_eq(ctx, producedRate, consumedRate));
                    // hasNonSynchronousInput = true;
                }
                Z3_solver_assert(ctx, solver, constraint);

            } else if (LLVM_UNLIKELY(rate.isGreedy())) {
                ++numOfGreedyRates;
            } else if (LLVM_UNLIKELY(rate.isUnknown())) {

                SmallVector<char, 256> tmp;
                raw_svector_ostream msg(tmp);
                msg << getKernel(kernel)->getName() << "."
                    << binding.getName()
                    << " may not have an unknown input rate.";
                report_fatal_error(msg.str());

            } else {

                const auto inputRateVar = bounded_variable(inputRate);
                const auto consumedRate = multiply(stridesPerSegmentVar, inputRateVar);
                const auto constraint = Z3_mk_ge(ctx, producedRate, consumedRate);
                Z3_solver_assert(ctx, solver, constraint);
                // To consume any data, we must guarantee at least one full stride of work
                const auto atLeastOneStride = Z3_mk_ge(ctx, consumedRate, maximum(inputRate));
                Z3_solver_assert(ctx, solver, atLeastOneStride);
                if (type == DataflowCalculationType::LowerBound) {
                    assumptions.push_back(Z3_mk_ge(ctx, inputRateVar, average(inputRate)));
                    assumptions.push_back(Z3_mk_eq(ctx, inputRateVar, maximum(inputRate)));
                } else {
                    assumptions.push_back(Z3_mk_le(ctx, inputRateVar, average(inputRate)));
                    assumptions.push_back(Z3_mk_eq(ctx, consumedRate, minimum(inputRate)));
                }
                // hasNonSynchronousInput = true;
            }
        }
#if 0
        // To reduce the number of soft constraints we add to the formula later when
        // trying to the maximum number of strides per kernel, we keep just one root
        // per partition. Because all kernels within the same partition must be
        // synchronous, all # of strides within a partition will increase together.
        if (hasNonSynchronousInput) {
            const auto id = KernelPartitionId[kernel];
            if (partitionRoots[id] == NO_ROOT) {
                partitionRoots[id] = kernel;
            }
        }
#endif

        // Any kernel with all greedy rates must exhaust its input in a single iteration.
        if (LLVM_UNLIKELY(numOfGreedyRates == in_degree(kernel, G))) {
            const auto constraint = Z3_mk_eq(ctx, stridesPerSegmentVar, ONE);
            Z3_solver_assert(ctx, solver, constraint);
        }

        for (const auto output : make_iterator_range(out_edges(kernel, G))) {
            const BufferRateData & outputRate = G[output];
            const Binding & binding = outputRate.Binding;
            const ProcessingRate & rate = binding.getRate();
            const auto buffer = target(output, G);
            if (LLVM_UNLIKELY(rate.isUnknown())) {
                // TODO: is there  a better way to handle unknown outputs? This
                // free variable represents the ideal amount of data to transfer
                // to subsequent kernels but that cannot be determined a priori;
                // thus this variable isn't very meaningful.
                VarList[buffer] = lower_bounded_variable(outputRate);
            } else {
                Z3_ast outputRateVar;
                if (LLVM_LIKELY(rate.isFixed())) {
                    outputRateVar = maximum(outputRate);
                } else {
                    outputRateVar = bounded_variable(outputRate);
                }
                const auto producedRate = multiply(stridesPerSegmentVar, outputRateVar);
                VarList[buffer] = producedRate;
                if (type == DataflowCalculationType::LowerBound) {
                    assumptions.push_back(Z3_mk_le(ctx, outputRateVar, average(outputRate)));
                    assumptions.push_back(Z3_mk_eq(ctx, producedRate, minimum(outputRate)));
                } else {
                    assumptions.push_back(Z3_mk_ge(ctx, outputRateVar, average(outputRate)));
                    assumptions.push_back(Z3_mk_eq(ctx, outputRateVar, maximum(outputRate)));
                }
            }
        }
    }

    std::vector<Rational> bounds(PipelineOutput + 1);

    auto update_model = [&]() {
        const auto model = Z3_solver_get_model(ctx, solver);
        Z3_model_inc_ref(ctx, model);
        for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
            Z3_ast const kernelVar = VarList[kernel];
            Z3_ast value;
            if (LLVM_UNLIKELY(Z3_model_eval(ctx, model, kernelVar, Z3_L_TRUE, &value) != Z3_L_TRUE)) {
                report_fatal_error("Unexpected Z3 error when attempting to obtain value from model!");
            }
            __int64 num, denom;
            if (LLVM_UNLIKELY(Z3_get_numeral_rational_int64(ctx, value, &num, &denom) != Z3_L_TRUE)) {
                report_fatal_error("Unexpected Z3 error when attempting to convert model value to number!");
            }
            assert (num > 0);
            bounds[kernel] = Rational{num, denom};
        }
        Z3_model_dec_ref(ctx, model);
    };

    if (LLVM_UNLIKELY(Z3_solver_check(ctx, solver) != Z3_L_TRUE)) {
        report_fatal_error("Unexpected Z3 error: unsatisfiable initial dataflow graph");
    }

//    Z3_solver_push(ctx, solver);
    maxsat(ctx, solver, assumptions);

//    errs() << "maxsat=" << m << " of " << assumptions.size() << "\n\n";

//    if (LLVM_UNLIKELY(m < 1)) {
//        Z3_solver_pop(ctx, solver, 1);
//    }
    update_model();

#if 0

    // Filter any identifier that is not a root of some partition
    partitionRoots.erase(
        std::remove_if(partitionRoots.begin(), partitionRoots.end(),
           [](unsigned val) { return val == NO_ROOT; }),
        partitionRoots.end());


//        Z3_params params = Z3_mk_params(ctx);
//        Z3_params_inc_ref(ctx, params);
//        Z3_symbol timeout = Z3_mk_string_symbol(ctx, "timeout");
//        Z3_params_set_uint(ctx, params, timeout, 100);
//        Z3_solver_set_params(ctx, solver, params);
//        Z3_params_dec_ref(ctx, params);

    Rational k;
    unsigned init_val = 0;

    if (type == DataflowCalculationType::LowerBound) {
        k = Rational{1, 2};
        init_val = std::numeric_limits<unsigned>::max();
    } else {
        k = Rational{2, 1};
        init_val = std::numeric_limits<unsigned>::min();
    }

    std::vector<Rational> prior(bounds.size(), init_val);

    unsigned rounds = 0;

    for (;;) {

        assumptions.clear();

        errs() << "------\n";

        if (type == DataflowCalculationType::LowerBound) {

            for (auto kernel : partitionRoots) {
                assert (kernel != NO_ROOT);
                const auto a = bounds[kernel];
                const auto t = prior[kernel];
                if (a < t) {
                    const auto r = a * k;

                    errs() << "  K" << kernel << " <= " << r.numerator() << "/" << r.denominator() << "\n";

                    const auto avgStridesPerSegmentVar = VarList[kernel];
                    assert (a > 0);
                    const auto constraint = Z3_mk_le(ctx, avgStridesPerSegmentVar, constant(r));
                    assumptions.push_back(constraint);
                }
            }

        } else {

            for (auto kernel : partitionRoots) {
                assert (kernel != NO_ROOT);
                const auto a = bounds[kernel];
                const auto t = prior[kernel];
                if (a > t) {
                    const auto r = a * k;

                    errs() << "  K" << kernel << " >= " << r.numerator() << "/" << r.denominator() << "\n";

                    const auto avgStridesPerSegmentVar = VarList[kernel];
                    assert (a > 0);
                    const auto constraint = Z3_mk_ge(ctx, avgStridesPerSegmentVar, constant(r));
                    assumptions.push_back(constraint);
                }
            }
        }

        Z3_solver_push(ctx, solver);
        const auto m = maxsat(ctx, solver, assumptions);
        if (m < 1) {
            if (++rounds == 4) {
                break;
            }
            k = Rational{k.numerator() + 1, k.denominator() + 1};
        } else {
            prior.swap(bounds);
            update_model();
        }
        Z3_solver_pop(ctx, solver, 1);
    }

#endif

    Z3_solver_dec_ref(ctx, solver);
    Z3_del_context(ctx);

    // Z3_finalize_memory();

    return bounds;
}

} // end of kernel namespace

#endif // DATAFLOW_ANALYSIS_HPP
