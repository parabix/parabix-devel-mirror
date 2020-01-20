#ifndef DATAFLOW_ANALYSIS_HPP
#define DATAFLOW_ANALYSIS_HPP

#include "../pipeline_compiler.hpp"
#include <util/maxsat.hpp>
#include <random>

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeParitionDataFlowRates
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::computeDataFlowRates(BufferGraph & G) {

    ExpectedNumOfStrides.resize(PipelineOutput + 1);

    printBufferGraph(G, errs());

    std::vector<Rational> bounds(PipelineOutput + 1);
    // calculateExpectedNumOfStridesPerSegment(G, DataflowCalculationType::Expected, bounds);
    calculateExpectedNumOfStridesPerSegment(G, DataflowCalculationType::UpperBound, bounds);

    const auto firstKernel = out_degree(PipelineInput, G) == 0 ? FirstKernel : PipelineInput;
    const auto lastKernel = in_degree(PipelineOutput, G) == 0 ? LastKernel : PipelineOutput;

    unsigned denom_lcm = 1;
    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
        const auto a = bounds[kernel];
        if (a.denominator() != 1) {
            denom_lcm = boost::lcm(denom_lcm, a.denominator());
        }
    }

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
        const auto a = bounds[kernel] * denom_lcm;
        assert (a.denominator() == 1);
        ExpectedNumOfStrides[kernel] = a.numerator();
    }

    estimateDataFlowBounds(G, ExpectedNumOfStrides);

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateExpectedNumOfStridesPerSegment
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::calculateExpectedNumOfStridesPerSegment(const BufferGraph & G,
                                                               const DataflowCalculationType type,
                                                               std::vector<Rational> & bounds) const {

    Z3_config cfg = Z3_mk_config();
    Z3_set_param_value(cfg, "MODEL", "true");
    Z3_set_param_value(cfg, "proof", "false");
    auto ctx = Z3_mk_context(cfg);
    Z3_del_config(cfg);
    auto solver = Z3_mk_solver(ctx);
    Z3_solver_inc_ref(ctx, solver);


    Z3_sort varType = Z3_mk_real_sort(ctx);;
//    if (type == DataflowCalculationType::Expected) {
//        varType = Z3_mk_int_sort(ctx);
//    } else {
//        varType = Z3_mk_real_sort(ctx);
//    }

    auto constant = [&](const Rational value) {
//        if (type == DataflowCalculationType::Expected) {
//            if (LLVM_LIKELY(value.denominator() == 1)) {
//                return Z3_mk_int(ctx, value.numerator(), varType);
//            }
//        }
        return Z3_mk_real(ctx, value.numerator(), value.denominator());
    };

    auto average = [&](const BufferRateData & rate) {
        return constant((rate.Minimum + rate.Maximum) * Rational{1, 2});
    };

    const auto ONE = constant(1);

    auto free_variable = [&](Z3_ast lb) {
        auto v = Z3_mk_fresh_const(ctx, nullptr, varType);
        auto c1 = Z3_mk_ge(ctx, v, lb);
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
//            Z3_ast lb;
//            if (type == DataflowCalculationType::Expected) {
//                lb = ONE;
//            } else {
//                lb = constant(bounds[kernel]);
//            }
            VarList[kernel] = free_variable(ONE);
        }
    }

    std::vector<unsigned> partitionRoots;
    BitVector P(PartitionCount + 1);

    std::vector<Z3_ast> assumptions;

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {

        const auto avgStridesPerSegmentVar = VarList[kernel];

        unsigned numOfGreedyRates = 0;
        bool hasNonSynchronousInput = false;

        for (const auto input : make_iterator_range(in_edges(kernel, G))) {
            const auto buffer = source(input, G);

            const BufferRateData & inputRate = G[input];
            const Binding & binding = inputRate.Binding;
            const ProcessingRate & rate = binding.getRate();

            const auto producedRate = VarList[buffer]; assert (producedRate);

            if (LLVM_LIKELY(rate.isFixed())) {

                auto consumedRate = constant(inputRate.Maximum);
                consumedRate = multiply(avgStridesPerSegmentVar, consumedRate);
                Z3_ast constraint = nullptr;
                // A partition-local stream must always consume everything
                const auto producer = parent(buffer, G);
                if (KernelPartitionId[producer] == KernelPartitionId[kernel]) {
                    constraint = Z3_mk_eq(ctx, producedRate, consumedRate);
                } else {
                    constraint = Z3_mk_ge(ctx, producedRate, consumedRate);
                    assumptions.push_back(Z3_mk_eq(ctx, producedRate, consumedRate));
                    hasNonSynchronousInput = true;
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
                Z3_ast r;
                if (type == DataflowCalculationType::Expected) {
                    r = average(inputRate);
                } else {
                    r = bounded_variable(inputRate);
                }
                const auto consumedRate = multiply(avgStridesPerSegmentVar, r);
                const auto constraint = Z3_mk_ge(ctx, producedRate, consumedRate);
                assumptions.push_back(Z3_mk_eq(ctx, producedRate, consumedRate));
                Z3_solver_assert(ctx, solver, constraint);
                hasNonSynchronousInput = true;
            }
        }

        if (hasNonSynchronousInput) {
            // Each partition is synchronous so we only need to consider single root of each
            const auto partId = KernelPartitionId[kernel];
            if (!P.test(partId)) {
                partitionRoots.push_back(kernel);
                P.set(partId);
            }
        }

        // Any kernel with all greedy rates must exhaust its input in a single iteration.
        if (LLVM_UNLIKELY(numOfGreedyRates == in_degree(kernel, G))) {
            const auto constraint = Z3_mk_eq(ctx, avgStridesPerSegmentVar, ONE);
            Z3_solver_assert(ctx, solver, constraint);
        }

        for (const auto output : make_iterator_range(out_edges(kernel, G))) {
            const BufferRateData & outputRate = G[output];
            const auto buffer = target(output, G);
            if (LLVM_UNLIKELY(outputRate.Maximum.numerator() == 0)) {
                // TODO: is there  a better way to handle unknown outputs? This
                // free variable represents the ideal amount of data to transfer
                // to subsequent kernels but that cannot be determined a priori;
                // thus this variable isn't very meaningful.
                VarList[buffer] = lower_bounded_variable(outputRate);
            } else {
                Z3_ast r;
                if (type == DataflowCalculationType::Expected) {
                    r = average(outputRate);
                } else {
                    r = bounded_variable(outputRate);
                }
                const auto producedRate = multiply(avgStridesPerSegmentVar, r);
                VarList[buffer] = producedRate;
            }
        }
    }


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

    Z3_solver_push(ctx, solver);
    const auto m = maxsat(ctx, solver, assumptions);
    if (m < 1) {
        Z3_solver_pop(ctx, solver, 1);
    }
    update_model();

    if (type == DataflowCalculationType::UpperBound) {
        assumptions.clear();
        assumptions.reserve(partitionRoots.size());
        std::vector<Rational> prior(bounds.size(), 0);

//        Z3_params params = Z3_mk_params(ctx);
//        Z3_params_inc_ref(ctx, params);
//        Z3_symbol timeout = Z3_mk_string_symbol(ctx, "timeout");
//        Z3_params_set_uint(ctx, params, timeout, 100);
//        Z3_solver_set_params(ctx, solver, params);
//        Z3_params_dec_ref(ctx, params);

        for (;;) {

            assumptions.clear();
            for (auto kernel : partitionRoots) {
                const auto a = bounds[kernel];
                const auto t = prior[kernel];
                if (a > t) {
                    const auto r = a * Rational{2,1};
                    const auto avgStridesPerSegmentVar = VarList[kernel];
                    const auto constraint = Z3_mk_ge(ctx, avgStridesPerSegmentVar, constant(r));
                    assumptions.push_back(constraint);
                }
            }

            Z3_solver_push(ctx, solver);
            const auto m = maxsat(ctx, solver, assumptions);
            if (m < 1) {
                break;
            }
            prior.swap(bounds);
            update_model();
            Z3_solver_pop(ctx, solver, 1);
        }

        Z3_solver_pop(ctx, solver, 1);

        for (;;) {

            assumptions.clear();
            for (auto kernel : partitionRoots) {
                const auto a = bounds[kernel];
                const auto t = prior[kernel];
                if (a > t) {
                    const auto r = a + Rational{1};
                    const auto avgStridesPerSegmentVar = VarList[kernel];
                    const auto constraint = Z3_mk_ge(ctx, avgStridesPerSegmentVar, constant(r));
                    assumptions.push_back(constraint);
                }
            }

            Z3_solver_push(ctx, solver);
            const auto m = maxsat(ctx, solver, assumptions);
            if (m < 1) {
                break;
            }
            prior.swap(bounds);
            update_model();
            Z3_solver_pop(ctx, solver, 1);
        }
    }

    Z3_solver_dec_ref(ctx, solver);
    Z3_del_context(ctx);
}



/** ------------------------------------------------------------------------------------------------------------- *
 * @brief estimateDataFlowBounds
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::estimateDataFlowBounds(BufferGraph & G, const std::vector<unsigned> & expected) const {

    auto roundUpTo = [](const Rational a, const Rational b) {
        // m = mod(a, b)
        Rational n(a.numerator() * b.denominator(), b.numerator() * a.denominator());
        const auto m = a - Rational{floor(n)} * b;
        if (LLVM_UNLIKELY(m.numerator() != 0)) {
            const auto r = (a - m) + b;
            assert (r.denominator() == 1);
            return r.numerator();
        }
        assert (a.denominator() == 1);
        return a.numerator();
    };

    // Since we do not want to create an artifical bottleneck by constructing output buffers that
    // cannot accommodate the full amount of data we could produce given the expected inputs, the
    // next loop will resize them accordingly.

    for (auto streamSet = FirstStreamSet; streamSet <= LastStreamSet; ++streamSet) {

        const auto output = in_edge(streamSet, G);

        const BufferRateData & pData = G[output];

        const auto producer = source(output, G);

        const auto pMin = pData.Minimum * expected[producer];
        const auto pMax = pData.Maximum * expected[producer];

        Rational consumeMin(std::numeric_limits<unsigned>::max());
        Rational consumeMax(std::numeric_limits<unsigned>::min());
        Rational requiredSpace(pMax);

        unsigned lookBehind{0};
        unsigned reflectionSpace{0};
        unsigned lookAheadSpace{0};
        const auto copyBackSpace = ceiling(pData.Maximum - pData.Minimum);

        const Binding & outputBinding = pData.Binding;
        for (const Attribute & attr : outputBinding.getAttributes()) {
            switch (attr.getKind()) {
                case AttrId::LookBehind:
                    lookBehind = std::max(lookBehind, attr.amount());
                    break;
                case AttrId::Delayed:
                    reflectionSpace = std::max(reflectionSpace, attr.amount());
                    break;
                default: break;
            }
        }

        Rational requiredSizeFactor{1};
        if (pData.Maximum == pData.Minimum) {
            requiredSizeFactor = pData.Maximum;
        }

        for (const auto input : make_iterator_range(out_edges(streamSet, G))) {

            const BufferRateData & cData = G[input];

            const auto consumer = target(input, G);

            const auto cMin = cData.Minimum * expected[consumer];
            const auto cMax = cData.Maximum * expected[consumer];

            consumeMin = std::min(consumeMin, cMin);
            consumeMax = std::min(consumeMax, cMax);
            if (cData.Maximum == cData.Minimum) {
                requiredSizeFactor = lcm(requiredSizeFactor, cData.Maximum);
            }

            const Binding & inputBinding = cData.Binding;
            // Get output overflow size
            unsigned lookAhead = 0;
            for (const Attribute & attr : inputBinding.getAttributes()) {
                switch (attr.getKind()) {
                    case AttrId::LookAhead:
                        lookAhead = std::max(lookAhead, attr.amount());
                        break;
                    case AttrId::LookBehind:
                        lookBehind = std::max(lookBehind, attr.amount());
                        break;
                    default: break;
                }
            }

            lookAhead += ceiling(cData.Maximum - cData.Minimum);
            lookAheadSpace = std::max(lookAheadSpace, lookAhead);
        }

        if (consumeMax > consumeMin) {
            requiredSpace += (consumeMax - consumeMin);
        }

        BufferNode & bn = G[streamSet];
        bn.LookBehind = lookBehind;
        bn.LookBehindReflection = reflectionSpace;
        bn.CopyBack = copyBackSpace;
        bn.LookAhead = lookAheadSpace;
        bn.RequiredSpace = roundUpTo(requiredSpace, requiredSizeFactor);

        assert (bn.RequiredSpace > 0);

    }

}

} // end of kernel namespace

#endif // DATAFLOW_ANALYSIS_HPP
