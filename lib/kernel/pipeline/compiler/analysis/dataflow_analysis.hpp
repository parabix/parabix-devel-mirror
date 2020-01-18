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

    ExpectedNumOfStrides = calculateExpectedNumOfStridesPerSegment(G, StridesPerSegmentCalculationType::Expected);

    estimateDataFlowBounds(G, ExpectedNumOfStrides);

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateExpectedNumOfStridesPerSegment
 ** ------------------------------------------------------------------------------------------------------------- */
std::vector<unsigned> PipelineCompiler::calculateExpectedNumOfStridesPerSegment(const BufferGraph & G, const StridesPerSegmentCalculationType type) const {

    Z3_config cfg = Z3_mk_config();
    Z3_set_param_value(cfg, "MODEL", "true");
    Z3_set_param_value(cfg, "proof", "false");
    auto ctx = Z3_mk_context(cfg);
    Z3_del_config(cfg);
    auto solver = Z3_mk_solver(ctx);
    Z3_solver_inc_ref(ctx, solver);

    const auto intType = Z3_mk_int_sort(ctx);
    const auto realType = Z3_mk_real_sort(ctx);

    auto constant = [&](const Rational value) {
        if (value.denominator() == 1) {
            return Z3_mk_int(ctx, value.numerator(), intType);
        }
        return Z3_mk_real(ctx, value.numerator(), value.denominator());
    };

    auto lower_bounded_variable = [&](Z3_ast lb) {
        auto v = Z3_mk_fresh_const(ctx, nullptr, realType);
        auto c1 = Z3_mk_ge(ctx, v, lb);
        Z3_solver_assert(ctx, solver, c1);
        return v;
    };

    auto bounded_variable = [&](Z3_ast lb, Z3_ast ub) {
        auto v = Z3_mk_fresh_const(ctx, nullptr, realType);
        auto c1 = Z3_mk_ge(ctx, v, lb);
        Z3_solver_assert(ctx, solver, c1);
        auto c2 = Z3_mk_le(ctx, v, ub);
        Z3_solver_assert(ctx, solver, c2);
        return v;
    };

    auto multiply =[&](Z3_ast X, Z3_ast Y) {
        Z3_ast args[2] = { X, Y };
        return Z3_mk_mul(ctx, 2, args);
    };

    // For Fixed/Greedy rates within the same partition:
    //    VAR(PRODUCER(c)) * PRODUCED(c) = VAR(CONSUMER(c)) * CONSUMED(c)

    // For Fixed/Greedy rates in differing partitions:
    //    VAR(PRODUCER(c)) * PRODUCED(c) >= VAR(CONSUMER(c)) * CONSUMED(c)

    // For variable rates regardless of partition:
    //    VAR(PRODUCER(c)) * PRODUCED(c) >= VAR(CONSUMER(c)) * CONSUMED(c)
    //  Where for a given rate of [n,m]:
    //    RATE(c) := (STRIDES) * (n + m) / 2

//    auto calculateInputBound = [&](const BufferRateData & r) {
//        switch (type) {
//            case StridesPerSegmentCalculationType::LowerBound:
//                return r.Maximum;
//            case StridesPerSegmentCalculationType::Expected:
//                return (r.Minimum + r.Maximum) * Rational{1,2};
//            case StridesPerSegmentCalculationType::UpperBound:
//                return r.Minimum;
//        }
//        llvm_unreachable("");
//    };

//    auto calculateOutputBound = [&](const BufferRateData & r) {
//        switch (type) {
//            case StridesPerSegmentCalculationType::LowerBound:
//                return r.Minimum;
//            case StridesPerSegmentCalculationType::Expected:
//                return (r.Minimum + r.Maximum) * Rational{1,2};
//            case StridesPerSegmentCalculationType::UpperBound:
//                return r.Maximum;
//        }
//        llvm_unreachable("");
//    };

    auto average = [](const BufferRateData & r) {
        return std::max((r.Minimum + r.Maximum) * Rational{1,2}, Rational{1});
    };

    const auto firstKernel = out_degree(PipelineInput, G) == 0 ? FirstKernel : PipelineInput;
    const auto lastKernel = in_degree(PipelineOutput, G) == 0 ? LastKernel : PipelineOutput;

    std::vector<Z3_ast> VarList(LastStreamSet + 1);

    const auto ONE = constant(1);

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
        // Source kernels always perform exactly one iteration
        if (in_degree(kernel, G) == 0) {
            VarList[kernel] = ONE;
        } else {

            auto v = Z3_mk_fresh_const(ctx, nullptr, realType);
            auto c1 = Z3_mk_ge(ctx, v, ONE);
            Z3_solver_assert(ctx, solver, c1);


            VarList[kernel] = v;
        }
    }

    flat_set<unsigned> crossesPartition;

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {

        const auto avgStridesPerSegmentVar = VarList[kernel];

        unsigned numOfGreedyRates = 0;

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
                    crossesPartition.insert(kernel);
                } else {
                    constraint = Z3_mk_ge(ctx, producedRate, consumedRate);
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
                // const auto r = constant(average(inputRate));
                // const auto r = constant(inputRate.Minimum);
                const auto r = bounded_variable(constant(inputRate.Minimum), constant(inputRate.Maximum));
                const auto consumedRate = multiply(avgStridesPerSegmentVar, r);
                const auto constraint = Z3_mk_ge(ctx, producedRate, consumedRate);
                Z3_solver_assert(ctx, solver, constraint);
                crossesPartition.insert(kernel);
            }
        }

        // Any kernel with a greedy rate must exhaust its input in a single iteration.
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
                VarList[buffer] = lower_bounded_variable(constant(outputRate.Minimum));
            } else {
                // const auto r = constant(outputRate.Maximum);
                // const auto r = constant(average(outputRate));

                const auto r = bounded_variable(constant(outputRate.Minimum), constant(outputRate.Maximum));
                const auto producedRate = multiply(avgStridesPerSegmentVar, r);
                VarList[buffer] = producedRate;
            }
        }
    }

    std::vector<Rational> avgNumOfStrides(PipelineOutput + 1);

    if (LLVM_UNLIKELY(Z3_solver_check(ctx, solver) != Z3_L_TRUE)) {
        report_fatal_error("Unexpected Z3 error: unsolvable dataflow graph.");
    }

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
            report_fatal_error("Unexpected Z3 error when attempting to convert model value to integer!");
        }
        assert (num > 0);
        avgNumOfStrides[kernel] = Rational{num, denom};
    }
    Z3_model_dec_ref(ctx, model);

    std::vector<unsigned> testNumOfStrides(PipelineOutput + 1, 0);

    const auto n = crossesPartition.size();

    std::vector<Z3_ast> assumptions;
    assumptions.reserve(4);

    std::vector<unsigned> check(crossesPartition.begin(), crossesPartition.end());

    unsigned iteration = 0;

    std::default_random_engine generator;

    Z3_params params = Z3_mk_params(ctx);
    Z3_params_inc_ref(ctx, params);
    Z3_symbol timeout = Z3_mk_string_symbol(ctx, "timeout");
    Z3_params_set_uint(ctx, params, timeout, 100);
    Z3_solver_set_params(ctx, solver, params);
    Z3_params_dec_ref(ctx, params);

    for (;;) {

        if (LLVM_UNLIKELY(check.empty())) {
            break;
        }

        const auto k = 1UL; // std::min(check.size(), 4UL);

 //       std::shuffle(check.begin(), check.end(), generator);

        assumptions.clear();

        Z3_solver_push(ctx, solver);

        errs() << "Z3: I=" << iteration << "\n"; // << " K=" << k;

        for (unsigned i = 0; i < k; ++i) {
            const auto kernel = check[i];
            const auto a = avgNumOfStrides[kernel];
            const auto avgStridesPerSegmentVar = VarList[kernel];
            const auto constraint = Z3_mk_gt(ctx, avgStridesPerSegmentVar, constant(a));
            assumptions.push_back(constraint);
            Z3_solver_assert(ctx, solver, constraint);
        }

        ++iteration;

//        auto m = k;

//        if (LLVM_UNLIKELY(Z3_solver_check(ctx, solver) != Z3_L_TRUE)) {
//            Z3_solver_pop(ctx, solver, 1);
//            Z3_solver_push(ctx, solver);
//            m = maxsat(ctx, solver, assumptions);
//        }

//        errs() << " M=" << m << "\n";

//        if (m < 1) {

        const auto r = Z3_solver_check(ctx, solver);

        if (LLVM_UNLIKELY(r != Z3_L_TRUE)) {
            check.erase(check.begin(), check.begin() + k);
        } else {


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
                    report_fatal_error("Unexpected Z3 error when attempting to convert model value to integer!");
                }
                assert (num > 0);
                avgNumOfStrides[kernel] = num;
            }
            Z3_model_dec_ref(ctx, model);
        }
        Z3_solver_pop(ctx, solver, 1);

    }

    Z3_solver_dec_ref(ctx, solver);
    Z3_del_context(ctx);

    unsigned denom_lcm = 1;

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
        const auto a = avgNumOfStrides[kernel];
        if (a.denominator() != 1) {
            denom_lcm = boost::lcm(denom_lcm, a.denominator());
        }
    }

    std::vector<unsigned> result(PipelineOutput + 1);

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
        const auto a = avgNumOfStrides[kernel] * denom_lcm;
        assert (a.denominator() == 1);
        result[kernel] = a.numerator();
    }

    return result;
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
