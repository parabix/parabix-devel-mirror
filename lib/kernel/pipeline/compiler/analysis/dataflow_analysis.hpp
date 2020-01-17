#ifndef DATAFLOW_ANALYSIS_HPP
#define DATAFLOW_ANALYSIS_HPP

#include "../pipeline_compiler.hpp"

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
    auto ctx = Z3_mk_context(cfg);
    Z3_del_config(cfg);
    auto solver = Z3_mk_solver(ctx);
    Z3_solver_inc_ref(ctx, solver);

    auto constant = [&](const Rational value) {
        if (value.denominator() == 1) {
            return Z3_mk_int(ctx, value.numerator(), Z3_mk_int_sort(ctx));
        } else {
            return Z3_mk_real(ctx, value.numerator(), value.denominator());
        }
    };

    auto bounded_variable = [&](Z3_ast lb) {
        auto v = Z3_mk_fresh_const(ctx, nullptr, Z3_mk_int_sort(ctx));
        auto c1 = Z3_mk_ge(ctx, v, lb);
        Z3_solver_assert(ctx, solver, c1);
        return v;
    };

    auto multiply =[&](Z3_ast X, Z3_ast Y) {
        Z3_ast args[2] = { X, Y };
        return Z3_mk_mul(ctx, 2, args);
    };

    auto ONE = Z3_mk_int(ctx, 1, Z3_mk_int_sort(ctx));

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

    const auto firstKernel = out_degree(PipelineInput, G) == 0 ? FirstKernel : PipelineInput;
    const auto lastKernel = in_degree(PipelineOutput, G) == 0 ? LastKernel : PipelineOutput;

    std::vector<Z3_ast> VarList(LastStreamSet + 1);
    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
        // Source kernels always perform exactly one iteration
        if (in_degree(kernel, G) == 0) {
            VarList[kernel] = ONE;
        } else {
            VarList[kernel] = bounded_variable(ONE);
        }
    }

    const Rational HALF{1, 2};

    const Rational R_ONE{1, 1};

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {

        const auto avgStridesPerSegmentVar = VarList[kernel];

        bool hasGreedyRate = false;

        for (const auto input : make_iterator_range(in_edges(kernel, G))) {
            const auto buffer = source(input, G);
            const auto producer = parent(buffer, G);

            const BufferRateData & inputRate = G[input];
            const Binding & binding = inputRate.Binding;
            const ProcessingRate & rate = binding.getRate();

            const auto producedRate = VarList[buffer];
            assert (producedRate);

            if (rate.isFixed() || rate.isGreedy()) {

                Z3_ast strideRate = nullptr;
                if (LLVM_LIKELY(rate.isFixed())) {
                    strideRate = constant(inputRate.Minimum);
                } else {
                    strideRate = producedRate;
                    hasGreedyRate = true;
                }
                const auto consumedRate = multiply(avgStridesPerSegmentVar, strideRate);
                Z3_ast constraint = nullptr;
                // A greedy rate or partition-local stream must always consume everything
                if (KernelPartitionId[producer] == KernelPartitionId[kernel]) { // rate.isGreedy() ||
                    constraint = Z3_mk_eq(ctx, producedRate, consumedRate);
                } else {
                    constraint = Z3_mk_ge(ctx, producedRate, consumedRate);
                }
                Z3_solver_assert(ctx, solver, constraint);

            } else if (LLVM_UNLIKELY(rate.isUnknown())) {

                SmallVector<char, 256> tmp;
                raw_svector_ostream msg(tmp);
                msg << getKernel(kernel)->getName() << "."
                    << binding.getName()
                    << " may not have an unknown input rate.";
                report_fatal_error(msg.str());

            } else {

                const auto r = constant(std::max(inputRate.Minimum, R_ONE));
                const auto consumedRate = multiply(avgStridesPerSegmentVar, r);
                const auto constraint = Z3_mk_ge(ctx, producedRate, consumedRate);
                Z3_solver_assert(ctx, solver, constraint);

            }
        }

        // Any kernel with a greedy rate must exhaust its input within a single iteration.
        if (LLVM_UNLIKELY(hasGreedyRate)) {
            const auto constraint = Z3_mk_eq(ctx, avgStridesPerSegmentVar, ONE);
            Z3_solver_assert(ctx, solver, constraint);
        }

        for (const auto output : make_iterator_range(out_edges(kernel, G))) {
            const BufferRateData & outputRate = G[output];
            const auto buffer = target(output, G);
            if (LLVM_UNLIKELY(outputRate.Maximum.numerator() == 0)) {
                VarList[buffer] = bounded_variable(constant(0));
            } else {
                const auto r = constant(outputRate.Maximum);
                const auto producedRate = multiply(avgStridesPerSegmentVar, r);
                VarList[buffer] = producedRate;
            }
        }
    }

    if (LLVM_UNLIKELY(Z3_solver_check(ctx, solver) != Z3_L_TRUE)) {
        report_fatal_error("Unexpected error: unsolvable dataflow graph.");
    }

    Z3_model model = Z3_solver_get_model(ctx, solver);
    Z3_model_inc_ref(ctx, model);

    std::vector<unsigned> expected(PipelineOutput + 1);

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
        Z3_ast const kernelVar = VarList[kernel];
        Z3_ast value;
        if (LLVM_UNLIKELY(Z3_model_eval(ctx, model, kernelVar, Z3_L_TRUE, &value) != Z3_L_TRUE)) {
            report_fatal_error("Unexpected Z3 error when attempting to obtain value from model!");
        }
        unsigned num;
        if (LLVM_UNLIKELY(Z3_get_numeral_uint(ctx, value, &num) != Z3_L_TRUE)) {
            report_fatal_error("Unexpected Z3 error when attempting to convert model value to integer!");
        }
        assert (num > 0);

        errs() << "Kernel " << kernel << " : " << num << "\n";

        expected[kernel] = num;
    }
    Z3_model_dec_ref(ctx, model);

    Z3_solver_dec_ref(ctx, solver);
    Z3_del_context(ctx);

    return expected;
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
