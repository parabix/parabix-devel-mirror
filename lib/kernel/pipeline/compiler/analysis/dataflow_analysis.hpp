#ifndef DATAFLOW_ANALYSIS_HPP
#define DATAFLOW_ANALYSIS_HPP

#include "../pipeline_compiler.hpp"

#include <z3++.h>

namespace kernel {

#if 1

///** ------------------------------------------------------------------------------------------------------------- *
// * @brief computeParitionDataFlowRates
// ** ------------------------------------------------------------------------------------------------------------- */
//void PipelineCompiler::computeDataFlowRates(BufferGraph & G) {

//    enum {
//        LowerBound = 0,
//        UpperBound = 1
//    };

//    using BoundedVars = std::array<Z3_ast, 2>;

//    using Bound = std::array<Rational, 2>;

//    const auto cfg = Z3_mk_config();
//    Z3_set_param_value(cfg, "MODEL", "true");
//    Z3_set_param_value(cfg, "proof", "false");
//    const auto ctx = Z3_mk_context(cfg);
//    Z3_del_config(cfg);
//    const auto solver = Z3_mk_solver(ctx);
//    Z3_solver_inc_ref(ctx, solver);

//    const auto varType = Z3_mk_real_sort(ctx);

//    auto constant = [&](const Rational value) {
//        return Z3_mk_real(ctx, value.numerator(), value.denominator());
//    };

//    const auto ONE = constant(1);

//    auto average = [&](const BufferRateData & rate) {
//        return constant((rate.Minimum + rate.Maximum) * Rational{1, 2});
//    };

//    auto maximum = [&](const BufferRateData & rate) {
//        return constant(rate.Maximum);
//    };

//    auto minimum = [&](const BufferRateData & rate) {
//        return constant(std::max(rate.Minimum, Rational{1}));
//    };

//    auto free_variable = [&](Z3_ast min) {
//        auto lb = Z3_mk_fresh_const(ctx, nullptr, varType);
//        auto c1 = Z3_mk_ge(ctx, lb, min);
//        Z3_solver_assert(ctx, solver, c1);
//        auto ub = Z3_mk_fresh_const(ctx, nullptr, varType);
//        auto c2 = Z3_mk_ge(ctx, ub, lb);
//        Z3_solver_assert(ctx, solver, c2);
//        return BoundedVars{lb, ub};
//    };

//    auto lower_bounded_variable = [&](const BufferRateData & rate) {
//        return free_variable(minimum(rate));
//    };

//    auto bounded_variable = [&](const BufferRateData & rate) {
//        assert (rate.Minimum < rate.Maximum);
//        auto var = lower_bounded_variable(rate);
//        auto c2 = Z3_mk_le(ctx, var[UpperBound], maximum(rate));
//        Z3_solver_assert(ctx, solver, c2);
//        return var;
//    };

//    auto multiply =[&](Z3_ast X, Z3_ast Y) {
//        Z3_ast args[2] = { X, Y };
//        return Z3_mk_mul(ctx, 2, args);
//    };

//    const auto firstKernel = out_degree(PipelineInput, G) == 0 ? FirstKernel : PipelineInput;
//    const auto lastKernel = in_degree(PipelineOutput, G) == 0 ? LastKernel : PipelineOutput;

//    std::vector<unsigned> PartitionKernelCount(PartitionCount, 0);

//    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
//        const auto partition = KernelPartitionId[kernel];
//        PartitionKernelCount[partition]++;
//    }

//    std::vector<BoundedVars> PartitionVarList(PartitionCount);

//    for (unsigned i = 0U; i < PartitionCount; ++i) {
//        if (PartitionKernelCount[i] > 1) {
//            PartitionVarList[i] = free_variable(ONE);
//        }
//    }

//    std::vector<BoundedVars> VarList(LastStreamSet + 1);

//    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
//        // Source kernels always perform exactly one iteration
//        BoundedVars & var = VarList[kernel];
//        if (in_degree(kernel, G) == 0) {
//            var[LowerBound] = ONE;
//            var[UpperBound] = ONE;
//        } else {
//            const auto partition = KernelPartitionId[kernel];
//            if (PartitionKernelCount[partition] > 1) {
//                auto ratio = Z3_mk_fresh_const(ctx, nullptr, varType);
//                auto c1 = Z3_mk_ge(ctx, ratio, ONE);
//                Z3_solver_assert(ctx, solver, c1);
//                const auto partition = KernelPartitionId[kernel];
//                const BoundedVars & P = PartitionVarList[partition];
//                var[LowerBound] = multiply(ratio, P[LowerBound]);
//                var[UpperBound] = multiply(ratio, P[UpperBound]);
//            } else {
//                var = free_variable(ONE);
//            }
//        }
//    }

//    std::vector<Z3_ast> assumptions;

//    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {

//        const auto stridesPerSegmentVar = VarList[kernel];
//        const auto partitionId = KernelPartitionId[kernel];

//        unsigned numOfGreedyRates = 0;

//        for (const auto input : make_iterator_range(in_edges(kernel, G))) {
//            const auto buffer = source(input, G);

//            const BufferRateData & inputRate = G[input];
//            const Binding & binding = inputRate.Binding;
//            const ProcessingRate & rate = binding.getRate();

//            const auto producedRate = VarList[buffer];

//            if (LLVM_LIKELY(rate.isFixed())) {

//                const auto fixedInputRateVal = maximum(inputRate);
//                const auto lb = multiply(stridesPerSegmentVar[LowerBound], fixedInputRateVal);
//                const auto ub = multiply(stridesPerSegmentVar[UpperBound], fixedInputRateVal);
//                // A partition-local stream must *always* consume everything but streams that
//                // cross a partition simply need to produce enough to satisfy its consumer(s)
//                const auto producer = parent(buffer, G);
//                const auto consumeEverything1 = Z3_mk_eq(ctx, producedRate[LowerBound], lb);
//                const auto consumeEverything2 = Z3_mk_eq(ctx, producedRate[UpperBound], ub);
//                if (KernelPartitionId[producer] == partitionId) {
//                    Z3_solver_assert(ctx, solver, consumeEverything1);
//                    Z3_solver_assert(ctx, solver, consumeEverything2);
//                } else {
//                    const auto produceEnough1 = Z3_mk_ge(ctx, producedRate[LowerBound], lb);
//                    Z3_solver_assert(ctx, solver, produceEnough1);
//                    const auto produceEnough2 = Z3_mk_ge(ctx, producedRate[UpperBound], ub);
//                    Z3_solver_assert(ctx, solver, produceEnough2);
//                    // Assume we can consume all of the data
//                    assumptions.push_back(consumeEverything1);
//                    assumptions.push_back(consumeEverything2);
//                }
//            } else if (LLVM_UNLIKELY(rate.isGreedy())) {
//                ++numOfGreedyRates;
//            } else if (LLVM_UNLIKELY(rate.isUnknown())) {

//                SmallVector<char, 256> tmp;
//                raw_svector_ostream msg(tmp);
//                msg << getKernel(kernel)->getName() << "."
//                    << binding.getName()
//                    << " may not have an unknown input rate.";
//                report_fatal_error(msg.str());

//            } else {

//                const auto inputRateVar = bounded_variable(inputRate);
//                const auto lb = multiply(stridesPerSegmentVar[LowerBound], inputRateVar[LowerBound]);
//                const auto ub = multiply(stridesPerSegmentVar[UpperBound], inputRateVar[UpperBound]);

//                const auto produceEnough1 = Z3_mk_ge(ctx, producedRate[LowerBound], lb);
//                Z3_solver_assert(ctx, solver, produceEnough1);
//                const auto produceEnough2 = Z3_mk_ge(ctx, producedRate[UpperBound], ub);
//                Z3_solver_assert(ctx, solver, produceEnough2);

//                // Since we are trying to determine the lower and upper bound on the number
//                // of strides per segment, to determine the lower bound we assume that a
//                // kernel consumes the maximum amount of data but produces the minimum.
//                // The upper bound is similar except we consume the minimum and produce
//                // the maximum.

//                const auto avgInputRate = average(inputRate);

//                assumptions.push_back(Z3_mk_ge(ctx, inputRateVar[LowerBound], avgInputRate));
//                assumptions.push_back(Z3_mk_eq(ctx, inputRateVar[LowerBound], maximum(inputRate)));

//                assumptions.push_back(Z3_mk_le(ctx, inputRateVar[UpperBound], avgInputRate));
//                assumptions.push_back(Z3_mk_eq(ctx, ub, minimum(inputRate)));
//            }
//        }

//        // Any kernel with all greedy rates must exhaust its input in a single iteration.
//        if (LLVM_UNLIKELY(numOfGreedyRates == in_degree(kernel, G))) {
//            const auto constraint1 = Z3_mk_eq(ctx, stridesPerSegmentVar[LowerBound], ONE);
//            Z3_solver_assert(ctx, solver, constraint1);
//            const auto constraint2 = Z3_mk_eq(ctx, stridesPerSegmentVar[UpperBound], ONE);
//            Z3_solver_assert(ctx, solver, constraint2);
//        }

//        for (const auto output : make_iterator_range(out_edges(kernel, G))) {
//            const BufferRateData & outputRate = G[output];
//            const Binding & binding = outputRate.Binding;
//            const ProcessingRate & rate = binding.getRate();
//            const auto buffer = target(output, G);
//            if (LLVM_LIKELY(rate.isFixed())) {
//                const auto fixedOutputRateVal = maximum(outputRate);
//                const auto lb = multiply(stridesPerSegmentVar[LowerBound], fixedOutputRateVal);
//                const auto ub = multiply(stridesPerSegmentVar[UpperBound], fixedOutputRateVal);



//                VarList[buffer] = { lb, ub };
//            } else if (LLVM_UNLIKELY(rate.isUnknown())) {
//                // TODO: is there a better way to handle unknown outputs? This represents
//                // the ideal amount of data to transfer to subsequent kernels but that
//                // isn't very meaningful here.
//                VarList[buffer] = lower_bounded_variable(outputRate);
//            } else {
//                const auto outputRateVar = bounded_variable(outputRate);
//                const auto lb = multiply(stridesPerSegmentVar[LowerBound], outputRateVar[LowerBound]);
//                const auto ub = multiply(stridesPerSegmentVar[UpperBound], outputRateVar[UpperBound]);
//                VarList[buffer] = { lb, ub };
//                // Like above, when calculating the lower bound of the number of kernel strides,
//                // we assume we've produced the minimum amount of data and for the upper bound,
//                // the maximum.

//                const auto avgOutputRate = average(outputRate);
//                assumptions.push_back(Z3_mk_le(ctx, outputRateVar[LowerBound], avgOutputRate));
//                assumptions.push_back(Z3_mk_eq(ctx, lb, minimum(outputRate)));

//                assumptions.push_back(Z3_mk_ge(ctx, outputRateVar[UpperBound], avgOutputRate));
//                assumptions.push_back(Z3_mk_eq(ctx, outputRateVar[UpperBound], maximum(outputRate)));
//            }
//        }
//    }

//    std::vector<Bound> current(PipelineOutput + 1);

////    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
////        current[kernel] = Bound{{Rational{-1U}, Rational{0U}}};
////    }
////    std::vector<Bound> prior(PipelineOutput + 1);

////    for (;;) {

//        Z3_solver_push(ctx, solver);
//        auto m = Z3_maxsat(ctx, solver, assumptions);
////        if (LLVM_UNLIKELY(m < 1)) {
////            break;
////        }

////        errs() << m << " of " << assumptions.size() << "\n";

////        current.swap(prior);

//        const auto model = Z3_solver_get_model(ctx, solver);
//        Z3_model_inc_ref(ctx, model);
//        for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
//            auto stridesPerSegmentVar = VarList[kernel];
//            for (unsigned bound = LowerBound; bound <= UpperBound; ++bound) {
//                Z3_ast value;
//                if (LLVM_UNLIKELY(Z3_model_eval(ctx, model, stridesPerSegmentVar[bound], Z3_L_TRUE, &value) != Z3_L_TRUE)) {
//                    report_fatal_error("Unexpected Z3 error when attempting to obtain value from model!");
//                }
//                __int64 num, denom;
//                if (LLVM_UNLIKELY(Z3_get_numeral_rational_int64(ctx, value, &num, &denom) != Z3_L_TRUE)) {
//                    report_fatal_error("Unexpected Z3 error when attempting to convert model value to number!");
//                }
//                assert (num > 0);
//                current[kernel][bound] = Rational{num, denom};
//            }
//        }
//        Z3_model_dec_ref(ctx, model);

////        break;

////        assumptions.clear();

////        for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
////            const auto & C = current[kernel];
////            const auto & P = prior[kernel];
////            if (C[LowerBound] > Rational{1} && C[LowerBound] < P[LowerBound]) {
////                assumptions.push_back(Z3_mk_lt(ctx, VarList[kernel][LowerBound], constant(C[LowerBound])));
////            }
////            if (C[UpperBound] > P[UpperBound]) {
////                assumptions.push_back(Z3_mk_gt(ctx, VarList[kernel][UpperBound], constant(C[UpperBound])));
////            }
////        }

////        if (assumptions.empty()) {
////            break;
////        }

////    }

//    // Scale all numbers up to integers
//    const Bound & bound = current[firstKernel];
//    auto g = gcd(bound[LowerBound], bound[UpperBound]);
//    for (auto kernel = firstKernel + 1; kernel <= lastKernel; ++kernel) {
//        const Bound & bound = current[kernel];
//        g = gcd(g, bound[LowerBound]);
//        g = gcd(g, bound[UpperBound]);
//    }
//    if (LLVM_UNLIKELY(g.numerator() > g.denominator())) {
//        for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
//            current[kernel][LowerBound] /= g;
//            current[kernel][UpperBound] /= g;
//        }
//    }

//    // Then write them out
//    MinimumNumOfStrides.resize(PipelineOutput + 1);
//    MaximumNumOfStrides.resize(PipelineOutput + 1);

//    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
//        MinimumNumOfStrides[kernel] = floor(current[kernel][LowerBound]);
//        MaximumNumOfStrides[kernel] = ceiling(current[kernel][UpperBound]);
//    }

////    auto min = std::numeric_limits<unsigned>::max();
////    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
////        const auto a = bounds[kernel][LowerBound] * denom_lcm;
////        assert (a.denominator() == 1);
////        min = std::min(min, a.numerator());
////        MinimumNumOfStrides[kernel] = a.numerator();
////        const auto b = bounds[kernel][UpperBound] * denom_lcm;
////        assert (b.denominator() == 1);
////        assert (a <= b);
////        MaximumNumOfStrides[kernel] = b.numerator();
////    }

////    auto gcd = min;
////    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
////        if (LLVM_LIKELY(gcd == 1)) break;
////        gcd = boost::gcd(gcd, MinimumNumOfStrides[kernel]);
////        if (LLVM_LIKELY(gcd == 1)) break;
////        gcd = boost::gcd(gcd, MaximumNumOfStrides[kernel]);
////    }

////    if (LLVM_UNLIKELY(gcd != 1)) {
////        for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
////            MinimumNumOfStrides[kernel] /= gcd;
////            MaximumNumOfStrides[kernel] /= gcd;
////        }
////    }

//    Z3_solver_dec_ref(ctx, solver);
//    Z3_del_context(ctx);

//    Z3_finalize_memory();

//}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeParitionDataFlowRates
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::computeDataFlowRates(BufferGraph & G) {

    enum {
        LowerBound = 0,
        UpperBound = 1
    };

    using BoundedVars = std::array<Z3_ast, 2>;

    using Bound = std::array<Rational, 2>;

    const auto cfg = Z3_mk_config();
    Z3_set_param_value(cfg, "MODEL", "true");
    Z3_set_param_value(cfg, "proof", "false");
    const auto ctx = Z3_mk_context(cfg);
    Z3_del_config(cfg);
    const auto solver = Z3_mk_optimize(ctx);
    Z3_optimize_inc_ref(ctx, solver);

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

    auto Z3_assert = [&](Z3_ast a) {
        Z3_optimize_assert(ctx, solver, a);
    };

    auto free_variable = [&](Z3_ast min) {
        auto lb = Z3_mk_fresh_const(ctx, nullptr, varType);
        auto c1 = Z3_mk_ge(ctx, lb, min);
        Z3_assert(c1);
        auto ub = Z3_mk_fresh_const(ctx, nullptr, varType);
        auto c2 = Z3_mk_ge(ctx, ub, lb);
        Z3_assert(c2);
        return BoundedVars{lb, ub};
    };

    auto lower_bounded_variable = [&](const BufferRateData & rate) {
        return free_variable(minimum(rate));
    };

    auto bounded_variable = [&](const BufferRateData & rate) {
        assert (rate.Minimum < rate.Maximum);
        auto var = lower_bounded_variable(rate);
        auto c2 = Z3_mk_le(ctx, var[UpperBound], maximum(rate));
        Z3_assert(c2);
        return var;
    };

    auto Z3_assert_soft = [&](Z3_ast a) {
        Z3_optimize_assert_soft(ctx, solver, a, "1000", 0);
    };

    auto multiply =[&](Z3_ast X, Z3_ast Y) {
        Z3_ast args[2] = { X, Y };
        return Z3_mk_mul(ctx, 2, args);
    };

    const auto firstKernel = out_degree(PipelineInput, G) == 0 ? FirstKernel : PipelineInput;
    const auto lastKernel = in_degree(PipelineOutput, G) == 0 ? LastKernel : PipelineOutput;

//    std::vector<unsigned> PartitionKernelCount(PartitionCount, 0);

//    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
//        const auto partition = KernelPartitionId[kernel];
//        PartitionKernelCount[partition]++;
//    }

//    std::vector<BoundedVars> PartitionVarList(PartitionCount);

//    for (unsigned i = 0U; i < PartitionCount; ++i) {
////        if (PartitionKernelCount[i] > 1) {
//            const auto var = free_variable(ONE);
//            PartitionVarList[i] = var;
//            Z3_optimize_minimize(ctx, solver, var[LowerBound]);
//            Z3_optimize_maximize(ctx, solver, var[UpperBound]);
////        }
//    }

    std::vector<BoundedVars> VarList(LastStreamSet + 1);

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
        // Source kernels always perform exactly one iteration
        BoundedVars & var = VarList[kernel];
        if (in_degree(kernel, G) == 0) {
            var[LowerBound] = ONE;
            var[UpperBound] = ONE;
        } else {
//            const auto partition = KernelPartitionId[kernel];
////            if (PartitionKernelCount[partition] > 1) {
//                auto ratio = Z3_mk_fresh_const(ctx, nullptr, varType);
//                auto c1 = Z3_mk_ge(ctx, ratio, ONE);
//                Z3_assert(c1);
//                const BoundedVars & P = PartitionVarList[partition];
//                var[LowerBound] = multiply(ratio, P[LowerBound]);
//                var[UpperBound] = multiply(ratio, P[UpperBound]);
////            } else {
////                var = free_variable(ONE);
////            }
            var = free_variable(ONE);
            Z3_optimize_minimize(ctx, solver, var[LowerBound]);
            Z3_optimize_maximize(ctx, solver, var[UpperBound]);

        }
    }


    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {

        const auto stridesPerSegmentVar = VarList[kernel];
        const auto partitionId = KernelPartitionId[kernel];

        unsigned numOfGreedyRates = 0;

        for (const auto input : make_iterator_range(in_edges(kernel, G))) {
            const auto buffer = source(input, G);

            const BufferRateData & inputRate = G[input];
            const Binding & binding = inputRate.Binding;
            const ProcessingRate & rate = binding.getRate();

            const auto producedRate = VarList[buffer];

            if (LLVM_LIKELY(rate.isFixed())) {

                const auto fixedInputRateVal = maximum(inputRate);
                const auto lb = multiply(stridesPerSegmentVar[LowerBound], fixedInputRateVal);
                const auto ub = multiply(stridesPerSegmentVar[UpperBound], fixedInputRateVal);
                // A partition-local stream must *always* consume everything but streams that
                // cross a partition simply need to produce enough to satisfy its consumer(s)
                const auto producer = parent(buffer, G);
                const auto consumeEverything1 = Z3_mk_eq(ctx, producedRate[LowerBound], lb);
                const auto consumeEverything2 = Z3_mk_eq(ctx, producedRate[UpperBound], ub);
                if (KernelPartitionId[producer] == partitionId) {
                    Z3_assert(consumeEverything1);
                    Z3_assert(consumeEverything2);
                } else {
                    const auto produceEnough1 = Z3_mk_ge(ctx, producedRate[LowerBound], lb);
                    Z3_assert(produceEnough1);
                    const auto produceEnough2 = Z3_mk_ge(ctx, producedRate[UpperBound], ub);
                    Z3_assert(produceEnough2);
                    // Assume we can consume all of the data
                    Z3_assert_soft(consumeEverything1);
                    Z3_assert_soft(consumeEverything2);
                }
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
                const auto lb = multiply(stridesPerSegmentVar[LowerBound], inputRateVar[LowerBound]);
                const auto ub = multiply(stridesPerSegmentVar[UpperBound], inputRateVar[UpperBound]);

                const auto produceEnough1 = Z3_mk_ge(ctx, producedRate[LowerBound], lb);
                Z3_assert(produceEnough1);
                const auto produceEnough2 = Z3_mk_ge(ctx, producedRate[UpperBound], ub);
                Z3_assert(produceEnough2);

                const auto consumeEverything1 = Z3_mk_eq(ctx, producedRate[LowerBound], lb);
                const auto consumeEverything2 = Z3_mk_eq(ctx, producedRate[UpperBound], ub);

                Z3_assert_soft(consumeEverything1);
                Z3_assert_soft(consumeEverything2);

                // Since we are trying to determine the lower and upper bound on the number
                // of strides per segment, to determine the lower bound we assume that a
                // kernel consumes the maximum amount of data but produces the minimum.
                // The upper bound is similar except we consume the minimum and produce
                // the maximum.

//                const auto avgInputRate = average(inputRate);

//                Z3_assert_soft(Z3_mk_ge(ctx, inputRateVar[LowerBound], avgInputRate));
//                Z3_assert_soft(Z3_mk_eq(ctx, inputRateVar[LowerBound], maximum(inputRate)));

//                Z3_assert_soft(Z3_mk_le(ctx, inputRateVar[UpperBound], avgInputRate));
//                Z3_assert_soft(Z3_mk_eq(ctx, ub, minimum(inputRate)));
            }
        }

        // Any kernel with all greedy rates must exhaust its input in a single iteration.
        if (LLVM_UNLIKELY(numOfGreedyRates == in_degree(kernel, G))) {
            const auto constraint1 = Z3_mk_eq(ctx, stridesPerSegmentVar[LowerBound], ONE);
            Z3_assert(constraint1);
            const auto constraint2 = Z3_mk_eq(ctx, stridesPerSegmentVar[UpperBound], ONE);
            Z3_assert(constraint2);
        }

        for (const auto output : make_iterator_range(out_edges(kernel, G))) {
            const BufferRateData & outputRate = G[output];
            const Binding & binding = outputRate.Binding;
            const ProcessingRate & rate = binding.getRate();
            const auto buffer = target(output, G);
            if (LLVM_LIKELY(rate.isFixed())) {
                const auto fixedOutputRateVal = maximum(outputRate);
                const auto lb = multiply(stridesPerSegmentVar[LowerBound], fixedOutputRateVal);
                const auto ub = multiply(stridesPerSegmentVar[UpperBound], fixedOutputRateVal);



                VarList[buffer] = { lb, ub };
            } else if (LLVM_UNLIKELY(rate.isUnknown())) {
                // TODO: is there a better way to handle unknown outputs? This represents
                // the ideal amount of data to transfer to subsequent kernels but that
                // isn't very meaningful here.
                VarList[buffer] = lower_bounded_variable(outputRate);
            } else {
                const auto outputRateVar = bounded_variable(outputRate);
                const auto lb = multiply(stridesPerSegmentVar[LowerBound], outputRateVar[LowerBound]);
                const auto ub = multiply(stridesPerSegmentVar[UpperBound], outputRateVar[UpperBound]);
                VarList[buffer] = { lb, ub };
                // Like above, when calculating the lower bound of the number of kernel strides,
                // we assume we've produced the minimum amount of data and for the upper bound,
                // the maximum.

//                const auto avgOutputRate = average(outputRate);
//                Z3_assert_soft(Z3_mk_le(ctx, outputRateVar[LowerBound], avgOutputRate));
//                Z3_assert_soft(Z3_mk_eq(ctx, lb, minimum(outputRate)));

//                Z3_assert_soft(Z3_mk_ge(ctx, outputRateVar[UpperBound], avgOutputRate));
//                Z3_assert_soft(Z3_mk_eq(ctx, outputRateVar[UpperBound], maximum(outputRate)));
            }
        }
    }

    std::vector<Bound> current(PipelineOutput + 1);

    const auto r = Z3_optimize_check(ctx, solver);

    if (r == Z3_L_FALSE) {
        report_fatal_error("Z3 failed to find dataflow solution");
    }

    const auto model = Z3_optimize_get_model(ctx, solver);
    Z3_model_inc_ref(ctx, model);
    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
        auto stridesPerSegmentVar = VarList[kernel];
        for (unsigned bound = LowerBound; bound <= UpperBound; ++bound) {
            Z3_ast value;
            if (LLVM_UNLIKELY(Z3_model_eval(ctx, model, stridesPerSegmentVar[bound], Z3_L_TRUE, &value) != Z3_L_TRUE)) {
                report_fatal_error("Unexpected Z3 error when attempting to obtain value from model!");
            }
            __int64 num, denom;
            if (LLVM_UNLIKELY(Z3_get_numeral_rational_int64(ctx, value, &num, &denom) != Z3_L_TRUE)) {
                report_fatal_error("Unexpected Z3 error when attempting to convert model value to number!");
            }
            assert (num > 0);
            current[kernel][bound] = Rational{num, denom};
        }
    }
    Z3_model_dec_ref(ctx, model);

    Z3_optimize_dec_ref(ctx, solver);
    Z3_del_context(ctx);

    Z3_finalize_memory();


    // Scale all numbers up to integers
    const Bound & bound = current[firstKernel];
    auto g = gcd(bound[LowerBound], bound[UpperBound]);
    for (auto kernel = firstKernel + 1; kernel <= lastKernel; ++kernel) {
        const Bound & bound = current[kernel];
        g = gcd(g, bound[LowerBound]);
        g = gcd(g, bound[UpperBound]);
    }
    if (LLVM_UNLIKELY(g.numerator() > g.denominator())) {
        for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
            current[kernel][LowerBound] /= g;
            current[kernel][UpperBound] /= g;
        }
    }

    // Then write them out
    MinimumNumOfStrides.resize(PipelineOutput + 1);
    MaximumNumOfStrides.resize(PipelineOutput + 1);

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
        MinimumNumOfStrides[kernel] = floor(current[kernel][LowerBound]);
        MaximumNumOfStrides[kernel] = ceiling(current[kernel][UpperBound]);
    }

}


#else

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeParitionDataFlowRates
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::computeDataFlowRates(BufferGraph & G) {

    using Bound = std::array<Rational, 2>;

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

    enum {
        LowerBound = 0,
        UpperBound = 1,
        Common = 2
    };

    std::array<std::vector<Z3_ast>, 3> assumptions;

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {

        const auto stridesPerSegmentVar = VarList[kernel];
        const auto partitionId = KernelPartitionId[kernel];

        unsigned numOfGreedyRates = 0;

        for (const auto input : make_iterator_range(in_edges(kernel, G))) {
            const auto buffer = source(input, G);

            const BufferRateData & inputRate = G[input];
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
                const auto producer = parent(buffer, G);
                if (KernelPartitionId[producer] == partitionId) {
                    Z3_solver_assert(ctx, solver, consumeEverything);
                } else {
                    const auto produceEnough = Z3_mk_ge(ctx, producedRate, consumedRate);
                    Z3_solver_assert(ctx, solver, produceEnough);
                    // Assume we can consume all of the data
                    assumptions[Common].push_back(consumeEverything);
                }
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
        if (LLVM_UNLIKELY(numOfGreedyRates == in_degree(kernel, G))) {
            const auto constraint = Z3_mk_eq(ctx, stridesPerSegmentVar, ONE);
            Z3_solver_assert(ctx, solver, constraint);
        }

        for (const auto output : make_iterator_range(out_edges(kernel, G))) {
            const BufferRateData & outputRate = G[output];
            const Binding & binding = outputRate.Binding;
            const ProcessingRate & rate = binding.getRate();
            const auto buffer = target(output, G);
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

    Z3_solver_push(ctx, solver);
    const auto m = Z3_maxsat(ctx, solver, assumptions[Common]);
    if (LLVM_UNLIKELY(m < 1)) {
        Z3_solver_pop(ctx, solver, 1);
    }
    std::vector<Bound> bounds(PipelineOutput + 1);

    for (;;) {

        for (unsigned bound = LowerBound; bound <= UpperBound; ++bound) {

            Z3_solver_push(ctx, solver);

            Z3_maxsat(ctx, solver, assumptions[bound]);

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

    MinimumNumOfStrides.resize(PipelineOutput + 1);
    MaximumNumOfStrides.resize(PipelineOutput + 1);

    auto min = std::numeric_limits<unsigned>::max();
    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
        const auto a = floor(bounds[kernel][LowerBound]);
        MinimumNumOfStrides[kernel] = a;
        min = std::min(min, a);
        const auto b = ceiling(bounds[kernel][UpperBound]);
        assert (a <= b);
        MaximumNumOfStrides[kernel] = b;
    }

    auto gcd = min;
    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
        if (LLVM_LIKELY(gcd == 1)) break;
        gcd = boost::gcd(gcd, MinimumNumOfStrides[kernel]);
        if (LLVM_LIKELY(gcd == 1)) break;
        gcd = boost::gcd(gcd, MaximumNumOfStrides[kernel]);
    }

    if (LLVM_UNLIKELY(gcd != 1)) {
        for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
            MinimumNumOfStrides[kernel] /= gcd;
            MaximumNumOfStrides[kernel] /= gcd;
        }
    }

    Z3_solver_dec_ref(ctx, solver);
    Z3_del_context(ctx);

    Z3_finalize_memory();

}

#endif

#if 0

/// Below is an attempt to use the Z3_optimize solver to better represent the problem; it reports "invalid argument"
/// for most problems, however, but may serve for a basis for building a correct implementation.

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeParitionDataFlowRates
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::computeDataFlowRates(BufferGraph & G) {

    enum DataFlowType {
        LowerBound = 0,
        UpperBound = 1,
    };

    auto computeDataFlowRate = [&](const DataFlowType type) {

        const auto cfg = Z3_mk_config();
        Z3_set_param_value(cfg, "MODEL", "true");
        Z3_set_param_value(cfg, "proof", "false");
        const auto ctx = Z3_mk_context(cfg);
        Z3_del_config(cfg);
        const auto solver = Z3_mk_optimize(ctx);
        Z3_optimize_inc_ref(ctx, solver);

        const auto varType = Z3_mk_real_sort(ctx);

        auto constant = [&](const Rational value) {
            return Z3_mk_real(ctx, value.numerator(), value.denominator());
        };

        const auto ONE = constant(1);

        auto Z3_assert = [&](Z3_ast cond) {
            Z3_optimize_assert(ctx, solver, cond);
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
            Z3_assert(c1);
            return v;
        };

        auto lower_bounded_variable = [&](const BufferRateData & rate) {
            auto v = Z3_mk_fresh_const(ctx, nullptr, varType);
            auto c1 = Z3_mk_ge(ctx, v, minimum(rate));
            Z3_assert(c1);
            return v;
        };

        auto bounded_variable = [&](const BufferRateData & rate) {
            assert (rate.Minimum < rate.Maximum);
            auto v = Z3_mk_fresh_const(ctx, nullptr, varType);
            auto c1 = Z3_mk_ge(ctx, v, minimum(rate));
            Z3_assert(c1);
            auto c2 = Z3_mk_le(ctx, v, maximum(rate));
            Z3_assert(c2);
            return v;
        };

        auto multiply = [&](Z3_ast X, Z3_ast Y) {
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

        const auto n = std::min<unsigned>(3, lastKernel);

        for (auto kernel = firstKernel; kernel <= n; ++kernel) {

            const auto stridesPerSegmentVar = VarList[kernel];
            const auto partitionId = KernelPartitionId[kernel];

            unsigned numOfGreedyRates = 0;

            for (const auto input : make_iterator_range(in_edges(kernel, G))) {
                const auto buffer = source(input, G);

                const BufferRateData & inputRate = G[input];
                const Binding & binding = inputRate.Binding;
                const ProcessingRate & rate = binding.getRate();

                const auto producedRate = VarList[buffer]; Z3_assert (producedRate);

                if (LLVM_LIKELY(rate.isFixed())) {

                    const auto fixedRateVal = minimum(inputRate);
                    const auto consumedRate = multiply(stridesPerSegmentVar, fixedRateVal);
                    // To consume any data, we want to guarantee at least one full stride of work.
                    Z3_assert(Z3_mk_ge(ctx, consumedRate, fixedRateVal));

                    // A partition-local stream must *always* consume everything but streams that
                    // cross a partition simply need to produce enough to satisfy its consumer(s)
                    const auto producer = parent(buffer, G);
                    if (KernelPartitionId[producer] == partitionId) {
                        Z3_assert(Z3_mk_eq(ctx, producedRate, consumedRate));
                    } else {
                        // Ensure the producing kernel produces enough data when we consider
                        // a potential solution.
                        Z3_assert(Z3_mk_ge(ctx, producedRate, consumedRate));
                        // Assume we can consume all of the data
                        Z3_optimize_maximize(ctx, solver, producedRate);
                    }
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
                    Z3_assert(Z3_mk_ge(ctx, producedRate, consumedRate));
                    // To consume any data, we want to guarantee at least one full stride of work
                    Z3_assert(Z3_mk_ge(ctx, consumedRate, maximum(inputRate)));

                    // Since we are trying to determine the lower and upper bound on the number
                    // of strides per segment, to determine the lower bound we assume that a
                    // kernel consumes the maximum amount of data but produces the minimum.
                    // The upper bound is similar except we consume the minimum and produce
                    // the maximum.

                    if (type == LowerBound) {
                        Z3_optimize_maximize(ctx, solver, consumedRate);
                    } else if (type == UpperBound) {
                        Z3_optimize_minimize(ctx, solver, consumedRate);
                    }

                }
            }

            // Any kernel with all greedy rates must exhaust its input in a single iteration.
            if (LLVM_UNLIKELY(numOfGreedyRates == in_degree(kernel, G))) {
                Z3_assert(Z3_mk_eq(ctx, stridesPerSegmentVar, ONE));
            }

            for (const auto output : make_iterator_range(out_edges(kernel, G))) {
                const BufferRateData & outputRate = G[output];
                const Binding & binding = outputRate.Binding;
                const ProcessingRate & rate = binding.getRate();
                const auto buffer = target(output, G);
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
                    if (type == LowerBound) {
                        Z3_optimize_minimize(ctx, solver, producedRate);
                    } else if (type == UpperBound) {
                        Z3_optimize_maximize(ctx, solver, producedRate);
                    }
                }
            }
        }

        if (LLVM_UNLIKELY(Z3_optimize_check(ctx, solver) != Z3_TRUE)) {
            report_fatal_error("Z3 could not solve the initial dataflow graph");
        }

        std::vector<Rational> bounds(PipelineOutput + 1);

        const auto model = Z3_optimize_get_model(ctx, solver);
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
            bounds[kernel] = Rational{num, denom};
        }

        Z3_model_dec_ref(ctx, model);

        Z3_optimize_dec_ref(ctx, solver);
        Z3_del_context(ctx);

        return bounds;
    };

    const auto lb = computeDataFlowRate(DataFlowType::LowerBound);
    const auto ub = computeDataFlowRate(DataFlowType::UpperBound);

    // Scale all numbers up to integers

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

    // Then write them out

    MinimumNumOfStrides.resize(PipelineOutput + 1);
    MaximumNumOfStrides.resize(PipelineOutput + 1);

    for (auto kernel = PipelineInput; kernel <= PipelineOutput; ++kernel) {
        const auto a = lb[kernel] * denom_lcm;
        assert (a.denominator() == 1);
        MinimumNumOfStrides[kernel] = a.numerator();
        const auto b = ub[kernel] * denom_lcm;
        assert (b.denominator() == 1);
        assert (a <= b);
        MaximumNumOfStrides[kernel] = b.numerator();
    }

    Z3_finalize_memory();

}



#endif

} // end of kernel namespace

#endif // DATAFLOW_ANALYSIS_HPP
