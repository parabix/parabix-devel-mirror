#ifndef DATAFLOW_ANALYSIS_HPP
#define DATAFLOW_ANALYSIS_HPP

#include "../pipeline_compiler.hpp"
#include <boost/dynamic_bitset.hpp>
#include "rate_math.hpp"

namespace kernel {



/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeParitionDataFlowRates
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::computeDataFlowRates(BufferGraph & G) const {

    struct PartitionEdgeData {
        unsigned        KernelId;
        StreamSetPort   OutputPort;
        Expr            DataFlow;
    };


    using PartitionGraph = adjacency_list<vecS, vecS, bidirectionalS, no_property, Expr>;

    const auto first = out_degree(PipelineInput, G) == 0 ? FirstKernel : PipelineInput;
    const auto last = in_degree(PipelineOutput, G) == 0 ? LastKernel : PipelineOutput;

    std::vector<unsigned> kernelStrideRates(last);

    SmallVector<unsigned, 16> kernels;
    unsigned partitionId = 0;

    for (auto nextKernel = first;;) {

        kernels.clear();
        for (; nextKernel <= last; ++nextKernel) {
            if (KernelPartitionId[nextKernel] != partitionId) {
                break;
            }
            kernels.push_back(nextKernel);
        }

        computeStaticDataFlowRatesForParition(G, kernels, partitionId, kernelStrideRates);

        if (nextKernel > last) {
            break;
        }
        partitionId = KernelPartitionId[nextKernel];
    }

    errs() << kernelStrideRates[first];
    for (auto kernel = first + 1; kernel <= last; ++kernel) {
        errs() << ',' << kernelStrideRates[kernel];
    }
    errs() << "\n";


}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeDataFlowRatesForParition
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::computeStaticDataFlowRatesForParition(const BufferGraph & G,
                                                             const SmallVector<unsigned, 16> & partition,
                                                             const unsigned partitionId,
                                                             std::vector<unsigned> & kernelStrideRates) const {

    // When considering the dataflow rates of a single partition, we ignore any inputs / outputs that cross
    // a partition boundary. The reason is we want to determine --- given the optimal input -- how many
    // invocations of each kernel within the partition occur. From there, we can decide how much data is
    // required to cross the partition invocation threshold.

    using ReferenceGraph = adjacency_list<vecS, vecS, bidirectionalS>;

    using BufferRateExprMap = flat_map<unsigned, Expr>;

    BufferRateExprMap B;

    auto getVar = [&B](const unsigned v) {
        const auto f = B.find(v);
        assert (f != B.end());
        return f->second;
    };

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

    auto multiply =[&](Expr X, Expr Y) {
        Z3_ast args[2] = { X, Y };
        return Z3_mk_mul(ctx, 2, args);
    };

    auto ONE = Z3_mk_int(ctx, 1, Z3_mk_int_sort(ctx));

    errs() << "------------------------------------------\n";


    // VAR(PRODUCER(c)) * PRODUCED(c) = VAR(CONSUMER(c)) * CONSUMED(c)

    for (const auto kernel : partition) {

        errs() << " kernel: " << kernel << "\n";

        const Kernel * const kernelObj = getKernel(kernel);
        const auto strideSize = kernelObj->getStride();

        Expr kernelVar;
        if (LLVM_UNLIKELY(in_degree(kernel, G) == 0)) {
            kernelVar = ONE;
        } else {
            const auto S = Z3_mk_int_sort(ctx);
            kernelVar = Z3_mk_fresh_const(ctx, nullptr, S);
            // we only care about positive values
            const auto c1 = Z3_mk_ge(ctx, kernelVar, ONE);
            Z3_solver_assert(ctx, solver, c1);
        }
        B.emplace(kernel, kernelVar);

        for (const auto input : make_iterator_range(in_edges(kernel, G))) {
            const auto buffer = source(input, G);
            const auto producer = parent(buffer, G);
            if (KernelPartitionId[producer] == partitionId) {

                const BufferRateData & inputRate = G[input];
                const Binding & binding = inputRate.Binding;
                const ProcessingRate & rate = binding.getRate();

                errs() << " input: " << buffer << "\n";

                Expr const producedRate = getVar(buffer); assert (producedRate);

                Expr strideRate = nullptr;
                switch (rate.getKind()) {
                    case RateId::Fixed:
                        strideRate = constant(rate.getRate() * strideSize);
                        break;
                    case RateId::Greedy:
                        strideRate = producedRate;
                        break;
                    default: llvm_unreachable("unexpected or unknown input rate type");
                }
                const auto consumedRate = multiply(kernelVar, strideRate);
                const auto constraint = Z3_mk_eq(ctx, producedRate, consumedRate);
                Z3_solver_assert(ctx, solver, constraint);
            }
        }

        for (const auto output : make_iterator_range(out_edges(kernel, G))) {
            const BufferRateData & outputRate = G[output];
            const Binding & binding = outputRate.Binding;
            const ProcessingRate & rate = binding.getRate();
            // any non-fixed rates will cross a partition
            if (rate.isFixed()) {
                const auto buffer = target(output, G);
                const auto strideRate = constant(rate.getRate() * strideSize);
                const auto producedRate = multiply(kernelVar, strideRate);

                errs() << " output: " << buffer << "\n";

                B.emplace(buffer, producedRate);
            }
        }
    }

    if (LLVM_UNLIKELY(Z3_solver_check(ctx, solver) != Z3_L_TRUE)) {
        report_fatal_error("unsolvable SDF graph?");
    }

    Z3_model model = Z3_solver_get_model(ctx, solver);
    Z3_model_inc_ref(ctx, model);
    for (const auto kernel : partition) {
        Expr const kernelVar = getVar(kernel);

        Z3_ast value;
        if (LLVM_UNLIKELY(Z3_model_eval(ctx, model, kernelVar, Z3_L_TRUE, &value) != Z3_L_TRUE)) {
            throw std::runtime_error("Unexpected Z3 error when attempting to obtain value from model!");
        }
        unsigned num;
        if (LLVM_UNLIKELY(Z3_get_numeral_uint(ctx, value, &num) != Z3_L_TRUE)) {
            throw std::runtime_error("Unexpected Z3 error when attempting to convert model value to integer!");
        }
        kernelStrideRates[kernel] = num;
    }
    Z3_model_dec_ref(ctx, model);

    Z3_solver_dec_ref(ctx, solver);
    Z3_del_context(ctx);
}



}


#endif // DATAFLOW_ANALYSIS_HPP
