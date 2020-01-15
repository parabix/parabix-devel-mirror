#ifndef DATAFLOW_ANALYSIS_HPP
#define DATAFLOW_ANALYSIS_HPP

#include "../pipeline_compiler.hpp"
#include <boost/dynamic_bitset.hpp>
#include "rate_math.hpp"

namespace kernel {

struct PartitionData : public RefWrapper<BufferRateData> {
    unsigned                    KernelId;
    unsigned                    BufferId;
    Expr                        DataFlow;

    PartitionData() { }

    PartitionData(const unsigned kernelId, const unsigned bufferId, BufferRateData & rateData)
    : RefWrapper<BufferRateData>(rateData)
    , KernelId(kernelId)
    , BufferId(bufferId)
    , DataFlow(nullptr) {

    }
};


using PartitionGraph = adjacency_list<vecS, vecS, bidirectionalS, no_property, PartitionData>;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeParitionDataFlowRates
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::computeDataFlowRates(BufferGraph & G) const {


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

    auto bounded_variable = [&](Expr lb) {
        auto v = Z3_mk_fresh_const(ctx, nullptr, Z3_mk_int_sort(ctx));
        auto c1 = Z3_mk_ge(ctx, v, lb);
        Z3_solver_assert(ctx, solver, c1);
        return v;
    };

    auto multiply =[&](Expr X, Expr Y) {
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

    const auto firstKernel = out_degree(PipelineInput, G) == 0 ? FirstKernel : PipelineInput;
    const auto lastKernel = in_degree(PipelineOutput, G) == 0 ? LastKernel : PipelineOutput;

    std::vector<Expr> VarList(LastStreamSet + 1);
    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
        if (in_degree(kernel, G) == 0) {
            VarList[kernel] = ONE;
        } else {
            VarList[kernel] = bounded_variable(ONE);
        }
    }

    // https://github.com/Z3Prover/z3/blob/master/src/opt/maxsmt.cpp ?

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {

        const Kernel * const kernelObj = getKernel(kernel);
        const auto strideSize = kernelObj->getStride();

        auto kernelVar = VarList[kernel];

        bool hasGreedyRate = false;

        for (const auto input : make_iterator_range(in_edges(kernel, G))) {
            const auto buffer = source(input, G);
            const auto producer = parent(buffer, G);

            const BufferRateData & inputRate = G[input];
            const Binding & binding = inputRate.Binding;
            const ProcessingRate & rate = binding.getRate();

            const auto producedRate = VarList[buffer]; assert (producedRate);

            if (rate.isFixed() || rate.isGreedy()) {

                Expr strideRate = nullptr;
                if (LLVM_LIKELY(rate.isFixed())) {
                    strideRate = constant(rate.getRate() * strideSize);
                } else {
                    strideRate = producedRate;
                    hasGreedyRate = true;
                }
                const auto consumedRate = multiply(kernelVar, strideRate);
                Expr constraint = nullptr;
                if (KernelPartitionId[producer] == KernelPartitionId[kernel]) {
                    constraint = Z3_mk_eq(ctx, producedRate, consumedRate);
                } else {
                    constraint = Z3_mk_ge(ctx, producedRate, consumedRate);
                }
                Z3_solver_assert(ctx, solver, constraint);

//            } else if (LLVM_UNLIKELY(rate.isGreedy())) {
//                hasGreedyRate = true;
            } else {

                const auto s = Rational{strideSize, 2} * (rate.getLowerBound() + rate.getUpperBound());
                const auto strideRate = constant(s);
                const auto estConsumedRate = multiply(kernelVar, strideRate);
                const auto constraint = Z3_mk_ge(ctx, producedRate, estConsumedRate);
                Z3_solver_assert(ctx, solver, constraint);

            }
        }

        if (LLVM_UNLIKELY(hasGreedyRate)) {
            const auto constraint = Z3_mk_eq(ctx, kernelVar, ONE);
            Z3_solver_assert(ctx, solver, constraint);
        }

        for (const auto output : make_iterator_range(out_edges(kernel, G))) {
            const BufferRateData & outputRate = G[output];
            const Binding & binding = outputRate.Binding;
            const ProcessingRate & rate = binding.getRate();
            const auto buffer = target(output, G);
            const auto s = Rational{strideSize, 2} * (rate.getLowerBound() + rate.getUpperBound());
            const auto strideRate = constant(s);
            const auto estProducedRate = multiply(kernelVar, strideRate);
            VarList[buffer] = estProducedRate;
        }
    }

    if (LLVM_UNLIKELY(Z3_solver_check(ctx, solver) != Z3_L_TRUE)) {
        report_fatal_error("Unexpected error: unsolvable fixed-rate dataflow graph.");
    }

    Z3_model model = Z3_solver_get_model(ctx, solver);
    Z3_model_inc_ref(ctx, model);

    std::vector<unsigned> strideRateFactor(lastKernel + 1, 0);

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
        Expr const kernelVar = VarList[kernel];
        Z3_ast value;
        if (LLVM_UNLIKELY(Z3_model_eval(ctx, model, kernelVar, Z3_L_TRUE, &value) != Z3_L_TRUE)) {
            report_fatal_error("Unexpected Z3 error when attempting to obtain value from model!");
        }
        unsigned num;
        if (LLVM_UNLIKELY(Z3_get_numeral_uint(ctx, value, &num) != Z3_L_TRUE)) {
            report_fatal_error("Unexpected Z3 error when attempting to convert model value to integer!");
        }
        strideRateFactor[kernel] = num;
    }
    Z3_model_dec_ref(ctx, model);

    Z3_solver_dec_ref(ctx, solver);
    Z3_del_context(ctx);


    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
        const Kernel * const kernelObj = getKernel(kernel);
        errs() << kernel << ") " << kernelObj->getName() << " : " << strideRateFactor[kernel] << "\n";
    }

}

#if 0

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeParitionDataFlowRates
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::computeDataFlowRates(BufferGraph & G) const {

    const auto first = out_degree(PipelineInput, G) == 0 ? FirstKernel : PipelineInput;
    const auto last = in_degree(PipelineOutput, G) == 0 ? LastKernel : PipelineOutput;

    std::vector<unsigned> strideRateFactor(last + 1, 0);


    // Begin by determing how many strides each kernel within a partition must perform
    // in order to fully consume all produced data (with the assumption that any input
    // to the partition is sufficient for the needs of the partition.)

    for (auto nextKernel = first;;) {
        const auto firstKernel = nextKernel;
        const auto partitionId = KernelPartitionId[firstKernel];
        while (++nextKernel <= last) {
            if (KernelPartitionId[nextKernel] != partitionId) {
                break;
            }
        }
        const auto lastKernel = nextKernel - 1;
        computeStaticDataFlowRatesForParition(G, firstKernel, lastKernel, partitionId, strideRateFactor);
        if (lastKernel == last) { // Exit after the last kernel
            break;
        }
    }

    // In the next phase, we want to simulate a demand-driven schedule. The goal is to
    // determine how much input data is required to invoke each kernel. The difficulty is
    // that cross-partition rates will require [n,m] items, which when n=0 implies that
    // we should always execute a partition.

    // However, we only invoke a kernel with a Bounded rate when the buffer has at least
    // m items. If we assume that we'll want [(STRIDES - 1) * (n + m) / 2] + m to progress
    // efficiently, this gives us a more useful lower bound.


    for (auto priorKernel = last;;) {
        const auto lastKernel = priorKernel;
        const auto partitionId = KernelPartitionId[lastKernel];
        while (--priorKernel >= first) {
            if (KernelPartitionId[priorKernel] != partitionId) {
                break;
            }
        }
        const auto firstKernel = priorKernel + 1;
        computeDynamicDataFlowRatesForParition(G, firstKernel, lastKernel, partitionId, strideRateFactor);
        if (firstKernel == first) {
            break;
        }
    }

}
#endif
#if 0

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief summarizePartition
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::summarizePartition(BufferGraph & G,
                                          const unsigned firstKernel,
                                          const unsigned lastKernel,
                                          const unsigned partitionId,
                                          PartitionGraph & P) const {

    for (const auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {

        for (const auto input : make_iterator_range(in_edges(kernel, G))) {
            const auto buffer = source(input, G);
            const auto producer = parent(buffer, G);
            const auto producerPartitionId = KernelPartitionId[producer];
            if (producerPartitionId != partitionId) {
                add_edge(partitionId, producerPartitionId, PartitionData{kernel, buffer, G[input]}, P);
            }
        }

        for (const auto output : make_iterator_range(out_edges(kernel, G))) {
            const auto buffer = target(output, G);
            for (const auto input : make_iterator_range(out_edges(buffer, G))) {
                const auto consumer = target(input, G);
                const auto consumerPartitionId = KernelPartitionId[consumer];
                if (consumerPartitionId != partitionId) {
                    add_edge(consumerPartitionId, partitionId, PartitionData{kernel, buffer, G[output]}, P);
                }
            }
        }

    }

}

#endif

#if 0

BufferRateData & getRateDataForPortNum(const unsigned kernel, const unsigned portNum, BufferGraph & G) {
    const auto numOfInputs = in_degree(kernel, G);
    BufferGraph::edge_descriptor e;
    if (portNum < numOfInputs) { // Input
        graph_traits<BufferGraph>::in_edge_iterator ei, ei_end;
        std::tie(ei, ei_end) = in_edges(kernel, G);
        e = *(ei + portNum);
    } else { // Output
        graph_traits<BufferGraph>::out_edge_iterator ei, ei_end;
        std::tie(ei, ei_end) = out_edges(kernel, G);
        e = *(ei + (portNum - numOfInputs));
    }
    BufferRateData & result = G[e];
    assert (result.Port ==
            StreamSetPort{
                portNum < numOfInputs ? PortType::Input : PortType::Output
                , portNum < numOfInputs ? portNum : portNum - numOfInputs });
    return result;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief identifySymbolicRates
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::identifySymbolicRates(BufferGraph & G) const {

    // Identify the symbolic rates of each kernel, where Fixed rates inherit the
    // symbolic rate of the kernel, popcount/partial sum/relative rates share the
    // same rate when referencing the same source, and bounded/unknown rates are
    // given a new rate. When a kernel has inputs whose rate differ, the kernel is
    // assigned a new rate otherwise the kernel inherits the rate of its inputs.

    using ReferenceGraph = adjacency_list<vecS, vecS, bidirectionalS>;

    using CommonPartialSumRateMap = flat_map<std::pair<unsigned, unsigned>, Expr>;

    CommonPartialSumRateMap P;


    const auto first = out_degree(PipelineInput, G) == 0 ? FirstKernel : PipelineInput;
    const auto last = in_degree(PipelineOutput, G) == 0 ? LastKernel : PipelineOutput;


    // With the assumption that we have an unbounded input from any sources and infinite length buffers,
    // calculate the upper/lower bound for the number of items/strides we can process in a single segment.

    for (auto kernel = first; kernel <= last; ++kernel) {

        // Determine whether we have any relative or partialsum bindings.
        // Relative rates will be temporarily ignored and then filled in
        // after we determine the reference rates.


        const auto numOfInputs = in_degree(kernel, G);
        const auto numOfOutputs = out_degree(kernel, G);
        const Kernel * const kernelObj = getKernel(kernel);
        const auto strideSize = kernelObj->getStride();

        auto toPortIndex = [&](const StreamSetPort port) {
            if (port.Type == PortType::Input) {
                return port.Number;
            } else {
                return port.Number + numOfInputs;
            }
        };

        SmallVector<Expr, 32> symbolicRates(numOfInputs + numOfOutputs);

        auto calculateSymbolicRate = [&](const BufferRateData & ioRate) -> Expr {

            const Binding & binding = ioRate.Binding;
            const ProcessingRate & rate = binding.getRate();

            using RateId = ProcessingRate::KindId;

            Expr symRate = nullptr;

            // calculate base rate

            switch (rate.getKind()) {
                case RateId::Fixed:
                    symRate = constant(rate.getRate() * strideSize);
                    break;
                case RateId::Bounded:
                    symRate = bounded_variable(rate.getLowerBound() * strideSize, rate.getUpperBound() * strideSize);
                    break;
                case RateId::PartialSum:
                    BEGIN_SCOPED_REGION
                    const auto ref = getReferenceBufferVertex(kernel, ioRate.Port);
                    const auto key = std::make_pair(ref, refSymRate);
                    const auto f = P.find(key);
                    if (LLVM_LIKELY(f == P.end())) {
                        Expr const popCountRate = bounded_variable(constant(0), refSymRate);
                        P.emplace(key, popCountRate);
                        symRate = popCountRate;
                    } else {
                        symRate = f->second;
                    }
                    END_SCOPED_REGION
                    break;
                case RateId::Greedy:
                    assert (refSymRate);
                    symRate = refSymRate;
                    break;
                case RateId::Relative:
                    BEGIN_SCOPED_REGION
                    const StreamSetPort ref = getReference(ioRate.Port);
                    symRate = symbolicRates[toPortIndex(ref)];
                    END_SCOPED_REGION
                case RateId::Unknown:
                    symRate = free_variable();
                    break;
                default: llvm_unreachable("invalid or unhandled rate type");
            }

            // apply rate attributes

            bool isDeferred = false;
            unsigned sub_lb = 0;

            for (const Attribute & attr : binding.getAttributes()) {
                switch (attr.getKind()) {
                    case AttrId::Deferred:
                        // TODO: check if its necessary to flag this. if the lb is already
                        // zero then this is unnecessary.
                        break;
                    case AttrId::Delayed:
                        sub_lb += attr.amount();
                        break;
                    case AttrId::LookAhead:
                        sub_lb += attr.amount();
                        break;
                    default: break;
                }
            }

            if (LLVM_UNLIKELY(isDeferred || sub_lb)) {
                Expr lb = symRate;
                Expr ub = symRate;
                if (isDeferred) {
                    lb = constant(0);
                }
                if (sub_lb) {
                    lb = subtract(symRate, constant(sub_lb));
                }
                symRate = bounded_variable(lb, ub);
            }

            return symRate;
        };

        // Any kernel without an input must be a source kernel
        if (LLVM_UNLIKELY(numOfInputs == 0)) {

            for (const auto e : make_iterator_range(out_edges(kernel, G))) {
                const BufferRateData & outputRate = G[e];
                // is this a relative rate?
                if (LLVM_UNLIKELY(out_degree(outputRate.Port.Number, R) != 0)) {
                    continue;
                }
                outputRate.SymbolicRate = calculateSymbolicRate(outputRate, nullptr);
            }

        } else { // non-source kernel

            /* mPortEvaluationOrder = */ determineEvaluationOrderOfKernelIO(kernel, G);

            // Identify the input rates

            ExprSet dataFlowRates;

            for (const auto i : mPortEvaluationOrder) {
                if (i < numOfInputs) {

                } else {

                }
            }


            for (const auto input : make_iterator_range(in_edges(kernel, G))) {
                BufferRateData & inputRate = G[input];
                // is this a relative rate?
                if (LLVM_UNLIKELY(out_degree(inputRate.Port.Number, R) != 0)) {
                    continue;
                }

                const auto output = in_edge(source(input, G), G);
                assert (out_degree(inputRate.Port.Number, R) == 0);
                const BufferRateData & outputRate = G[output];
                auto sr = calculateSymbolicRate(inputRate, outputRate.SymbolicRate);
                inputRate.SymbolicRate = sr;
                symbolicRates[inputRate.Port.Number] = sr;

                // check if we already saw that num/denom pair?

                Expr relativeDataFlow = divide(outputRate.SymbolicRate, sr);
                dataFlowRates.insert(relativeDataFlow);
            }

            // Characterize the expected number of strides
            auto expected = bounded_variable(mk_min(dataFlowRates), mk_max(dataFlowRates));

            for (const auto e : make_iterator_range(out_edges(kernel, G))) {
                const BufferRateData & outputRate = G[e];
                // is this a relative rate?
                if (LLVM_UNLIKELY(out_degree(outputRate.Port.Number, R) != 0)) {
                    continue;
                }

                auto symrate = calculateSymbolicRate(outputRate, outputRate);
                outputRate.SymbolicRate = symrate;
            }



        }



        // Correct the input rate to be relative to the newly computed symbolic rate.
        for (const auto input : make_iterator_range(in_edges(kernel, G))) {
            BufferRateData & inputRate = G[input];
            const Binding & binding = inputRate.Binding;
            const ProcessingRate & rate = binding.getRate();
            if (LLVM_LIKELY(rate.isFixed())) {
                inputRate.SymbolicRate = symbolicRate;
            } else if (LLVM_UNLIKELY(rate.isPartialSum())) {
                inputRate.SymbolicRate = getReferenceSymbolicRate(inputRate, symbolicRate);
            }
        }

        // Determine the output rates
        for (const auto e : make_iterator_range(out_edges(kernel, G))) {
            BufferRateData & outputRate = G[e];
            // is this a relative rate?
            if (LLVM_UNLIKELY(out_degree(outputRate.Port.Number + numOfInputs, R) != 0)) {
                continue;
            }
            const Binding & binding = outputRate.Binding;
            const ProcessingRate & rate = binding.getRate();
            if (LLVM_LIKELY(rate.isFixed())) {
                outputRate.SymbolicRate = symbolicRate;
            } else if (LLVM_UNLIKELY(rate.isPartialSum())) {
                outputRate.SymbolicRate = getReferenceSymbolicRate(outputRate, symbolicRate);
            } else { // if (rate.isBounded() || rate.isUnknown()) {
                outputRate.SymbolicRate = add_vertex(M);
                rateIsBoundedBy(outputRate.SymbolicRate, symbolicRate);
            }
        }


    }
}

#endif

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeStaticDataFlowRatesForParition
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::computeStaticDataFlowRatesForParition(BufferGraph & G,
                                                             const unsigned firstKernel,
                                                             const unsigned lastKernel,
                                                             const unsigned partitionId,
                                                             std::vector<unsigned> & strideRateFactor) const {

    // When considering the dataflow rates of a single partition, we ignore any inputs / outputs that cross
    // a partition boundary. The reason is we want to determine --- given the optimal input -- how many
    // strides of each kernel within the partition will occur. From there, we can decide how much data is
    // required to cross the partition invocation threshold.

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

    // For Fixed/Greedy rates:
    //    VAR(PRODUCER(c)) * PRODUCED(c) = VAR(CONSUMER(c)) * CONSUMED(c)

    // For variable rates:
    //    VAR(PRODUCER(c)) * PRODUCED(c) <= VAR(CONSUMER(c)) * CONSUMED(c)
    //  Where for a given rate [n,m]:
    //    PRODUCED(c) := (STRIDES) * (n + m) / 2
    //    CONSUMED(c) := [(STRIDES - 1) * (n + m) / 2] + m



    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {

        const Kernel * const kernelObj = getKernel(kernel);
        const auto strideSize = kernelObj->getStride();

        Expr kernelVar;
        const auto S = Z3_mk_int_sort(ctx);
        kernelVar = Z3_mk_fresh_const(ctx, nullptr, S);
        // we only care about positive values
        const auto c1 = Z3_mk_ge(ctx, kernelVar, ONE);
        Z3_solver_assert(ctx, solver, c1);
        B.emplace(kernel, kernelVar);

        for (const auto input : make_iterator_range(in_edges(kernel, G))) {
            const auto buffer = source(input, G);
            const auto producer = parent(buffer, G);
            if (KernelPartitionId[producer] == partitionId) {

                // TODO: should this consider Fixed rates from other partitions?

                Expr const producedRate = getVar(buffer); assert (producedRate);

                const BufferRateData & inputRate = G[input];
                const Binding & binding = inputRate.Binding;
                const ProcessingRate & rate = binding.getRate();

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
                B.emplace(buffer, producedRate);
            }
        }
    }

    if (LLVM_UNLIKELY(Z3_solver_check(ctx, solver) != Z3_L_TRUE)) {
        report_fatal_error("Unexpected error: unsolvable fixed-rate dataflow graph.");
    }

    Z3_model model = Z3_solver_get_model(ctx, solver);
    Z3_model_inc_ref(ctx, model);
    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
        Expr const kernelVar = getVar(kernel);
        Z3_ast value;
        if (LLVM_UNLIKELY(Z3_model_eval(ctx, model, kernelVar, Z3_L_TRUE, &value) != Z3_L_TRUE)) {
            report_fatal_error("Unexpected Z3 error when attempting to obtain value from model!");
        }
        unsigned num;
        if (LLVM_UNLIKELY(Z3_get_numeral_uint(ctx, value, &num) != Z3_L_TRUE)) {
            report_fatal_error("Unexpected Z3 error when attempting to convert model value to integer!");
        }
        strideRateFactor[kernel] = num;
    }
    Z3_model_dec_ref(ctx, model);

    Z3_solver_dec_ref(ctx, solver);
    Z3_del_context(ctx);
}

#if 0

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeDynamicDataFlowRatesForParition
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::computeDynamicDataFlowRatesForParition(const BufferGraph & G,
                                                              const unsigned firstKernel,
                                                              const unsigned lastKernel,
                                                              const unsigned partitionId,
                                                              std::vector<unsigned> & strideRateFactor) const {

    using CommonPartialSumRateMap = flat_map<std::pair<unsigned, unsigned>, Expr>;
    using BufferRateExprMap = flat_map<unsigned, Expr>;

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

    auto bounded_variable = [&](Rational lb) {
        auto v = Z3_mk_fresh_const(ctx, nullptr, Z3_mk_real_sort(ctx));
        auto c1 = Z3_mk_ge(ctx, v, constant(lb));
        Z3_solver_assert(ctx, solver, c1);
        return v;
    };




    auto multiply = [&](Expr X, Expr Y) {
        Z3_ast args[2] = { X, Y };
        return Z3_mk_mul(ctx, 2, args);
    };

    CommonPartialSumRateMap P;

    BufferRateExprMap B;

    auto getOrMakeVar = [&B](const unsigned vertex) {
        const auto f = B.find(vertex);
        if (f == B.end()) {
            auto var = Z3_mk_fresh_const(ctx, nullptr, Z3_mk_real_sort(ctx));
            B.emplace(vertex, var);
            return var;
        }
        return f->second;
    };

    for (const auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {

        const Kernel * const kernelObj = getKernel(kernel);
        const auto strideSize = kernelObj->getStride();
        const auto strideCount = strideRateFactor[kernel];

        for (const auto input : make_iterator_range(in_edges(kernel, G))) {
            const auto buffer = source(input, G);
            const auto output = in_edge(buffer, G);
            const auto producer = source(output, G);

            if (KernelPartitionId[producer] != partitionId) {

                const BufferRateData & inputRate = G[input];
                const Binding & binding = inputRate.Binding;
                const ProcessingRate & rate = binding.getRate();
                const auto bufferVar = getOrMakeVar(buffer);

                // Assume for rate [n,m] we'll consume [(STRIDES - 1) * (n + m) / 2] + m items

                const Rational F{strideCount - 1, 2};
                const auto M = F * (rate.getLowerBound() + rate.getUpperBound());
                auto C = (M + rate.getUpperBound()) * strideSize;

                for (const Attribute & attr : binding.getAttributes()) {
                    switch (attr.getKind()) {
                        case AttrId::Delayed:
                        case AttrId::LookAhead:
                            C += attr.amount();
                            break;
                        default: break;
                    }
                }

                const auto c1 = Z3_mk_ge(ctx, bufferVar, constant(C));
                Z3_solver_assert(ctx, solver, c1);







            }
        }


    }

}

#endif

}


#endif // DATAFLOW_ANALYSIS_HPP
