#ifndef TERMINATION_ANALYSIS_HPP
#define TERMINATION_ANALYSIS_HPP

#include "pipeline_analysis.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief identifyTerminationChecks
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::identifyTerminationChecks() {

    using TerminationGraph = adjacency_list<hash_setS, vecS, bidirectionalS>;

    TerminationGraph G(PartitionCount);

    // Although every kernel will eventually terminate, we only need to observe one kernel
    // in each partition terminating to know whether *all* of the kernels will terminate.
    // Since only the root of a partition could be a kernel that explicitly terminates,
    // we can "share" its termination state.

    const auto terminal = PartitionCount - 1U;

    for (auto consumer = FirstKernel; consumer <= PipelineOutput; ++consumer) {
        const auto cid = KernelPartitionId[consumer];
        for (const auto e : make_iterator_range(in_edges(consumer, mBufferGraph))) {
            const auto buffer = source(e, mBufferGraph);
            const auto producer = parent(buffer, mBufferGraph);
            const auto pid = KernelPartitionId[producer];
            assert (pid <= cid);
            if (pid != cid) {
                add_edge(pid, cid, G);
            }
        }
    }

    for (auto i = FirstKernel; i <= LastKernel; ++i) {
        const Kernel * const kernelObj = getKernel(i); assert (kernelObj);
        if (LLVM_UNLIKELY(kernelObj->hasAttribute(AttrId::SideEffecting))) {
            const auto pid = KernelPartitionId[i];
            add_edge(pid, terminal, G);
        }
    }

    assert (FirstCall == (PipelineOutput + 1U));

    for (auto consumer = PipelineOutput; consumer <= LastCall; ++consumer) {
        for (const auto relationship : make_iterator_range(in_edges(consumer, mScalarGraph))) {
            const auto scalar = source(relationship, mScalarGraph);
            for (const auto production : make_iterator_range(in_edges(scalar, mScalarGraph))) {
                const auto producer = source(production, mScalarGraph);
                const auto partitionId = KernelPartitionId[producer];
                assert ("cannot occur" && partitionId != terminal);
                add_edge(partitionId, terminal, G);
            }
        }
    }

    assert ("pipeline has no observable outputs?" && (in_degree(terminal, G) > 0));

    transitive_reduction_dag(G);

    mTerminationCheck.resize(PartitionCount, 0U);

    // we are only interested in the incoming edges of the pipeline output
    for (const auto e : make_iterator_range(in_edges(terminal, G))) {
        mTerminationCheck[source(e, G)] = TerminationCheckFlag::Soft;
    }

    // hard terminations
    for (auto i = FirstKernel; i <= LastKernel; ++i) {
        const Kernel * const kernelObj = getKernel(i);
        if (LLVM_UNLIKELY(kernelObj->hasAttribute(AttrId::MayFatallyTerminate))) {
            mTerminationCheck[KernelPartitionId[i]] |= TerminationCheckFlag::Hard;
        }
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeTerminationPropagationGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::makeTerminationPropagationGraph() {

    // When a partition terminates, we want to inform any kernels that supply information to it that
    // one of their consumers has finished processing data. In a pipeline with a single output, this
    // isn't necessary but if a pipeline has multiple outputs, we could end up needlessly producing
    // data that will never be consumed.

    mTerminationPropagationGraph = TerminationPropagationGraph(PartitionCount);

    unsigned outputs = 0;
    for (unsigned i = 0; i < PartitionCount; ++i) {
        if (mTerminationCheck[i] & TerminationCheckFlag::Soft) {
            ++outputs;
        }
    }

    if (outputs < 2) {
        return;
    }

    BitVector inputs(PartitionCount);

    for (auto start = FirstKernel; start <= PipelineOutput; ) {
        const auto pid = KernelPartitionId[start];
        auto end = start + 1;
        for (; end <= PipelineOutput; ++end) {
            if (pid != KernelPartitionId[end]) {
                break;
            }
        }

        for (auto i = start; i < end; ++i) {
            for (const auto e : make_iterator_range(in_edges(i, mBufferGraph))) {
                const auto streamSet = source(e, mBufferGraph);
                const auto producer = parent(streamSet, mBufferGraph);
                const auto partitionId = KernelPartitionId[producer];
                inputs.set(partitionId);
            }
        }
        inputs.reset(pid);
        for (const auto i : inputs.set_bits()) {
            add_edge(pid, i, mTerminationPropagationGraph);
        }
        inputs.reset();

        start = end;
    }

    transitive_reduction_dag(mTerminationPropagationGraph);

    for (auto end = LastKernel; end >= FirstKernel; ) {
        const auto pid = KernelPartitionId[end];
        auto start = end;
        for (; start > FirstKernel; --start) {
            if (pid != KernelPartitionId[start - 1U]) {
                break;
            }
        }



        const Kernel * const kernelObj = getKernel(start);

        auto prior = pid;
        if (kernelObj->canSetTerminateSignal()) {
            auto fork = pid;
            for (;;) {
                const auto n = out_degree(fork, mTerminationPropagationGraph);
                if (n != 1) {
                    break;
                }
                prior = fork;
                fork = child(fork, mTerminationPropagationGraph);
            }
        }

        clear_out_edges(pid, mTerminationPropagationGraph);
        if (prior != pid) {
            add_edge(pid, prior, mTerminationPropagationGraph);
        }

        end = start - 1U;
    }

    printGraph(mTerminationPropagationGraph, errs(), "TP");

}

}

#endif // TERMINATION_ANALYSIS_HPP
