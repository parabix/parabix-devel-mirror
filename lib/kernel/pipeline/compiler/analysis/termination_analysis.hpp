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
            const auto producer = parent(scalar, mScalarGraph);
            const auto partitionId = KernelPartitionId[producer];
            assert ("cannot occur" && partitionId != terminal);
            add_edge(partitionId, terminal, G);
        }
    }

    assert ("pipeline has no observable outputs?" && (in_degree(terminal, G) > 0));

    printGraph(G, errs(), "T1");

    transitive_reduction_dag(G);

    printGraph(G, errs(), "T2");

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

}

#endif // TERMINATION_ANALYSIS_HPP
