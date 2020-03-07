#ifndef TERMINATION_ANALYSIS_HPP
#define TERMINATION_ANALYSIS_HPP

#include "pipeline_analysis.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeTerminationGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::makeTerminationGraph() {

    mTerminationGraph = TerminationGraph(PartitionCount);

    // Although every kernel will eventually terminate, we only need to observe one kernel
    // in each partition terminating to know whether *all* of the kernels will terminate.
    // Since only the root of a partition could be a kernel that explicitly terminates,
    // we can "share" its termination state.

    const auto terminal = PartitionCount - 1;

    for (auto consumer = FirstKernel; consumer <= PipelineOutput; ++consumer) {
        const auto cid = KernelPartitionId[consumer];
        for (const auto e : make_iterator_range(in_edges(consumer, mBufferGraph))) {
            const auto buffer = source(e, mBufferGraph);
            const auto producer = parent(buffer, mBufferGraph);
            const auto pid = KernelPartitionId[producer];
            assert (pid <= cid);
            if (pid != cid) {
                add_edge(pid, cid, false, mTerminationGraph);
            }
        }
    }

    for (auto consumer = FirstCall; consumer <= LastCall; ++consumer) {
        for (const auto relationship : make_iterator_range(in_edges(consumer, mScalarGraph))) {
            const auto r = source(relationship, mScalarGraph);
            for (const auto producer : make_iterator_range(in_edges(r, mScalarGraph))) {
                const auto k = source(producer, mScalarGraph);
                assert ("cannot occur" && k != PipelineOutput);
                const auto pid = KernelPartitionId[k];
                add_edge(pid, terminal, false, mTerminationGraph);
            }
        }
    }

    auto any_termination = [](const Kernel * const kernel) {
        for (const Attribute & attr : kernel->getAttributes()) {
            switch (attr.getKind()) {
                case AttrId::CanTerminateEarly:
                case AttrId::MustExplicitlyTerminate:
                case AttrId::MayFatallyTerminate:
                case AttrId::SideEffecting:
                    return true;
                default: break;
            }
        }
        return false;
    };

    for (auto i = FirstKernel; i <= LastKernel; ++i) {
        if (any_termination(getKernel(i))) {
            const auto pid = KernelPartitionId[i];
            add_edge(pid, terminal, false, mTerminationGraph);
        }
    }

    transitive_reduction_dag(mTerminationGraph);

    // we are only interested in the incoming edges of the pipeline output
    for (unsigned i = 0; i < terminal; ++i) {
        clear_in_edges(i, mTerminationGraph);
    }

    // hard terminations
    auto hard_termination = [](const Kernel * const kernel) {
        for (const Attribute & attr : kernel->getAttributes()) {
            switch (attr.getKind()) {
                case AttrId::MayFatallyTerminate:
                    return true;
                default: break;
            }
        }
        return false;
    };

    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        if (hard_termination(getKernel(i))) {
            // in case we already have the edge in G, set the
            // hard termination flag to true after "adding" it.
            const auto pid = KernelPartitionId[i];
            mTerminationGraph[add_edge(pid, terminal, true, mTerminationGraph).first] = true;
        }
    }

    assert ("a pipeline with no sinks ought to produce no observable data"
            && in_degree(terminal, mTerminationGraph) > 0);
    assert ("termination graph construction error?"
            && out_degree(terminal, mTerminationGraph) == 0);

}

}

#endif // TERMINATION_ANALYSIS_HPP
