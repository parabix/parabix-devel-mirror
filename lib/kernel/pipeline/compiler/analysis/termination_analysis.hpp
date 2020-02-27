#ifndef TERMINATION_ANALYSIS_HPP
#define TERMINATION_ANALYSIS_HPP

#include "../pipeline_compiler.hpp"

namespace kernel {


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeTerminationGraph
 ** ------------------------------------------------------------------------------------------------------------- */
TerminationGraph PipelineCompiler::makeTerminationGraph() const {

    TerminationGraph G(PartitionCount + 1);

    // Although every kernel will eventually terminate, we only need to observe one kernel
    // in each partition terminating to know whether *all* of the kernels will terminate.
    // Since only the root of a partition could be a kernel that explicitly terminates,
    // we can "share" its termination state.

    for (auto consumer = FirstKernel; consumer <= PipelineOutput; ++consumer) {
        const auto cid = KernelPartitionId[consumer];
        for (const auto e : make_iterator_range(in_edges(consumer, mBufferGraph))) {
            const auto buffer = source(e, mBufferGraph);
            const auto producer = parent(buffer, mBufferGraph);
            const auto pid = KernelPartitionId[producer];
            assert (pid <= cid);
            if (pid != cid) {
                add_edge(pid, cid, false, G);
            }
        }
    }

    for (auto consumer = PipelineOutput; consumer <= LastCall; ++consumer) {
        for (const auto relationship : make_iterator_range(in_edges(consumer, mScalarGraph))) {
            const auto r = source(relationship, mScalarGraph);
            for (const auto producer : make_iterator_range(in_edges(r, mScalarGraph))) {
                const auto k = source(producer, mScalarGraph);
                assert ("cannot occur" && k != PipelineOutput);
                const auto pid = KernelPartitionId[k];
                add_edge(pid, PartitionCount, false, G);
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
            add_edge(pid, PartitionCount, false, G);
        }
    }

    transitive_reduction_dag(G);

    // we are only interested in the incoming edges of the pipeline output
    for (unsigned i = 0; i < PartitionCount; ++i) {
        clear_in_edges(i, G);
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
            G[add_edge(pid, PartitionCount, true, G).first] = true;
        }
    }

    printGraph(G, errs(), "T");

    assert ("a pipeline with no sinks ought to produce no observable data"
            && in_degree(PartitionCount, G) > 0);
    assert ("termination graph construction error?"
            && out_degree(PartitionCount, G) == 0);

    return G;
}

}

#endif // TERMINATION_ANALYSIS_HPP
