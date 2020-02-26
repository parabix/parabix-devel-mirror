#ifndef TERMINATION_ANALYSIS_HPP
#define TERMINATION_ANALYSIS_HPP

#include "../pipeline_compiler.hpp"

namespace kernel {


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeTerminationGraph
 *
 * The termination graph is a minimum vertex-disjoint path cover of the transitive reduction of I/O dependencies.
 * It "kernel path" models which kernels to are controlled by a specific termination "counter" and the incoming
 * edges to the P_{out} vertex indicates which counters must be checked by the pipeline to determine whether the
 * work has finished.
 ** ------------------------------------------------------------------------------------------------------------- */
TerminationGraph PipelineCompiler::makeTerminationGraph() const {

    TerminationGraph G(PipelineOutput + 1);

    for (auto consumer = FirstKernel; consumer <= PipelineOutput; ++consumer) {
        for (const auto e : make_iterator_range(in_edges(consumer, mBufferGraph))) {
            const auto buffer = source(e, mBufferGraph);
            const auto producer = parent(buffer, mBufferGraph);
            add_edge(producer, consumer, false, G);
        }
    }

    for (auto consumer = PipelineOutput; consumer <= LastCall; ++consumer) {
        for (const auto relationship : make_iterator_range(in_edges(consumer, mScalarGraph))) {
            const auto r = source(relationship, mScalarGraph);
            for (const auto producer : make_iterator_range(in_edges(r, mScalarGraph))) {
                const auto k = source(producer, mScalarGraph);
                assert ("cannot occur" && k != PipelineOutput);
                add_edge(k, PipelineOutput, false, G);
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

    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        if (any_termination(getKernel(i))) {
            add_edge(i, PipelineOutput, false, G);
        }
    }

    transitive_reduction_dag(G);

    clear_out_edges(PipelineInput, G);

    // we are only interested in the incoming edges of the pipeline output
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
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
            // incase we already have the edge in G, set the
            // hard termination flag to true after "adding" it.
            G[add_edge(i, PipelineOutput, true, G).first] = true;
        }
    }

    assert ("a pipeline with no sinks ought to produce no observable data"
            && in_degree(PipelineOutput, G) > 0);
    assert ("termination graph construction error?"
            && out_degree(PipelineOutput, G) == 0);

    return G;
}

}

#endif // TERMINATION_ANALYSIS_HPP
