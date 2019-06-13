#ifndef TERMINATION_LOGIC_HPP
#define TERMINATION_LOGIC_HPP

#include "pipeline_compiler.hpp"
#include <boost/graph/topological_sort.hpp>
#include <boost/graph/edmonds_karp_max_flow.hpp>

#define DISABLE_TERMINATION_SIGNAL_COUNTING_VARIABLES

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeTerminationGraph
 *
 * The termination graph is a minimum vertex-disjoint path cover of the transitive reduction of I/O dependencies.
 * It "kernel path" models which kernels to are controlled by a specific termination "counter" and the incoming
 * edges to the P_{out} vertex indicates which counters must be checked by the pipeline to determine whether the
 * work has finished.
 ** ------------------------------------------------------------------------------------------------------------- */
TerminationGraph PipelineCompiler::makeTerminationGraph() {
    using Edge = TerminationGraph::edge_descriptor;

    TerminationGraph G(PipelineOutput + 1);

    // 1) copy and summarize producer -> consumer relations from the buffer graph
    for (unsigned consumer = FirstKernel; consumer <= PipelineOutput; ++consumer) {
        for (const auto & e : make_iterator_range(in_edges(consumer, mBufferGraph))) {
            const auto buffer = source(e, mBufferGraph);
            const auto producer = parent(buffer, mBufferGraph);
            const BufferRateData & rd = mBufferGraph[e];
            const auto mayConsumeNoInput = (rd.Minimum.numerator() == 0) && (consumer != PipelineOutput);
            if (LLVM_UNLIKELY(mayConsumeNoInput)) {
                continue;
            }
            add_edge(producer, consumer, G);
        }
    }
    clear_out_edges(PipelineInput, G);

    // 2) copy and summarize any input scalars to internal calls or output scalars of the pipeline
    assert (FirstCall == PipelineOutput + 1);
    for (unsigned consumer = PipelineOutput; consumer <= LastCall; ++consumer) {
        for (const auto & relationship : make_iterator_range(in_edges(consumer, mScalarGraph))) {
            const auto r = source(relationship, mScalarGraph);
            for (const auto & producer : make_iterator_range(in_edges(r, mScalarGraph))) {
                const auto k = source(producer, mScalarGraph);
                assert ("cannot occur" && k != PipelineOutput);
                add_edge(k, PipelineOutput, G);
            }
        }
    }

    // 3) remove any incoming edges to a kernel that must explicitly terminate;
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        const Kernel * const kernel = getKernel(i);
        if (LLVM_UNLIKELY(kernel->hasAttribute(AttrId::SideEffecting))) {
            add_edge(i, PipelineOutput, G);
        }
    }

    transitive_reduction_dag(G);

    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        remove_out_edge_if(i, [&](Edge e) { return target(e, G) != PipelineOutput; }, G);
    }

    mTerminationSignals.resize(LastKernel + 1, nullptr);

    assert ("a pipeline with no sinks ought to produce no observable data"
            && in_degree(PipelineOutput, G) > 0);
    assert ("termination graph construction error?"
            && out_degree(PipelineOutput, G) == 0);

    return G;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addTerminationProperties
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addTerminationProperties(BuilderRef b, const unsigned kernel) {
    mPipelineKernel->addInternalScalar(b->getSizeTy(), TERMINATION_PREFIX + std::to_string(kernel));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initiallyTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::initiallyTerminated(BuilderRef b) {
    b->setKernel(mPipelineKernel);
    Value * const ptr = b->getScalarFieldPtr(TERMINATION_PREFIX + std::to_string(mKernelIndex));
    b->setKernel(mKernel);
    Value * signal = b->CreateLoad(ptr, true);
    mTerminationSignals[mKernelIndex] = signal;
    mTerminatedInitially = signal;
    return hasKernelTerminated(b, mKernelIndex);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief hasKernelTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::hasKernelTerminated(BuilderRef b, const unsigned kernel, const bool normally) const {
    // any pipeline input streams are considered produced by the P_{in} vertex.
    if (LLVM_UNLIKELY(kernel == PipelineInput)) {
        return mPipelineKernel->isFinal();
    } else {
        Value * const terminated = mTerminationSignals[kernel];
        if (normally) {
            Constant * terminatedButNotAborted = getTerminationSignal(b, TerminationSignal::Terminated);
            return b->CreateICmpEQ(terminated, terminatedButNotAborted);
        } else {
            Constant * notTerminated = getTerminationSignal(b, TerminationSignal::None);
            return b->CreateICmpNE(terminated, notTerminated);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getTerminationSignal
 ** ------------------------------------------------------------------------------------------------------------- */
Constant * PipelineCompiler::getTerminationSignal(BuilderRef b, const TerminationSignal type) {
    return b->getSize(static_cast<unsigned>(type));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief signalAbnormalTermination
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::signalAbnormalTermination(BuilderRef b) {
    const Kernel * const kernel = getKernel(mKernelIndex);
    TerminationSignal signal = TerminationSignal::Terminated;
    if (kernel->hasAttribute(AttrId::CanTerminateEarly)) {
        signal = TerminationSignal::Aborted;
    }
    Constant * const aborted = getTerminationSignal(b, signal);
    mTerminatedSignalPhi->addIncoming(aborted, b->GetInsertBlock());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isClosed
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::isClosed(BuilderRef b, const unsigned inputPort) {
    const auto buffer = getInputBufferVertex(inputPort);
    const auto producer = parent(buffer, mBufferGraph);
    return hasKernelTerminated(b, producer, false);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isClosedNormally
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::isClosedNormally(BuilderRef b, const unsigned inputPort) {
    const auto buffer = getInputBufferVertex(inputPort);
    const auto producer = parent(buffer, mBufferGraph);
    const Kernel * const kernel = getKernel(mKernelIndex);
    return hasKernelTerminated(b, producer, kernel->hasAttribute(AttrId::CanTerminateEarly));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updateTerminationSignal
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::updateTerminationSignal(Value * const signal) {
    mTerminationSignals[mKernelIndex] = signal;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::setTerminated(BuilderRef b, Value * const signal) {
    b->setKernel(mPipelineKernel);
    Value * const ptr = b->getScalarFieldPtr(TERMINATION_PREFIX + std::to_string(mKernelIndex));
    b->CreateStore(signal, ptr, true);
    b->setKernel(mKernel);
}

}

#endif // TERMINATION_LOGIC_HPP
