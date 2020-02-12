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

    TerminationGraph G(PipelineOutput + 1);

    for (unsigned consumer = FirstKernel; consumer <= PipelineOutput; ++consumer) {
        for (const auto e : make_iterator_range(in_edges(consumer, mBufferGraph))) {
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

    for (unsigned consumer = PipelineOutput; consumer <= LastCall; ++consumer) {
        for (const auto relationship : make_iterator_range(in_edges(consumer, mScalarGraph))) {
            const auto r = source(relationship, mScalarGraph);
            for (const auto producer : make_iterator_range(in_edges(r, mScalarGraph))) {
                const auto k = source(producer, mScalarGraph);
                assert ("cannot occur" && k != PipelineOutput);
                add_edge(k, PipelineOutput, G);
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
inline void PipelineCompiler::addTerminationProperties(BuilderRef b, const size_t kernel) {
    mTarget->addInternalScalar(b->getSizeTy(), TERMINATION_PREFIX + std::to_string(kernel));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief hasKernelTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::hasKernelTerminated(BuilderRef b, const size_t kernel, const bool normally) const {
    // any pipeline input streams are considered produced by the P_{in} vertex.
    if (LLVM_UNLIKELY(kernel == PipelineInput)) {
        return isFinal();
    } else {
        assert (kernel != PipelineOutput);
        Value * const terminated = mTerminationSignals[kernel];
        if (normally) {
            Constant * const completed = getTerminationSignal(b, TerminationSignal::Completed);
            return b->CreateICmpEQ(terminated, completed);
        } else {
            Constant * const unterminated = getTerminationSignal(b, TerminationSignal::None);
            return b->CreateICmpNE(terminated, unterminated);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief pipelineTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::hasPipelineTerminated(BuilderRef b) const {

    Value * hard = mIsFinal;
    Value * soft = b->getTrue();

    Constant * const unterminated = getTerminationSignal(b, TerminationSignal::None);
    Constant * const aborted = getTerminationSignal(b, TerminationSignal::Aborted);
    Constant * const fatal = getTerminationSignal(b, TerminationSignal::Fatal);

    // check whether every sink has terminated
    for (const auto e : make_iterator_range(in_edges(PipelineOutput, mTerminationGraph))) {
        const auto kernel = source(e, mTerminationGraph);
        Value * const signal = mTerminationSignals[kernel];
        assert (signal);
        assert (signal->getType() == unterminated->getType());
        // if this is a hard termination, such as a fatal error, any can terminate the pipeline.
        // however, a kernel that can terminate with a fatal error, may not necessarily do so.
        // otherwise its a soft termination and all must agree that the pipeline has terminated
        if (mTerminationGraph[e]) {
            hard = b->CreateOr(hard, b->CreateICmpEQ(signal, fatal));
        }
        soft = b->CreateAnd(soft, b->CreateICmpNE(signal, unterminated));
    }

    return b->CreateSelect(hard, fatal, b->CreateSelect(soft, aborted, unterminated));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getTerminationSignal
 ** ------------------------------------------------------------------------------------------------------------- */
/* static */ Constant * PipelineCompiler::getTerminationSignal(BuilderRef b, const TerminationSignal type) {
    return b->getSize(static_cast<unsigned>(type));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief signalAbnormalTermination
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::signalAbnormalTermination(BuilderRef b) {
    Constant * const aborted = getTerminationSignal(b, TerminationSignal::Aborted);
    mTerminatedSignalPhi->addIncoming(aborted, b->GetInsertBlock());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isClosed
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::isClosed(BuilderRef b, const StreamSetPort inputPort) {
    const auto buffer = getInputBufferVertex(inputPort);
    const auto producer = parent(buffer, mBufferGraph);
    return hasKernelTerminated(b, producer, false);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isClosed
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::isClosed(BuilderRef b, const unsigned streamSet) {
    const auto producer = parent(streamSet, mBufferGraph);
    return hasKernelTerminated(b, producer, false);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isClosedNormally
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::isClosedNormally(BuilderRef b, const StreamSetPort inputPort) {
    const auto buffer = getInputBufferVertex(inputPort);
    const auto producer = parent(buffer, mBufferGraph);
    const Kernel * const kernel = getKernel(producer);
    bool normally = false;
    for (const Attribute & attr : kernel->getAttributes()) {
        switch (attr.getKind()) {
            case AttrId::CanTerminateEarly:
            case AttrId::MayFatallyTerminate:
                normally = true;
            default: continue;
        }
    }
    return hasKernelTerminated(b, producer, normally);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updateTerminationSignal
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::updateTerminationSignal(Value * const signal) {
    assert (mKernelIndex < mTerminationSignals.size());
    mTerminationSignals[mKernelIndex] = signal;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initiallyTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::initiallyTerminated(BuilderRef b) {
    Value * const signal = readTerminationSignal(b);
    updateTerminationSignal(signal);
    mTerminatedInitially = signal;
    return hasKernelTerminated(b, mKernelIndex);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initiallyTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::readTerminationSignal(BuilderRef b) {
    Value * const ptr = b->getScalarFieldPtr(TERMINATION_PREFIX + std::to_string(mKernelIndex));
    return b->CreateLoad(ptr, true);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeTerminationSignal(BuilderRef b, Value * const signal) {
    Value * const ptr = b->getScalarFieldPtr(TERMINATION_PREFIX + std::to_string(mKernelIndex));
    b->CreateStore(signal, ptr, true);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief loadItemCountsOfCountableRateStreams
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::readCountableItemCountsAfterAbnormalTermination(BuilderRef b) {

    auto isCountableType = [this](const Value * const ptr, const Binding & binding) {
        if (ptr == nullptr || mKernelIsInternallySynchronized) {
            return false;
        }
        return isCountable(binding);
    };

    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    Vec<Value *> finalProcessed(numOfInputs);
    for (unsigned i = 0; i < numOfInputs; i++) {
        const StreamSetPort port (PortType::Input, i);
        finalProcessed[i] = mProcessedItemCount(port);
        if (isCountableType(mReturnedProcessedItemCountPtr(port), getInputBinding(port))) {
            finalProcessed[i] = b->CreateLoad(mReturnedProcessedItemCountPtr(port));
        }
    }
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    Vec<Value *> finalProduced(numOfOutputs);
    for (unsigned i = 0; i < numOfOutputs; i++) {
        const StreamSetPort port (PortType::Output, i);
        finalProduced[i] = mProducedItemCount(port);
        if (isCountableType(mReturnedProducedItemCountPtr(port), getOutputBinding(port))) {
            finalProduced[i] = b->CreateLoad(mReturnedProducedItemCountPtr(port));
        }
    }
    BasicBlock * const exitBlock = b->GetInsertBlock();
    for (unsigned i = 0; i < numOfInputs; i++) {
        const StreamSetPort port (PortType::Input, i);
        mFinalProcessedPhi(port)->addIncoming(finalProcessed[i], exitBlock);
    }
    for (unsigned i = 0; i < numOfOutputs; i++) {
        const StreamSetPort port (PortType::Output, i);
        mFinalProducedPhi(port)->addIncoming(finalProduced[i], exitBlock);
    }
}

}

#endif // TERMINATION_LOGIC_HPP
