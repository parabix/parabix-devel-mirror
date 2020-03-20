#ifndef TERMINATION_LOGIC_HPP
#define TERMINATION_LOGIC_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addTerminationProperties
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addTerminationProperties(BuilderRef b, const size_t kernel) {
    const auto id = KernelPartitionId[kernel];
    mTarget->addInternalScalar(b->getSizeTy(), TERMINATION_PREFIX + std::to_string(id), kernel);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief hasKernelTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::initializePipelineInputTerminationSignal(BuilderRef b) {
    // any pipeline input streams are considered produced by the P_{in} vertex.
    if (out_degree(PipelineInput, mBufferGraph) > 0) {
        assert (KernelPartitionId[PipelineInput] == 0);
        Constant * const completed = getTerminationSignal(b, TerminationSignal::Completed);
        Constant * const unterminated = getTerminationSignal(b, TerminationSignal::None);
        mPartitionTerminationSignal[0] = b->CreateSelect(mIsFinal, completed, unterminated);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief hasKernelTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::hasKernelTerminated(BuilderRef b, const size_t kernel, const bool normally) const {
    const auto partitionId = KernelPartitionId[kernel];
    Value * const signal = mPartitionTerminationSignal[partitionId];
    if (signal == nullptr) errs() << partitionId << "\n\n";
    assert (signal);
    if (normally) {
        Constant * const completed = getTerminationSignal(b, TerminationSignal::Completed);
        return b->CreateICmpEQ(signal, completed);
    } else {
        Constant * const unterminated = getTerminationSignal(b, TerminationSignal::None);
        return b->CreateICmpNE(signal, unterminated);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief pipelineTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::hasPipelineTerminated(BuilderRef b) const {

    Value * hard = mExhaustedInput;
    Value * soft = nullptr;

    Constant * const unterminated = getTerminationSignal(b, TerminationSignal::None);
    Constant * const aborted = getTerminationSignal(b, TerminationSignal::Aborted);
    Constant * const fatal = getTerminationSignal(b, TerminationSignal::Fatal);

    // check whether every sink has terminated
    for (const auto e : make_iterator_range(in_edges((PartitionCount - 1), mTerminationGraph))) {
        const auto partitionId = source(e, mTerminationGraph);
        Value * const signal = mPartitionTerminationSignal[partitionId]; assert (signal);
        assert (signal->getType() == unterminated->getType());
        // if this is a hard termination, such as a fatal error, any can terminate the pipeline.
        // however, a kernel that can terminate with a fatal error, may not necessarily do so.
        // otherwise its a soft termination and all must agree that the pipeline has terminated
        if (mTerminationGraph[e]) {
            Value * const final = b->CreateICmpEQ(signal, fatal);
            if (hard) {
                hard = b->CreateOr(hard, final);
            } else {
                hard = final;
            }
        }
        Value * const final = b->CreateICmpNE(signal, unterminated);
        if (soft) {
            soft = b->CreateAnd(soft, final);
        } else {
            soft = final;
        }
    }
    assert (soft);
    Value * signal = b->CreateSelect(soft, aborted, unterminated);
    if (hard) {
        signal = b->CreateSelect(hard, fatal, signal);
    }
    return signal;

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
Value * PipelineCompiler::isClosed(BuilderRef b, const StreamSetPort inputPort) const {
    const auto buffer = getInputBufferVertex(inputPort);
    const auto producer = parent(buffer, mBufferGraph);
    return hasKernelTerminated(b, producer, false);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isClosed
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::isClosed(BuilderRef b, const unsigned streamSet) const {
    const auto producer = parent(streamSet, mBufferGraph);
    return hasKernelTerminated(b, producer, false);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isClosedNormally
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::isClosedNormally(BuilderRef b, const StreamSetPort inputPort) const {
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
 * @brief initiallyTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::initiallyTerminated(BuilderRef b) {
    if (mIsPartitionRoot) {
        Value * const signal = readTerminationSignal(b, KernelPartitionId[mKernelId]);
        mTerminatedInitially = signal; assert (signal);
        setCurrentPartitionTerminationSignal(signal);
        return hasKernelTerminated(b, mKernelId);
    }
    return nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initiallyTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::readTerminationSignal(BuilderRef b, const unsigned partitionId) {
    const auto name = TERMINATION_PREFIX + std::to_string(partitionId);
    Value * const ptr = b->getScalarFieldPtr(name);
    return b->CreateLoad(ptr, true, name);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeTerminationSignal(BuilderRef b, Value * const signal) {
    const auto id = KernelPartitionId[mKernelId];
    Value * const ptr = b->getScalarFieldPtr(TERMINATION_PREFIX + std::to_string(id));
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

    const auto numOfInputs = numOfStreamInputs(mKernelId);
    Vec<Value *> finalProcessed(numOfInputs);
    for (unsigned i = 0; i < numOfInputs; i++) {
        const StreamSetPort port (PortType::Input, i);
        finalProcessed[i] = mProcessedItemCount(port);
        if (isCountableType(mReturnedProcessedItemCountPtr(port), getInputBinding(port))) {
            finalProcessed[i] = b->CreateLoad(mReturnedProcessedItemCountPtr(port));
        }
    }
    const auto numOfOutputs = numOfStreamOutputs(mKernelId);
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
