#ifndef TERMINATION_LOGIC_HPP
#define TERMINATION_LOGIC_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addTerminationProperties
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addTerminationProperties(BuilderRef b, const size_t kernel) {
    const auto id = KernelPartitionId[kernel];
    IntegerType * const sizeTy = b->getSizeTy();
    mTarget->addInternalScalar(sizeTy, TERMINATION_PREFIX + std::to_string(id), kernel);
    if (in_degree(id, mTerminationPropagationGraph) > 0) {
        mTarget->addInternalScalar(sizeTy, CONSUMER_TERMINATION_COUNT_PREFIX + std::to_string(id), kernel);
    }
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
 * @brief setCurrentTerminationSignal
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::setCurrentTerminationSignal(BuilderRef /* b */, Value * const signal) {
    assert (mCurrentPartitionId == KernelPartitionId[mKernelId]);
    mPartitionTerminationSignal[mCurrentPartitionId] = signal;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getCurrentTerminationSignal
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getCurrentTerminationSignal() const {
    assert (mCurrentPartitionId == KernelPartitionId[mKernelId]);
    return mPartitionTerminationSignal[mCurrentPartitionId];
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief hasKernelTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::hasKernelTerminated(BuilderRef b, const size_t kernel, const bool normally) const {
    const auto partitionId = KernelPartitionId[kernel];
    Value * const signal = mPartitionTerminationSignal[partitionId]; assert (signal);
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

    for (auto partitionId = 0u; partitionId < PartitionCount; ++partitionId) {
        Value * const signal = mPartitionTerminationSignal[partitionId];
        const auto type = mTerminationCheck[partitionId];
        if (type & TerminationCheckFlag::Hard) {
            Value * const final = b->CreateICmpEQ(signal, fatal);
            if (hard) {
                hard = b->CreateOr(hard, final);
            } else {
                hard = final;
            }
        }
        if (type & TerminationCheckFlag::Soft) {
            Value * const final = b->CreateICmpNE(signal, unterminated);
            if (soft) {
                soft = b->CreateAnd(soft, final);
            } else {
                soft = final;
            }
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
    return hasKernelTerminated(b, producer, kernelCanTerminateAbnormally(producer));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief kernelCanTerminateAbnormally
 ** ------------------------------------------------------------------------------------------------------------- */
bool PipelineCompiler::kernelCanTerminateAbnormally(const unsigned kernel) const {
    for (const Attribute & attr : getKernel(kernel)->getAttributes()) {
        switch (attr.getKind()) {
            case AttrId::CanTerminateEarly:
            case AttrId::MayFatallyTerminate:
                return true;
            default: continue;
        }
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkIfKernelIsAlreadyTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::checkIfKernelIsAlreadyTerminated(BuilderRef b) {
    if (mIsPartitionRoot) {
        const auto id = KernelPartitionId[mKernelId];
        Value * const signal = readTerminationSignal(b, id);
        mInitialTerminationSignal = signal; assert (signal);
        setCurrentTerminationSignal(b, signal);
        Value * terminated = hasKernelTerminated(b, mKernelId);

        const auto n = in_degree(id, mTerminationPropagationGraph);
        if (LLVM_UNLIKELY(n > 0)) {
            Value * const ptr = b->getScalarFieldPtr(CONSUMER_TERMINATION_COUNT_PREFIX + std::to_string(id));
            Value * const conSignal = b->CreateLoad(ptr);
            Value * const allConsumersFinished = b->CreateICmpEQ(conSignal, b->getSize(n));
            terminated = b->CreateOr(terminated, allConsumersFinished);
        }
        mInitiallyTerminated = terminated;
    }
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
 * @brief readCountableItemCountsAfterAbnormalTermination
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::readCountableItemCountsAfterAbnormalTermination(BuilderRef b) {

    auto isCountableType = [this](const Value * const ptr, const Binding & binding) {
        if (ptr == nullptr || mKernelIsInternallySynchronized) {
            return false;
        }
        return isCountable(binding);
    };

    const auto numOfOutputs = numOfStreamOutputs(mKernelId);
    Vec<Value *> finalProduced(numOfOutputs);
    for (unsigned i = 0; i < numOfOutputs; i++) {
        const StreamSetPort port (PortType::Output, i);
        finalProduced[i] = mProducedItemCount[port];
        if (isCountableType(mReturnedProducedItemCountPtr[port], getOutputBinding(port))) {
            finalProduced[i] = b->CreateLoad(mReturnedProducedItemCountPtr[port]);
            #ifdef PRINT_DEBUG_MESSAGES
            debugPrint(b, makeBufferName(mKernelId, port) +
                       "_producedAfterAbnormalTermination = %" PRIu64, finalProduced[i]);
            #endif
        }
    }
    BasicBlock * const exitBlock = b->GetInsertBlock();

    for (unsigned i = 0; i < numOfOutputs; i++) {
        const StreamSetPort port (PortType::Output, i);
        mProducedAtTerminationPhi[port]->addIncoming(finalProduced[i], exitBlock);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief informInputKernelsOfTermination
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::informInputKernelsOfTermination(BuilderRef b) {

    if (CheckAssertions) {

        bool hasPrincipal = false;
        Value * atLeastOneExhausted = nullptr;
        for (const auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
            const BufferPort & br = mBufferGraph[e];
            Value * const closed = isClosed(b, br.Port);

            Value * fullyConsumed = nullptr;
            if (LLVM_UNLIKELY(br.IsZeroExtended)) {
                fullyConsumed = closed;
            } else {
                Value * const avail = getLocallyAvailableItemCount(b, br.Port);
                Value * const processed = mProcessedItemCount[br.Port]; assert (processed);
                fullyConsumed = b->CreateAnd(closed, b->CreateICmpULE(avail, processed));
            }

            if (LLVM_UNLIKELY(br.IsPrincipal)) {
                if (atLeastOneExhausted) {
                    RecursivelyDeleteTriviallyDeadInstructions(atLeastOneExhausted);
                }
                atLeastOneExhausted = fullyConsumed;
                hasPrincipal = true;
                break;
            }
            if (atLeastOneExhausted) {
                atLeastOneExhausted = b->CreateOr(atLeastOneExhausted, fullyConsumed);
            } else {
                atLeastOneExhausted = fullyConsumed;
            }
        }

        if (atLeastOneExhausted) {
            Constant * const completed = getTerminationSignal(b, TerminationSignal::Completed);
            Value * const notTerminatedNormally = b->CreateICmpNE(mTerminatedSignalPhi, completed);
            Value * const expectedTermination = b->CreateOr(notTerminatedNormally, atLeastOneExhausted);
            SmallVector<char, 256> tmp;
            raw_svector_ostream out(tmp);
            out << "Kernel %s in partition %" PRId64 " was terminated before exhausting ";
            if (hasPrincipal) {
                out << "its principal input.";
            } else {
                out << "at least one of its inputs.";
            }
            b->CreateAssert(expectedTermination, out.str(),
                mCurrentKernelName, b->getSize(mCurrentPartitionId),
                mKernelName[FirstKernelInPartition]);
        }

    }


    // When a partition terminates, we want to inform any kernels that supply information to it that
    // one of their consumers has finished processing data.
    ConstantInt * const ONE = b->getSize(1);
    for (const auto e : make_iterator_range(out_edges(mCurrentPartitionId, mTerminationPropagationGraph))) {
        const auto id = target(e, mTerminationPropagationGraph);
        Value * const signal = b->getScalarFieldPtr(CONSUMER_TERMINATION_COUNT_PREFIX + std::to_string(id));
        b->CreateAtomicFetchAndAdd(ONE, signal);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief verifyPostInvocationTerminationSignal
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::verifyPostInvocationTerminationSignal(BuilderRef b) {

    if (mIsPartitionRoot) {
        #ifndef INITIALLY_TERMINATED_KERNELS_JUMP_TO_NEXT_PARTITION
        Value * anyUnterminated = nullptr;
        ConstantInt * const ZERO = b->getSize(0);
        if (mCurrentPartitionId > 0) {
            Value * const signal = mPartitionTerminationSignal[mCurrentPartitionId - 1];
            anyUnterminated = b->CreateICmpEQ(signal, ZERO);
        }

        for (const auto e : make_iterator_range(in_edges(mCurrentPartitionId, mPartitionJumpTree))) {
            const auto partitionId = source(e, mPartitionJumpTree);
            Value * const signal = mPartitionTerminationSignal[partitionId];
            Value * const nonterm = b->CreateICmpEQ(signal, ZERO);
            if (anyUnterminated) {
                anyUnterminated = b->CreateOr(anyUnterminated, nonterm);
            } else {
                anyUnterminated = nonterm;
            }
        }

        if (anyUnterminated) {
            Value * const allTerminated = b->CreateNot(anyUnterminated);
            Constant * const completed = getTerminationSignal(b, TerminationSignal::Completed);
            Value * const notTerminatedNormally = b->CreateICmpNE(mTerminatedAtExitPhi, completed);
            Value * const valid = b->CreateOr(notTerminatedNormally, allTerminated);
            constexpr auto msg =
                "Kernel %s of partition %" PRId64 " was flagged as complete "
                "before any of its source partitions were terminated.";
            b->CreateAssert(valid, msg,
                mCurrentKernelName, b->getSize(mCurrentPartitionId));
        }
        #endif
        mPartitionRootTerminationSignal = b->CreateIsNotNull(mTerminatedAtExitPhi);
        constexpr auto msg =
            "Partition root %s in partition %" PRId64 " should have been flagged as terminated "
            "after invocation.";
        Value * const valid = b->CreateOr(b->CreateNot(mFinalPartitionSegment), mPartitionRootTerminationSignal);
        b->CreateAssert(valid, msg, mCurrentKernelName, b->getSize(mCurrentPartitionId));
    } else if (!mKernelCanTerminateEarly) {
        Value * const isTerminated = b->CreateIsNotNull(mTerminatedAtExitPhi);
        constexpr auto msg =
            "Kernel %s in partition %" PRId64 " should have been flagged as terminated "
            "after partition root %s was terminated.";
        Value * const valid = b->CreateICmpEQ(mPartitionRootTerminationSignal, isTerminated);
        b->CreateAssert(valid, msg,
            mCurrentKernelName, b->getSize(mCurrentPartitionId),
            mKernelName[FirstKernelInPartition]);
    }

}

}

#endif // TERMINATION_LOGIC_HPP
