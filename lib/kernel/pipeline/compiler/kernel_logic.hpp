#include "pipeline_compiler.hpp"

#include <llvm/Support/ErrorHandling.h>

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief beginKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::setActiveKernel(BuilderRef b, const unsigned index, const bool allowThreadLocal) {
    assert (index >= FirstKernel && index <= LastKernel);
    mKernelId = index;
    mKernel = getKernel(index);
    mKernelSharedHandle = nullptr;
    if (LLVM_LIKELY(mKernel->isStateful())) {
        Value * handle = b->getScalarField(makeKernelName(index));
        if (LLVM_UNLIKELY(mKernel->externallyInitialized())) {
            handle = b->CreatePointerCast(handle, mKernel->getSharedStateType()->getPointerTo());
        }
        mKernelSharedHandle = handle;
    }
    mKernelThreadLocalHandle = nullptr;
    if (mKernel->hasThreadLocal() && allowThreadLocal) {
        mKernelThreadLocalHandle = b->CreateLoad(getThreadLocalHandlePtr(b, mKernelId));
    }
    mCurrentKernelName = mKernelName[mKernelId];
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeFullyProcessedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::computeFullyProcessedItemCounts(BuilderRef b) {
    const auto numOfInputs = numOfStreamInputs(mKernelId);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const StreamSetPort port{PortType::Input, i};
        const Binding & input = getInputBinding(port);
        Value * processed = nullptr;
        if (mUpdatedProcessedDeferredPhi(port)) {
            processed = mUpdatedProcessedDeferredPhi(port);
        } else {
            processed = mUpdatedProcessedPhi(port);
        }
        Value * const fullyProcessed = truncateBlockSize(b, input, processed, mTerminatedAtLoopExitPhi);
        mFullyProcessedItemCount(port) = fullyProcessed;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeFullyProducedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::computeFullyProducedItemCounts(BuilderRef b) {

    const auto numOfOutputs = numOfStreamOutputs(mKernelId);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const StreamSetPort port{PortType::Output, i};
        Value * produced = mUpdatedProducedPhi(port);
        Value * const fullyProduced = computeFullyProducedItemCount(b, mKernelId, port, produced, mTerminatedAtLoopExitPhi);
        mFullyProducedItemCount(port)->addIncoming(fullyProduced, mKernelLoopExitPhiCatch);
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeFullyProducedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::computeFullyProducedItemCount(BuilderRef b,
                                                        const size_t kernel,
                                                        const StreamSetPort port,
                                                        Value * produced, Value * const terminationSignal) {

    // TODO: we only need to consider the blocksize attribute if it's possible this
    // stream could be read before being fully written. This might occur if one of
    // it's consumers has a non-Fixed rate that does not have a matching BlockSize
    // attribute.

    const Binding & output = getOutputBinding(kernel, port);
    if (LLVM_UNLIKELY(output.hasAttribute(AttrId::Delayed))) {
        const auto & D = output.findAttribute(AttrId::Delayed);
        Value * const delayed = b->CreateSaturatingSub(produced, b->getSize(D.amount()));
        Value * const terminated = b->CreateIsNotNull(terminationSignal);
        produced = b->CreateSelect(terminated, produced, delayed);
    }
    return truncateBlockSize(b, output, produced, terminationSignal);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addLookahead
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::addLookahead(BuilderRef b, const StreamSetPort inputPort, Value * const itemCount) const {
    Constant * const lookAhead = getLookahead(b, inputPort);
    if (LLVM_LIKELY(lookAhead == nullptr)) {
        return itemCount;
    }
    return b->CreateAdd(itemCount, lookAhead);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief subtractLookahead
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::subtractLookahead(BuilderRef b, const StreamSetPort inputPort, Value * const itemCount) {
    Constant * const lookAhead = getLookahead(b, inputPort);
    if (LLVM_LIKELY(lookAhead == nullptr)) {
        return itemCount;
    }
    Value * const closed = isClosed(b, inputPort);
    if (LLVM_UNLIKELY(mCheckAssertions)) {
        const Binding & binding = getInputBinding(inputPort);
        b->CreateAssert(b->CreateOr(b->CreateICmpUGE(itemCount, lookAhead), closed),
                        "%s.%s: look ahead exceeds item count",
                        mCurrentKernelName,
                        b->GetString(binding.getName()));
    }
    Value * const reducedItemCount = b->CreateSub(itemCount, lookAhead);
    return b->CreateSelect(closed, itemCount, reducedItemCount);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getLookahead
 ** ------------------------------------------------------------------------------------------------------------- */
Constant * PipelineCompiler::getLookahead(BuilderRef b, const StreamSetPort inputPort) const {
    const Binding & input = getInputBinding(inputPort);
    if (LLVM_UNLIKELY(input.hasLookahead())) {
        return b->getSize(input.getLookahead());
    }
    return nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief maskBlockSize
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::truncateBlockSize(BuilderRef b, const Binding & binding, Value * itemCount, Value * const terminationSignal) const {
    // TODO: if we determine all of the inputs of a stream have a blocksize attribute, or the output has one,
    // we can skip masking it on input



    if (LLVM_UNLIKELY(binding.hasAttribute(AttrId::BlockSize))) {
        // If the input rate has a block size attribute then --- for the purpose of determining how many
        // items have been consumed --- we consider a stream set to be fully processed when an entire
        // stride has been processed.
        Constant * const BLOCK_WIDTH = b->getSize(b->getBitBlockWidth());
        Value * const maskedItemCount = b->CreateAnd(itemCount, ConstantExpr::getNeg(BLOCK_WIDTH));
        Value * const terminated = b->CreateIsNotNull(terminationSignal);
        itemCount = b->CreateSelect(terminated, itemCount, maskedItemCount);
    }
    return itemCount;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitializationFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getKernelInitializeFunction(BuilderRef b) const {
    Function * const init = mKernel->getInitializeFunction(b);
    assert (!mKernel->hasFamilyName());
    return init;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getKernelAllocateSharedInternalStreamSetsFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getKernelAllocateSharedInternalStreamSetsFunction(BuilderRef b) const {
    Function * const term = mKernel->getAllocateSharedInternalStreamSetsFunction(b, false);
    if (mKernel->hasFamilyName()) {
        return getFamilyFunctionFromKernelState(b, term->getType(), ALLOCATE_SHARED_INTERNAL_STREAMSETS_FUNCTION_POINTER_SUFFIX);
    }
    return term;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitializationThreadLocalFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getKernelInitializeThreadLocalFunction(BuilderRef b) const {
    Function * const init = mKernel->getInitializeThreadLocalFunction(b);
    if (mKernel->hasFamilyName()) {
        return getFamilyFunctionFromKernelState(b, init->getType(), INITIALIZE_THREAD_LOCAL_FUNCTION_POINTER_SUFFIX);
    }
    return init;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getKernelAllocateThreadLocalInternalStreamSetsFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getKernelAllocateThreadLocalInternalStreamSetsFunction(BuilderRef b) const {
    Function * const term = mKernel->getAllocateThreadLocalInternalStreamSetsFunction(b, false);
    if (mKernel->hasFamilyName()) {
        return getFamilyFunctionFromKernelState(b, term->getType(), ALLOCATE_THREAD_LOCAL_INTERNAL_STREAMSETS_FUNCTION_POINTER_SUFFIX);
    }
    return term;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getDoSegmentFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getKernelDoSegmentFunction(BuilderRef b) const {
    Function * const doSegment = mKernel->getDoSegmentFunction(b);
    if (mKernel->hasFamilyName()) {
        return getFamilyFunctionFromKernelState(b, doSegment->getType(), DO_SEGMENT_FUNCTION_POINTER_SUFFIX);
    }
    return doSegment;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitializationThreadLocalFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getKernelFinalizeThreadLocalFunction(BuilderRef b) const {
    Function * const finalize = mKernel->getFinalizeThreadLocalFunction(b);
    if (mKernel->hasFamilyName()) {
        return getFamilyFunctionFromKernelState(b, finalize->getType(), FINALIZE_THREAD_LOCAL_FUNCTION_POINTER_SUFFIX);
    }
    return finalize;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getFinalizeFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getKernelFinalizeFunction(BuilderRef b) const {
    Function * const term = mKernel->getFinalizeFunction(b);
    if (mKernel->hasFamilyName()) {
        return getFamilyFunctionFromKernelState(b, term->getType(), FINALIZE_FUNCTION_POINTER_SUFFIX);
    }
    return term;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getThreadLocalHandlePtr
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getThreadLocalHandlePtr(BuilderRef b, const unsigned kernelIndex) const {
    const Kernel * const kernel = getKernel(kernelIndex);
    assert ("getThreadLocalHandlePtr should not have been called" && kernel->hasThreadLocal());
    const auto prefix = makeKernelName(kernelIndex);
    Value * handle = getScalarFieldPtr(b.get(), prefix + KERNEL_THREAD_LOCAL_SUFFIX);
    if (LLVM_UNLIKELY(kernel->externallyInitialized())) {
        PointerType * const localStateTy = kernel->getThreadLocalStateType()->getPointerTo();
        handle = b->CreatePointerCast(handle, localStateTy->getPointerTo());
    }
    return handle;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructInputPortMappings
 ** ------------------------------------------------------------------------------------------------------------- */
BufferPortMap PipelineCompiler::constructInputPortMappings() const {
    size_t n = 0;
    for (auto i = PipelineInput; i <= PipelineOutput; ++i) {
        n += in_degree(i, mBufferGraph);
    }
    BufferPortMap M;
    M.reserve(n);
    for (auto i = PipelineInput; i <= PipelineOutput; ++i) {
        const auto hint = M.nth(M.size());
        for (const auto e : make_iterator_range(in_edges(i, mBufferGraph))) {
            const BufferRateData & input = mBufferGraph[e];
            M.emplace_hint_unique(hint, i, input.Port.Number);
        }
    }
    assert (M.size() == n);
    return M;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructOutputPortMappings
 ** ------------------------------------------------------------------------------------------------------------- */
BufferPortMap PipelineCompiler::constructOutputPortMappings() const {
    size_t n = 0;
    for (auto i = PipelineInput; i <= PipelineOutput; ++i) {
        n += out_degree(i, mBufferGraph);
    }
    BufferPortMap M;
    M.reserve(n);
    for (auto i = PipelineInput; i <= PipelineOutput; ++i) {
        const auto hint = M.nth(M.size());
        for (const auto e : make_iterator_range(out_edges(i, mBufferGraph))) {
            const BufferRateData & output = mBufferGraph[e];
            M.emplace_hint_unique(hint, i, output.Port.Number);
        }
    }
    assert (M.size() == n);
    return M;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief supportsInternalSynchronization
 ** ------------------------------------------------------------------------------------------------------------- */
bool PipelineCompiler::supportsInternalSynchronization() const {
    if (LLVM_UNLIKELY(mKernel->hasAttribute(AttrId::InternallySynchronized))) {
        if (LLVM_UNLIKELY(mMayHaveNonLinearIO)) {
            SmallVector<char, 256> tmp;
            raw_svector_ostream out(tmp);
            out << "PipelineCompiler error: " <<
                   mKernel->getName() << " is internally synchronized"
                   " but permits non-linear I/O.";
            report_fatal_error(out.str());
        }
        return true;
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isBounded
 ** ------------------------------------------------------------------------------------------------------------- */
bool PipelineCompiler::isBounded() const {
    assert (mKernelId >= FirstKernel && mKernelId <= LastKernel);
    for (const auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
        const auto streamSet = source(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[streamSet];
        if (bn.NonLinear) {
            const BufferRateData & br = mBufferGraph[e];
            const Binding & binding = br.Binding;
            const ProcessingRate & rate = binding.getRate();
            switch (rate.getKind()) {
                case RateId::Bounded:
                case RateId::Fixed:
                case RateId::PartialSum:
                    return true;
                case RateId::Greedy:
                    if (rate.getLowerBound() > Rational{0, 1}) {
                        return true;
                    }
                default: break;
            }
        }
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief canTruncateInputBuffer
 ** ------------------------------------------------------------------------------------------------------------- */
bool PipelineCompiler::canTruncateInputBuffer() const {
    assert (mKernelId >= FirstKernel && mKernelId <= LastKernel);
    return out_degree(mKernelId, mInputTruncationGraph) != 0;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief identifyPipelineInputs
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::identifyPipelineInputs() {
    mHasPipelineInput.reset();
    mHasPipelineInput.resize(in_degree(mKernelId, mBufferGraph));

    if (LLVM_LIKELY(out_degree(PipelineInput, mBufferGraph) > 0)) {
        for (const auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
            const auto streamSet = source(e, mBufferGraph);
            const auto producer = parent(streamSet, mBufferGraph);
            if (LLVM_UNLIKELY(producer == PipelineInput)) {
                const BufferRateData & br = mBufferGraph[e];
                mHasPipelineInput.set(br.Port.Number);
            }
        }
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputBufferVertex
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned PipelineCompiler::getInputBufferVertex(const StreamSetPort inputPort) const {
    return getInputBufferVertex(mKernelId, inputPort);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputBuffer
 ** ------------------------------------------------------------------------------------------------------------- */
inline StreamSetBuffer * PipelineCompiler::getInputBuffer(const StreamSetPort inputPort) const {
    return getInputBuffer(mKernelId, inputPort);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputBinding
 ** ------------------------------------------------------------------------------------------------------------- */
inline const Binding & PipelineCompiler::getInputBinding(const StreamSetPort inputPort) const {
    return getInputBinding(mKernelId, inputPort);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputBufferVertex
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned PipelineCompiler::getOutputBufferVertex(const StreamSetPort outputPort) const {
    return getOutputBufferVertex(mKernelId, outputPort);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputBinding
 ** ------------------------------------------------------------------------------------------------------------- */
inline const Binding & PipelineCompiler::getOutputBinding(const StreamSetPort outputPort) const {
    return getOutputBinding(mKernelId, outputPort);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputBuffer
 ** ------------------------------------------------------------------------------------------------------------- */
inline StreamSetBuffer * PipelineCompiler::getOutputBuffer(const StreamSetPort outputPort) const {
    return getOutputBuffer(mKernelId, outputPort);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getBinding
 ** ------------------------------------------------------------------------------------------------------------- */
inline const Binding & PipelineCompiler::getBinding(const StreamSetPort port) const {
    return getBinding(mKernelId, port);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getReference
 ** ------------------------------------------------------------------------------------------------------------- */
inline const StreamSetPort PipelineCompiler::getReference(const StreamSetPort port) const {
    return PipelineCommonGraphFunctions::getReference(mKernelId, port);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief reset
 ** ------------------------------------------------------------------------------------------------------------- */
template <typename Vec>
inline void reset(Vec & vec, const size_t n) {
    vec.resize(n);
    std::memset(vec.data(), 0, n * sizeof(typename Vec::value_type));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief clearInternalStateForCurrentKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::clearInternalStateForCurrentKernel() {
    mNumOfAddressableItemCount = 0;
    mNumOfVirtualBaseAddresses = 0;
    mNumOfTruncatedInputBuffers = 0;

    mHasZeroExtendedInput = nullptr;
    mZeroExtendBufferPhi = nullptr;
    mAnyRemainingInput = nullptr;
    mExhaustedPipelineInputPhi = nullptr;

    mKernelInsufficientInput = nullptr;
    mKernelInsufficientInputExit = nullptr;
    mKernelTerminated = nullptr;
    mKernelInitiallyTerminated = nullptr;
    mKernelInitiallyTerminatedExit = nullptr;

    mMaximumNumOfStrides = nullptr;

    assert (mKernelId >= FirstKernel);
    assert (mKernelId <= LastKernel);

    const auto numOfInputs = in_degree(mKernelId, mBufferGraph);
    reset(mAccessibleInputItems, numOfInputs);

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelAssertions
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::initializeKernelAssertions(BuilderRef b) {
    SmallVector<char, 256> tmp;
    for (auto kernel = FirstKernel; kernel <= LastKernel; ++kernel) {
        raw_svector_ostream out(tmp);
        out << kernel << "." << getKernel(kernel)->getName();
        mKernelName[kernel] = b->GetString(out.str());
        tmp.clear();
    }
}

}
