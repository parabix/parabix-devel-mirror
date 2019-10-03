#include "pipeline_compiler.hpp"

#include <llvm/Support/ErrorHandling.h>

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief reset
 ** ------------------------------------------------------------------------------------------------------------- */
template <typename Vec>
inline void reset(Vec & vec, const unsigned n) {
    vec.resize(n);
    std::fill_n(vec.begin(), n, nullptr);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief beginKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::setActiveKernel(BuilderRef b, const unsigned index) {
    assert (index >= FirstKernel && index <= LastKernel);
    mKernelIndex = index;
    mKernel = getKernel(index);
    b->setKernel(mPipelineKernel);
    if (LLVM_LIKELY(mKernel->isStateful())) {
        Value * handle = nullptr;
        if (mKernel->hasFamilyName()) {
            handle = b->getScalarField(makeFamilyPrefix(index));
            handle = b->CreateBitCast(handle, mKernel->getSharedStateType()->getPointerTo());
        } else {
            handle = b->getScalarField(makeKernelName(index));
        }
        mKernel->setHandle(b, handle);
    }
    b->setKernel(mKernel);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief zeroInputAfterFinalItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::zeroInputAfterFinalItemCount(BuilderRef b, const Vec<Value *> & accessibleItems, Vec<Value *> & inputBaseAddress) {

    const auto numOfInputs = accessibleItems.size();
    const auto blockWidth = b->getBitBlockWidth();
    const auto log2BlockWidth = floor_log2(blockWidth);
    Constant * const BLOCK_MASK = b->getSize(blockWidth - 1);
    Constant * const LOG_2_BLOCK_WIDTH = b->getSize(log2BlockWidth);
    Constant * const ZERO = b->getSize(0);
    Constant * const ONE = b->getSize(1);

    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = getInputBinding(i);
        const ProcessingRate & rate = input.getRate();

        // TODO: if this streamset has 0 streams, it exists only to have a produced/processed count. Ignore it.

        inputBaseAddress[i] = mInputEpoch[i];

        assert (inputBaseAddress[i]);

        #ifndef DISABLE_INPUT_ZEROING
        if (LLVM_LIKELY(rate.isFixed())) {

            // create a stack entry for this buffer at the start of the pipeline
            const auto ip = b->saveIP();
            b->SetInsertPoint(mPipelineEntryBranch);
            PointerType * const int8PtrTy = b->getInt8PtrTy();
            AllocaInst * const bufferStorage = b->CreateAlloca(int8PtrTy, nullptr, "TruncatedInputBuffer");
            b->CreateStore(ConstantPointerNull::get(int8PtrTy), bufferStorage);
            b->restoreIP(ip);
            mTruncatedInputBuffer.push_back(bufferStorage);



            const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Input, i});
            const StreamSetBuffer * const buffer = getInputBuffer(i);
            const auto itemWidth = getItemWidth(buffer->getBaseType());
            Constant * const ITEM_WIDTH = b->getSize(itemWidth);
            const RateValue stridesPerBlock(mKernel->getStride(), blockWidth);
            const auto strideRate = rate.getRate() * stridesPerBlock;
            const auto stridesPerSegment = ceiling(strideRate); assert (stridesPerSegment >= 1);
            Constant * const STRIDES_PER_SEGMENT = b->getSize(stridesPerSegment);

            BasicBlock * const maskedInput = b->CreateBasicBlock(prefix + "_genMaskedInput", mKernelLoopCall);
            BasicBlock * const maskedInputLoop = b->CreateBasicBlock(prefix + "_genMaskedInputLoop", mKernelLoopCall);
            BasicBlock * const selectedInput = b->CreateBasicBlock(prefix + "_selectedInput", mKernelLoopCall);



            Value * const tooMany = b->CreateICmpULT(accessibleItems[i], mAccessibleInputItems[i]);
            Value * computeMask = tooMany;
            if (mIsInputZeroExtended[i]) {
                computeMask = b->CreateAnd(tooMany, b->CreateNot(mIsInputZeroExtended[i]));
            }
            BasicBlock * const entryBlock = b->GetInsertBlock();
            b->CreateUnlikelyCondBr(computeMask, maskedInput, selectedInput);

            b->SetInsertPoint(maskedInput);

            // if this is a deferred fixed rate stream, we cannot be sure how many
            // blocks will have to be provided to the kernel in order to mask out
            // the truncated input stream.

            #ifdef PRINT_DEBUG_MESSAGES
            b->CallPrintInt(prefix + " truncating item count (" + std::to_string(itemWidth) + ") to ", accessibleItems[i]);
            #endif

            // TODO: if we can prove that this will be the last kernel invocation that will ever touch this stream)
            // and is not an input to the pipeline (which we cannot prove will have space after the last item), we
            // can avoid copying the buffer and instead just mask out the surpressed items.

            ExternalBuffer tmp(b, input.getType(), true, 0);

            Value * const start = b->CreateLShr(mAlreadyProcessedPhi[i], LOG_2_BLOCK_WIDTH);

            DataLayout DL(b->getModule());
            Value * const startPtr = tmp.getStreamBlockPtr(b, mInputEpoch[i], ZERO, start);
            Type * const intPtrTy = DL.getIntPtrType(startPtr->getType());
            Value * const startPtrInt = b->CreatePtrToInt(startPtr, intPtrTy);

            Value * const limit = b->CreateAdd(start, STRIDES_PER_SEGMENT);
            Value * const limitPtr = tmp.getStreamBlockPtr(b, mInputEpoch[i], ZERO, limit);
            Value * const limitPtrInt = b->CreatePtrToInt(limitPtr, intPtrTy);

            Value * const strideBytes = b->CreateSub(limitPtrInt, startPtrInt);
            Value * segmentBytes = strideBytes;

            Value * initial = start;
            Value * initialPtr = startPtr;
            Value * initialPtrInt = startPtrInt;

            Value * end = start;
            Value * fullBytesToCopy = nullptr;

            if (stridesPerSegment != 1 || input.isDeferred()) {
                if (input.isDeferred()) {
                    initial = b->CreateLShr(mAlreadyProcessedDeferredPhi[i], LOG_2_BLOCK_WIDTH);
                    initialPtr = tmp.getStreamBlockPtr(b, mInputEpoch[i], ZERO, initial);
                    initialPtrInt = b->CreatePtrToInt(initialPtr, intPtrTy);
                }
                // if a kernel reads in multiple strides of data per segment, we may be able to
                // copy over a portion of it with a single memcpy.
                Value * endPtrInt = startPtrInt;
                if (stridesPerSegment  != 1) {
                    end = b->CreateAdd(mAlreadyProcessedPhi[i], accessibleItems[i]);
                    end = b->CreateLShr(end, LOG_2_BLOCK_WIDTH);
                    Value * const endPtr = tmp.getStreamBlockPtr(b, mInputEpoch[i], ZERO, end);
                    endPtrInt = b->CreatePtrToInt(endPtr, intPtrTy);
                }
                fullBytesToCopy = b->CreateSub(endPtrInt, initialPtrInt);
                segmentBytes = b->CreateAdd(fullBytesToCopy, strideBytes);
            }

            Value * maskedBuffer = b->CreateAlignedMalloc(segmentBytes, blockWidth / 8);
            b->CreateStore(maskedBuffer, bufferStorage);
            PointerType * const bufferType = tmp.getPointerType();
            maskedBuffer = b->CreatePointerCast(maskedBuffer, bufferType, "maskedBuffer");

            if (fullBytesToCopy) {
                b->CreateMemCpy(maskedBuffer, initialPtr, fullBytesToCopy, blockWidth / 8);
            }

            Value * maskedEpoch = tmp.getStreamBlockPtr(b, maskedBuffer, ZERO, b->CreateNeg(initial));
            maskedEpoch = b->CreatePointerCast(maskedEpoch, bufferType);
            Value * packIndex = nullptr;
            Value * maskOffset = b->CreateAnd(accessibleItems[i], BLOCK_MASK);
            if (itemWidth > 1) {
                Value * const position = b->CreateMul(maskOffset, ITEM_WIDTH);
                packIndex = b->CreateLShr(position, LOG_2_BLOCK_WIDTH);
                maskOffset = b->CreateAnd(position, BLOCK_MASK);
            }
            Value * const mask = b->CreateNot(b->bitblock_mask_from(maskOffset));
            Value * const numOfStreams = buffer->getStreamSetCount(b);
            BasicBlock * const loopEntryBlock = b->GetInsertBlock();
            b->CreateBr(maskedInputLoop);

            b->SetInsertPoint(maskedInputLoop);
            PHINode * const streamIndex = b->CreatePHI(b->getSizeTy(), 2);
            streamIndex->addIncoming(ZERO, loopEntryBlock);

            Value * inputPtr = tmp.getStreamBlockPtr(b, mInputEpoch[i], streamIndex, end);
            Value * outputPtr = tmp.getStreamBlockPtr(b, maskedEpoch, streamIndex, end);

            if (itemWidth > 1) {
                Value * const endPtr = inputPtr;
                Value * const endPtrInt = b->CreatePtrToInt(endPtr, intPtrTy);
                inputPtr = tmp.getStreamPackPtr(b, mInputEpoch[i], streamIndex, end, packIndex);
                Value * const inputPtrInt = b->CreatePtrToInt(inputPtr, intPtrTy);
                Value * const bytesToCopy = b->CreateSub(inputPtrInt, endPtrInt);
                b->CreateMemCpy(outputPtr, endPtr, bytesToCopy, blockWidth / 8);
                outputPtr = tmp.getStreamPackPtr(b, maskedEpoch, streamIndex, end, packIndex);
            }


            assert (inputPtr->getType() == outputPtr->getType());
            Value * const val = b->CreateBlockAlignedLoad(inputPtr);
            Value * const maskedVal = b->CreateAnd(val, mask);
            b->CreateBlockAlignedStore(maskedVal, outputPtr);

            if (itemWidth > 1) {
                Value * const nextPackIndex = b->CreateAdd(packIndex, ONE);
                Value * const clearPtr = tmp.getStreamPackPtr(b, maskedEpoch, streamIndex, end, nextPackIndex);
                Value * const clearPtrInt = b->CreatePtrToInt(clearPtr, intPtrTy);
                Value * const clearEndPtr = tmp.getStreamPackPtr(b, maskedEpoch, streamIndex, end, ITEM_WIDTH);
                Value * const clearEndPtrInt = b->CreatePtrToInt(clearEndPtr, intPtrTy);
                Value * const bytesToClear = b->CreateSub(clearEndPtrInt, clearPtrInt);
                b->CreateMemZero(clearPtr, bytesToClear, blockWidth / 8);
            }

            Value * const nextIndex = b->CreateAdd(streamIndex, ONE);
            Value * const notDone = b->CreateICmpNE(nextIndex, numOfStreams);
            streamIndex->addIncoming(nextIndex, maskedInputLoop);
            b->CreateCondBr(notDone, maskedInputLoop, selectedInput);

            b->SetInsertPoint(selectedInput);
            PHINode * const finalEpoch = b->CreatePHI(bufferType, 2);
            finalEpoch->addIncoming(mInputEpoch[i], entryBlock);
            finalEpoch->addIncoming(maskedEpoch, maskedInputLoop);

            inputBaseAddress[i] = finalEpoch;
        }
        #endif
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief prepareLocalZeroExtendSpace
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::prepareLocalZeroExtendSpace(BuilderRef b) {
    if (mHasZeroExtendedStream) {
        const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
        mZeroExtendBufferPhi = nullptr;
        const auto strideSize = mKernel->getStride();
        const auto blockWidth = b->getBitBlockWidth();
        Value * requiredSpace = nullptr;

        Constant * const ZERO = b->getSize(0);
        Constant * const ONE = b->getSize(1);
        Value * const numOfStrides = b->CreateUMax(mNumOfLinearStrides, ONE);

        for (unsigned i = 0; i < numOfInputs; ++i) {
            if (mIsInputZeroExtended[i]) {
                const auto bufferVertex = getInputBufferVertex(i);
                const BufferNode & bn = mBufferGraph[bufferVertex];
                const Binding & input = getInputBinding(i);

                const auto itemWidth = getItemWidth(input.getType());
                Constant * const strideFactor = b->getSize(itemWidth * strideSize / 8);
                Value * requiredBytes = b->CreateMul(numOfStrides, strideFactor); assert (requiredBytes);
                if (bn.LookAhead) {
                    const auto lh = (bn.LookAhead * itemWidth);
                    requiredBytes = b->CreateAdd(requiredBytes, b->getSize(lh));
                }
                if (LLVM_LIKELY(itemWidth < blockWidth)) {
                    Constant * const factor = b->getSize(blockWidth / itemWidth);
                    requiredBytes = b->CreateRoundUp(requiredBytes, factor);
                }
                requiredBytes = b->CreateMul(requiredBytes, bn.Buffer->getStreamSetCount(b));

                const auto fieldWidth = input.getFieldWidth();
                if (fieldWidth < 8) {
                    requiredBytes = b->CreateUDiv(requiredBytes, b->getSize(8 / fieldWidth));
                } else if (fieldWidth > 8) {
                    requiredBytes = b->CreateMul(requiredBytes, b->getSize(fieldWidth / 8));
                }
                requiredBytes = b->CreateSelect(mIsInputZeroExtended[i], requiredBytes, ZERO);
                requiredSpace = b->CreateUMax(requiredSpace, requiredBytes);
            }
        }
        if (requiredSpace) {
            const auto prefix = makeKernelName(mKernelIndex);
            BasicBlock * const entry = b->GetInsertBlock();
            BasicBlock * const expandZeroExtension =
                b->CreateBasicBlock(prefix + "_expandZeroExtensionBuffer", mKernelLoopCall);
            BasicBlock * const executeKernel =
                b->CreateBasicBlock(prefix + "_executeKernelAfterZeroExtension", mKernelLoopCall);
            Value * const currentSpace = b->CreateLoad(mZeroExtendSpace);
            Value * const currentBuffer = b->CreateLoad(mZeroExtendBuffer);
            requiredSpace = b->CreateRoundUp(requiredSpace, b->getSize(b->getCacheAlignment()));

            Value * const largeEnough = b->CreateICmpUGE(currentSpace, requiredSpace);
            b->CreateLikelyCondBr(largeEnough, executeKernel, expandZeroExtension);

            b->SetInsertPoint(expandZeroExtension);
            assert (b->getCacheAlignment() >= (b->getBitBlockWidth() / 8));
            b->CreateFree(currentBuffer);
            Value * const newBuffer = b->CreateCacheAlignedMalloc(requiredSpace);
            b->CreateMemZero(newBuffer, requiredSpace, b->getCacheAlignment());
            b->CreateStore(requiredSpace, mZeroExtendSpace);
            b->CreateStore(newBuffer, mZeroExtendBuffer);
            b->CreateBr(executeKernel);

            b->SetInsertPoint(executeKernel);
            PHINode * const zeroBuffer = b->CreatePHI(b->getVoidPtrTy(), 2);
            zeroBuffer->addIncoming(currentBuffer, entry);
            zeroBuffer->addIncoming(newBuffer, expandZeroExtension);
            mZeroExtendBufferPhi = zeroBuffer;

        }
    }
}

#if 0

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkForLastPartialSegment
 *
 * If this kernel is internally synchronized, determine whether there are any more segments.
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::checkForLastPartialSegment(BuilderRef b, Value * isFinal) {
    assert (b->getKernel() == mKernel);
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    mLastPartialSegment = nullptr;
    if (mKernel->hasAttribute(AttrId::InternallySynchronized)) {
        mLastPartialSegment = isFinal ? isFinal : b->getFalse();
        for (const auto i : mPortEvaluationOrder) {
            if (i < numOfInputs) {
                mLastPartialSegment = b->CreateOr(mLastPartialSegment, noMoreInputData(b, i));
            } else {
                mLastPartialSegment = b->CreateOr(mLastPartialSegment, noMoreOutputData(b, i - numOfInputs));
            }
        }
        assert (mLastPartialSegment);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief noMoreInputData
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::noMoreInputData(BuilderRef b, const unsigned inputPort) {
    const StreamSetBuffer * const buffer = getInputBuffer(inputPort);
    Value * const available = getLocallyAvailableItemCount(b, inputPort);
    Value * const pending = b->CreateAdd(mAlreadyProcessedPhi[inputPort], mLinearInputItemsPhi[inputPort]);
    Value * const accessible = buffer->getLinearlyAccessibleItems(b, pending, available);
    Value * const strideLength = getInputStrideLength(b, inputPort);
    Value * const required = addLookahead(b, inputPort, strideLength);
    return b->CreateICmpULE(required, accessible);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief noMoreOutputData
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::noMoreOutputData(BuilderRef b, const unsigned outputPort) {
    // TODO: not right for popcount rates. will have to branch (to account for # of ref items)
    // then check with the ref rate's pending offset.
    const StreamSetBuffer * const buffer = getOutputBuffer(outputPort);
    if (isa<DynamicBuffer>(buffer)) {
        return b->getFalse();
    }
    Value * const consumed = mConsumedItemCount[outputPort]; assert (consumed);
    Value * const pending = b->CreateAdd(mAlreadyProducedPhi[outputPort], mLinearOutputItemsPhi[outputPort]);
    Value * const writable = buffer->getLinearlyWritableItems(b, pending, consumed);
    Value * const required = getOutputStrideLength(b, outputPort);
    return b->CreateICmpULE(required, writable);
}

#endif

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeKernelCall
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeKernelCall(BuilderRef b) {
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);

    mNumOfAddressableItemCount = 0;

    // TODO: add MProtect to buffers and their handles.

    // TODO: send in the # of output items we want in the external buffers



    ArgVec args;
    args.reserve(4 + (numOfInputs + numOfOutputs) * 4);
    if (LLVM_LIKELY(mKernel->isStateful())) {
        args.push_back(mKernel->getHandle()); assert (mKernel->getHandle());
    }
    if (LLVM_UNLIKELY(mKernel->hasThreadLocal())) {
        b->setKernel(mPipelineKernel);
        const auto prefix = makeKernelName(mKernelIndex);
        Value * const threadLocal = b->getScalarFieldPtr(prefix + KERNEL_THREAD_LOCAL_SUFFIX);
        b->setKernel(mKernel);
        args.push_back(threadLocal);
    }



    // If a kernel is internally synchronized, pass the iteration
    // count. Note: this is not the same as the pipeline's logical
    // segment number since unless we can prove that a kernel,
    // regardless of buffer state, will be called only once per
    // segment, we can only state the iteration count is >= the
    // segment number.

    // We may hit the same kernel with both threads simultaneously
    // before the first has finished updating?

    // Can we pass in the outer pipeline's synch num for this kernel
    // and increment it early? We'd need to pass in a temp one until
    // we know this is the last iteration of the segment.

    // That requires being able to know apriori what our resulting
    // state will be after completion for the input positions. We
    // can rely on the kernel itself to handle output synchronization.

//    if (mLastPartialSegment) {
//        Value * const iterationPtr = b->getScalarFieldPtr(makeKernelName(mKernelIndex) + ITERATION_COUNT_SUFFIX);
//        Value * const iterationCount = b->CreateAtomicFetchAndAdd(b->getSize(1), iterationPtr);
//        args.push_back(iterationCount);
//    }
    args.push_back(mNumOfLinearStrides); assert (mNumOfLinearStrides);

    if (mFixedRateFactorPhi) {
        args.push_back(mFixedRateFactorPhi);
    }

    RelationshipType prior_in{};

    for (const auto & e : make_iterator_range(in_edges(mKernelIndex, mBufferGraph))) {
        const BufferRateData & rt = mBufferGraph[e];
        assert (rt.Port.Type == PortType::Input);
        assert (prior_in < rt.Port);
        prior_in = rt.Port;

        if (LLVM_LIKELY(rt.Port.Reason == ReasonType::Explicit)) {

            // calculate the deferred processed item count
            PHINode * processed = nullptr;
            bool deferred = false;

            const auto i = rt.Port.Number;
            if (mAlreadyProcessedDeferredPhi[i]) {
                processed = mAlreadyProcessedDeferredPhi[i];
                deferred = true;
            } else {
                processed = mAlreadyProcessedPhi[i];
            }

            const Binding & input = rt.Binding;
            #ifndef NDEBUG
            const auto buffer = source(e, mBufferGraph);
            const BufferNode & bn = mBufferGraph[buffer];
            assert ("input buffer type mismatch?" && (input.getType() == bn.Buffer->getBaseType()));
            #endif
            args.push_back(mInputEpochPhi[i]);

            mReturnedProcessedItemCountPtr[i] = addItemCountArg(b, input, deferred, processed, args);

            if (LLVM_UNLIKELY(requiresItemCount(input))) {
                // calculate how many linear items are from the *deferred* position
                Value * inputItems = mLinearInputItemsPhi[i];
                if (deferred) {
                    Value * diff = b->CreateSub(mAlreadyProcessedPhi[i], mAlreadyProcessedDeferredPhi[i]);
                    inputItems = b->CreateAdd(inputItems, diff);
                }
                args.push_back(inputItems); assert (inputItems);
            }

            if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresPopCountArray))) {
                //args.push_back(getPositivePopCountArray(b, i));
                assert (false);
            }
            if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresNegatedPopCountArray))) {
                // args.push_back(getNegativePopCountArray(b, i));
                assert (false);
            }

        }
    }

    const auto canTerminate = mKernel->canSetTerminateSignal();

    RelationshipType prior_out{};
    for (const auto & e : make_iterator_range(out_edges(mKernelIndex, mBufferGraph))) {
        const BufferRateData & rt = mBufferGraph[e];
        assert (rt.Port.Reason == ReasonType::Explicit);
        assert (rt.Port.Type == PortType::Output);
        assert (prior_out < rt.Port);
        prior_out = rt.Port;
        const auto i = rt.Port.Number;

        PHINode * const produced = mAlreadyProducedPhi[i];
        const auto buffer = target(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[buffer];
        const Binding & output = rt.Binding;

        assert ("output buffer type mismatch?" && (output.getType() == bn.Buffer->getBaseType()));

        if (LLVM_LIKELY(bn.Type != BufferType::Managed)) {
            args.push_back(epoch(b, output, bn.Buffer, produced, nullptr));
        }
        mReturnedProducedItemCountPtr[i] = addItemCountArg(b, output, canTerminate, produced, args);
        if (LLVM_LIKELY(bn.Type == BufferType::Managed)) {
            args.push_back(mConsumedItemCount[i]); assert (mConsumedItemCount[i]);
        } else if (LLVM_UNLIKELY(requiresItemCount(output))) {
            args.push_back(mLinearOutputItemsPhi[i]);  assert (mLinearOutputItemsPhi[i]);
        }
    }

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mPipelineKernel->getHandle(), CBuilder::Protect::NONE);
    }

    #ifdef PRINT_DEBUG_MESSAGES
    const auto prefix = makeKernelName(mKernelIndex);
    b->CallPrintInt("* " + prefix + "_executing", mNumOfLinearStrides);
    #endif

    startCycleCounter(b, CycleCounter::BEFORE_KERNEL_CALL);

    Value * const doSegment = getDoSegmentFunction(b);
    if (mRethrowException) {
        BasicBlock * const invokeOk = b->CreateBasicBlock("", mKernelTerminationCheck);
        mTerminatedExplicitly = b->CreateInvoke(doSegment, invokeOk, mRethrowException, args);
        b->SetInsertPoint(invokeOk);
    } else {
        mTerminatedExplicitly = b->CreateCall(getDoSegmentFunction(b), args);
    }

    updateCycleCounter(b, CycleCounter::BEFORE_KERNEL_CALL, CycleCounter::AFTER_KERNEL_CALL);

    mUpdatedNumOfStrides = b->CreateAdd(mCurrentNumOfStrides, mNumOfLinearStrides);
    if (LLVM_LIKELY(!canTerminate)) {
        mTerminatedExplicitly = getTerminationSignal(b, TerminationSignal::None);
    }

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mPipelineKernel->getHandle(), CBuilder::Protect::WRITE);
    }

    // calculate or read the item counts (assuming this kernel did not terminate)
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = getInputBinding(i);
        const ProcessingRate & rate = input.getRate();
        if (rate.isFixed() || rate.isPartialSum() || rate.isGreedy()) {
            mProcessedItemCount[i] = b->CreateAdd(mAlreadyProcessedPhi[i], mLinearInputItemsPhi[i]);
            if (mAlreadyProcessedDeferredPhi[i]) {
                assert (mReturnedProcessedItemCountPtr[i]);
                mProcessedDeferredItemCount[i] = b->CreateLoad(mReturnedProcessedItemCountPtr[i]);
                if (LLVM_UNLIKELY(mCheckAssertions)) {
                    Value * const deferred = mProcessedDeferredItemCount[i];
                    Value * const processed = mProcessedItemCount[i];
                    Value * const isDeferred = b->CreateICmpULE(deferred, processed);
                    b->CreateAssert(isDeferred, input.getName() +
                                    ": deferred processed item count (%d) "
                                    "exceeds non-deferred (%d)",
                                    deferred, processed);
                }
                #ifdef PRINT_DEBUG_MESSAGES
                const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Input, i});
                b->CallPrintInt(prefix + "_processed_deferred'", mProcessedDeferredItemCount[i]);
                #endif
            }
        } else if (rate.isBounded() || rate.isUnknown()) {
            mProcessedItemCount[i] = b->CreateLoad(mReturnedProcessedItemCountPtr[i]);
        } else {
            llvm_unreachable("unexpected input rate");
        }
        #ifdef PRINT_DEBUG_MESSAGES
        const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Input, i});
        b->CallPrintInt(prefix + "_processed'", mProcessedItemCount[i]);
        #endif
    }



    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = getOutputBinding(i);
        const ProcessingRate & rate = output.getRate();
        if (rate.isFixed() || rate.isPartialSum()) {
            mProducedItemCount[i] = b->CreateAdd(mAlreadyProducedPhi[i], mLinearOutputItemsPhi[i]);
        } else if (rate.isBounded() || rate.isUnknown()) {
            mProducedItemCount[i] = b->CreateLoad(mReturnedProducedItemCountPtr[i]);
        } else {
            llvm_unreachable("unexpected output rate");
        }

        #ifdef PRINT_DEBUG_MESSAGES
        const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Output, i});
        b->CallPrintInt(prefix + "_produced'", mProducedItemCount[i]);
        #endif
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInternallySynchronizedArg
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::addInternallySynchronizedArg(BuilderRef b, ArgVec & args) {
    if (mKernel->hasAttribute(AttrId::InternallySynchronized)) {
        Value * const iterationPtr = b->getScalarFieldPtr(makeKernelName(mKernelIndex) + ITERATION_COUNT_SUFFIX);
        Value * const iterationCount = b->CreateAtomicFetchAndAdd(b->getSize(1), iterationPtr);
        args.push_back(iterationCount);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addItemCountArg
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::addItemCountArg(BuilderRef b, const Binding & binding,
                                          const bool addressable,
                                          PHINode * const itemCount,
                                          ArgVec & args) {
    const ProcessingRate & rate = binding.getRate();
    if (LLVM_UNLIKELY(rate.isRelative())) {
        return nullptr;
    }
    Value * ptr = nullptr;
    if (addressable || isAddressable(binding)) {
        if (mNumOfAddressableItemCount == mAddressableItemCountPtr.size()) {
            BasicBlock * const bb = b->GetInsertBlock();
            b->SetInsertPoint(mPipelineEntryBranch);
            AllocaInst * const aic = b->CreateAlloca(b->getSizeTy(), nullptr, "AddressableItemCount");
            b->SetInsertPoint(bb);
            mAddressableItemCountPtr.push_back(aic);
        }
        ptr = mAddressableItemCountPtr[mNumOfAddressableItemCount++];
        b->CreateStore(itemCount, ptr);
        args.push_back(ptr);
    } else if (isCountable(binding)) {
        args.push_back(itemCount);
    }
    return ptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief loadItemCountsOfCountableRateStreams
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::loadItemCountsOfCountableRateStreams(BuilderRef b) {

    auto load = [](const Value * const ptr, const Binding & binding) {
        if (ptr == nullptr) {
            return false;
        }
        const ProcessingRate & rate = binding.getRate();
        return rate.isFixed() || rate.isPartialSum();
    };

    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    for (unsigned i = 0; i < numOfInputs; i++) {
        const Binding & input = getInputBinding(i);
        if (load(mReturnedProcessedItemCountPtr[i], input)) {
            mProcessedItemCount[i] = b->CreateLoad(mReturnedProcessedItemCountPtr[i]);
        }
    }
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; i++) {
        const Binding & output = getOutputBinding(i);
        if (load(mReturnedProducedItemCountPtr[i], output)) {
            mProducedItemCount[i] = b->CreateLoad(mReturnedProducedItemCountPtr[i]);
        }
    }
    BasicBlock * const exitBlock = b->GetInsertBlock();
    for (unsigned i = 0; i < numOfInputs; i++) {
        mFinalProcessedPhi[i]->addIncoming(mProcessedItemCount[i], exitBlock);
    }
    for (unsigned i = 0; i < numOfOutputs; i++) {
        mFinalProducedPhi[i]->addIncoming(mProducedItemCount[i], exitBlock);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief clearUnwrittenOutputData
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::clearUnwrittenOutputData(BuilderRef b) {
    #ifndef DISABLE_OUTPUT_ZEROING
    const auto blockWidth = b->getBitBlockWidth();
    const auto log2BlockWidth = floor_log2(blockWidth);
    Constant * const LOG_2_BLOCK_WIDTH = b->getSize(log2BlockWidth);
    Constant * const ZERO = b->getSize(0);
    Constant * const ONE = b->getSize(1);
    Constant * const BLOCK_MASK = b->getSize(blockWidth - 1);

    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const StreamSetBuffer * const buffer = getOutputBuffer(i);
        if (LLVM_UNLIKELY(isa<ExternalBuffer>(buffer))) {
            continue;
        }
        const auto itemWidth = getItemWidth(buffer->getBaseType());
        // Determine the maximum lookahead dependency on this stream and zero fill
        // the appropriate number of additional blocks.
        unsigned maximumLookahead = 0;
        RateValue strideLength{0};
        const auto bufferVertex = getOutputBufferVertex(i);
        for (const auto & e : make_iterator_range(out_edges(bufferVertex, mBufferGraph))) {
            const BufferRateData & rd = mBufferGraph[e];
            const Binding & input = rd.Binding;
            strideLength = std::max(strideLength, rd.Maximum);
            if (LLVM_UNLIKELY(input.hasLookahead())) {
                maximumLookahead = std::max(maximumLookahead, input.getLookahead());
            }
        }

        const RateValue itemWidthFactor{itemWidth, blockWidth};
        const auto numOfBlocks = ceiling(strideLength * itemWidthFactor);
        const auto numOfLookaheadBlocks = ceiling(RateValue{maximumLookahead} * itemWidthFactor);
        const auto blocksToZero = numOfBlocks + numOfLookaheadBlocks;

        const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Output, i});
        Value * const produced = mFinalProducedPhi[i];
        Value * const blockIndex = b->CreateLShr(produced, LOG_2_BLOCK_WIDTH);
        Constant * const ITEM_WIDTH = b->getSize(itemWidth);
        Value * packIndex = nullptr;
        Value * maskOffset = b->CreateAnd(produced, BLOCK_MASK);

        if (itemWidth > 1) {
            Value * const position = b->CreateMul(maskOffset, ITEM_WIDTH);
            packIndex = b->CreateLShr(position, LOG_2_BLOCK_WIDTH);
            maskOffset = b->CreateAnd(position, BLOCK_MASK);
        }

        Value * const mask = b->CreateNot(b->bitblock_mask_from(maskOffset));
        BasicBlock * const maskLoop = b->CreateBasicBlock(prefix + "_zeroFillLoop", mKernelLoopExit);
        BasicBlock * const maskExit = b->CreateBasicBlock(prefix + "_zeroFillExit", mKernelLoopExit);
        Value * const numOfStreams = buffer->getStreamSetCount(b);
        Value * const baseAddress = buffer->getBaseAddress(b);
        #ifdef PRINT_DEBUG_MESSAGES
        Value * const epoch = buffer->getStreamPackPtr(b, baseAddress, ZERO, ZERO, ZERO);
        #endif
        BasicBlock * const entry = b->GetInsertBlock();
        b->CreateBr(maskLoop);

        b->SetInsertPoint(maskLoop);
        PHINode * const streamIndex = b->CreatePHI(b->getSizeTy(), 2);
        streamIndex->addIncoming(ZERO, entry);
        Value * ptr = nullptr;
        if (itemWidth > 1) {
            ptr = buffer->getStreamPackPtr(b, baseAddress, streamIndex, blockIndex, packIndex);
        } else {
            ptr = buffer->getStreamBlockPtr(b, baseAddress, streamIndex, blockIndex);
        }
        Value * const value = b->CreateBlockAlignedLoad(ptr);
        Value * const maskedValue = b->CreateAnd(value, mask);
        b->CreateBlockAlignedStore(maskedValue, ptr);

        DataLayout DL(b->getModule());
        Type * const intPtrTy = DL.getIntPtrType(ptr->getType());
        #ifdef PRINT_DEBUG_MESSAGES
        Value * const epochInt = b->CreatePtrToInt(epoch, intPtrTy);
        #endif
        if (itemWidth > 1) {
            // Since packs are laid out sequentially in memory, it will hopefully be cheaper to zero them out here
            // because they may be within the same cache line.
            Value * const nextPackIndex = b->CreateAdd(packIndex, ONE);
            Value * const start = buffer->getStreamPackPtr(b, baseAddress, streamIndex, blockIndex, nextPackIndex);
            Value * const startInt = b->CreatePtrToInt(start, intPtrTy);
            Value * const end = buffer->getStreamPackPtr(b, baseAddress, streamIndex, blockIndex, ITEM_WIDTH);
            Value * const endInt = b->CreatePtrToInt(end, intPtrTy);
            Value * const remainingPackBytes = b->CreateSub(endInt, startInt);
            #ifdef PRINT_DEBUG_MESSAGES
            b->CallPrintInt(prefix + "_zeroFill_packStart", b->CreateSub(startInt, epochInt));
            b->CallPrintInt(prefix + "_zeroFill_remainingPackBytes", remainingPackBytes);
            #endif
            b->CreateMemZero(start, remainingPackBytes, blockWidth / 8);
        }
        BasicBlock * const maskLoopExit = b->GetInsertBlock();
        Value * const nextStreamIndex = b->CreateAdd(streamIndex, ONE);
        streamIndex->addIncoming(nextStreamIndex, maskLoopExit);
        Value * const notDone = b->CreateICmpNE(nextStreamIndex, numOfStreams);
        b->CreateCondBr(notDone, maskLoop, maskExit);

        b->SetInsertPoint(maskExit);
        // Zero out any blocks we could potentially touch
        if (blocksToZero > 1) {
            Value * const nextStrideBlockIndex = b->CreateAdd(blockIndex, ONE);
            Value * const startPtr = buffer->getStreamBlockPtr(b, baseAddress, ZERO, nextStrideBlockIndex);
            Constant * const NUM_OF_BLOCKS = b->getSize(numOfBlocks);
            Constant * const BLOCKS_TO_ZERO = b->getSize(blocksToZero);
            Value * const blockOffset = b->CreateURem(nextStrideBlockIndex, NUM_OF_BLOCKS);
            Value * const remainingBlocks = b->CreateSub(BLOCKS_TO_ZERO, blockOffset);
            Constant * const BLOCK_SIZE = b->getSize(blockWidth / 8);
            Value * const remainingBytes = b->CreateMul(remainingBlocks, BLOCK_SIZE);
            #ifdef PRINT_DEBUG_MESSAGES            
            Value * const startPtrInt = b->CreatePtrToInt(startPtr, intPtrTy);
            b->CallPrintInt(prefix + "_zeroFill_bufferStart", b->CreateSub(startPtrInt, epochInt));
            b->CallPrintInt(prefix + "_zeroFill_remainingBufferBytes", remainingBytes);
            #endif
            b->CreateMemZero(startPtr, remainingBytes, blockWidth / 8);
        }
    }
    #endif
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeFullyProcessedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::computeFullyProcessedItemCounts(BuilderRef b) {
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = getInputBinding(i);
        Value * processed = nullptr;
        if (mUpdatedProcessedDeferredPhi[i]) {
            processed = mUpdatedProcessedDeferredPhi[i];
        } else {
            processed = mUpdatedProcessedPhi[i];
        }
        processed = truncateBlockSize(b, input, processed);
        mFullyProcessedItemCount[i] = processed;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeFullyProducedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::computeFullyProducedItemCounts(BuilderRef b) {

    // TODO: we only need to consider the blocksize attribute if it's possible this
    // stream could be read before being fully written. This might occur if one of
    // it's consumers has a non-Fixed rate that does not have a matching BlockSize
    // attribute.

    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = getOutputBinding(i);
        Value * produced = mUpdatedProducedPhi[i];
        if (LLVM_UNLIKELY(output.hasAttribute(AttrId::Delayed))) {
            const auto & D = output.findAttribute(AttrId::Delayed);
            Value * const delayed = b->CreateSaturatingSub(produced, b->getSize(D.amount()));
            Value * const terminated = b->CreateIsNotNull(mTerminatedPhi);
            produced = b->CreateSelect(terminated, produced, delayed);
        }
        produced = truncateBlockSize(b, output, produced);
        mFullyProducedItemCount[i]->addIncoming(produced, mKernelLoopExitPhiCatch);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getTotalItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getLocallyAvailableItemCount(BuilderRef /* b */, const unsigned inputPort) const {
    const auto bufferVertex = getInputBufferVertex(inputPort);
    return mLocallyAvailableItems[getBufferIndex(bufferVertex)];
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addLookahead
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::addLookahead(BuilderRef b, const unsigned inputPort, Value * const itemCount) const {
    Constant * const lookAhead = getLookahead(b, inputPort);
    if (LLVM_LIKELY(lookAhead == nullptr)) {
        return itemCount;
    }
    return b->CreateAdd(itemCount, lookAhead);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief subtractLookahead
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::subtractLookahead(BuilderRef b, const unsigned inputPort, Value * const itemCount) {
    Constant * const lookAhead = getLookahead(b, inputPort);
    if (LLVM_LIKELY(lookAhead == nullptr)) {
        return itemCount;
    }
    Value * const reducedItemCount = b->CreateSub(itemCount, lookAhead);
    return b->CreateSelect(isClosed(b, inputPort), itemCount, reducedItemCount);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getLookahead
 ** ------------------------------------------------------------------------------------------------------------- */
Constant * PipelineCompiler::getLookahead(BuilderRef b, const unsigned inputPort) const {
    const Binding & input = getInputBinding(inputPort);
    if (LLVM_UNLIKELY(input.hasLookahead())) {
        return b->getSize(input.getLookahead());
    }
    return nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief maskBlockSize
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::truncateBlockSize(BuilderRef b, const Binding & binding, Value * itemCount) const {
    // TODO: if we determine all of the inputs of a stream have a blocksize attribute, or the output has one,
    // we can skip masking it on input



    if (LLVM_UNLIKELY(binding.hasAttribute(AttrId::BlockSize))) {
        // If the input rate has a block size attribute then --- for the purpose of determining how many
        // items have been consumed --- we consider a stream set to be fully processed when an entire
        // stride has been processed.
        Constant * const BLOCK_WIDTH = b->getSize(b->getBitBlockWidth());
        Value * const maskedItemCount = b->CreateAnd(itemCount, ConstantExpr::getNeg(BLOCK_WIDTH));
        Value * const terminated = hasKernelTerminated(b, mKernelIndex);
        itemCount = b->CreateSelect(terminated, itemCount, maskedItemCount);
    }
    return itemCount;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitializationFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getFunctionFromKernelState(BuilderRef b, Type * const type, const std::string & suffix) const {
    const auto prefix = makeFamilyPrefix(mKernelIndex);
    b->setKernel(mPipelineKernel);
    Value * const funcPtr = b->getScalarField(prefix + suffix);
    if (LLVM_UNLIKELY(mCheckAssertions)) {
        b->CreateAssert(funcPtr, prefix + suffix + " is null");
    }
    b->setKernel(mKernel);
    return b->CreateBitCast(funcPtr, type);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitializationFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getInitializeFunction(BuilderRef b) const {
    Function * const init = mKernel->getInitializeFunction(b);
    if (mKernel->hasFamilyName()) {
        return getFunctionFromKernelState(b, init->getType(), INITIALIZE_FUNCTION_POINTER_SUFFIX);
    }
    return init;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitializationThreadLocalFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getInitializeThreadLocalFunction(BuilderRef b) const {
    Function * const init = mKernel->getInitializeThreadLocalFunction(b);
    if (mKernel->hasFamilyName()) {
        return getFunctionFromKernelState(b, init->getType(), INITIALIZE_THREAD_LOCAL_FUNCTION_POINTER_SUFFIX);
    }
    return init;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getDoSegmentFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getDoSegmentFunction(BuilderRef b) const {
    Function * const doSegment = mKernel->getDoSegmentFunction(b);
    if (mKernel->hasFamilyName()) {
        return getFunctionFromKernelState(b, doSegment->getType(), DO_SEGMENT_FUNCTION_POINTER_SUFFIX);
    }
    return doSegment;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitializationThreadLocalFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getFinalizeThreadLocalFunction(BuilderRef b) const {
    Function * const init = mKernel->getFinalizeThreadLocalFunction(b);
    if (mKernel->hasFamilyName()) {
        return getFunctionFromKernelState(b, init->getType(), FINALIZE_THREAD_LOCAL_FUNCTION_POINTER_SUFFIX);
    }
    return init;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getFinalizeFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getFinalizeFunction(BuilderRef b) const {
    Function * const term = mKernel->getFinalizeFunction(b);
    if (mKernel->hasFamilyName()) {
        return getFunctionFromKernelState(b, term->getType(), FINALIZE_FUNCTION_POINTER_SUFFIX);
    }
    return term;
}

#if 0

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief verifyInputItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::verifyInputItemCount(BuilderRef b, Value * processed, const unsigned inputPort) const {
    if (LLVM_UNLIKELY(mCheckAssertions)) {
        const Binding & input = getInputBinding(inputPort);
        Value * const expected = b->CreateAdd(mAlreadyProcessedPhi[inputPort], mLinearInputItemsPhi[inputPort]);
        itemCountSanityCheck(b, input, "processed", processed, expected);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief verifyOutputItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::verifyOutputItemCount(BuilderRef b, Value * produced, const unsigned outputPort) const {
    if (LLVM_UNLIKELY(mCheckAssertions)) {
        const Binding & output = getOutputBinding(outputPort);
        Value * const expected = b->CreateAdd(mAlreadyProducedPhi[outputPort], mLinearOutputItemsPhi[outputPort]);
        itemCountSanityCheck(b, output, "produced", produced, expected);
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief itemCountSanityCheck
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::itemCountSanityCheck(BuilderRef b, const Binding & binding,
                                            const std::string & label,
                                            Value * const itemCount, Value * const expected) const {

    const auto prefix = makeBufferName(mKernelIndex, binding);
    const auto lb = mKernel->getLowerBound(binding);
    if (lb > 0 && !binding.hasAttribute(AttrId::Deferred)) {
        Constant * const strideSize = b->getSize(ceiling(lb * mKernel->getStride()));
        Value * hasEnough = b->CreateICmpULE(itemCount, strideSize);
        hasEnough = b->CreateOr(hasEnough, mTerminationExplicitly);
        b->CreateAssert(hasEnough, prefix + " " + label + " fewer items than expected");
    }
    Value * const withinBounds = b->CreateICmpULE(itemCount, expected);
    b->CreateAssert(withinBounds, prefix + " " + label + " more items than expected");

}

#endif

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief phiOutItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::phiOutItemCounts(BuilderRef b,
                                        const Vec<Value *> & accessibleItems,
                                        const Vec<Value *> & inputEpoch,
                                        const Vec<Value *> & writableItems,
                                        Value * const fixedRateFactor) const {
    BasicBlock * const exitBlock = b->GetInsertBlock();
    const auto numOfInputs = accessibleItems.size();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        mLinearInputItemsPhi[i]->addIncoming(accessibleItems[i], exitBlock);
        mInputEpochPhi[i]->addIncoming(inputEpoch[i], exitBlock);
    }
    const auto numOfOutputs = writableItems.size();
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        mLinearOutputItemsPhi[i]->addIncoming(writableItems[i], exitBlock);
    }
    if (fixedRateFactor) {
        mFixedRateFactorPhi->addIncoming(fixedRateFactor, exitBlock);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief resetMemoizedFields
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::resetMemoizedFields() {
    const auto numOfInputs = in_degree(mKernelIndex, mBufferGraph);
    reset(mIsInputZeroExtended, numOfInputs);
    reset(mInitiallyProcessedItemCount, numOfInputs);
    reset(mInitiallyProcessedDeferredItemCount, numOfInputs);
    reset(mAlreadyProcessedPhi, numOfInputs);
    reset(mAlreadyProcessedDeferredPhi, numOfInputs);
    reset(mInputEpoch, numOfInputs);
    reset(mInputEpochPhi, numOfInputs);
    reset(mFirstInputStrideLength, numOfInputs);
    reset(mAccessibleInputItems, numOfInputs);
    reset(mLinearInputItemsPhi, numOfInputs);
    reset(mReturnedProcessedItemCountPtr, numOfInputs);
    reset(mProcessedItemCount, numOfInputs);
    reset(mProcessedDeferredItemCount, numOfInputs);
    reset(mFinalProcessedPhi, numOfInputs);
    reset(mUpdatedProcessedPhi, numOfInputs);
    reset(mUpdatedProcessedDeferredPhi, numOfInputs);
    reset(mFullyProcessedItemCount, numOfInputs);
    const auto numOfOutputs = out_degree(mKernelIndex, mBufferGraph);
    reset(mInitiallyProducedItemCount, numOfOutputs);
    reset(mInitiallyProducedDeferredItemCount, numOfOutputs);
    reset(mAlreadyProducedPhi, numOfOutputs);
    reset(mAlreadyProducedDeferredPhi, numOfOutputs);
    reset(mFirstOutputStrideLength, numOfOutputs);
    reset(mWritableOutputItems, numOfOutputs);
    reset(mConsumedItemCount, numOfOutputs);
    reset(mLinearOutputItemsPhi, numOfOutputs);
    reset(mReturnedProducedItemCountPtr, numOfOutputs);
    reset(mProducedItemCount, numOfOutputs);
    reset(mProducedDeferredItemCount, numOfOutputs);
    reset(mFinalProducedPhi, numOfOutputs);
    reset(mUpdatedProducedPhi, numOfOutputs);
    reset(mUpdatedProducedDeferredPhi, numOfOutputs);
    reset(mFullyProducedItemCount, numOfOutputs);
}

}
