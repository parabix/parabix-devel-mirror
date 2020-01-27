#include "pipeline_compiler.hpp"

#include <llvm/Support/ErrorHandling.h>

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief beginKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::setActiveKernel(BuilderRef b, const unsigned index) {
    assert (index >= FirstKernel && index <= LastKernel);
    mKernelIndex = index;
    mKernel = getKernel(index);
    mKernelHandle = nullptr;
    if (LLVM_LIKELY(mKernel->isStateful())) {
        Value * handle = b->getScalarField(makeKernelName(index));
        if (LLVM_UNLIKELY(mKernel->externallyInitialized())) {
            handle = b->CreatePointerCast(handle, mKernel->getSharedStateType()->getPointerTo());
        }
        mKernelHandle = handle;
    }
    if (LLVM_UNLIKELY(mCheckAssertions)) {
        SmallVector<char, 256> tmp;
        raw_svector_ostream out(tmp);
        out << mKernelIndex << "." << mKernel->getName();
        mKernelAssertionName = b->GetString(out.str());
    }
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
        const Binding & input = getInputBinding(StreamSetPort{PortType::Input, i});
        const ProcessingRate & rate = input.getRate();

        // TODO: if this streamset has 0 streams, it exists only to have a produced/processed count. Ignore it.

        inputBaseAddress[i] = mInputEpoch[i];

        assert (inputBaseAddress[i]);

        #ifndef DISABLE_INPUT_ZEROING
        if (LLVM_LIKELY(rate.isFixed())) {

            // create a stack entry for this buffer at the start of the pipeline
            PointerType * const int8PtrTy = b->getInt8PtrTy();
            AllocaInst * const bufferStorage = b->CreateAllocaAtEntryPoint(int8PtrTy);
            Instruction * const nextNode = bufferStorage->getNextNode(); assert (nextNode);
            new StoreInst(ConstantPointerNull::get(int8PtrTy), bufferStorage, nextNode);
            mTruncatedInputBuffer.push_back(bufferStorage);

            const auto inputPort = StreamSetPort{PortType::Input, i};
            const auto prefix = makeBufferName(mKernelIndex, inputPort);
            const StreamSetBuffer * const buffer = getInputBuffer(inputPort);
            const auto itemWidth = getItemWidth(buffer->getBaseType());
            Constant * const ITEM_WIDTH = b->getSize(itemWidth);
            const Rational stridesPerBlock(mKernel->getStride(), blockWidth);
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
            debugPrint(b, prefix + " truncating item count from %" PRIu64 " to %" PRIu64, mAccessibleInputItems[i], accessibleItems[i]);
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
                const auto inputPort = StreamSetPort{PortType::Input, i};
                const auto bufferVertex = getInputBufferVertex(inputPort);
                const BufferNode & bn = mBufferGraph[bufferVertex];
                const Binding & input = getInputBinding(inputPort);

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
        const auto outputPort = StreamSetPort{PortType::Output, i};
        const StreamSetBuffer * const buffer = getOutputBuffer(outputPort);
        if (LLVM_UNLIKELY(isa<ExternalBuffer>(buffer))) {
            // If this stream is either controlled by this kernel or is an external
            // stream, any clearing of data is the responsibility of the owner.
            // Simply ignore any external buffers for the purpose of zeroing out
            // unnecessary data.
            continue;
        }
        const auto itemWidth = getItemWidth(buffer->getBaseType());

        const auto prefix = makeBufferName(mKernelIndex, outputPort);
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
        BasicBlock * const maskLoop = b->CreateBasicBlock(prefix + "_zeroFillLoop", mKernelInsufficientIOExit);
        BasicBlock * const maskExit = b->CreateBasicBlock(prefix + "_zeroFillExit", mKernelInsufficientIOExit);
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
            debugPrint(b, prefix + "_zeroFill_packStart = %" PRIu64, b->CreateSub(startInt, epochInt));
            debugPrint(b, prefix + "_zeroFill_remainingPackBytes = %" PRIu64, remainingPackBytes);
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

        Rational strideLength{0};
        const auto bufferVertex = getOutputBufferVertex(outputPort);
        for (const auto e : make_iterator_range(out_edges(bufferVertex, mBufferGraph))) {
            const BufferRateData & rd = mBufferGraph[e];
            const Binding & input = rd.Binding;

            Rational R{rd.Maximum};
            if (LLVM_UNLIKELY(input.hasLookahead())) {
                R += input.getLookahead();
            }
            strideLength = std::max(strideLength, R);
        }

        const auto blocksToZero = ceiling(strideLength * Rational{1, blockWidth});

        if (blocksToZero > 1) {
            Value * const nextBlockIndex = b->CreateAdd(blockIndex, ONE);
            Value * const nextOffset = buffer->modByCapacity(b, nextBlockIndex);
            Value * const startPtr = buffer->StreamSetBuffer::getStreamBlockPtr(b, baseAddress, ZERO, nextOffset);
            Value * const startPtrInt = b->CreatePtrToInt(startPtr, intPtrTy);
            Constant * const BLOCKS_TO_ZERO = b->getSize(blocksToZero);
            Value * const endOffset = b->CreateRoundUp(nextOffset, BLOCKS_TO_ZERO);
            Value * const endPtr = buffer->StreamSetBuffer::getStreamBlockPtr(b, baseAddress, ZERO, endOffset);
            Value * const endPtrInt = b->CreatePtrToInt(endPtr, intPtrTy);
            Value * const remainingBytes = b->CreateSub(endPtrInt, startPtrInt);
            #ifdef PRINT_DEBUG_MESSAGES
            debugPrint(b, prefix + "_zeroFill_bufferStart = %" PRIu64, b->CreateSub(startPtrInt, epochInt));
            debugPrint(b, prefix + "_zeroFill_remainingBufferBytes = %" PRIu64, remainingBytes);
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
        const Binding & input = getInputBinding(StreamSetPort{PortType::Input, i});
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
        const Binding & output = getOutputBinding(StreamSetPort{PortType::Output, i});
        Value * produced = mUpdatedProducedPhi[i];
        if (LLVM_UNLIKELY(output.hasAttribute(AttrId::Delayed))) {
            const auto & D = output.findAttribute(AttrId::Delayed);
            Value * const delayed = b->CreateSaturatingSub(produced, b->getSize(D.amount()));
            Value * const terminated = b->CreateIsNotNull(mTerminatedAtLoopExitPhi);
            produced = b->CreateSelect(terminated, produced, delayed);
        }
        produced = truncateBlockSize(b, output, produced);
        mFullyProducedItemCount[i]->addIncoming(produced, mKernelLoopExitPhiCatch);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getTotalItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getLocallyAvailableItemCount(BuilderRef /* b */, const StreamSetPort inputPort) const {
    const auto bufferVertex = getInputBufferVertex(inputPort);
    return mLocallyAvailableItems[getBufferIndex(bufferVertex)];
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
                        mKernelAssertionName,
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
Value * PipelineCompiler::getKernelInitializeFunction(BuilderRef b) const {    
    Function * const init = mKernel->getInitializeFunction(b);
    assert (!mKernel->hasFamilyName());
    return init;
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
 * @brief reset
 ** ------------------------------------------------------------------------------------------------------------- */
namespace {
template <typename Vec>
inline void reset(Vec & vec, const size_t n) {
    vec.resize(n);
    std::fill_n(vec.begin(), n, nullptr);
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
    reset(mReturnedOutputVirtualBaseAddressPtr, numOfOutputs);
    reset(mReturnedProducedItemCountPtr, numOfOutputs);
    reset(mProducedItemCount, numOfOutputs);
    reset(mProducedDeferredItemCount, numOfOutputs);
    reset(mFinalProducedPhi, numOfOutputs);
    reset(mUpdatedProducedPhi, numOfOutputs);
    reset(mUpdatedProducedDeferredPhi, numOfOutputs);
    reset(mFullyProducedItemCount, numOfOutputs);
    mNumOfAddressableItemCount = 0;
    mNumOfVirtualBaseAddresses = 0;
    mHasClosedInputStream = nullptr;
}

}
