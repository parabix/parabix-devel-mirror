#ifndef BUFFER_MANIPULATION_LOGIC_HPP
#define BUFFER_MANIPULATION_LOGIC_HPP

#include "../pipeline_compiler.hpp"

namespace kernel {


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief allocateLocalZeroExtensionSpace
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::allocateLocalZeroExtensionSpace(BuilderRef b, BasicBlock * const insertBefore) const {
    #ifndef DISABLE_ZERO_EXTEND
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    const auto strideSize = mKernel->getStride();
    const auto blockWidth = b->getBitBlockWidth();
    Value * requiredSpace = nullptr;

    Constant * const ZERO = b->getSize(0);
    Constant * const ONE = b->getSize(1);
    Value * const numOfStrides = b->CreateUMax(mNumOfLinearStrides, ONE);

    for (unsigned i = 0; i < numOfInputs; ++i) {
        const StreamSetPort port{PortType::Input, i};
        if (mIsInputZeroExtended(port)) {
            const auto bufferVertex = getInputBufferVertex(port);
            const BufferNode & bn = mBufferGraph[bufferVertex];
            const Binding & input = getInputBinding(port);

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
            requiredBytes = b->CreateSelect(mIsInputZeroExtended(port), requiredBytes, ZERO);
            requiredSpace = b->CreateUMax(requiredSpace, requiredBytes);
        }
    }
    assert (requiredSpace);
    const auto prefix = makeKernelName(mKernelIndex);
    BasicBlock * const entry = b->GetInsertBlock();
    BasicBlock * const expandZeroExtension =
        b->CreateBasicBlock(prefix + "_expandZeroExtensionBuffer", insertBefore);
    BasicBlock * const hasSufficientZeroExtendSpace =
        b->CreateBasicBlock(prefix + "_hasSufficientZeroExtendSpace", insertBefore);
    Value * const currentSpace = b->CreateLoad(mZeroExtendSpace);
    Value * const currentBuffer = b->CreateLoad(mZeroExtendBuffer);
    requiredSpace = b->CreateRoundUp(requiredSpace, b->getSize(b->getCacheAlignment()));

    Value * const largeEnough = b->CreateICmpUGE(currentSpace, requiredSpace);
    b->CreateLikelyCondBr(largeEnough, hasSufficientZeroExtendSpace, expandZeroExtension);

    b->SetInsertPoint(expandZeroExtension);
    assert (b->getCacheAlignment() >= (b->getBitBlockWidth() / 8));
    b->CreateFree(currentBuffer);
    Value * const newBuffer = b->CreateCacheAlignedMalloc(requiredSpace);
    b->CreateMemZero(newBuffer, requiredSpace, b->getCacheAlignment());
    b->CreateStore(requiredSpace, mZeroExtendSpace);
    b->CreateStore(newBuffer, mZeroExtendBuffer);
    b->CreateBr(hasSufficientZeroExtendSpace);

    b->SetInsertPoint(hasSufficientZeroExtendSpace);
    PHINode * const zeroBuffer = b->CreatePHI(b->getVoidPtrTy(), 2);
    zeroBuffer->addIncoming(currentBuffer, entry);
    zeroBuffer->addIncoming(newBuffer, expandZeroExtension);
    return zeroBuffer;
    #else
    return nullptr;
    #endif
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getZeroExtendedInputVirtualBaseAddresses
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::getZeroExtendedInputVirtualBaseAddresses(BuilderRef b,
                                                                const Vec<Value *> & baseAddresses,
                                                                Value * const zeroExtensionSpace,
                                                                Vec<Value *> & zeroExtendedVirtualBaseAddress) const {
    #ifndef DISABLE_ZERO_EXTEND
    for (const auto e : make_iterator_range(in_edges(mKernelIndex, mBufferGraph))) {
        const BufferRateData & rt = mBufferGraph[e];
        assert (rt.Port.Type == PortType::Input);
        Value * const zeroExtended = mIsInputZeroExtended(rt.Port);
        if (zeroExtended) {
            PHINode * processed = nullptr;
            if (mAlreadyProcessedDeferredPhi(rt.Port)) {
                processed = mAlreadyProcessedDeferredPhi(rt.Port);
            } else {
                processed = mAlreadyProcessedPhi(rt.Port);
            }
            const BufferNode & bn = mBufferGraph[source(e, mBufferGraph)];
            const Binding & binding = rt.Binding;
            const StreamSetBuffer * const buffer = bn.Buffer;

            Constant * const LOG_2_BLOCK_WIDTH = b->getSize(floor_log2(b->getBitBlockWidth()));
            Constant * const ZERO = b->getSize(0);
            PointerType * const bufferType = buffer->getPointerType();
            Value * const blockIndex = b->CreateLShr(processed, LOG_2_BLOCK_WIDTH);

            // allocateLocalZeroExtensionSpace guarantees this will be large enough to satisfy the kernel
            ExternalBuffer tmp(b, binding.getType(), true, buffer->getAddressSpace());
            Value * zeroExtension = b->CreatePointerCast(zeroExtensionSpace, bufferType);
            Value * addr = tmp.getStreamBlockPtr(b, zeroExtension, ZERO, b->CreateNeg(blockIndex));
            addr = b->CreatePointerCast(addr, bufferType);
            const auto i = rt.Port.Number;
            assert (addr->getType() == baseAddresses[i]->getType());
            addr = b->CreateSelect(zeroExtended, addr, baseAddresses[i]);
            zeroExtendedVirtualBaseAddress[i] = addr;
        }
    }
    #endif
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief zeroInputAfterFinalItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::zeroInputAfterFinalItemCount(BuilderRef b, const Vec<Value *> & accessibleItems, Vec<Value *> & inputBaseAddresses) {
    #ifndef DISABLE_INPUT_ZEROING
    const auto numOfInputs = accessibleItems.size();
    const auto blockWidth = b->getBitBlockWidth();
    const auto log2BlockWidth = floor_log2(blockWidth);
    Constant * const BLOCK_MASK = b->getSize(blockWidth - 1);
    Constant * const LOG_2_BLOCK_WIDTH = b->getSize(log2BlockWidth);
    Constant * const ZERO = b->getSize(0);
    Constant * const ONE = b->getSize(1);

    for (unsigned i = 0; i < numOfInputs; ++i) {
        const StreamSetPort port(PortType::Input, i);
        const Binding & input = getInputBinding(port);
        const ProcessingRate & rate = input.getRate();

        // TODO: if this streamset has 0 streams, it exists only to have a produced/processed count. Ignore it.

        if (LLVM_LIKELY(rate.isFixed())) {

            AllocaInst * bufferStorage = nullptr;
            bool reuse = false;
            if (mNumOfTruncatedInputBuffers < mTruncatedInputBuffer.size()) {
                bufferStorage = mTruncatedInputBuffer[i];
                reuse = true;
            } else { // create a stack entry for this buffer at the start of the pipeline
                PointerType * const int8PtrTy = b->getInt8PtrTy();
                bufferStorage = b->CreateAllocaAtEntryPoint(int8PtrTy);
                Instruction * const nextNode = bufferStorage->getNextNode(); assert (nextNode);
                new StoreInst(ConstantPointerNull::get(int8PtrTy), bufferStorage, nextNode);
                mTruncatedInputBuffer.push_back(bufferStorage);
            }
            ++mNumOfTruncatedInputBuffers;

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

            Value * const tooMany = b->CreateICmpULT(accessibleItems[i], mAccessibleInputItems(port));
            Value * computeMask = tooMany;
            if (mIsInputZeroExtended(port)) {
                computeMask = b->CreateAnd(tooMany, b->CreateNot(mIsInputZeroExtended(port)));
            }
            BasicBlock * const entryBlock = b->GetInsertBlock();
            b->CreateUnlikelyCondBr(computeMask, maskedInput, selectedInput);

            b->SetInsertPoint(maskedInput);

            // if this is a deferred fixed rate stream, we cannot be sure how many
            // blocks will have to be provided to the kernel in order to mask out
            // the truncated input stream.

            #ifdef PRINT_DEBUG_MESSAGES
            debugPrint(b, prefix + " truncating item count from %" PRIu64 " to %" PRIu64,
                       mAccessibleInputItems(inputPort), accessibleItems[i]);
            #endif

            if (reuse) {
                b->CreateFree(b->CreateLoad(bufferStorage));
            }

            // TODO: if we can prove that this will be the last kernel invocation that will ever touch this stream)
            // and is not an input to the pipeline (which we cannot prove will have space after the last item), we
            // can avoid copying the buffer and instead just mask out the surpressed items.

            ExternalBuffer tmp(b, input.getType(), true, 0);

            Value * const start = b->CreateLShr(mAlreadyProcessedPhi(port), LOG_2_BLOCK_WIDTH);

            DataLayout DL(b->getModule());
            Value * const inputAddress = inputBaseAddresses[i];
            Value * const startPtr = tmp.getStreamBlockPtr(b, inputAddress, ZERO, start);
            Type * const intPtrTy = DL.getIntPtrType(startPtr->getType());
            Value * const startPtrInt = b->CreatePtrToInt(startPtr, intPtrTy);

            Value * const limit = b->CreateAdd(start, STRIDES_PER_SEGMENT);
            Value * const limitPtr = tmp.getStreamBlockPtr(b, inputAddress, ZERO, limit);
            Value * const limitPtrInt = b->CreatePtrToInt(limitPtr, intPtrTy);

            Value * const strideBytes = b->CreateSub(limitPtrInt, startPtrInt);

            b->CallPrintInt("strideBytes", strideBytes);

            Value * segmentBytes = strideBytes;

            Value * initial = start;
            Value * initialPtr = startPtr;
            Value * initialPtrInt = startPtrInt;

            Value * end = start;
            Value * fullBytesToCopy = nullptr;

            if (stridesPerSegment != 1 || input.isDeferred()) {
                if (input.isDeferred()) {
                    initial = b->CreateLShr(mAlreadyProcessedDeferredPhi(port), LOG_2_BLOCK_WIDTH);
                    initialPtr = tmp.getStreamBlockPtr(b, inputAddress, ZERO, initial);
                    initialPtrInt = b->CreatePtrToInt(initialPtr, intPtrTy);
                }
                // if a kernel reads in multiple strides of data per segment, we may be able to
                // copy over a portion of it with a single memcpy.
                Value * endPtrInt = startPtrInt;
                if (stridesPerSegment  != 1) {
                    end = b->CreateAdd(mAlreadyProcessedPhi(port), accessibleItems[i]);
                    end = b->CreateLShr(end, LOG_2_BLOCK_WIDTH);
                    Value * const endPtr = tmp.getStreamBlockPtr(b, inputAddress, ZERO, end);
                    endPtrInt = b->CreatePtrToInt(endPtr, intPtrTy);
                }
                fullBytesToCopy = b->CreateSub(endPtrInt, initialPtrInt);
                segmentBytes = b->CreateAdd(fullBytesToCopy, strideBytes);
            }

            b->CallPrintInt("segmentBytes", segmentBytes);

            Value * maskedBuffer = b->CreateAlignedMalloc(segmentBytes, blockWidth / 8);

            b->CallPrintInt("maskedBuffer", maskedBuffer);

            b->CreateStore(maskedBuffer, bufferStorage);
            PointerType * const bufferType = tmp.getPointerType();
            maskedBuffer = b->CreatePointerCast(maskedBuffer, bufferType, "maskedBuffer");

            if (fullBytesToCopy) {
                b->CallPrintInt("fullBytesToCopy", fullBytesToCopy);

                b->CreateMemCpy(maskedBuffer, initialPtr, fullBytesToCopy, blockWidth / 8);
            }

            Value * maskedAddress = tmp.getStreamBlockPtr(b, maskedBuffer, ZERO, b->CreateNeg(initial));
            maskedAddress = b->CreatePointerCast(maskedAddress, bufferType);

            b->CallPrintInt("maskedAddress", maskedAddress);

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

            Value * inputPtr = tmp.getStreamBlockPtr(b, inputAddress, streamIndex, end);
            Value * outputPtr = tmp.getStreamBlockPtr(b, maskedAddress, streamIndex, end);

            if (itemWidth > 1) {
                Value * const endPtr = inputPtr;
                Value * const endPtrInt = b->CreatePtrToInt(endPtr, intPtrTy);
                inputPtr = tmp.getStreamPackPtr(b, inputAddress, streamIndex, end, packIndex);
                Value * const inputPtrInt = b->CreatePtrToInt(inputPtr, intPtrTy);
                Value * const bytesToCopy = b->CreateSub(inputPtrInt, endPtrInt);
                b->CreateMemCpy(outputPtr, endPtr, bytesToCopy, blockWidth / 8);
                outputPtr = tmp.getStreamPackPtr(b, maskedAddress, streamIndex, end, packIndex);
            }

            assert (inputPtr->getType() == outputPtr->getType());
            Value * const val = b->CreateBlockAlignedLoad(inputPtr);
            Value * const maskedVal = b->CreateAnd(val, mask);
            b->CreateBlockAlignedStore(maskedVal, outputPtr);

            if (itemWidth > 1) {
                Value * const nextPackIndex = b->CreateAdd(packIndex, ONE);
                Value * const clearPtr = tmp.getStreamPackPtr(b, maskedAddress, streamIndex, end, nextPackIndex);
                Value * const clearPtrInt = b->CreatePtrToInt(clearPtr, intPtrTy);
                Value * const clearEndPtr = tmp.getStreamPackPtr(b, maskedAddress, streamIndex, end, ITEM_WIDTH);
                Value * const clearEndPtrInt = b->CreatePtrToInt(clearEndPtr, intPtrTy);
                Value * const bytesToClear = b->CreateSub(clearEndPtrInt, clearPtrInt);
                b->CreateMemZero(clearPtr, bytesToClear, blockWidth / 8);
            }

            Value * const nextIndex = b->CreateAdd(streamIndex, ONE);
            Value * const notDone = b->CreateICmpNE(nextIndex, numOfStreams);
            streamIndex->addIncoming(nextIndex, maskedInputLoop);


            BasicBlock * const maskedInputLoopExit = b->GetInsertBlock();
            b->CreateCondBr(notDone, maskedInputLoop, selectedInput);

            b->SetInsertPoint(selectedInput);
            PHINode * const phi = b->CreatePHI(bufferType, 2);
            phi->addIncoming(inputAddress, entryBlock);
            phi->addIncoming(maskedAddress, maskedInputLoopExit);
            inputBaseAddresses[i] = phi;
        }
    }
    #endif
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
        const StreamSetPort port{PortType::Output, i};
        const StreamSetBuffer * const buffer = getOutputBuffer(port);
        if (LLVM_UNLIKELY(isa<ExternalBuffer>(buffer))) {
            // If this stream is either controlled by this kernel or is an external
            // stream, any clearing of data is the responsibility of the owner.
            // Simply ignore any external buffers for the purpose of zeroing out
            // unnecessary data.
            continue;
        }
        const auto itemWidth = getItemWidth(buffer->getBaseType());

        const auto prefix = makeBufferName(mKernelIndex, port);
        Value * const produced = mFinalProducedPhi(port);
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
        const auto bufferVertex = getOutputBufferVertex(port);
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




}

#endif // BUFFER_MANIPULATION_LOGIC_HPP
