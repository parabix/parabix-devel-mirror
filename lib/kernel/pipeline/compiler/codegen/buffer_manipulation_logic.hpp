#ifndef BUFFER_MANIPULATION_LOGIC_HPP
#define BUFFER_MANIPULATION_LOGIC_HPP

#include "../pipeline_compiler.hpp"

namespace kernel {


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief allocateLocalZeroExtensionSpace
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::allocateLocalZeroExtensionSpace(BuilderRef b, BasicBlock * const insertBefore) const {
    #ifndef DISABLE_ZERO_EXTEND
    const auto strideSize = mKernel->getStride();
    const auto blockWidth = b->getBitBlockWidth();
    Value * requiredSpace = nullptr;

    Constant * const ZERO = b->getSize(0);
    Constant * const ONE = b->getSize(1);
    Value * const numOfStrides = b->CreateUMax(mNumOfLinearStrides, ONE);

    for (const auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {

        const BufferPort & br = mBufferGraph[e];
        if (br.IsZeroExtended) {

            assert (HasZeroExtendedStream);

            const auto streamSet = source(e, mBufferGraph);
            const BufferNode & bn = mBufferGraph[streamSet];
            const Binding & input = br.Binding;

            const auto itemWidth = getItemWidth(input.getType());
            Constant * const strideFactor = b->getSize(itemWidth * strideSize / 8);
            Value * requiredBytes = b->CreateMul(numOfStrides, strideFactor); assert (requiredBytes);
            if (br.LookAhead) {
                const auto lh = (br.LookAhead * itemWidth);
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
            requiredBytes = b->CreateSelect(mIsInputZeroExtended[br.Port], requiredBytes, ZERO, "zeroExtendRequiredBytes");
            requiredSpace = b->CreateUMax(requiredSpace, requiredBytes);
        }
    }
    assert (requiredSpace);    
    const auto prefix = makeKernelName(mKernelId);
    BasicBlock * const entry = b->GetInsertBlock();
    BasicBlock * const expandZeroExtension =
        b->CreateBasicBlock(prefix + "_expandZeroExtensionBuffer", insertBefore);
    BasicBlock * const hasSufficientZeroExtendSpace =
        b->CreateBasicBlock(prefix + "_hasSufficientZeroExtendSpace", insertBefore);

    Value * const zeroExtendSpace = b->getScalarFieldPtr(ZERO_EXTENDED_SPACE);
    Value * const currentSpace = b->CreateLoad(zeroExtendSpace);

    Value * const zeroExtendBuffer = b->getScalarFieldPtr(ZERO_EXTENDED_BUFFER);
    Value * const currentBuffer = b->CreateLoad(zeroExtendBuffer);

    requiredSpace = b->CreateRoundUp(requiredSpace, b->getSize(b->getCacheAlignment()));

    Value * const largeEnough = b->CreateICmpUGE(currentSpace, requiredSpace);
    b->CreateLikelyCondBr(largeEnough, hasSufficientZeroExtendSpace, expandZeroExtension);

    b->SetInsertPoint(expandZeroExtension);
    assert (b->getCacheAlignment() >= (b->getBitBlockWidth() / 8));
    b->CreateFree(currentBuffer);
    Value * const newBuffer = b->CreateCacheAlignedMalloc(requiredSpace);
    b->CreateMemZero(newBuffer, requiredSpace, b->getCacheAlignment());
    b->CreateStore(requiredSpace, zeroExtendSpace);
    b->CreateStore(newBuffer, zeroExtendBuffer);
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
    for (const auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
        const BufferPort & rt = mBufferGraph[e];
        assert (rt.Port.Type == PortType::Input);
        Value * const zeroExtended = mIsInputZeroExtended[rt.Port];
        if (zeroExtended) {
            PHINode * processed = nullptr;
            if (mAlreadyProcessedDeferredPhi[rt.Port]) {
                processed = mAlreadyProcessedDeferredPhi[rt.Port];
            } else {
                processed = mAlreadyProcessedPhi[rt.Port];
            }
            const BufferNode & bn = mBufferGraph[source(e, mBufferGraph)];
            const Binding & binding = rt.Binding;
            const StreamSetBuffer * const buffer = bn.Buffer;

            Constant * const LOG_2_BLOCK_WIDTH = b->getSize(floor_log2(b->getBitBlockWidth()));
            Constant * const ZERO = b->getSize(0);
            PointerType * const bufferType = buffer->getPointerType();
            Value * const blockIndex = b->CreateLShr(processed, LOG_2_BLOCK_WIDTH);

            // allocateLocalZeroExtensionSpace guarantees this will be large enough to satisfy the kernel
            ExternalBuffer tmp(0, b, binding.getType(), true, buffer->getAddressSpace());
            Value * zeroExtension = b->CreatePointerCast(zeroExtensionSpace, bufferType);
            Value * addr = tmp.getStreamBlockPtr(b, zeroExtension, ZERO, b->CreateNeg(blockIndex));
            addr = b->CreatePointerCast(addr, bufferType);
            const auto i = rt.Port.Number;
            assert (addr->getType() == baseAddresses[i]->getType());

            addr = b->CreateSelect(zeroExtended, addr, baseAddresses[i], "zeroExtendAddr");
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

    Constant * const sz_ZERO = b->getSize(0);
    Constant * const sz_ONE = b->getSize(1);

    for (const auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {

        const auto streamSet = source(e, mBufferGraph);

        const BufferNode & bn = mBufferGraph[streamSet];
        const StreamSetBuffer * const buffer = bn.Buffer;
        const BufferPort & port = mBufferGraph[e];
        const auto inputPort = port.Port;
        assert (inputPort.Type == PortType::Input);
        const Binding & input = port.Binding;
        const ProcessingRate & rate = input.getRate();

        if (LLVM_LIKELY(rate.isFixed())) {

            // TODO: support popcount/partialsum

            // TODO: for fixed rate inputs, so long as the actual number of items is aa even
            // multiple of the stride*rate, we can ignore masking.

            // TODO: if we can prove that this will be the last kernel invocation that will ever touch this stream)
            // and is not an input to the pipeline (which we cannot prove will have space after the last item), we
            // can avoid copying the buffer and instead just mask out the surpressed items.


            AllocaInst * bufferStorage = nullptr;
            if (mNumOfTruncatedInputBuffers < mTruncatedInputBuffer.size()) {
                bufferStorage = mTruncatedInputBuffer[mNumOfTruncatedInputBuffers];
            } else { // create a stack entry for this buffer at the start of the pipeline
                PointerType * const int8PtrTy = b->getInt8PtrTy();
                bufferStorage = b->CreateAllocaAtEntryPoint(int8PtrTy);
                Instruction * const nextNode = bufferStorage->getNextNode(); assert (nextNode);
                new StoreInst(ConstantPointerNull::get(int8PtrTy), bufferStorage, nextNode);
                mTruncatedInputBuffer.push_back(bufferStorage);
            }
            ++mNumOfTruncatedInputBuffers;

            const auto prefix = makeBufferName(mKernelId, inputPort);
            const auto itemWidth = getItemWidth(buffer->getBaseType());

            if (LLVM_UNLIKELY(itemWidth == 0)) {
                continue;
            }

            Constant * const ITEM_WIDTH = b->getSize(itemWidth);

            PointerType * const bufferType = buffer->getPointerType();
            PointerType * const int8PtrTy = b->getInt8PtrTy();

            BasicBlock * const maskedInput = b->CreateBasicBlock(prefix + "_maskInput", mKernelCheckOutputSpace);
            BasicBlock * const selectedInput = b->CreateBasicBlock(prefix + "_selectInput", mKernelCheckOutputSpace);

            Value * selected = accessibleItems[inputPort.Number];
            Value * totalNumOfItems = getAccessibleInputItems(b, port);
            Value * const tooMany = b->CreateICmpULT(selected, totalNumOfItems);
            Value * computeMask = tooMany;
            if (mIsInputZeroExtended[inputPort]) {
                computeMask = b->CreateAnd(tooMany, b->CreateNot(mIsInputZeroExtended[inputPort]));
            }

            BasicBlock * const entryBlock = b->GetInsertBlock();
            b->CreateUnlikelyCondBr(computeMask, maskedInput, selectedInput);

            b->SetInsertPoint(maskedInput);

            // if this is a deferred fixed rate stream, we cannot be sure how many
            // blocks will have to be provided to the kernel in order to mask out
            // the truncated input stream.


            // Generate a name to describe this masking function.
            SmallVector<char, 32> tmp;
            raw_svector_ostream name(tmp);

            name << "__maskInput" << itemWidth;

            Module * const m = b->getModule();

            Function * maskInput = m->getFunction(name.str());

            if (maskInput == nullptr) {

                IntegerType * const sizeTy = b->getSizeTy();

                const auto blockWidth = b->getBitBlockWidth();
                const auto log2BlockWidth = floor_log2(blockWidth);
                Constant * const BLOCK_MASK = b->getSize(blockWidth - 1);
                Constant * const LOG_2_BLOCK_WIDTH = b->getSize(log2BlockWidth);

                const auto ip = b->saveIP();

                FixedArray<Type *, 7> params;
                params[0] = int8PtrTy; // input buffer
                params[1] = sizeTy; // bytes per stride
                params[2] = sizeTy; // processed
                params[3] = sizeTy; // processed (deferred)
                params[4] = sizeTy; // accessible
                params[5] = sizeTy; // numOfStreams
                params[6] = int8PtrTy->getPointerTo(); // masked buffer storage ptr

                LLVMContext & C = m->getContext();

                FunctionType * const funcTy = FunctionType::get(int8PtrTy, params, false);
                maskInput = Function::Create(funcTy, Function::InternalLinkage, name.str(), m);
                b->SetInsertPoint(BasicBlock::Create(C, "entry", maskInput));

                auto arg = maskInput->arg_begin();
                auto nextArg = [&]() {
                    assert (arg != maskInput->arg_end());
                    Value * const v = &*arg;
                    std::advance(arg, 1);
                    return v;
                };


                DataLayout DL(b->getModule());
                Type * const intPtrTy = DL.getIntPtrType(int8PtrTy);

                Value * const inputBuffer = nextArg();
                inputBuffer->setName("inputBuffer");
                Value * const itemsPerStride = nextArg();
                itemsPerStride->setName("itemsPerStride");
                Value * const processed = nextArg();
                processed->setName("processed");
                Value * const consumed = nextArg();
                consumed->setName("consumed");
                Value * const accessible = nextArg();
                accessible->setName("accessible");
                Value * const numOfStreams = nextArg();
                numOfStreams->setName("numOfStreams");
                Value * const bufferStorage = nextArg();
                bufferStorage->setName("bufferStorage");
                assert (arg == maskInput->arg_end());


                Type * const singleElementStreamSetTy = ArrayType::get(llvm_version::getVectorType(IntegerType::get(C, itemWidth), 0), 1);
                ExternalBuffer tmp(0, b, singleElementStreamSetTy, true, 0);
                PointerType * const bufferPtrTy = tmp.getPointerType();

                Value * const inputAddress = b->CreatePointerCast(inputBuffer, bufferPtrTy);
                Value * const initial = b->CreateMul(b->CreateLShr(consumed, LOG_2_BLOCK_WIDTH), numOfStreams);
                Value * const initialPtr = tmp.getStreamBlockPtr(b, inputAddress, sz_ZERO, initial);
                Value * const initialPtrInt = b->CreatePtrToInt(initialPtr, intPtrTy);

                Value * const total = b->CreateAdd(processed, accessible);

                Value * const required = b->CreateMul(b->CreateLShr(b->CreateRoundUp(total, itemsPerStride), LOG_2_BLOCK_WIDTH), numOfStreams);
                Value * const requiredPtr = tmp.getStreamBlockPtr(b, inputAddress, sz_ZERO, required);
                Value * const requiredPtrInt = b->CreatePtrToInt(requiredPtr, intPtrTy);
                Value * const requiredBytes = b->CreateSub(requiredPtrInt, initialPtrInt);

                const auto blockSize = b->getBitBlockWidth() / 8;

                Value * const maskedBuffer = b->CreateAlignedMalloc(requiredBytes, blockSize);
                b->CreateMemZero(maskedBuffer, requiredBytes, blockSize);
                // TODO: look into checking whether the OS supports aligned realloc.
                b->CreateFree(b->CreateLoad(bufferStorage));
                b->CreateStore(maskedBuffer, bufferStorage);
                Value * const mallocedAddress = b->CreatePointerCast(maskedBuffer, bufferPtrTy);
                Value * const fullCopyEnd = b->CreateMul(b->CreateLShr(total, LOG_2_BLOCK_WIDTH), numOfStreams);
                Value * const fullCopyEndPtr = tmp.getStreamBlockPtr(b, inputAddress, sz_ZERO, fullCopyEnd);
                Value * const fullCopyEndPtrInt = b->CreatePtrToInt(fullCopyEndPtr, intPtrTy);
                Value * const fullBytesToCopy = b->CreateSub(fullCopyEndPtrInt, initialPtrInt);

                b->CreateMemCpy(mallocedAddress, initialPtr, fullBytesToCopy, blockSize);

                // Value * const base = b->CreateMul(b->CreateLShr(processed, LOG_2_BLOCK_WIDTH), numOfStreams);
                Value * const outputVBA = tmp.getStreamBlockPtr(b, mallocedAddress, sz_ZERO, b->CreateNeg(initial));
                Value * const maskedAddress = b->CreatePointerCast(outputVBA, bufferPtrTy);
                assert (maskedAddress->getType() == inputAddress->getType());

                Value * packIndex = nullptr;
                Value * maskOffset = b->CreateAnd(accessible, BLOCK_MASK);
                if (itemWidth > 1) {
                    Value * const position = b->CreateMul(maskOffset, ITEM_WIDTH);
                    packIndex = b->CreateLShr(position, LOG_2_BLOCK_WIDTH);
                    maskOffset = b->CreateAnd(position, BLOCK_MASK);
                }
                Value * const mask = b->CreateNot(b->bitblock_mask_from(maskOffset));
                BasicBlock * const loopEntryBlock = b->GetInsertBlock();

                BasicBlock * const maskedInputLoop = BasicBlock::Create(C, "maskInputLoop", maskInput);
                BasicBlock * const maskedInputExit = BasicBlock::Create(C, "maskInputExit", maskInput);
                b->CreateBr(maskedInputLoop);

                b->SetInsertPoint(maskedInputLoop);
                PHINode * const streamIndex = b->CreatePHI(b->getSizeTy(), 2);
                streamIndex->addIncoming(sz_ZERO, loopEntryBlock);

                Value * inputPtr = tmp.getStreamBlockPtr(b, inputAddress, streamIndex, fullCopyEnd);
                Value * outputPtr = tmp.getStreamBlockPtr(b, maskedAddress, streamIndex, fullCopyEnd);
                assert (inputPtr->getType() == outputPtr->getType());
                if (itemWidth > 1) {
                    Value * const partialCopyInputEndPtr = tmp.getStreamPackPtr(b, inputAddress, streamIndex, fullCopyEnd, packIndex);
                    Value * const partialCopyInputEndPtrInt = b->CreatePtrToInt(partialCopyInputEndPtr, intPtrTy);
                    Value * const partialCopyInputStartPtrInt = b->CreatePtrToInt(inputPtr, intPtrTy);
                    Value * const bytesToCopy = b->CreateSub(partialCopyInputEndPtrInt, partialCopyInputStartPtrInt);

                    b->CreateMemCpy(outputPtr, inputPtr, bytesToCopy, blockSize);
                    inputPtr = partialCopyInputEndPtr;
                    Value * const afterCopyOutputPtr = tmp.getStreamPackPtr(b, maskedAddress, streamIndex, fullCopyEnd, packIndex);
                    outputPtr = afterCopyOutputPtr;
                }
                assert (inputPtr->getType() == outputPtr->getType());
                Value * const val = b->CreateBlockAlignedLoad(inputPtr);
                Value * const maskedVal = b->CreateAnd(val, mask);
                b->CreateBlockAlignedStore(maskedVal, outputPtr);

                Value * const nextIndex = b->CreateAdd(streamIndex, sz_ONE);
                Value * const notDone = b->CreateICmpNE(nextIndex, numOfStreams);
                streamIndex->addIncoming(nextIndex, maskedInputLoop);

                b->CreateCondBr(notDone, maskedInputLoop, maskedInputExit);

                b->SetInsertPoint(maskedInputExit);
                b->CreateRet(b->CreatePointerCast(maskedAddress, int8PtrTy));

                b->restoreIP(ip);
            }

            FixedArray<Value *, 7> args;
            args[0] = b->CreatePointerCast(inputBaseAddresses[inputPort.Number], int8PtrTy);
            const auto itemsPerStride = rate.getRate() * mKernel->getStride();
            assert (itemsPerStride.denominator() == 1);
            args[1] = b->getSize(itemsPerStride.numerator());
            args[2] = mAlreadyProcessedPhi[inputPort];
            if (port.IsDeferred) {
                args[3] = mAlreadyProcessedDeferredPhi[inputPort];
            } else {
                args[3] = mAlreadyProcessedPhi[inputPort];
            }
            args[4] = accessibleItems[inputPort.Number];
            args[5] = buffer->getStreamSetCount(b);
            args[6] = bufferStorage;

            #ifdef PRINT_DEBUG_MESSAGES
            debugPrint(b, prefix + " truncating item count from %" PRIu64 " to %" PRIu64,
                      totalNumOfItems, accessibleItems[inputPort.Number]);
            #endif

            Value * const maskedAddress = b->CreatePointerCast(b->CreateCall(maskInput, args), bufferType);
            BasicBlock * const maskedInputLoopExit = b->GetInsertBlock();
            b->CreateBr(selectedInput);

            b->SetInsertPoint(selectedInput);
            PHINode * const phi = b->CreatePHI(bufferType, 2);
            phi->addIncoming(inputBaseAddresses[inputPort.Number], entryBlock);
            phi->addIncoming(maskedAddress, maskedInputLoopExit);
            inputBaseAddresses[inputPort.Number] = phi;

        }
    }
    #endif
}



#if 0

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief clearUnwrittenOutputData
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::clearUnwrittenOutputData(BuilderRef b) {
    #ifndef DISABLE_OUTPUT_ZEROING

    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const auto streamSet = target(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[streamSet];

        // If this stream is either controlled by this kernel or is an external
        // stream, any clearing of data is the responsibility of the owner.
        // Simply ignore any external buffers for the purpose of zeroing out
        // unnecessary data.

        const StreamSetBuffer * const buffer = bn.Buffer;

        if (bn.isUnowned() || buffer->isEmptySet()) {
            continue;
        }

        const BufferRateData & rt = mBufferGraph[e];
        assert (rt.Port.Type == PortType::Output);
        const auto port = rt.Port;

        const auto itemWidth = getItemWidth(buffer->getBaseType());

        SmallVector<char, 256> tmp;
        raw_svector_ostream name(tmp);
        name << "__maskOutput" << itemWidth;

        Module * const m = b->getModule();
        PointerType * const voidPtrTy = b->getVoidPtrTy();

        const auto blockWidth = b->getBitBlockWidth();

        Function * maskOutput = m->getFunction(name.str());

        if (maskOutput == nullptr) {

            IntegerType * const sizeTy = b->getSizeTy();
            FixedArray<Type *, 4> params;
            params[0] = voidPtrTy; // base address
            params[1] = sizeTy; // produced
            params[2] = sizeTy; // numOfStreams
            params[3] = sizeTy; // blocksToZero

            LLVMContext & C = m->getContext();

            const auto ip = b->saveIP();

            FunctionType * const funcTy = FunctionType::get(b->getVoidTy(), params, false);
            maskOutput = Function::Create(funcTy, Function::InternalLinkage, name.str(), m);
            b->SetInsertPoint(BasicBlock::Create(C, "entry", maskOutput));

            auto arg = maskOutput->arg_begin();
            auto nextArg = [&]() {
                assert (arg != maskOutput->arg_end());
                Value * const v = &*arg;
                std::advance(arg, 1);
                return v;
            };

            Value * baseAddress = nextArg();
            baseAddress->setName("inputAddress");

            PointerType * const bufferType = buffer->getPointerType();
            Type * const elemTy = bufferType->getPointerElementType();
            ArrayType * const baseStreamSetTy = ArrayType::get(elemTy->getArrayElementType(), 1);
            PointerType * const streamSetTy = baseStreamSetTy->getPointerTo();

            baseAddress = b->CreatePointerCast(baseAddress, streamSetTy);

            Value * const produced = nextArg();
            produced->setName("produced");
            Value * const numOfStreams = nextArg();
            numOfStreams->setName("numOfStreams");
            Value * const blocksToZero = nextArg();
            blocksToZero->setName("blocksToZero");
            assert (arg == maskOutput->arg_end());


            const auto log2BlockWidth = floor_log2(blockWidth);
            Constant * const LOG_2_BLOCK_WIDTH = b->getSize(log2BlockWidth);
            Constant * const ZERO = b->getSize(0);
            Constant * const ONE = b->getSize(1);
            Constant * const BLOCK_MASK = b->getSize(blockWidth - 1);

            Value * const blockIndex = b->CreateMul(b->CreateLShr(produced, LOG_2_BLOCK_WIDTH), numOfStreams);

            Constant * const ITEM_WIDTH = b->getSize(itemWidth);
            Value * packIndex = nullptr;
            Value * maskOffset = b->CreateAnd(produced, BLOCK_MASK);
            if (itemWidth > 1) {
                Value * const position = b->CreateMul(maskOffset, ITEM_WIDTH);
                packIndex = b->CreateLShr(position, LOG_2_BLOCK_WIDTH);
                maskOffset = b->CreateAnd(position, BLOCK_MASK);
            }
            Value * const mask = b->CreateNot(b->bitblock_mask_from(maskOffset));
            BasicBlock * const maskLoop = BasicBlock::Create(C, "maskLoop", maskOutput);
            BasicBlock * const maskLoopEnd = BasicBlock::Create(C, "maskLoopExit", maskOutput);
            BasicBlock * const maskZeroRemaining = BasicBlock::Create(C, "maskZeroRemaining", maskOutput);
            BasicBlock * const maskExit = BasicBlock::Create(C, "maskExit", maskOutput);

            BasicBlock * const entry = b->GetInsertBlock();
            b->CreateBr(maskLoop);

            b->SetInsertPoint(maskLoop);
            PHINode * const streamIndex = b->CreatePHI(b->getSizeTy(), 2);
            streamIndex->addIncoming(ZERO, entry);
            Value * ptr = nullptr;
            Value * const idx = b->CreateAdd(blockIndex, streamIndex);
            if (itemWidth > 1) {
                ptr = b->CreateInBoundsGEP(baseAddress, { idx, packIndex });
            } else {
                ptr = b->CreateInBoundsGEP(baseAddress, { idx });
            }
            Value * const value = b->CreateBlockAlignedLoad(ptr);
            Value * const maskedValue = b->CreateAnd(value, mask);
            b->CreateBlockAlignedStore(maskedValue, ptr);

            DataLayout DL(b->getModule());
            Type * const intPtrTy = DL.getIntPtrType(ptr->getType());
            if (itemWidth > 1) {
                // Since packs are laid out sequentially in memory, it will hopefully be cheaper to zero them out here
                // because they may be within the same cache line.
                Value * const nextPackIndex = b->CreateAdd(packIndex, ONE);
                Value * const start = b->CreateInBoundsGEP(baseAddress, { idx, nextPackIndex} );
                Value * const startInt = b->CreatePtrToInt(start, intPtrTy);
                Value * const end = b->CreateInBoundsGEP(baseAddress, { idx, ITEM_WIDTH });
                Value * const endInt = b->CreatePtrToInt(end, intPtrTy);
                Value * const remainingPackBytes = b->CreateSub(endInt, startInt);
                b->CreateMemZero(start, remainingPackBytes, blockWidth / 8);
            }
            BasicBlock * const maskLoopExit = b->GetInsertBlock();
            Value * const nextStreamIndex = b->CreateAdd(streamIndex, ONE);
            streamIndex->addIncoming(nextStreamIndex, maskLoopExit);
            Value * const notDone = b->CreateICmpNE(nextStreamIndex, numOfStreams);
            b->CreateCondBr(notDone, maskLoop, maskLoopEnd);

            b->SetInsertPoint(maskLoopEnd);
            // Zero out any blocks we could potentially touch
            Value * const anyToZero = b->CreateICmpUGE(blocksToZero, ONE);
            b->CreateCondBr(anyToZero, maskZeroRemaining, maskExit);

            b->SetInsertPoint(maskZeroRemaining);
            Value * const nextOffset = b->CreateAdd(blockIndex, numOfStreams);
            Value * const startPtr = b->CreateInBoundsGEP(baseAddress, nextOffset);
            Value * const startPtrInt = b->CreatePtrToInt(startPtr, intPtrTy);
            Value * const endOffset = b->CreateRoundUp(nextOffset, b->CreateMul(blocksToZero, numOfStreams));
            Value * const endPtr = b->CreateInBoundsGEP(baseAddress, endOffset);
            Value * const endPtrInt = b->CreatePtrToInt(endPtr, intPtrTy);
            Value * const remainingBytes = b->CreateSub(endPtrInt, startPtrInt);
            b->CreateMemZero(startPtr, remainingBytes, blockWidth / 8);
            b->CreateBr(maskExit);

            b->SetInsertPoint(maskExit);
            b->CreateRetVoid();

            b->restoreIP(ip);
        }

        Value * produced = nullptr;
        if (mKernelIsInternallySynchronized) {
            produced = mProducedItemCount[port];
        } else {
            produced = mProducedAtTerminationPhi[port];
        }

        FixedArray<Value *, 4> args;
        args[0] = b->CreatePointerCast(buffer->getHandle(), voidPtrTy);
        args[1] = produced;
        args[2] = buffer->getStreamSetCount(b);
        Rational strideLength{0};
        for (const auto e : make_iterator_range(out_edges(streamSet, mBufferGraph))) {
            const BufferRateData & rd = mBufferGraph[e];
            const Binding & input = rd.Binding;

            Rational R{rd.Maximum};
            if (LLVM_UNLIKELY(input.hasLookahead())) {
                R += input.getLookahead();
            }
            strideLength = std::max(strideLength, R);
        }
        args[3] = b->getSize(ceiling(strideLength * Rational{1, blockWidth}));
        b->CreateCall(maskOutput, args);
    }
    #endif
}


#else

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

    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const auto streamSet = target(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[streamSet];

        // If this stream is either controlled by this kernel or is an external
        // stream, any clearing of data is the responsibility of the owner.
        // Simply ignore any external buffers for the purpose of zeroing out
        // unnecessary data.
        if (bn.isUnowned()) {
            continue;
        }

        const StreamSetBuffer * const buffer = bn.Buffer;
        const BufferPort & rt = mBufferGraph[e];
        assert (rt.Port.Type == PortType::Output);
        const auto port = rt.Port;

        const auto itemWidth = getItemWidth(buffer->getBaseType());

        const auto prefix = makeBufferName(mKernelId, port);
        Value * produced = nullptr;
        if (LLVM_UNLIKELY(bn.OutputItemCountId != streamSet)) {
            produced = mLocallyAvailableItems[bn.OutputItemCountId];
        } else if (mKernelIsInternallySynchronized) {
            produced = mProducedItemCount[port];
        } else {
            produced = mProducedAtTerminationPhi[port];
        }

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
            const BufferPort & rd = mBufferGraph[e];
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
            Value * const endOffset = b->CreateRoundUp(nextOffset, b->getSize(blocksToZero));
            Value * const endPtr = buffer->StreamSetBuffer::getStreamBlockPtr(b, baseAddress, ZERO, endOffset);
            Value * const endPtrInt = b->CreatePtrToInt(endPtr, intPtrTy);
            Value * const remainingBytes = b->CreateSub(endPtrInt, startPtrInt);
            #ifdef PRINT_DEBUG_MESSAGES
            debugPrint(b, prefix + "_zeroFill_blockIndex = %" PRIu64, blockIndex);
            debugPrint(b, prefix + "_zeroFill_nextOffset = %" PRIu64, nextOffset);
            debugPrint(b, prefix + "_zeroFill_endOffset = %" PRIu64, endOffset);
            debugPrint(b, prefix + "_zeroFill_bufferStart = %" PRIu64, b->CreateSub(startPtrInt, epochInt));
            debugPrint(b, prefix + "_zeroFill_remainingBufferBytes = %" PRIu64, remainingBytes);
            #endif
            b->CreateMemZero(startPtr, remainingBytes, blockWidth / 8);
        }
    }
    #endif
}

#endif

}

#endif // BUFFER_MANIPULATION_LOGIC_HPP
