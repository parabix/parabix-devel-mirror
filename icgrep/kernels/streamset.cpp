/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

    
#include <kernels/streamset.h>
#include <vector>
#include <IDISA/idisa_builder.h>
#include <llvm/IR/Type.h>

using namespace parabix;

enum SS_struct_index {iProducer_pos = 0, iConsumer_pos = 1, iEnd_of_input = 2, iBuffer_ptr = 3};

llvm::PointerType * StreamSetBuffer::getStreamBufferPointerType() {
    return PointerType::get(mStreamSetType, mAddrSpace);
}

llvm::PointerType * StreamSetBuffer::getStreamSetStructPointerType() {
    return PointerType::get(mStreamSetStructType, 0);
}

llvm::Value * StreamSetBuffer::getProducerPosPtr(Value * bufferStructPtr) {
    return iBuilder->CreateGEP(bufferStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iProducer_pos)});
}

void StreamSetBuffer::setProducerPos(Value * bufferStructPtr, llvm::Value * pos){
    iBuilder->CreateStore(pos, iBuilder->CreateGEP(bufferStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iProducer_pos)}));
}

llvm::Value * StreamSetBuffer::getConsumerPosPtr(Value * bufferStructPtr) {
    return iBuilder->CreateGEP(bufferStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iConsumer_pos)});
}

void StreamSetBuffer::setConsumerPos(Value * bufferStructPtr, Value * pos){
    iBuilder->CreateStore(pos, iBuilder->CreateGEP(bufferStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iConsumer_pos)}));
}

llvm::Value * StreamSetBuffer::hasEndOfInputPtr(Value * bufferStructPtr) {
    return iBuilder->CreateGEP(bufferStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iEnd_of_input)});
}

void StreamSetBuffer::setEndOfInput(Value * bufferStructPtr){
    iBuilder->CreateStore(ConstantInt::get(iBuilder->getInt1Ty(), 1), iBuilder->CreateGEP(bufferStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iEnd_of_input)}));
}

llvm::Value * StreamSetBuffer::getStreamSetStructPtr(){
    return mStreamSetStructPtr;
}

llvm::Value * StreamSetBuffer::allocateBuffer() {
    Type * const size_ty = iBuilder->getSizeTy();
    Type * const int1ty = iBuilder->getInt1Ty();
    mStreamSetBufferPtr = iBuilder->CreateCacheAlignedAlloca(mStreamSetType, ConstantInt::get(iBuilder->getSizeTy(), mBufferBlocks));
    mStreamSetStructPtr = iBuilder->CreateCacheAlignedAlloca(mStreamSetStructType);

    iBuilder->CreateStore(ConstantInt::get(size_ty, 0), iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iProducer_pos)}));
    iBuilder->CreateStore(ConstantInt::get(size_ty, 0), iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iConsumer_pos)}));
    iBuilder->CreateStore(ConstantInt::get(int1ty, 0), iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iEnd_of_input)}));
    iBuilder->CreateStore(mStreamSetBufferPtr, iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iBuffer_ptr)}));
    
    return mStreamSetBufferPtr;
}

llvm::Value * StreamSetBuffer::getStreamSetBlockPointer(llvm::Value * bufferStructPtr, llvm::Value * blockNo) {
    Value * handle = iBuilder->CreateGEP(bufferStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iBuffer_ptr)});
    return iBuilder->CreateGEP(iBuilder->CreateLoad(handle), {blockNo});
}

// Single Block Buffer
// For a single block buffer, the block pointer is always the buffer base pointer.
llvm::Value * SingleBlockBuffer::getStreamSetBlockPointer(llvm::Value * bufferStructPtr, llvm::Value * blockNo) {
    Value * handle = iBuilder->CreateGEP(bufferStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iBuffer_ptr)});
    return iBuilder->CreateLoad(handle);
}


// External File Buffer

void ExternalFileBuffer::setStreamSetBuffer(llvm::Value * ptr, Value * fileSize) {
    
    Type * const size_ty = iBuilder->getSizeTy();
    Type * const int1ty = iBuilder->getInt1Ty();
    
    PointerType * t = getStreamBufferPointerType();    
    mStreamSetBufferPtr = iBuilder->CreatePointerBitCastOrAddrSpaceCast(ptr, t);
    
    mStreamSetStructPtr = iBuilder->CreateCacheAlignedAlloca(mStreamSetStructType);
    iBuilder->CreateStore(fileSize, iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iProducer_pos)}));
    iBuilder->CreateStore(ConstantInt::get(size_ty, 0), iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iConsumer_pos)}));
    iBuilder->CreateStore(ConstantInt::get(int1ty, 1), iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iEnd_of_input)}));
    iBuilder->CreateStore(mStreamSetBufferPtr, iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iBuffer_ptr)}));
}

void ExternalFileBuffer::setEmptyBuffer(llvm::Value * ptr) {
    
    Type * const size_ty = iBuilder->getSizeTy();
    Type * const int1ty = iBuilder->getInt1Ty();
    
    PointerType * t = getStreamBufferPointerType();    
    mStreamSetBufferPtr = iBuilder->CreatePointerBitCastOrAddrSpaceCast(ptr, t);
    
    mStreamSetStructPtr = iBuilder->CreateCacheAlignedAlloca(mStreamSetStructType);
    iBuilder->CreateStore(ConstantInt::get(size_ty, 0), iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iProducer_pos)}));
    iBuilder->CreateStore(ConstantInt::get(size_ty, 0), iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iConsumer_pos)}));
    iBuilder->CreateStore(ConstantInt::get(int1ty,0), iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iEnd_of_input)}));
    iBuilder->CreateStore(mStreamSetBufferPtr, iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iBuffer_ptr)}));
}

llvm::Value * ExternalFileBuffer::allocateBuffer() {
    throw std::runtime_error("External buffers cannot be allocated.");
}

llvm::Value * ExternalFileBuffer::getStreamSetBlockPointer(llvm::Value * bufferStructPtr, llvm::Value * blockNo) {
    Value * handle = iBuilder->CreateGEP(bufferStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iBuffer_ptr)});
    return iBuilder->CreateGEP(iBuilder->CreateLoad(handle), {blockNo});
}

// Circular Stack Allocated Buffer

llvm::Value * CircularBuffer::getStreamSetBlockPointer(llvm::Value * bufferStructPtr, llvm::Value * blockNo) {
    Value * handle = iBuilder->CreateGEP(bufferStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iBuffer_ptr)});
    // Circular access is based on blockNo mod mBufferBlocks.  For power of 2 buffer sizes (required), we
    // use bitwise masking to efficiently compute the mod function  (blockNo & (mBufferBlocks - 1)
    Value * bufPtr = iBuilder->CreateLoad(handle);
    //iBuilder->CallPrintInt("CircularBuffer bufPtr", iBuilder->CreatePtrToInt(bufPtr, iBuilder->getSizeTy()));
    return iBuilder->CreateGEP(bufPtr, {iBuilder->CreateAnd(blockNo, ConstantInt::get(iBuilder->getSizeTy(), mBufferBlocks-1))});
}

llvm::Value * LinearCopybackBuffer::getStreamSetBlockPointer(llvm::Value * bufferStructPtr, llvm::Value * blockNo) {
    Constant * blockWidth = ConstantInt::get(iBuilder->getSizeTy(), iBuilder->getStride());
    Value * consumerPos_ptr = iBuilder->CreateGEP(bufferStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iConsumer_pos)});
    Value * consumerPos = iBuilder->CreateLoad(consumerPos_ptr);
    Value * consumerBlock = iBuilder->CreateUDiv(consumerPos, blockWidth);
    Value * handle = iBuilder->CreateGEP(bufferStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iBuffer_ptr)});
    Value * bufPtr = iBuilder->CreateLoad(handle);
    //iBuilder->CallPrintInt("LinearBuffer bufPtr", iBuilder->CreatePtrToInt(bufPtr, iBuilder->getSizeTy()));
    return iBuilder->CreateGEP(bufPtr, {iBuilder->CreateSub(blockNo, consumerBlock)});
}

void LinearCopybackBuffer::setConsumerPos(Value * bufferStructPtr, Value * new_consumer_pos) {
    Type * const i1 = iBuilder->getInt1Ty();
    Type * const i8 = iBuilder->getInt8Ty();
    Type * const i32 = iBuilder->getInt32Ty();
    Type * const i8_ptr = PointerType::get(i8, mAddrSpace);


    Module * M = iBuilder->getModule();

    IntegerType * sizeTy = iBuilder->getSizeTy();

    Function * memmoveFunc = cast<Function>(M->getOrInsertFunction("llvm.memmove.p0i8.p0i8.i" + std::to_string(sizeTy->getBitWidth()),
                                                                  iBuilder->getVoidTy(), i8_ptr, i8_ptr, sizeTy, i32, i1, nullptr));
    Function * current = iBuilder->GetInsertBlock()->getParent();
    BasicBlock * copyBackBody = BasicBlock::Create(M->getContext(), "copy_back", current, 0);
    BasicBlock * setConsumerPosExit = BasicBlock::Create(M->getContext(), "setConsumerPos_done", current, 0);
    Constant * blockWidth = ConstantInt::get(sizeTy, iBuilder->getStride());
    Constant * one = ConstantInt::get(sizeTy, 1);
    Value * consumerPos_ptr = iBuilder->CreateGEP(bufferStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iConsumer_pos)});
    Value * consumerPos = iBuilder->CreateLoad(consumerPos_ptr);
    Value * consumerBlock = iBuilder->CreateUDiv(consumerPos, blockWidth);
    // Ensure that the new consumer position is no less than the current position.
    new_consumer_pos = iBuilder->CreateSelect(iBuilder->CreateICmpULT(new_consumer_pos, consumerPos), consumerPos, new_consumer_pos);
    Value * producerPos = iBuilder->CreateLoad(iBuilder->CreateGEP(bufferStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iProducer_pos)}));
    // Ensure that the new consumer position is no greater than the current producer position.
    Value * new_pos_lt_producer_pos = iBuilder->CreateICmpULT(new_consumer_pos, producerPos);
    new_consumer_pos = iBuilder->CreateSelect(new_pos_lt_producer_pos, new_consumer_pos, producerPos);
    // Now, the new_consumer_pos is at most = to the producer_pos; if =, we're done.
    iBuilder->CreateCondBr(new_pos_lt_producer_pos, copyBackBody, setConsumerPosExit);
    iBuilder->SetInsertPoint(copyBackBody);
    
    Value * new_consumer_block = iBuilder->CreateUDiv(new_consumer_pos, blockWidth);
    
    Value * lastProducerBlock = iBuilder->CreateUDiv(iBuilder->CreateSub(producerPos, one), blockWidth);
    //iBuilder->CallPrintInt("new_consumer_block", new_consumer_block);
    //iBuilder->CallPrintInt("lastProducerBlock", lastProducerBlock);

    Value * copyBlocks = iBuilder->CreateAdd(iBuilder->CreateSub(lastProducerBlock, new_consumer_block), one);

    DataLayout dl(iBuilder->getModule());

    Constant * blockBytes = ConstantInt::get(sizeTy, dl.getTypeAllocSize(mStreamSetType) * iBuilder->getStride());
    Value * copyLength = iBuilder->CreateMul(copyBlocks, blockBytes);
    //iBuilder->CallPrintInt("memmove copyLength", copyLength);
    // Must copy back one full block for each of the streams in the stream set.
    Value * handle = iBuilder->CreateGEP(bufferStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iBuffer_ptr)});
    Value * bufferPtr = iBuilder->CreateLoad(handle);
    //iBuilder->CallPrintInt("memmove bufferPtr", iBuilder->CreatePtrToInt(bufferPtr, sizeTy));

    Value * copyFrom = iBuilder->CreateGEP(bufferPtr, {iBuilder->CreateSub(new_consumer_block, consumerBlock)});
    //iBuilder->CallPrintInt("memmove copyFrom", iBuilder->CreatePtrToInt(copyFrom, sizeTy));
    Value * alignment = ConstantInt::get(iBuilder->getInt32Ty(), iBuilder->getBitBlockWidth()/8);
    
    iBuilder->CreateCall(memmoveFunc, {iBuilder->CreateBitCast(bufferPtr, i8_ptr), iBuilder->CreateBitCast(copyFrom, i8_ptr), copyLength, alignment, ConstantInt::getNullValue(i1)});
    iBuilder->CreateBr(setConsumerPosExit);
    // Copy back done, store the new consumer position.
    iBuilder->SetInsertPoint(setConsumerPosExit);
    iBuilder->CreateStore(new_consumer_pos, consumerPos_ptr);
}    
