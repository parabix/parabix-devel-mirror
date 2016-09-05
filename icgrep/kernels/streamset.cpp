/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

    
#include <kernels/streamset.h>
#include <vector>
#include <IDISA/idisa_builder.h>
#include <llvm/IR/Type.h>
#include <iostream>
    
using namespace parabix;

enum SS_struct_index {iProducer_pos = 0, iConsumer_pos = 1, iEnd_of_input = 2, iBuffer_ptr = 3};

llvm::Type * StreamSetType::getStreamSetBlockType(IDISA::IDISA_Builder * iBuilder) {
    llvm::Type * streamType = mFieldWidth == 1 ? iBuilder->getBitBlockType() : ArrayType::get(iBuilder->getBitBlockType(), mFieldWidth);
    return ArrayType::get(streamType, mStreamCount);
}

llvm::PointerType * StreamSetBuffer::getStreamBufferPointerType() {
    return PointerType::get(mStreamSetType.getStreamSetBlockType(iBuilder), 0);
}

llvm::PointerType * StreamSetBuffer::getStreamSetStructPointerType() {
    return PointerType::get(mStreamSetStructType, 0);
}

llvm::Value * StreamSetBuffer::getProducerPosPtr(Value * ptr) {
    return iBuilder->CreateGEP(ptr, {iBuilder->getInt32(0), iBuilder->getInt32(iProducer_pos)});
}

void StreamSetBuffer::setProducerPos(Value * ptr, llvm::Value * pos){
    iBuilder->CreateStore(pos, iBuilder->CreateGEP(ptr, {iBuilder->getInt32(0), iBuilder->getInt32(iProducer_pos)}));
}

llvm::Value * StreamSetBuffer::getComsumerPosPtr(Value * ptr) {
    return iBuilder->CreateGEP(ptr, {iBuilder->getInt32(0), iBuilder->getInt32(iConsumer_pos)});
}

void StreamSetBuffer::setConsumerPos(Value * ptr, Value * pos){
    iBuilder->CreateStore(pos, iBuilder->CreateGEP(ptr, {iBuilder->getInt32(0), iBuilder->getInt32(iConsumer_pos)}));
}

llvm::Value * StreamSetBuffer::hasEndOfInputPtr(Value * ptr) {
    return iBuilder->CreateGEP(ptr, {iBuilder->getInt32(0), iBuilder->getInt32(iEnd_of_input)});
}

void StreamSetBuffer::setEndOfInput(Value * ptr){
    iBuilder->CreateStore(ConstantInt::get(iBuilder->getInt8Ty(), 1), iBuilder->CreateGEP(ptr, {iBuilder->getInt32(0), iBuilder->getInt32(iEnd_of_input)}));
}

llvm::Value * StreamSetBuffer::getStreamSetStructPtr(){
    return mStreamSetStructPtr;
}
// Single Block Buffer

size_t SingleBlockBuffer::getBufferSize() {
    return 1; //iBuilder->getBitBlockWidth();
}

llvm::Value * SingleBlockBuffer::allocateBuffer() {
    Type * const size_ty = iBuilder->getSizeTy();
    Type * const int8ty = iBuilder->getInt8Ty();
    mStreamSetBufferPtr = iBuilder->CreateAlloca(mStreamSetType.getStreamSetBlockType(iBuilder));
    mStreamSetStructPtr = iBuilder->CreateAlloca(mStreamSetStructType);
    iBuilder->CreateStore(ConstantInt::get(size_ty, 0), iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iProducer_pos)}));
    iBuilder->CreateStore(ConstantInt::get(size_ty, 0), iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iConsumer_pos)}));
    iBuilder->CreateStore(ConstantInt::get(int8ty, 0), iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iEnd_of_input)}));
    iBuilder->CreateStore(mStreamSetBufferPtr, iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iBuffer_ptr)}));
    return mStreamSetBufferPtr;
}

// For a single block buffer, the block pointer is always the buffer base pointer.
llvm::Value * SingleBlockBuffer::getStreamSetBlockPointer(llvm::Value * basePtr, llvm::Value * blockNo) {
    Value * handle = iBuilder->CreateGEP(basePtr, {iBuilder->getInt32(0), iBuilder->getInt32(iBuffer_ptr)});
    return iBuilder->CreateLoad(handle);
}


// External Unbounded Buffer

size_t ExternalFileBuffer::getBufferSize() {
    return 0;
}

void ExternalFileBuffer::setStreamSetBuffer(llvm::Value * ptr, Value * fileSize) {

    Type * const size_ty = iBuilder->getSizeTy();
    Type * const int8ty = iBuilder->getInt8Ty();

    PointerType * t = getStreamBufferPointerType();    
    mStreamSetBufferPtr = iBuilder->CreatePointerBitCastOrAddrSpaceCast(ptr, t);

    mStreamSetStructPtr = iBuilder->CreateAlloca(mStreamSetStructType);
    iBuilder->CreateStore(fileSize, iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iProducer_pos)}));
    iBuilder->CreateStore(ConstantInt::get(size_ty, 0), iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iConsumer_pos)}));
    iBuilder->CreateStore(ConstantInt::get(int8ty, 1), iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iEnd_of_input)}));
    iBuilder->CreateStore(mStreamSetBufferPtr, iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iBuffer_ptr)}));
}

llvm::PointerType * ExternalFileBuffer::getStreamBufferPointerType() {
    return PointerType::get(mStreamSetType.getStreamSetBlockType(iBuilder), mAddrSpace);
}

llvm::Value * ExternalFileBuffer::allocateBuffer() {
    throw std::runtime_error("External buffers cannot be allocated.");
}

llvm::Value * ExternalFileBuffer::getStreamSetBlockPointer(llvm::Value * basePtr, llvm::Value * blockNo) {
    Value * handle = iBuilder->CreateGEP(basePtr, {iBuilder->getInt32(0), iBuilder->getInt32(iBuffer_ptr)});
    return iBuilder->CreateGEP(iBuilder->CreateLoad(handle), {blockNo});
}


// Circular Stack Allocated Buffer

size_t CircularBuffer::getBufferSize() {
    return mBufferBlocks; // * iBuilder->getBitBlockWidth();
}

llvm::Value * CircularBuffer::allocateBuffer() {
    Type * const size_ty = iBuilder->getSizeTy();
    Type * const int8ty = iBuilder->getInt8Ty();
    mStreamSetBufferPtr = iBuilder->CreateAlloca(mStreamSetType.getStreamSetBlockType(iBuilder), ConstantInt::get(iBuilder->getSizeTy(), mBufferBlocks));
    mStreamSetStructPtr = iBuilder->CreateAlloca(mStreamSetStructType);
    iBuilder->CreateStore(ConstantInt::get(size_ty, 0), iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iProducer_pos)}));
    iBuilder->CreateStore(ConstantInt::get(size_ty, 0), iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iConsumer_pos)}));
    iBuilder->CreateStore(ConstantInt::get(int8ty, 0), iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iEnd_of_input)}));
    iBuilder->CreateStore(mStreamSetBufferPtr, iBuilder->CreateGEP(mStreamSetStructPtr, {iBuilder->getInt32(0), iBuilder->getInt32(iBuffer_ptr)}));

    return mStreamSetBufferPtr;
}

llvm::Value * CircularBuffer::getStreamSetBlockPointer(llvm::Value * basePtr, llvm::Value * blockNo) {
    Value * handle = iBuilder->CreateGEP(basePtr, {iBuilder->getInt32(0), iBuilder->getInt32(iBuffer_ptr)});
    return iBuilder->CreateGEP(iBuilder->CreateLoad(handle), {iBuilder->CreateAnd(blockNo, ConstantInt::get(iBuilder->getSizeTy(), mBufferBlocks-1))});
}

