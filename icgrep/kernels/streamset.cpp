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

llvm::Type * StreamSetType::getStreamSetBlockType(IDISA::IDISA_Builder * iBuilder) {
    llvm::Type * streamType = mFieldWidth == 1 ? iBuilder->getBitBlockType() : ArrayType::get(iBuilder->getBitBlockType(), mFieldWidth);
    return ArrayType::get(streamType, mStreamCount);
}

llvm::PointerType * StreamSetBuffer::getStreamBufferPointerType() {
    return PointerType::get(mStreamSetType.getStreamSetBlockType(iBuilder), 0);
}

// Single Block Buffer

size_t SingleBlockBuffer::getBufferSize() {
    return 1; //iBuilder->getBitBlockWidth();
}

llvm::Value * SingleBlockBuffer::allocateBuffer() {
    mStreamSetBufferPtr = iBuilder->CreateAlloca(mStreamSetType.getStreamSetBlockType(iBuilder));
    return mStreamSetBufferPtr;
}

// For a single block buffer, the block pointer is always the buffer base pointer.
llvm::Value * SingleBlockBuffer::getStreamSetBlockPointer(llvm::Value * bufferBasePtr, llvm::Value * blockNo) {
    return bufferBasePtr;
}


// External Unbounded Buffer

size_t ExternalUnboundedBuffer::getBufferSize() {
    return 0;
}

void ExternalUnboundedBuffer::setStreamSetBuffer(llvm::Value * ptr) {
    PointerType * t = PointerType::get(mStreamSetType.getStreamSetBlockType(iBuilder), mAddrSpace);
    
    mStreamSetBufferPtr = iBuilder->CreatePointerBitCastOrAddrSpaceCast(ptr, t);
    std::cerr << "mStreamSetBufferPtr\n";
}

llvm::PointerType * ExternalUnboundedBuffer::getStreamBufferPointerType() {
    return PointerType::get(mStreamSetType.getStreamSetBlockType(iBuilder), mAddrSpace);
}

llvm::Value * ExternalUnboundedBuffer::allocateBuffer() {
    throw std::runtime_error("External buffers cannot be allocated.");
}

llvm::Value * ExternalUnboundedBuffer::getStreamSetBlockPointer(llvm::Value * bufferBasePtr, llvm::Value * blockNo) {
    PointerType * t = getStreamBufferPointerType();
    return iBuilder->CreateGEP(iBuilder->CreatePointerBitCastOrAddrSpaceCast(bufferBasePtr, t), {blockNo});
}


// Circular Stack Allocated Buffer

size_t CircularBuffer::getBufferSize() {
    return mBufferBlocks; // * iBuilder->getBitBlockWidth();
}

llvm::Value * CircularBuffer::allocateBuffer() {
    mStreamSetBufferPtr = iBuilder->CreateAlloca(mStreamSetType.getStreamSetBlockType(iBuilder), ConstantInt::get(iBuilder->getSizeTy(), mBufferBlocks));
    return mStreamSetBufferPtr;
}

llvm::Value * CircularBuffer::getStreamSetBlockPointer(llvm::Value * bufferBasePtr, llvm::Value * blockNo) {
    return iBuilder->CreateGEP(mStreamSetType.getStreamSetBlockType(iBuilder), bufferBasePtr, {iBuilder->CreateAnd(blockNo, ConstantInt::get(iBuilder->getSizeTy(), mBufferBlocks-1))});
}

