/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

    
#include <kernels/streamset.h>
#include <vector>
#include <IDISA/idisa_builder.h>
#include <llvm/IR/Type.h>
    
using namespace kernel;

llvm::Type * StreamSetType::getStreamSetBlockType(IDISA::IDISA_Builder * iBuilder) {
    llvm::Type * streamType = mFieldWidth == 1 ? iBuilder->getBitBlockType() : ArrayType::get(iBuilder->getBitBlockType(), mFieldWidth);
    return ArrayType::get(streamType, mStreamCount);
}

llvm::Type * StreamSetBuffer::getStreamSetBlockType() {
    return mStreamSetType.getStreamSetBlockType(iBuilder);
}

llvm::Type * StreamSetBuffer::getStreamSetBufferType() {
    if (mSegmentSize == 1) return getStreamSetBlockType();
    return ArrayType::get(getStreamSetBlockType(), mSegmentSize);
}

llvm::Value * StreamSetBuffer::allocateBuffer() {
    if (mStreamSetBufferPtr == nullptr) {
        mStreamSetBufferPtr = iBuilder->CreateAlloca(getStreamSetBlockType(), iBuilder->getInt32(mSegmentSize));
    }
    return mStreamSetBufferPtr;
}

llvm::Value * StreamSetBuffer::getBlockPointer(llvm::Value * blockNo) {
    if (mSegmentSize == 1) return mStreamSetBufferPtr;
    if (mSegmentSize == 0) return
        iBuilder->CreateGEP(getStreamSetBlockType(), mStreamSetBufferPtr, {blockNo});
    Value * offset = iBuilder->CreateURem(blockNo, iBuilder->getInt64(mSegmentSize));
    return iBuilder->CreateGEP(getStreamSetBlockType(), mStreamSetBufferPtr, {iBuilder->getInt64(0), offset});
}

