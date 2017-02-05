/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "streamset.h"
#include <IR_Gen/idisa_builder.h>  // for IDISA_Builder
#include <assert.h>                // for assert
#include <llvm/IR/Type.h>          // for Type
#include <stdexcept>               // for runtime_error
#include <llvm/IR/BasicBlock.h>    // for BasicBlock
#include <llvm/IR/Constants.h>     // for ConstantInt
#include <llvm/IR/DataLayout.h>    // for DataLayout
#include <llvm/IR/DerivedTypes.h>  // for IntegerType (ptr only), PointerType
#include <llvm/IR/Module.h>        // for Module
#include <llvm/IR/Value.h>         // for Value
namespace llvm { class Constant; }
namespace llvm { class Function; }

using namespace parabix;
using namespace llvm;
using namespace IDISA;

Type * resolveVectorTy(IDISA_Builder * const b, Type * type) {
    if (LLVM_LIKELY(type->isVectorTy() && type->getVectorNumElements() == 0)) {
        type = type->getVectorElementType();
        if (LLVM_LIKELY(type->isIntegerTy())) {
            const auto fieldWidth = cast<IntegerType>(type)->getBitWidth();
            type = b->getBitBlockType();
            if (fieldWidth != 1) {
                type = llvm::ArrayType::get(type, fieldWidth);
            }
        }
    }
    return type;
}

Type * StreamSetBuffer::resolveStreamSetBufferType(Type * const type) const {
    if (type->isArrayTy()) {
        return ArrayType::get(resolveVectorTy(iBuilder, type->getArrayElementType()), type->getArrayNumElements());
    } else if (type->isVectorTy()) {
        return resolveVectorTy(iBuilder, type);
    }
    return type;
}

void StreamSetBuffer::allocateBuffer() {
    mStreamSetBufferPtr = iBuilder->CreateCacheAlignedAlloca(getType(), iBuilder->getSize(mBufferBlocks));
}

Value * StreamSetBuffer::getStream(Value * self, Value * blockNo, Value * index) const {
    return iBuilder->CreateGEP(getStreamSetPtr(self, blockNo), {iBuilder->getInt32(0), index});
}

Value * StreamSetBuffer::getStream(Value * self, Value * blockNo, Value * index1, Value * index2) const {
    return iBuilder->CreateGEP(getStreamSetPtr(self, blockNo), {iBuilder->getInt32(0), index1, index2});
}

Value * StreamSetBuffer::getStreamView(llvm::Type * type, llvm::Value * self, Value * blockNo, llvm::Value * index) const {
    return iBuilder->CreateGEP(iBuilder->CreatePointerCast(getStreamSetPtr(self, blockNo), type), index, "view");
}

Value * StreamSetBuffer::getLinearlyAccessibleItems(llvm::Value * fromPosition) const {
    if (isa<ArrayType>(mStreamSetType) && dyn_cast<ArrayType>(mStreamSetType)->getNumElements() > 1) {
        Constant * stride = iBuilder->getSize(iBuilder->getStride());
        return iBuilder->CreateSub(stride, iBuilder->CreateURem(fromPosition, stride));
    }
    else {
        Constant * bufSize = iBuilder->getSize(mBufferBlocks * iBuilder->getStride());
        return iBuilder->CreateSub(bufSize, iBuilder->CreateURem(fromPosition, bufSize));
    }
}


// Single Block Buffer

// For a single block buffer, the block pointer is always the buffer base pointer.
Value * SingleBlockBuffer::getStreamSetPtr(Value * self, Value *) const {
    return self;
}

// External File Buffer
void ExternalFileBuffer::setStreamSetBuffer(Value * ptr, Value * /* fileSize */) {
    mStreamSetBufferPtr = iBuilder->CreatePointerBitCastOrAddrSpaceCast(ptr, getPointerType());
}

void ExternalFileBuffer::setEmptyBuffer(Value * ptr) {    
    mStreamSetBufferPtr = iBuilder->CreatePointerBitCastOrAddrSpaceCast(ptr, getPointerType());
}

void ExternalFileBuffer::allocateBuffer() {
    throw std::runtime_error("External buffers cannot be allocated.");
}

Value * ExternalFileBuffer::getStreamSetPtr(Value * self, Value * blockNo) const {
    return iBuilder->CreateGEP(self, blockNo);
}

Value * ExternalFileBuffer::getLinearlyAccessibleItems(llvm::Value * fromPosition) const {
    throw std::runtime_error("External buffers: getLinearlyAccessibleItems not supported.");
}

// Circular Buffer

Value * CircularBuffer::getStreamSetPtr(Value * self, Value * blockNo) const {
    assert (blockNo->getType()->isIntegerTy());

    Value * offset = nullptr;
    if (mBufferBlocks == 1) {
        offset = ConstantInt::getNullValue(iBuilder->getSizeTy());
    } else if ((mBufferBlocks & (mBufferBlocks - 1)) == 0) { // is power of 2
        offset = iBuilder->CreateAnd(blockNo, ConstantInt::get(blockNo->getType(), mBufferBlocks - 1));
    } else {
        offset = iBuilder->CreateURem(blockNo, ConstantInt::get(blockNo->getType(), mBufferBlocks));
    }
    return iBuilder->CreateGEP(self, offset);
}


// CircularCopybackBuffer Buffer

void CircularCopybackBuffer::allocateBuffer() {
    mStreamSetBufferPtr = iBuilder->CreateCacheAlignedAlloca(getType(), iBuilder->getSize(mBufferBlocks + mOverflowBlocks));
}

void CircularCopybackBuffer::createCopyBack(Value * self, Value * overFlowItems) {
    // Must copy back one full block for each of the streams in the stream set.
    Constant * blockSize = iBuilder->getSize(iBuilder->getBitBlockWidth());
    Constant * blockSizeLess1 = iBuilder->getSize(iBuilder->getBitBlockWidth() - 1);
    Value * overFlowBlocks = iBuilder->CreateUDiv(iBuilder->CreateAdd(overFlowItems, blockSizeLess1), blockSize);
    Value * overFlowAreaPtr = iBuilder->CreateGEP(mStreamSetBufferPtr, iBuilder->getSize(mBufferBlocks));
    DataLayout dl(iBuilder->getModule());
    Constant * blockBytes = ConstantInt::get(iBuilder->getSizeTy(), dl.getTypeAllocSize(mStreamSetType) * iBuilder->getBitBlockWidth());
    Value * copyLength = iBuilder->CreateMul(overFlowBlocks, blockBytes);
    Type * i8ptr = iBuilder->getInt8Ty()->getPointerTo();
    unsigned alignment = iBuilder->getBitBlockWidth() / 8;
    iBuilder->CreateMemMove(iBuilder->CreateBitCast(mStreamSetBufferPtr, i8ptr), iBuilder->CreateBitCast(overFlowAreaPtr, i8ptr), copyLength, alignment);
}

Value * CircularCopybackBuffer::getStreamSetPtr(Value * self, Value * blockNo) const {
    assert (blockNo->getType()->isIntegerTy());
    
    Value * offset = nullptr;
    if (mBufferBlocks == 1) {
        offset = ConstantInt::getNullValue(iBuilder->getSizeTy());
    } else if ((mBufferBlocks & (mBufferBlocks - 1)) == 0) { // is power of 2
        offset = iBuilder->CreateAnd(blockNo, ConstantInt::get(blockNo->getType(), mBufferBlocks - 1));
    } else {
        offset = iBuilder->CreateURem(blockNo, ConstantInt::get(blockNo->getType(), mBufferBlocks));
    }
    return iBuilder->CreateGEP(self, offset);
}



// Expandable Buffer

Value * ExpandableBuffer::getStreamSetPtr(Value * self, Value * blockNo) const {
    return nullptr;
}

llvm::Value * ExpandableBuffer::getStream(llvm::Value * self, llvm::Value * blockNo, llvm::Value * index) const {
    return nullptr;
}

llvm::Value * ExpandableBuffer::getStream(llvm::Value * self, llvm::Value * blockNo, Value *index1, Value *index2) const {
    return nullptr;
}

llvm::Value * ExpandableBuffer::getStreamView(llvm::Type * type, llvm::Value * self, llvm::Value * blockNo, llvm::Value * index) const {
    return nullptr;
}

// Constructors

SingleBlockBuffer::SingleBlockBuffer(IDISA::IDISA_Builder * b, llvm::Type * type)
: StreamSetBuffer(BufferKind::BlockBuffer, b, type, 1, 0) {

}

ExternalFileBuffer::ExternalFileBuffer(IDISA::IDISA_Builder * b, llvm::Type * type, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::ExternalFileBuffer, b, type, 0, AddressSpace) {

}

CircularBuffer::CircularBuffer(IDISA::IDISA_Builder * b, llvm::Type * type, size_t bufferBlocks, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::CircularBuffer, b, type, bufferBlocks, AddressSpace) {

}

CircularCopybackBuffer::CircularCopybackBuffer(IDISA::IDISA_Builder * b, llvm::Type * type, size_t bufferBlocks, size_t overflowBlocks, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::CircularCopybackBuffer, b, type, bufferBlocks, AddressSpace), mOverflowBlocks(overflowBlocks) {

}


ExpandableBuffer::ExpandableBuffer(IDISA::IDISA_Builder * b, llvm::Type * type, size_t bufferBlocks, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::ExpandableBuffer, b, type, bufferBlocks, AddressSpace) {

}

Value * ExpandableBuffer::getLinearlyAccessibleItems(llvm::Value * fromPosition) const {
    throw std::runtime_error("Expandable buffers: getLinearlyAccessibleItems not supported.");
}


StreamSetBuffer::StreamSetBuffer(BufferKind k, IDISA::IDISA_Builder * b, Type * type, unsigned blocks, unsigned AddressSpace)
: mBufferKind(k)
, iBuilder(b)
, mStreamSetType(resolveStreamSetBufferType(type))
, mBufferBlocks(blocks)
, mAddressSpace(AddressSpace)
, mStreamSetBufferPtr(nullptr)
, mBaseStreamSetType(type) {

}
