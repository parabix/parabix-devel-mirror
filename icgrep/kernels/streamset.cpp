/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "streamset.h"
#include <IR_Gen/idisa_builder.h>  // for IDISA_Builder
#include <IR_Gen/types/streamtype.h>
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

Type * StreamSetBuffer::resolveStreamTypes(Type * type) {
    if (auto ty = dyn_cast<ArrayType>(type)) {
        unsigned numElems = ty->getNumElements();
        auto elemTy = ty->getElementType();
        if (isa<StreamType>(elemTy)) {
            return ArrayType::get(cast<StreamType>(elemTy)->resolveType(iBuilder), numElems);
        }
    }
    else if (auto ty = dyn_cast<StreamType>(type)) {
        return ty->resolveType(iBuilder);
    }
    return type;
}

void StreamSetBuffer::allocateBuffer() {
    mStreamSetBufferPtr = iBuilder->CreateCacheAlignedAlloca(mStreamSetType, iBuilder->getSize(mBufferBlocks));
}

Value * StreamSetBuffer::getStream(Value * self, Value * blockNo, Value * index) const {
    return iBuilder->CreateGEP(getStreamSetPtr(self, blockNo), {iBuilder->getInt32(0), index});
}

Value * StreamSetBuffer::getStream(Value * self, Value * blockNo, Value * index1, Value * index2) const {
    return iBuilder->CreateGEP(getStreamSetPtr(self, blockNo), {iBuilder->getInt32(0), index1, index2});
}

Value * StreamSetBuffer::getStreamView(llvm::Value * self, Value * blockNo, llvm::Value * index) const {
    return iBuilder->CreateGEP(getStreamSetPtr(self, blockNo), index, "view");
}

Value * StreamSetBuffer::getStreamView(llvm::Type * type, llvm::Value * self, Value * blockNo, llvm::Value * index) const {
    return iBuilder->CreateGEP(iBuilder->CreatePointerCast(getStreamSetPtr(self, blockNo), type), index, "view");
}

// Single Block Buffer

// For a single block buffer, the block pointer is always the buffer base pointer.
Value * SingleBlockBuffer::getStreamSetPtr(Value * self, Value *) const {
    return self;
}

// External File Buffer
void ExternalFileBuffer::setStreamSetBuffer(Value * ptr, Value * fileSize) {
    
    PointerType * t = getStreamBufferPointerType();    
    mStreamSetBufferPtr = iBuilder->CreatePointerBitCastOrAddrSpaceCast(ptr, t);
}

void ExternalFileBuffer::setEmptyBuffer(Value * ptr) {
    
    PointerType * t = getStreamBufferPointerType();    
    mStreamSetBufferPtr = iBuilder->CreatePointerBitCastOrAddrSpaceCast(ptr, t);
}

void ExternalFileBuffer::allocateBuffer() {
    throw std::runtime_error("External buffers cannot be allocated.");
}

Value * ExternalFileBuffer::getStreamSetPtr(Value * self, Value * blockNo) const {
    return iBuilder->CreateGEP(self, blockNo);
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

// Linear Copyback Buffer

Value * LinearCopybackBuffer::getStreamSetPtr(Value * self, Value * blockNo) const {
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

llvm::Value * ExpandableBuffer::getStreamView(llvm::Value * self, llvm::Value * blockNo, llvm::Value * index) const {
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

LinearCopybackBuffer::LinearCopybackBuffer(IDISA::IDISA_Builder * b, llvm::Type * type, size_t bufferBlocks, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::LinearCopybackBuffer, b, type, bufferBlocks, AddressSpace) {

}

ExpandableBuffer::ExpandableBuffer(IDISA::IDISA_Builder * b, llvm::Type * type, size_t bufferBlocks, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::ExpandableBuffer, b, type, bufferBlocks, AddressSpace) {

}

StreamSetBuffer::StreamSetBuffer(BufferKind k, IDISA::IDISA_Builder * b, Type * type, unsigned blocks, unsigned AddressSpace)
: mBufferKind(k)
, iBuilder(b)
, mStreamSetType(resolveStreamTypes(type))
, mBufferBlocks(blocks)
, mAddrSpace(AddressSpace)
, mStreamSetBufferPtr(nullptr) {

}
