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
    mStreamSetBufferPtr = iBuilder->CreateCacheAlignedAlloca(getType(), iBuilder->getSize(mBufferSize));
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

// Circular Buffer

Value * CircularBuffer::getStreamSetPtr(Value * self, Value * blockNo) const {
    assert (blockNo->getType()->isIntegerTy());

    Value * offset = nullptr;
    if (mBufferSize == 1) {
        offset = ConstantInt::getNullValue(iBuilder->getSizeTy());
    } else if ((mBufferSize & (mBufferSize - 1)) == 0) { // is power of 2
        offset = iBuilder->CreateAnd(blockNo, ConstantInt::get(blockNo->getType(), mBufferSize - 1));
    } else {
        offset = iBuilder->CreateURem(blockNo, ConstantInt::get(blockNo->getType(), mBufferSize));
    }
    return iBuilder->CreateGEP(self, offset);
}

// Linear Copyback Buffer

Value * LinearCopybackBuffer::getStreamSetPtr(Value * self, Value * blockNo) const {
    Value * offset = nullptr;
    if (mBufferSize == 1) {
        offset = ConstantInt::getNullValue(iBuilder->getSizeTy());
    } else if ((mBufferSize & (mBufferSize - 1)) == 0) { // is power of 2
        offset = iBuilder->CreateAnd(blockNo, ConstantInt::get(blockNo->getType(), mBufferSize - 1));
    } else {
        offset = iBuilder->CreateURem(blockNo, ConstantInt::get(blockNo->getType(), mBufferSize));
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

LinearCopybackBuffer::LinearCopybackBuffer(IDISA::IDISA_Builder * b, llvm::Type * type, size_t bufferBlocks, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::LinearCopybackBuffer, b, type, bufferBlocks, AddressSpace) {

}

ExpandableBuffer::ExpandableBuffer(IDISA::IDISA_Builder * b, llvm::Type * type, size_t bufferBlocks, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::ExpandableBuffer, b, type, bufferBlocks, AddressSpace) {

}

StreamSetBuffer::StreamSetBuffer(BufferKind k, IDISA::IDISA_Builder * b, Type * type, unsigned blocks, unsigned AddressSpace)
: mBufferKind(k)
, iBuilder(b)
, mStreamSetType(resolveStreamSetBufferType(type))
, mBufferSize(blocks)
, mAddressSpace(AddressSpace)
, mStreamSetBufferPtr(nullptr)
, mBaseStreamSetType(type) {

}
