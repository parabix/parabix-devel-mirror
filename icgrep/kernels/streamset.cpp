/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "streamset.h"
#include <IR_Gen/idisa_builder.h>  // for IDISA_Builder
#include <llvm/IR/BasicBlock.h>    // for BasicBlock
#include <llvm/IR/Constants.h>     // for ConstantInt
#include <llvm/IR/DataLayout.h>    // for DataLayout
#include <llvm/IR/DerivedTypes.h>  // for IntegerType (ptr only), PointerType
#include <llvm/IR/Module.h>        // for Module
#include <llvm/IR/Value.h>         // for Value
#include <llvm/Support/raw_ostream.h>

namespace llvm { class Constant; }
namespace llvm { class Function; }

using namespace parabix;
using namespace llvm;
using namespace IDISA;

ArrayType * resolveStreamSetType(IDISA_Builder * const b, Type * type);

StructType * resolveExpandableStreamSetType(IDISA_Builder * const b, Type * type);

void StreamSetBuffer::allocateBuffer() {
    mStreamSetBufferPtr = iBuilder->CreateCacheAlignedAlloca(getType(), iBuilder->getSize(mBufferBlocks));
}

Value * StreamSetBuffer::getStreamBlockPtr(Value * self, Value * streamIndex, Value * blockIndex) const {
    return iBuilder->CreateGEP(getStreamSetBlockPtr(self, blockIndex), {iBuilder->getInt32(0), streamIndex});
}

Value * StreamSetBuffer::getStreamPackPtr(Value * self, Value * streamIndex, Value * blockIndex, Value * packIndex) const {
    return iBuilder->CreateGEP(getStreamSetBlockPtr(self, blockIndex), {iBuilder->getInt32(0), streamIndex, packIndex});
}

llvm::Value * StreamSetBuffer::getStreamSetCount(Value *) const {
    uint64_t count = 1;
    if (isa<ArrayType>(mBaseType)) {
        count = mBaseType->getArrayNumElements();
    }
    return iBuilder->getInt32(count);
}

/**
 * @brief getRawItemPointer
 *
 * get a raw pointer the iN field at position absoluteItemPosition of the stream number streamIndex of the stream set.
 * In the case of a stream whose fields are less than one byte (8 bits) in size, the pointer is to the containing byte.
 * The type of the pointer is i8* for fields of 8 bits or less, otherwise iN* for N-bit fields.
 */
Value * StreamSetBuffer::getRawItemPointer(Value * self, Value * streamIndex, Value * absolutePosition) const {
    Value * ptr = self;
    if (isa<ConstantInt>(streamIndex) && cast<ConstantInt>(streamIndex)->isZero()) {
        ptr = iBuilder->CreateGEP(ptr, {iBuilder->getInt32(0), streamIndex});
    }
    IntegerType * const ty = cast<IntegerType>(mBaseType->getArrayElementType()->getVectorElementType());
    ptr = iBuilder->CreatePointerCast(ptr, ty->getPointerTo());
    if (LLVM_UNLIKELY(ty->getBitWidth() < 8)) {
        const auto bw = ty->getBitWidth();
        if (LLVM_LIKELY((bw & (bw - 1)) == 0)) { // is power of 2
            absolutePosition = iBuilder->CreateUDiv(absolutePosition, ConstantInt::get(absolutePosition->getType(), 8 / bw));
        } else {
            absolutePosition = iBuilder->CreateMul(absolutePosition, ConstantInt::get(absolutePosition->getType(), bw));
            absolutePosition = iBuilder->CreateUDiv(absolutePosition, ConstantInt::get(absolutePosition->getType(), 8));
        }
    }
    return iBuilder->CreateGEP(ptr, absolutePosition);
}

Value * StreamSetBuffer::getLinearlyAccessibleItems(llvm::Value * fromPosition) const {
    if (isa<ArrayType>(mType) && dyn_cast<ArrayType>(mType)->getNumElements() > 1) {
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
Value * SingleBlockBuffer::getStreamSetBlockPtr(Value * self, Value *) const {
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
    report_fatal_error("External buffers cannot be allocated.");
}

Value * ExternalFileBuffer::getStreamSetBlockPtr(Value * self, Value * blockNo) const {
    return iBuilder->CreateGEP(self, blockNo);
}

Value * ExternalFileBuffer::getLinearlyAccessibleItems(llvm::Value *) const {
    report_fatal_error("External buffers: getLinearlyAccessibleItems is not supported.");
}

// Circular Buffer

Value * CircularBuffer::getStreamSetBlockPtr(Value * self, Value * blockIndex) const {
    assert (blockIndex->getType()->isIntegerTy());
    Value * offset = nullptr;
    if (mBufferBlocks == 1) {
        offset = ConstantInt::getNullValue(iBuilder->getSizeTy());
    } else if ((mBufferBlocks & (mBufferBlocks - 1)) == 0) { // is power of 2
        offset = iBuilder->CreateAnd(blockIndex, ConstantInt::get(blockIndex->getType(), mBufferBlocks - 1));
    } else {
        offset = iBuilder->CreateURem(blockIndex, ConstantInt::get(blockIndex->getType(), mBufferBlocks));
    }
    return iBuilder->CreateGEP(self, offset);
}


// CircularCopybackBuffer Buffer

void CircularCopybackBuffer::allocateBuffer() {
    mStreamSetBufferPtr = iBuilder->CreateCacheAlignedAlloca(getType(), iBuilder->getSize(mBufferBlocks + mOverflowBlocks));
}

void CircularCopybackBuffer::createCopyBack(Value * self, Value * overFlowItems) const {
    Function * f = iBuilder->GetInsertBlock()->getParent();
    BasicBlock * wholeBlockCopy = BasicBlock::Create(iBuilder->getContext(), "wholeBlockCopy", f, 0);
    BasicBlock * partialBlockCopy = BasicBlock::Create(iBuilder->getContext(), "partialBlockCopy", f, 0);
    BasicBlock * copyBackDone = BasicBlock::Create(iBuilder->getContext(), "copyBackDone", f, 0);
    Type * i8ptr = iBuilder->getInt8PtrTy();
    unsigned numStreams = getType()->getArrayNumElements();
    auto elemTy = getType()->getArrayElementType();
    unsigned fieldWidth = isa<ArrayType>(elemTy) ? elemTy->getArrayNumElements() : 1;
    Constant * blockSize = iBuilder->getSize(iBuilder->getBitBlockWidth());
    Value * overFlowAreaPtr = iBuilder->CreateGEP(self, iBuilder->getSize(mBufferBlocks));
    Value * overFlowBlocks = iBuilder->CreateUDiv(overFlowItems, blockSize);
    Value * partialItems = iBuilder->CreateURem(overFlowItems, blockSize);
    iBuilder->CreateCondBr(iBuilder->CreateICmpUGT(overFlowBlocks, iBuilder->getSize(0)), wholeBlockCopy, partialBlockCopy);
    iBuilder->SetInsertPoint(wholeBlockCopy);
    unsigned alignment = iBuilder->getBitBlockWidth() / 8;
    Constant * blockBytes = iBuilder->getSize(fieldWidth * iBuilder->getBitBlockWidth()/8);
    Value * copyLength = iBuilder->CreateMul(overFlowBlocks, blockBytes);
    iBuilder->CreateMemMove(iBuilder->CreateBitCast(self, i8ptr), iBuilder->CreateBitCast(overFlowAreaPtr, i8ptr), copyLength, alignment);
    iBuilder->CreateCondBr(iBuilder->CreateICmpUGT(partialItems, iBuilder->getSize(0)), partialBlockCopy, copyBackDone);
    iBuilder->SetInsertPoint(partialBlockCopy);
    Value * partialBlockTargetPtr = iBuilder->CreateGEP(self, overFlowBlocks);
    Value * partialBlockSourcePtr = iBuilder->CreateGEP(overFlowAreaPtr, overFlowBlocks);
    Value * copyBits = iBuilder->CreateMul(overFlowItems, iBuilder->getSize(fieldWidth));
    Value * copyBytes = iBuilder->CreateLShr(iBuilder->CreateAdd(copyBits, iBuilder->getSize(7)), iBuilder->getSize(3));
    for (unsigned strm = 0; strm < numStreams; strm++) {
        Value * strmTargetPtr = iBuilder->CreateGEP(partialBlockTargetPtr, {iBuilder->getInt32(0), iBuilder->getInt32(strm)});
        Value * strmSourcePtr = iBuilder->CreateGEP(partialBlockSourcePtr, {iBuilder->getInt32(0), iBuilder->getInt32(strm)});
        iBuilder->CreateMemMove(iBuilder->CreateBitCast(strmTargetPtr, i8ptr), iBuilder->CreateBitCast(strmSourcePtr, i8ptr), copyBytes, alignment);
    }
    iBuilder->CreateBr(copyBackDone);
    iBuilder->SetInsertPoint(copyBackDone);
}

Value * CircularCopybackBuffer::getStreamSetBlockPtr(Value * self, Value * blockIndex) const {
    assert (blockIndex->getType()->isIntegerTy());
    
    Value * offset = nullptr;
    if (mBufferBlocks == 1) {
        offset = ConstantInt::getNullValue(iBuilder->getSizeTy());
    } else if ((mBufferBlocks & (mBufferBlocks - 1)) == 0) { // is power of 2
        offset = iBuilder->CreateAnd(blockIndex, ConstantInt::get(blockIndex->getType(), mBufferBlocks - 1));
    } else {
        offset = iBuilder->CreateURem(blockIndex, ConstantInt::get(blockIndex->getType(), mBufferBlocks));
    }
    return iBuilder->CreateGEP(self, offset);
}



// Expandable Buffer

void ExpandableBuffer::allocateBuffer() {
    mStreamSetBufferPtr = iBuilder->CreateCacheAlignedAlloca(getType());
    Value * const capacityPtr = iBuilder->CreateGEP(mStreamSetBufferPtr, {iBuilder->getInt32(0), iBuilder->getInt32(0)});
    iBuilder->CreateStore(iBuilder->getSize(mInitialCapacity), capacityPtr);
    Type * const bufferType = getType()->getStructElementType(1)->getPointerElementType();
    ConstantInt * const size = iBuilder->getSize(mBufferBlocks * mInitialCapacity);
    Value * const ptr = iBuilder->CreateAlignedMalloc(bufferType, size, iBuilder->getCacheAlignment());
    const auto alignment = bufferType->getPrimitiveSizeInBits() / 8;
    iBuilder->CreateMemZero(ptr, size, alignment);
    Value * const streamSetPtr = iBuilder->CreateGEP(mStreamSetBufferPtr, {iBuilder->getInt32(0), iBuilder->getInt32(1)});
    iBuilder->CreateStore(ptr, streamSetPtr);
}

std::pair<Value *, Value *> ExpandableBuffer::getExpandedStreamOffset(llvm::Value * self, llvm::Value * streamIndex, Value * blockIndex) const {

    // MDNode *Weights = MDBuilder(Ctx).createBranchWeights(42, 13);

    // ENTRY
    Value * const capacityPtr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(0)});
    Value * const capacity = iBuilder->CreateLoad(capacityPtr);
    Value * const streamSetPtr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(1)});
    Value * const streamSet = iBuilder->CreateLoad(streamSetPtr);

    // Are we guaranteed that we can access this stream?
    if (LLVM_UNLIKELY(isa<ConstantInt>(streamIndex))) {
        if (LLVM_LIKELY(cast<ConstantInt>(streamIndex)->getLimitedValue() < mInitialCapacity)) {
            return {streamSet, capacity};
        }
    }

    BasicBlock * const entry = iBuilder->GetInsertBlock();
    BasicBlock * const expand = BasicBlock::Create(iBuilder->getContext(), "expand", entry->getParent());
    BasicBlock * const resume = BasicBlock::Create(iBuilder->getContext(), "resume", entry->getParent());

    assert (streamIndex->getType() == capacity->getType());
    Value * cond = iBuilder->CreateICmpULT(streamIndex, capacity);
    iBuilder->CreateCondBr(cond, resume, expand);
    // EXPAND
    iBuilder->SetInsertPoint(expand);
    /// TODO: this should call a function rather than be inlined into the block. REVISIT once tested.
    Value * newCapacity = iBuilder->CreateMul(streamIndex, iBuilder->getSize(2));
    iBuilder->CreateStore(newCapacity, capacityPtr);
    Type * bufferType = getType()->getStructElementType(1)->getPointerElementType();
    Value * size = iBuilder->CreateMul(newCapacity, iBuilder->getSize(mBufferBlocks));
    Value * newStreamSet = iBuilder->CreateAlignedMalloc(bufferType, size, iBuilder->getCacheAlignment());
    iBuilder->CreateStore(newStreamSet, streamSetPtr);
    Value * const diffCapacity = iBuilder->CreateSub(newCapacity, capacity);
    const auto alignment = bufferType->getPrimitiveSizeInBits() / 8;
    for (unsigned i = 0; i < mBufferBlocks; ++i) {
        ConstantInt * const offset = iBuilder->getSize(i);
        Value * srcOffset = iBuilder->CreateMul(capacity, offset);
        Value * srcPtr = iBuilder->CreateGEP(streamSet, srcOffset);
        Value * destOffset = iBuilder->CreateMul(newCapacity, offset);
        Value * destPtr = iBuilder->CreateGEP(newStreamSet, destOffset);
        iBuilder->CreateMemCpy(destPtr, srcPtr, capacity, alignment);
        Value * destZeroOffset = iBuilder->CreateAdd(destOffset, capacity);
        Value * destZeroPtr = iBuilder->CreateGEP(newStreamSet, destZeroOffset);
        iBuilder->CreateMemZero(destZeroPtr, diffCapacity, alignment);
    }

    iBuilder->CreateAlignedFree(streamSet);
    iBuilder->CreateBr(resume);
    // RESUME
    iBuilder->SetInsertPoint(resume);

    PHINode * phiStreamSet = iBuilder->CreatePHI(streamSet->getType(), 2);
    phiStreamSet->addIncoming(streamSet, entry);
    phiStreamSet->addIncoming(newStreamSet, expand);

    PHINode * phiCapacity = iBuilder->CreatePHI(capacity->getType(), 2);
    phiCapacity->addIncoming(capacity, entry);
    phiCapacity->addIncoming(newCapacity, expand);

    Value * offset = iBuilder->CreateAdd(iBuilder->CreateMul(blockIndex, phiCapacity), streamIndex);

    return {phiStreamSet, offset};
}

llvm::Value * ExpandableBuffer::getStreamBlockPtr(llvm::Value * self, Value * streamIndex, Value * blockIndex) const {
    Value * ptr, * offset;
    std::tie(ptr, offset) = getExpandedStreamOffset(self, streamIndex, blockIndex);
    return iBuilder->CreateGEP(ptr, offset);
}

llvm::Value * ExpandableBuffer::getStreamPackPtr(llvm::Value * self, llvm::Value * streamIndex, Value * blockIndex, Value * packIndex) const {
    Value * ptr, * offset;
    std::tie(ptr, offset) = getExpandedStreamOffset(self, streamIndex, blockIndex);
    return iBuilder->CreateGEP(ptr, {offset, packIndex});
}

llvm::Value * ExpandableBuffer::getStreamSetCount(llvm::Value * self) const {
    return iBuilder->CreateLoad(iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(0)}));
}

Value * ExpandableBuffer::getStreamSetBlockPtr(Value *, Value *) const {
    report_fatal_error("Expandable buffers: getStreamSetBlockPtr is not supported.");
}

Value * ExpandableBuffer::getLinearlyAccessibleItems(llvm::Value *) const {
    report_fatal_error("Expandable buffers: getLinearlyAccessibleItems is not supported.");
}

// Constructors
SingleBlockBuffer::SingleBlockBuffer(IDISA::IDISA_Builder * b, llvm::Type * type)
: StreamSetBuffer(BufferKind::BlockBuffer, b, type, resolveStreamSetType(b, type), 1, 0) {

}

ExternalFileBuffer::ExternalFileBuffer(IDISA::IDISA_Builder * b, llvm::Type * type, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::ExternalFileBuffer, b, type, resolveStreamSetType(b, type), 0, AddressSpace) {

}

CircularBuffer::CircularBuffer(IDISA::IDISA_Builder * b, llvm::Type * type, size_t bufferBlocks, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::CircularBuffer, b, type, resolveStreamSetType(b, type), bufferBlocks, AddressSpace) {

}

CircularCopybackBuffer::CircularCopybackBuffer(IDISA::IDISA_Builder * b, llvm::Type * type, size_t bufferBlocks, size_t overflowBlocks, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::CircularCopybackBuffer, b, type, resolveStreamSetType(b, type), bufferBlocks, AddressSpace), mOverflowBlocks(overflowBlocks) {

}

ExpandableBuffer::ExpandableBuffer(IDISA::IDISA_Builder * b, llvm::Type * type, size_t bufferBlocks, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::ExpandableBuffer, b, type, resolveExpandableStreamSetType(b, type), bufferBlocks, AddressSpace)
, mInitialCapacity(type->getArrayNumElements()) {

}

inline StreamSetBuffer::StreamSetBuffer(BufferKind k, IDISA::IDISA_Builder * b, Type * baseType, Type * resolvedType, unsigned blocks, unsigned AddressSpace)
: mBufferKind(k)
, iBuilder(b)
, mType(resolvedType)
, mBufferBlocks(blocks)
, mAddressSpace(AddressSpace)
, mStreamSetBufferPtr(nullptr)
, mBaseType(baseType) {

}

// Helper routines
ArrayType * resolveStreamSetType(IDISA_Builder * const b, Type * type) {
    unsigned numElements = 1;
    if (LLVM_LIKELY(type->isArrayTy())) {
        numElements = type->getArrayNumElements();
        type = type->getArrayElementType();
    }
    if (LLVM_LIKELY(type->isVectorTy() && type->getVectorNumElements() == 0)) {
        type = type->getVectorElementType();
        if (LLVM_LIKELY(type->isIntegerTy())) {
            const auto fieldWidth = cast<IntegerType>(type)->getBitWidth();
            type = b->getBitBlockType();
            if (fieldWidth != 1) {
                type = ArrayType::get(type, fieldWidth);
            }
            return ArrayType::get(type, numElements);
        }
    }
    std::string tmp;
    raw_string_ostream out(tmp);
    type->print(out);
    out << " is an unvalid stream set buffer type.";
    report_fatal_error(out.str());
}

StructType * resolveExpandableStreamSetType(IDISA_Builder * const b, Type * type) {
    if (LLVM_LIKELY(type->isArrayTy())) {
        type = type->getArrayElementType();
    }
    if (LLVM_LIKELY(type->isVectorTy() && type->getVectorNumElements() == 0)) {
        type = type->getVectorElementType();
        if (LLVM_LIKELY(type->isIntegerTy())) {
            const auto fieldWidth = cast<IntegerType>(type)->getBitWidth();
            type = b->getBitBlockType();
            if (fieldWidth != 1) {
                type = ArrayType::get(type, fieldWidth);
            }
            return StructType::get(b->getSizeTy(), type->getPointerTo(), nullptr);
        }
    }
    std::string tmp;
    raw_string_ostream out(tmp);
    type->print(out);
    out << " is an unvalid stream set buffer type.";
    report_fatal_error(out.str());
}
