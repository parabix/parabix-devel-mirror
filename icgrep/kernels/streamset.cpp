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
#include <llvm/IR/CFG.h>

namespace llvm { class Constant; }
namespace llvm { class Function; }

using namespace parabix;
using namespace llvm;
using namespace IDISA;

ArrayType * resolveStreamSetType(IDISA_Builder * const b, Type * type);

StructType * resolveExpandableStreamSetType(IDISA_Builder * const b, Type * type);

void StreamSetBuffer::allocateBuffer() {
    Type * const ty = getType();
    ConstantInt * blocks = iBuilder->getSize(mBufferBlocks);
    mStreamSetBufferPtr = iBuilder->CreateCacheAlignedAlloca(ty, iBuilder->getSize(mBufferBlocks));
    Constant * width = ConstantExpr::getMul(ConstantExpr::getSizeOf(ty), blocks);
    iBuilder->CreateMemZero(mStreamSetBufferPtr, width, iBuilder->getCacheAlignment());
}

Value * StreamSetBuffer::getStreamBlockPtr(Value * self, Value * streamIndex, Value * blockIndex, const bool /* readOnly */) const {
    iBuilder->CreateAssert(iBuilder->CreateICmpULT(streamIndex, getStreamSetCount(self)), "StreamSetBuffer: out-of-bounds stream access");
    return iBuilder->CreateGEP(getStreamSetBlockPtr(getBaseAddress(self), blockIndex), {iBuilder->getInt32(0), streamIndex});
}

Value * StreamSetBuffer::getStreamPackPtr(Value * self, Value * streamIndex, Value * blockIndex, Value * packIndex, const bool /* readOnly */) const {
    iBuilder->CreateAssert(iBuilder->CreateICmpULT(streamIndex, getStreamSetCount(self)), "StreamSetBuffer: out-of-bounds stream access");
    return iBuilder->CreateGEP(getStreamSetBlockPtr(getBaseAddress(self), blockIndex), {iBuilder->getInt32(0), streamIndex, packIndex});
}

inline bool StreamSetBuffer::isCapacityGuaranteed(const Value * const index, const size_t capacity) const {
    if (LLVM_UNLIKELY(isa<ConstantInt>(index))) {
        if (LLVM_LIKELY(cast<ConstantInt>(index)->getLimitedValue() < capacity)) {
            return true;
        }
    }
    return false;
}

Value * StreamSetBuffer::getStreamSetCount(Value *) const {
    uint64_t count = 1;
    if (isa<ArrayType>(mBaseType)) {
        count = mBaseType->getArrayNumElements();
    }
    return iBuilder->getSize(count);
}

inline Value * StreamSetBuffer::modByBufferBlocks(Value * const offset) const {
    assert (offset->getType()->isIntegerTy());
    if (isCapacityGuaranteed(offset, mBufferBlocks)) {
        return offset;
    } else if (mBufferBlocks == 1) {
        return ConstantInt::getNullValue(iBuilder->getSizeTy());
    } else if ((mBufferBlocks & (mBufferBlocks - 1)) == 0) { // is power of 2
        return iBuilder->CreateAnd(offset, ConstantInt::get(offset->getType(), mBufferBlocks - 1));
    } else {
        return iBuilder->CreateURem(offset, ConstantInt::get(offset->getType(), mBufferBlocks));
    }
}

/**
 * @brief getRawItemPointer
 *
 * get a raw pointer the iN field at position absoluteItemPosition of the stream number streamIndex of the stream set.
 * In the case of a stream whose fields are less than one byte (8 bits) in size, the pointer is to the containing byte.
 * The type of the pointer is i8* for fields of 8 bits or less, otherwise iN* for N-bit fields.
 */
Value * StreamSetBuffer::getRawItemPointer(Value * self, Value * streamIndex, Value * absolutePosition) const {
    Value * ptr = getBaseAddress(self);
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

Value * StreamSetBuffer::getLinearlyAccessibleItems(Value * self, Value * fromPosition) const {
    if (isa<ArrayType>(mType) && dyn_cast<ArrayType>(mType)->getNumElements() > 1) {
        Constant * stride = iBuilder->getSize(iBuilder->getStride());
        return iBuilder->CreateSub(stride, iBuilder->CreateURem(fromPosition, stride));
    } else {
        Constant * bufSize = iBuilder->getSize(mBufferBlocks * iBuilder->getStride());
        return iBuilder->CreateSub(bufSize, iBuilder->CreateURem(fromPosition, bufSize));
    }
}

Value * StreamSetBuffer::getLinearlyAccessibleBlocks(Value * self, Value * fromBlock) const {
    Constant * bufBlocks = iBuilder->getSize(mBufferBlocks);
    return iBuilder->CreateSub(bufBlocks, iBuilder->CreateURem(fromBlock, bufBlocks));
}

void StreamSetBuffer::reserveBytes(Value * self, llvm::Value * position, llvm::Value *requested) const {
    report_fatal_error("reserve() can only be used with ExtensibleBuffers");
}

Value * StreamSetBuffer::getBaseAddress(Value * self) const {
    return self;
}

void StreamSetBuffer::releaseBuffer(Value * /* self */) {
    /* do nothing: memory is stack allocated */
}

// Single Block Buffer

// For a single block buffer, the block pointer is always the buffer base pointer.
Value * SingleBlockBuffer::getStreamSetBlockPtr(Value * self, Value *) const {
    return self;
}

// External File Buffer
void ExternalFileBuffer::setStreamSetBuffer(Value * ptr) {
    mStreamSetBufferPtr = iBuilder->CreatePointerBitCastOrAddrSpaceCast(ptr, getPointerType());
}

void ExternalFileBuffer::allocateBuffer() {
    report_fatal_error("External buffers cannot be allocated.");
}

Value * ExternalFileBuffer::getStreamSetBlockPtr(Value * self, Value * blockIndex) const {
    return iBuilder->CreateGEP(self, blockIndex);
}

Value * ExternalFileBuffer::getLinearlyAccessibleItems(Value * self, Value *) const {
    report_fatal_error("External buffers: getLinearlyAccessibleItems is not supported.");
}

// ExtensibleBuffer
Value * ExtensibleBuffer::getLinearlyAccessibleItems(Value * self, Value * fromPosition) const {
    Value * capacityPtr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(0)});
    Value * capacity = iBuilder->CreateLoad(capacityPtr);
    return iBuilder->CreateSub(capacity, fromPosition);
}

void ExtensibleBuffer::allocateBuffer() {
    Type * ty = getType();
    Value * instance = iBuilder->CreateCacheAlignedAlloca(ty);
    Value * const capacityPtr = iBuilder->CreateGEP(instance, {iBuilder->getInt32(0), iBuilder->getInt32(0)});
    Constant * initialSize = ConstantExpr::getSizeOf(ty->getStructElementType(1)->getPointerElementType());
    initialSize = ConstantExpr::getMul(initialSize, iBuilder->getSize(mBufferBlocks));
    initialSize = ConstantExpr::getIntegerCast(initialSize, iBuilder->getSizeTy(), false);
    iBuilder->CreateStore(initialSize, capacityPtr);
    Value * addr = iBuilder->CreateAnonymousMMap(initialSize);
    Value * const addrPtr = iBuilder->CreateGEP(instance, {iBuilder->getInt32(0), iBuilder->getInt32(1)});
    addr = iBuilder->CreatePointerCast(addr, addrPtr->getType()->getPointerElementType());
    iBuilder->CreateStore(addr, addrPtr);
    mStreamSetBufferPtr = instance;
}

Value * ExtensibleBuffer::getStreamSetBlockPtr(Value * self, Value * blockIndex) const {
    return iBuilder->CreateGEP(self, blockIndex);
}

void ExtensibleBuffer::reserveBytes(Value * const self, llvm::Value * const position, llvm::Value * const requested) const {
    Value * const capacityPtr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(0)});
    Value * const currentSize = iBuilder->CreateLoad(capacityPtr);
    Type * const intTy = currentSize->getType();
    assert (position->getType() == requested->getType());
    Constant * const blockSize = ConstantExpr::getIntegerCast(ConstantExpr::getIntegerCast(ConstantExpr::getSizeOf(getType()->getStructElementType(1)), intTy, false), requested->getType(), false);
    BasicBlock * const entry = iBuilder->GetInsertBlock();
    BasicBlock * const expand = BasicBlock::Create(iBuilder->getContext(), "expand", entry->getParent());
    BasicBlock * const resume = BasicBlock::Create(iBuilder->getContext(), "resume", entry->getParent());
    Value * const reserved = iBuilder->CreateAdd(iBuilder->CreateMul(position, blockSize), requested);
    iBuilder->CreateLikelyCondBr(iBuilder->CreateICmpULT(reserved, currentSize), resume, expand);
    iBuilder->SetInsertPoint(expand);
    Value * const reservedSize = iBuilder->CreateShl(reserved, 1);
    Value * const baseAddrPtr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(1)});
    Value * const baseAddr = iBuilder->CreateLoad(baseAddrPtr);
    Value * newAddr = iBuilder->CreateMRemap(baseAddr, currentSize, reservedSize);
    newAddr = iBuilder->CreatePointerCast(newAddr, baseAddr->getType());
    iBuilder->CreateStore(reservedSize, capacityPtr);
    iBuilder->CreateStore(newAddr, baseAddrPtr);
    iBuilder->CreateBr(resume);

    iBuilder->SetInsertPoint(resume);
}

Value * ExtensibleBuffer::getBaseAddress(Value * const self) const {
    return iBuilder->CreateLoad(iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(1)}));
}

void ExtensibleBuffer::releaseBuffer(Value * self) {
    Value * const sizePtr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(0)});
    Value * size = iBuilder->CreateLoad(sizePtr);
    iBuilder->CreateMUnmap(getBaseAddress(self), size);
}

// Circular Buffer

Value * CircularBuffer::getStreamSetBlockPtr(Value * const self, Value * const blockIndex) const {
    return iBuilder->CreateGEP(self, modByBufferBlocks(blockIndex));
}

// CircularCopybackBuffer Buffer

void CircularCopybackBuffer::allocateBuffer() {
    mStreamSetBufferPtr = iBuilder->CreateCacheAlignedAlloca(getType(), iBuilder->getSize(mBufferBlocks + mOverflowBlocks));
}

void CircularCopybackBuffer::createCopyBack(Value * self, Value * overFlowItems) const {
    Type * size_ty = iBuilder->getSizeTy();
    Type * i8ptr = iBuilder->getInt8PtrTy();
    Constant * blockSize = iBuilder->getSize(iBuilder->getBitBlockWidth());
    Function * f = iBuilder->GetInsertBlock()->getParent();
    BasicBlock * wholeBlockCopy = BasicBlock::Create(iBuilder->getContext(), "wholeBlockCopy", f, 0);
    BasicBlock * partialBlockCopy = BasicBlock::Create(iBuilder->getContext(), "partialBlockCopy", f, 0);
    BasicBlock * copyBackDone = BasicBlock::Create(iBuilder->getContext(), "copyBackDone", f, 0);
    unsigned numStreams = getType()->getArrayNumElements();
    auto elemTy = getType()->getArrayElementType();
    unsigned fieldWidth = isa<ArrayType>(elemTy) ? elemTy->getArrayNumElements() : 1;
    Value * overFlowAreaPtr = iBuilder->CreateGEP(self, iBuilder->getSize(mBufferBlocks));
    Value * overFlowBlocks = iBuilder->CreateUDiv(overFlowItems, blockSize);
    Value * partialItems = iBuilder->CreateURem(overFlowItems, blockSize);
    Value * partialBlockTargetPtr = iBuilder->CreateGEP(self, overFlowBlocks);
    Value * partialBlockSourcePtr = iBuilder->CreateGEP(overFlowAreaPtr, overFlowBlocks);
    iBuilder->CreateCondBr(iBuilder->CreateICmpUGT(overFlowBlocks, iBuilder->getSize(0)), wholeBlockCopy, partialBlockCopy);
    iBuilder->SetInsertPoint(wholeBlockCopy);
    unsigned alignment = iBuilder->getBitBlockWidth() / 8;
    Value * copyLength = iBuilder->CreateSub(iBuilder->CreatePtrToInt(partialBlockTargetPtr, size_ty), iBuilder->CreatePtrToInt(self, size_ty));
    iBuilder->CreateMemMove(iBuilder->CreateBitCast(self, i8ptr), iBuilder->CreateBitCast(overFlowAreaPtr, i8ptr), copyLength, alignment);
    iBuilder->CreateCondBr(iBuilder->CreateICmpUGT(partialItems, iBuilder->getSize(0)), partialBlockCopy, copyBackDone);
    iBuilder->SetInsertPoint(partialBlockCopy);
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
    return iBuilder->CreateGEP(self, modByBufferBlocks(blockIndex));
}

// SwizzledCopybackBuffer Buffer

void SwizzledCopybackBuffer::allocateBuffer() {
    mStreamSetBufferPtr = iBuilder->CreateCacheAlignedAlloca(getType(), iBuilder->getSize(mBufferBlocks + mOverflowBlocks));
}

void SwizzledCopybackBuffer::createCopyBack(Value * self, Value * overFlowItems) const {
    Type * size_ty = iBuilder->getSizeTy();
    Type * i8ptr = iBuilder->getInt8PtrTy();
    Constant * blockSize = iBuilder->getSize(iBuilder->getBitBlockWidth());
    Function * f = iBuilder->GetInsertBlock()->getParent();
    BasicBlock * wholeBlockCopy = BasicBlock::Create(iBuilder->getContext(), "wholeBlockCopy", f, 0);
    BasicBlock * partialBlockCopy = BasicBlock::Create(iBuilder->getContext(), "partialBlockCopy", f, 0);
    BasicBlock * copyBackDone = BasicBlock::Create(iBuilder->getContext(), "copyBackDone", f, 0);
    unsigned numStreams = getType()->getArrayNumElements();
    unsigned swizzleFactor = iBuilder->getBitBlockWidth()/mFieldWidth;
    auto elemTy = getType()->getArrayElementType();
    unsigned fieldWidth = isa<ArrayType>(elemTy) ? elemTy->getArrayNumElements() : 1;
    Value * overFlowAreaPtr = iBuilder->CreateGEP(self, iBuilder->getSize(mBufferBlocks));
    Value * overFlowBlocks = iBuilder->CreateUDiv(overFlowItems, blockSize);
    Value * partialItems = iBuilder->CreateURem(overFlowItems, blockSize);
    Value * partialBlockTargetPtr = iBuilder->CreateGEP(self, overFlowBlocks);
    Value * partialBlockSourcePtr = iBuilder->CreateGEP(overFlowAreaPtr, overFlowBlocks);
    iBuilder->CreateCondBr(iBuilder->CreateICmpUGT(overFlowBlocks, iBuilder->getSize(0)), wholeBlockCopy, partialBlockCopy);
    iBuilder->SetInsertPoint(wholeBlockCopy);
    unsigned alignment = iBuilder->getBitBlockWidth() / 8;
    Value * copyLength = iBuilder->CreateSub(iBuilder->CreatePtrToInt(partialBlockTargetPtr, size_ty), iBuilder->CreatePtrToInt(self, size_ty));
    iBuilder->CreateMemMove(iBuilder->CreateBitCast(self, i8ptr), iBuilder->CreateBitCast(overFlowAreaPtr, i8ptr), copyLength, alignment);
    iBuilder->CreateCondBr(iBuilder->CreateICmpUGT(partialItems, iBuilder->getSize(0)), partialBlockCopy, copyBackDone);
    iBuilder->SetInsertPoint(partialBlockCopy);
    Value * copyBits = iBuilder->CreateMul(overFlowItems, iBuilder->getSize(fieldWidth * swizzleFactor));
    Value * copyBytes = iBuilder->CreateLShr(iBuilder->CreateAdd(copyBits, iBuilder->getSize(7)), iBuilder->getSize(3));
    for (unsigned strm = 0; strm < numStreams; strm += swizzleFactor) {
        Value * strmTargetPtr = iBuilder->CreateGEP(partialBlockTargetPtr, {iBuilder->getInt32(0), iBuilder->getInt32(strm)});
        Value * strmSourcePtr = iBuilder->CreateGEP(partialBlockSourcePtr, {iBuilder->getInt32(0), iBuilder->getInt32(strm)});
        iBuilder->CreateMemMove(iBuilder->CreateBitCast(strmTargetPtr, i8ptr), iBuilder->CreateBitCast(strmSourcePtr, i8ptr), copyBytes, alignment);
    }
    iBuilder->CreateBr(copyBackDone);
    iBuilder->SetInsertPoint(copyBackDone);
}

Value * SwizzledCopybackBuffer::getStreamSetBlockPtr(Value * self, Value * blockIndex) const {
    return iBuilder->CreateGEP(self, modByBufferBlocks(blockIndex));
}

SwizzledCopybackBuffer::SwizzledCopybackBuffer(IDISA::IDISA_Builder * b, Type * type, size_t bufferBlocks, size_t overflowBlocks, unsigned fieldwidth, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::SwizzledCopybackBuffer, b, type, resolveStreamSetType(b, type), bufferBlocks, AddressSpace), mOverflowBlocks(overflowBlocks), mFieldWidth(fieldwidth) {
    
}

// Expandable Buffer

void ExpandableBuffer::allocateBuffer() {
    mStreamSetBufferPtr = iBuilder->CreateCacheAlignedAlloca(getType());
    Value * const capacityPtr = iBuilder->CreateGEP(mStreamSetBufferPtr, {iBuilder->getInt32(0), iBuilder->getInt32(0)});
    iBuilder->CreateStore(iBuilder->getSize(mInitialCapacity), capacityPtr);
    Type * const bufferType = getType()->getStructElementType(1)->getPointerElementType();
    Constant * const bufferWidth = ConstantExpr::getIntegerCast(ConstantExpr::getSizeOf(bufferType), iBuilder->getSizeTy(), false);
    Constant * const size = ConstantExpr::getMul(iBuilder->getSize(mBufferBlocks * mInitialCapacity), bufferWidth);
    Value * const ptr = iBuilder->CreateAlignedMalloc(size, iBuilder->getCacheAlignment());
    iBuilder->CreateMemZero(ptr, size, bufferType->getPrimitiveSizeInBits() / 8);
    Value * const streamSetPtr = iBuilder->CreateGEP(mStreamSetBufferPtr, {iBuilder->getInt32(0), iBuilder->getInt32(1)});
    iBuilder->CreateStore(iBuilder->CreatePointerCast(ptr, bufferType->getPointerTo()), streamSetPtr);
}

std::pair<Value *, Value *> ExpandableBuffer::getInternalStreamBuffer(Value * self, Value * streamIndex, Value * blockIndex, const bool readOnly) const {

    // ENTRY
    Value * const capacityPtr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(0)});
    Value * const capacity = iBuilder->CreateLoad(capacityPtr);
    Value * const streamSetPtr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(1)});
    Value * const streamSet = iBuilder->CreateLoad(streamSetPtr);
    blockIndex = modByBufferBlocks(blockIndex);

    assert (streamIndex->getType() == capacity->getType());
    Value * const cond = iBuilder->CreateICmpULT(streamIndex, capacity);

    // Are we guaranteed that we can access this stream?
    if (readOnly || isCapacityGuaranteed(streamIndex, mInitialCapacity)) {
        iBuilder->CreateAssert(cond, "ExpandableBuffer: out-of-bounds stream access");
        Value * offset = iBuilder->CreateAdd(iBuilder->CreateMul(blockIndex, capacity), streamIndex);
        return {streamSet, offset};
    }

    BasicBlock * const entry = iBuilder->GetInsertBlock();
    BasicBlock * const expand = BasicBlock::Create(iBuilder->getContext(), "expand", entry->getParent());
    BasicBlock * const resume = BasicBlock::Create(iBuilder->getContext(), "resume", entry->getParent());

    iBuilder->CreateLikelyCondBr(cond, resume, expand);

    // EXPAND
    iBuilder->SetInsertPoint(expand);

    Type * elementType = getType()->getStructElementType(1)->getPointerElementType();
    Constant * const vectorWidth = ConstantExpr::getIntegerCast(ConstantExpr::getSizeOf(elementType), capacity->getType(), false);

    Value * newCapacity = iBuilder->CreateAdd(streamIndex, iBuilder->getSize(1));
    newCapacity = iBuilder->CreateCeilLog2(newCapacity);
    newCapacity = iBuilder->CreateShl(iBuilder->getSize(1), newCapacity, "newCapacity");

    std::string tmp;
    raw_string_ostream out(tmp);
    out << "__expand";
    elementType->print(out);
    std::string name = out.str();

    Module * const m = iBuilder->getModule();
    Function * expandFunction = m->getFunction(name);

    if (expandFunction == nullptr) {

        const auto ip = iBuilder->saveIP();

        FunctionType * fty = FunctionType::get(elementType->getPointerTo(), {elementType->getPointerTo(), iBuilder->getSizeTy(), iBuilder->getSizeTy()}, false);
        expandFunction = Function::Create(fty, GlobalValue::PrivateLinkage, name, m);

        auto args = expandFunction->arg_begin();
        Value * streamSet = &*args++;
        Value * capacity = &*args++;
        Value * newCapacity = &*args;

        BasicBlock * entry = BasicBlock::Create(iBuilder->getContext(), "entry", expandFunction);
        iBuilder->SetInsertPoint(entry);

        Value * size = iBuilder->CreateMul(newCapacity, iBuilder->getSize(mBufferBlocks));
        Value * newStreamSet = iBuilder->CreatePointerCast(iBuilder->CreateAlignedMalloc(iBuilder->CreateMul(size, vectorWidth), iBuilder->getCacheAlignment()), elementType->getPointerTo());
        Value * const diffCapacity = iBuilder->CreateMul(iBuilder->CreateSub(newCapacity, capacity), vectorWidth);

        const auto alignment = elementType->getPrimitiveSizeInBits() / 8;
        for (unsigned i = 0; i < mBufferBlocks; ++i) {
            ConstantInt * const offset = iBuilder->getSize(i);
            Value * srcOffset = iBuilder->CreateMul(capacity, offset);
            Value * srcPtr = iBuilder->CreateGEP(streamSet, srcOffset);
            Value * destOffset = iBuilder->CreateMul(newCapacity, offset);
            Value * destPtr = iBuilder->CreateGEP(newStreamSet, destOffset);
            iBuilder->CreateMemCpy(destPtr, srcPtr, iBuilder->CreateMul(capacity, vectorWidth), alignment);
            Value * destZeroOffset = iBuilder->CreateAdd(destOffset, capacity);
            Value * destZeroPtr = iBuilder->CreateGEP(newStreamSet, destZeroOffset);
            iBuilder->CreateMemZero(destZeroPtr, diffCapacity, alignment);
        }

        iBuilder->CreateAlignedFree(streamSet);

        iBuilder->CreateRet(newStreamSet);

        iBuilder->restoreIP(ip);
    }

    Value * newStreamSet = iBuilder->CreateCall(expandFunction, {streamSet, capacity, newCapacity});
    iBuilder->CreateStore(newStreamSet, streamSetPtr);
    iBuilder->CreateStore(newCapacity, capacityPtr);

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

Value * ExpandableBuffer::getStreamBlockPtr(Value * self, Value * streamIndex, Value * blockIndex, const bool readOnly) const {
    Value * ptr, * offset;
    std::tie(ptr, offset) = getInternalStreamBuffer(self, streamIndex, blockIndex, readOnly);
    return iBuilder->CreateGEP(ptr, offset);
}

Value * ExpandableBuffer::getStreamPackPtr(Value * self, Value * streamIndex, Value * blockIndex, Value * packIndex, const bool readOnly) const {
    Value * ptr, * offset;
    std::tie(ptr, offset) = getInternalStreamBuffer(self, streamIndex, blockIndex, readOnly);
    return iBuilder->CreateGEP(ptr, {offset, packIndex});
}

Value * ExpandableBuffer::getStreamSetCount(Value * self) const {
    return iBuilder->CreateLoad(iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(0)}));
}

Value * ExpandableBuffer::getBaseAddress(Value * self) const {
    return iBuilder->CreateLoad(iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(1)}));
}

void ExpandableBuffer::releaseBuffer(Value * self) {
    iBuilder->CreateAlignedFree(getBaseAddress(self));
}

Value * ExpandableBuffer::getStreamSetBlockPtr(Value *, Value *) const {
    report_fatal_error("Expandable buffers: getStreamSetBlockPtr is not supported.");
}

Value * ExpandableBuffer::getLinearlyAccessibleItems(Value * self, Value *) const {
    report_fatal_error("Expandable buffers: getLinearlyAccessibleItems is not supported.");
}

// Constructors
SingleBlockBuffer::SingleBlockBuffer(IDISA::IDISA_Builder * b, Type * type)
: StreamSetBuffer(BufferKind::BlockBuffer, b, type, resolveStreamSetType(b, type), 1, 0) {

}

ExternalFileBuffer::ExternalFileBuffer(IDISA::IDISA_Builder * b, Type * type, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::ExternalFileBuffer, b, type, resolveStreamSetType(b, type), 0, AddressSpace) {

}

ExtensibleBuffer::ExtensibleBuffer(IDISA::IDISA_Builder * b, Type * type, size_t bufferBlocks, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::ExtensibleBuffer, b, type, StructType::get(b->getSizeTy(), resolveStreamSetType(b, type)->getPointerTo(), nullptr), bufferBlocks, AddressSpace) {

}

CircularBuffer::CircularBuffer(IDISA::IDISA_Builder * b, Type * type, size_t bufferBlocks, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::CircularBuffer, b, type, resolveStreamSetType(b, type), bufferBlocks, AddressSpace) {

}

CircularCopybackBuffer::CircularCopybackBuffer(IDISA::IDISA_Builder * b, Type * type, size_t bufferBlocks, size_t overflowBlocks, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::CircularCopybackBuffer, b, type, resolveStreamSetType(b, type), bufferBlocks, AddressSpace), mOverflowBlocks(overflowBlocks) {

}

ExpandableBuffer::ExpandableBuffer(IDISA::IDISA_Builder * b, Type * type, size_t bufferBlocks, unsigned AddressSpace)
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

StreamSetBuffer::~StreamSetBuffer() { }

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
