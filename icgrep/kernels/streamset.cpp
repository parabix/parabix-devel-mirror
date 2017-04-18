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
#include <kernels/kernel.h>
#include <kernels/toolchain.h>

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

void StreamSetBuffer::setBaseAddress(Value * /* self */, Value * /* addr */) const {
    report_fatal_error("setBaseAddress is not supported by this buffer type");
}

Value * StreamSetBuffer::getBufferedSize(Value * /* self */) const {
    report_fatal_error("getBufferedSize is not supported by this buffer type");
}

void StreamSetBuffer::setBufferedSize(Value * /* self */, llvm::Value * /* size */) const {
    report_fatal_error("setBufferedSize is not supported by this buffer type");
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
    if (!isa<ConstantInt>(streamIndex) || !cast<ConstantInt>(streamIndex)->isZero()) {
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

void StreamSetBuffer::reserveBytes(Value * self, llvm::Value *requested) const {
    report_fatal_error("reserve() can only be used with ExtensibleBuffers");
}

Value * StreamSetBuffer::getBaseAddress(Value * self) const {
    return self;
}

void StreamSetBuffer::releaseBuffer(Value * /* self */) const {
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

// Source File Buffer
Value * SourceFileBuffer::getBufferedSize(Value * self) const {
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(1)});
    return iBuilder->CreateLoad(ptr);
}

void SourceFileBuffer::setBufferedSize(Value * self, llvm::Value * size) const {
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(1)});
    iBuilder->CreateStore(size, ptr);
}

void SourceFileBuffer::setBaseAddress(Value * self, Value * addr) const {
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(0)});
    iBuilder->CreateStore(addr, ptr);
}

Value * SourceFileBuffer::getBaseAddress(Value * const self) const {
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(0)});
    return iBuilder->CreateLoad(ptr);
}

Value * SourceFileBuffer::getStreamSetBlockPtr(Value * self, Value * blockIndex) const {
    return iBuilder->CreateGEP(self, blockIndex);
}

Value * SourceFileBuffer::getLinearlyAccessibleItems(Value * self, Value *) const {
    report_fatal_error("External buffers: getLinearlyAccessibleItems is not supported.");
}

// ExtensibleBuffer
Value * ExtensibleBuffer::getLinearlyAccessibleItems(Value * self, Value * fromPosition) const {
    Value * capacityPtr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(0)});
    Value * capacity = iBuilder->CreateLoad(capacityPtr);
    return iBuilder->CreateSub(capacity, fromPosition);
}

Value * ExtensibleBuffer::roundUpToPageSize(Value * const value) const {
    const auto pageSize = getpagesize();
    assert ((pageSize & (pageSize - 1)) == 0);
    Constant * const pageMask = ConstantInt::get(value->getType(), pageSize - 1);
    return iBuilder->CreateAnd(iBuilder->CreateAdd(value, pageMask), iBuilder->CreateNot(pageMask));
}

void ExtensibleBuffer::allocateBuffer() {
    Type * ty = getType();
    Value * instance = iBuilder->CreateCacheAlignedAlloca(ty);
    Value * const capacityPtr = iBuilder->CreateGEP(instance, {iBuilder->getInt32(0), iBuilder->getInt32(0)});

    Type * const elementType = ty->getStructElementType(1)->getPointerElementType();
    Constant * size = ConstantExpr::getSizeOf(elementType);
    size = ConstantExpr::getMul(size, iBuilder->getSize(mBufferBlocks));
    size = ConstantExpr::getIntegerCast(size, iBuilder->getSizeTy(), false);
    Value * const initialSize = roundUpToPageSize(size);

    iBuilder->CreateStore(initialSize, capacityPtr);
    Value * addr = iBuilder->CreateAnonymousMMap(size);
    Value * const addrPtr = iBuilder->CreateGEP(instance, {iBuilder->getInt32(0), iBuilder->getInt32(1)});
    addr = iBuilder->CreatePointerCast(addr, addrPtr->getType()->getPointerElementType());
    iBuilder->CreateStore(addr, addrPtr);
    Value * const bufferSizePtr = iBuilder->CreateGEP(instance, {iBuilder->getInt32(0), iBuilder->getInt32(2)});
    iBuilder->CreateStore(ConstantInt::getNullValue(bufferSizePtr->getType()->getPointerElementType()), bufferSizePtr);
    mStreamSetBufferPtr = instance;
}

Value * ExtensibleBuffer::getStreamSetBlockPtr(Value * self, Value * blockIndex) const {
    return iBuilder->CreateGEP(self, blockIndex);
}

void ExtensibleBuffer::reserveBytes(Value * const self, llvm::Value * const requiredSize) const {

    Value * const capacityPtr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(0)});
    Value * const currentSize = iBuilder->CreateLoad(capacityPtr);
    BasicBlock * const entry = iBuilder->GetInsertBlock();
    Function * const parent = entry->getParent();
    IntegerType * const sizeTy = iBuilder->getSizeTy();
    ConstantInt * const zero = iBuilder->getInt32(0);
    ConstantInt * const one = iBuilder->getInt32(1);

    BasicBlock * const expand = BasicBlock::Create(iBuilder->getContext(), "expand", parent);
    BasicBlock * const resume = BasicBlock::Create(iBuilder->getContext(), "resume", parent);

    Value * noExpansionNeeded = iBuilder->CreateICmpULT(requiredSize, currentSize);

    kernel::KernelBuilder * const kernel = getProducer();
    auto consumers = kernel->getStreamOutputs();
    if (consumers.empty()) {
        iBuilder->CreateLikelyCondBr(noExpansionNeeded, resume, expand);
    } else { // we cannot risk expanding this buffer until all of the consumers have finished reading the data

        ConstantInt * const size0 = iBuilder->getSize(0);
        Value * const segNo = kernel->acquireLogicalSegmentNo();
        const auto n = consumers.size();

        BasicBlock * load[n + 1];
        BasicBlock * wait[n];
        for (unsigned i = 0; i < n; ++i) {
            load[i] = BasicBlock::Create(iBuilder->getContext(), consumers[i].name + "Load", parent);
            wait[i] = BasicBlock::Create(iBuilder->getContext(), consumers[i].name + "Wait", parent);
        }
        load[n] = expand;
        iBuilder->CreateLikelyCondBr(noExpansionNeeded, resume, load[0]);

        for (unsigned i = 0; i < n; ++i) {

            iBuilder->SetInsertPoint(load[i]);
            Value * const outputConsumers = kernel->getConsumerState(consumers[i].name);

            Value * const consumerCount = iBuilder->CreateLoad(iBuilder->CreateGEP(outputConsumers, {zero, zero}));
            Value * const consumerPtr = iBuilder->CreateLoad(iBuilder->CreateGEP(outputConsumers, {zero, one}));
            Value * const noConsumers = iBuilder->CreateICmpEQ(consumerCount, size0);
            iBuilder->CreateUnlikelyCondBr(noConsumers, load[i + 1], wait[i]);

            iBuilder->SetInsertPoint(wait[i]);
            PHINode * const consumerPhi = iBuilder->CreatePHI(sizeTy, 2);
            consumerPhi->addIncoming(size0, load[i]);

            Value * const conSegPtr = iBuilder->CreateLoad(iBuilder->CreateGEP(consumerPtr, consumerPhi));
            Value * const processedSegmentCount = iBuilder->CreateAtomicLoadAcquire(conSegPtr);
            Value * const ready = iBuilder->CreateICmpEQ(segNo, processedSegmentCount);
            assert (ready->getType() == iBuilder->getInt1Ty());
            Value * const nextConsumerIdx = iBuilder->CreateAdd(consumerPhi, iBuilder->CreateZExt(ready, sizeTy));
            consumerPhi->addIncoming(nextConsumerIdx, wait[i]);
            Value * const next = iBuilder->CreateICmpEQ(nextConsumerIdx, consumerCount);
            iBuilder->CreateCondBr(next, load[i + 1], wait[i]);

        }
        expand->moveAfter(wait[n - 1]);
        resume->moveAfter(expand);
    }
    iBuilder->SetInsertPoint(expand);
    Value * const reservedSize = roundUpToPageSize(iBuilder->CreateShl(requiredSize, 1));
    Value * const baseAddrPtr = iBuilder->CreateGEP(self, {zero, one});

    Value * const baseAddr = iBuilder->CreateLoad(baseAddrPtr);
    Value * newAddr = iBuilder->CreateMRemap(baseAddr, currentSize, reservedSize);
    newAddr = iBuilder->CreatePointerCast(newAddr, baseAddr->getType());
    iBuilder->CreateStore(newAddr, baseAddrPtr);
    iBuilder->CreateStore(reservedSize, capacityPtr);
    iBuilder->CreateBr(resume);
    iBuilder->SetInsertPoint(resume);
}

Value * ExtensibleBuffer::getBufferedSize(Value * self) const {
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(2)});
    return iBuilder->CreateLoad(ptr);
}

void ExtensibleBuffer::setBufferedSize(Value * self, llvm::Value * size) const {
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(2)});
    iBuilder->CreateStore(size, ptr);
}

Value * ExtensibleBuffer::getBaseAddress(Value * const self) const {
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(1)});
    return iBuilder->CreateLoad(ptr);
}

void ExtensibleBuffer::releaseBuffer(Value * self) const {
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
    mUniqueID = "SW" + std::to_string(fieldwidth) + ":" + std::to_string(bufferBlocks);
    if (mOverflowBlocks != 1) {
        mUniqueID += "_" + std::to_string(mOverflowBlocks);
    }
    if (AddressSpace > 0) {
        mUniqueID += "@" + std::to_string(AddressSpace);
    }
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

void ExpandableBuffer::releaseBuffer(Value * self) const {
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
    mUniqueID = "S";

}

ExternalFileBuffer::ExternalFileBuffer(IDISA::IDISA_Builder * b, Type * type, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::ExternalFileBuffer, b, type, resolveStreamSetType(b, type), 0, AddressSpace) {
    mUniqueID = "E";
    if (AddressSpace > 0) mUniqueID += "@" + std::to_string(AddressSpace);
}

SourceFileBuffer::SourceFileBuffer(IDISA::IDISA_Builder * b, Type * type, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::SourceFileBuffer, b, type, StructType::get(resolveStreamSetType(b, type)->getPointerTo(), b->getSizeTy(), nullptr), 0, AddressSpace) {

}

ExtensibleBuffer::ExtensibleBuffer(IDISA::IDISA_Builder * b, Type * type, size_t bufferBlocks, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::ExtensibleBuffer, b, type, StructType::get(b->getSizeTy(), resolveStreamSetType(b, type)->getPointerTo(), b->getSizeTy(), nullptr), bufferBlocks, AddressSpace) {
    mUniqueID = "XT" + std::to_string(bufferBlocks);
    if (AddressSpace > 0) mUniqueID += "@" + std::to_string(AddressSpace);
}

CircularBuffer::CircularBuffer(IDISA::IDISA_Builder * b, Type * type, size_t bufferBlocks, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::CircularBuffer, b, type, resolveStreamSetType(b, type), bufferBlocks, AddressSpace) {
    mUniqueID = "C" + std::to_string(bufferBlocks);
    if (AddressSpace > 0) mUniqueID += "@" + std::to_string(AddressSpace);

}

CircularCopybackBuffer::CircularCopybackBuffer(IDISA::IDISA_Builder * b, Type * type, size_t bufferBlocks, size_t overflowBlocks, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::CircularCopybackBuffer, b, type, resolveStreamSetType(b, type), bufferBlocks, AddressSpace), mOverflowBlocks(overflowBlocks) {
    mUniqueID = "CC" + std::to_string(bufferBlocks);
    if (mOverflowBlocks != 1) mUniqueID += "_" + std::to_string(mOverflowBlocks);
    if (AddressSpace > 0) mUniqueID += "@" + std::to_string(AddressSpace);
}

ExpandableBuffer::ExpandableBuffer(IDISA::IDISA_Builder * b, Type * type, size_t bufferBlocks, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::ExpandableBuffer, b, type, resolveExpandableStreamSetType(b, type), bufferBlocks, AddressSpace)
, mInitialCapacity(type->getArrayNumElements()) {
    mUniqueID = "XP" + std::to_string(bufferBlocks);
    if (AddressSpace > 0) mUniqueID += "@" + std::to_string(AddressSpace);
}

inline StreamSetBuffer::StreamSetBuffer(BufferKind k, IDISA::IDISA_Builder * b, Type * baseType, Type * resolvedType, unsigned blocks, unsigned AddressSpace)
: mBufferKind(k)
, iBuilder(b)
, mType(resolvedType)
, mBufferBlocks(blocks)
, mAddressSpace(AddressSpace)
, mStreamSetBufferPtr(nullptr)
, mBaseType(baseType)
, mProducer(nullptr) {

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
