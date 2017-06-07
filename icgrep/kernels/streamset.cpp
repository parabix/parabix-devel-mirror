/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "streamset.h"
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_ostream.h>
#include <kernels/kernel.h>
#include <kernels/kernel_builder.h>
#include <toolchain/toolchain.h>

namespace llvm { class Constant; }
namespace llvm { class Function; }

using namespace parabix;
using namespace llvm;
using namespace IDISA;

ArrayType * resolveStreamSetType(const std::unique_ptr<kernel::KernelBuilder> & b, Type * type);

StructType * resolveExpandableStreamSetType(const std::unique_ptr<kernel::KernelBuilder> & b, Type * type);

void StreamSetBuffer::allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    if (LLVM_LIKELY(mStreamSetBufferPtr == nullptr)) {
        Type * const ty = getType();
        mStreamSetBufferPtr = iBuilder->CreateCacheAlignedAlloca(ty, iBuilder->getSize(mBufferBlocks));
        iBuilder->CreateAlignedStore(Constant::getNullValue(ty), mStreamSetBufferPtr, iBuilder->getCacheAlignment());
    } else {
        report_fatal_error("StreamSetBuffer::allocateBuffer() was called twice on the same stream set");
    }
}

Value * StreamSetBuffer::getStreamBlockPtr(IDISA::IDISA_Builder * const iBuilder, Value * self, Value * streamIndex, Value * blockIndex, const bool /* readOnly */) const {
    if (codegen::EnableAsserts) {
        Value * const count = getStreamSetCount(iBuilder, self);
        Value * const index = iBuilder->CreateZExtOrTrunc(streamIndex, count->getType());
        Value * const cond = iBuilder->CreateICmpULT(index, count);
        iBuilder->CreateAssert(cond, "StreamSetBuffer: out-of-bounds stream access");
    }
    return iBuilder->CreateGEP(getStreamSetBlockPtr(iBuilder, self, blockIndex), {iBuilder->getInt32(0), streamIndex});
}

Value * StreamSetBuffer::getStreamPackPtr(IDISA::IDISA_Builder * const iBuilder, Value * self, Value * streamIndex, Value * blockIndex, Value * packIndex, const bool /* readOnly */) const {
    if (codegen::EnableAsserts) {
        Value * const count = getStreamSetCount(iBuilder, self);
        Value * const index = iBuilder->CreateZExtOrTrunc(streamIndex, count->getType());
        Value * const cond = iBuilder->CreateICmpULT(index, count);
        iBuilder->CreateAssert(cond, "StreamSetBuffer: out-of-bounds stream access");
    }
    return iBuilder->CreateGEP(getStreamSetBlockPtr(iBuilder, self, blockIndex), {iBuilder->getInt32(0), streamIndex, packIndex});
}

void StreamSetBuffer::setBaseAddress(IDISA::IDISA_Builder * const iBuilder, Value * /* self */, Value * /* addr */) const {
    report_fatal_error("setBaseAddress is not supported by this buffer type");
}

Value * StreamSetBuffer::getBufferedSize(IDISA::IDISA_Builder * const iBuilder, Value * /* self */) const {
    report_fatal_error("getBufferedSize is not supported by this buffer type");
}

void StreamSetBuffer::setBufferedSize(IDISA::IDISA_Builder * const iBuilder, Value * /* self */, llvm::Value * /* size */) const {
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

Value * StreamSetBuffer::getStreamSetCount(IDISA::IDISA_Builder * const iBuilder, Value *) const {
    uint64_t count = 1;
    if (isa<ArrayType>(mBaseType)) {
        count = mBaseType->getArrayNumElements();
    }
    return iBuilder->getSize(count);
}

inline Value * StreamSetBuffer::modByBufferBlocks(IDISA::IDISA_Builder * const iBuilder, Value * const offset) const {
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
Value * StreamSetBuffer::getRawItemPointer(IDISA::IDISA_Builder * const iBuilder, Value * self, Value * streamIndex, Value * absolutePosition) const {
    Value * ptr = iBuilder->CreateGEP(getBaseAddress(iBuilder, self), {iBuilder->getInt32(0), streamIndex});
    Value * relativePosition = absolutePosition;
    const auto bw = mBaseType->getArrayElementType()->getScalarSizeInBits();
    if (bw < 8) {
        assert (bw  == 1 || bw == 2 || bw == 4);
        relativePosition = iBuilder->CreateUDiv(relativePosition, ConstantInt::get(relativePosition->getType(), 8 / bw));
        ptr = iBuilder->CreatePointerCast(ptr, iBuilder->getInt8PtrTy());
    } else {
        ptr = iBuilder->CreatePointerCast(ptr, iBuilder->getIntNTy(bw)->getPointerTo());
    }
    return iBuilder->CreateGEP(ptr, relativePosition);
}

Value * StreamSetBuffer::getLinearlyAccessibleItems(IDISA::IDISA_Builder * const iBuilder, Value * fromPosition) const {
    if (isa<ArrayType>(mType) && dyn_cast<ArrayType>(mType)->getNumElements() > 1) {
        Constant * stride = iBuilder->getSize(iBuilder->getStride());
        return iBuilder->CreateSub(stride, iBuilder->CreateURem(fromPosition, stride));
    } else {
        Constant * bufSize = iBuilder->getSize(mBufferBlocks * iBuilder->getStride());
        return iBuilder->CreateSub(bufSize, iBuilder->CreateURem(fromPosition, bufSize));
    }
}

Value * StreamSetBuffer::getLinearlyAccessibleBlocks(IDISA::IDISA_Builder * const iBuilder, Value * fromBlock) const {
    Constant * bufBlocks = iBuilder->getSize(mBufferBlocks);
    return iBuilder->CreateSub(bufBlocks, iBuilder->CreateURem(fromBlock, bufBlocks));
}

Value * StreamSetBuffer::getLinearlyWritableItems(IDISA::IDISA_Builder * const iBuilder, Value * fromPosition) const {
    return getLinearlyAccessibleItems(iBuilder, fromPosition);
}

Value * StreamSetBuffer::getLinearlyWritableBlocks(IDISA::IDISA_Builder * const iBuilder, Value * fromBlock) const {
    return getLinearlyAccessibleBlocks(iBuilder, fromBlock);
}

Value * StreamSetBuffer::getBaseAddress(IDISA::IDISA_Builder * const iBuilder, Value * self) const {
    iBuilder->CreateAssert(self, "StreamSetBuffer base address cannot be 0");
    return self;
}

void StreamSetBuffer::releaseBuffer(IDISA::IDISA_Builder * const /* iBuilder */, Value * /* self */) const {
    /* do nothing: memory is stack allocated */
}

void StreamSetBuffer::createBlockCopy(IDISA::IDISA_Builder * const iBuilder, Value * targetBlockPtr, Value * sourceBlockPtr, Value * blocksToCopy) const {
    Type * i8ptr = iBuilder->getInt8PtrTy();
    unsigned alignment = iBuilder->getBitBlockWidth() / 8;
    uint64_t numStreams = 1;
    if (isa<ArrayType>(mBaseType)) {
        numStreams = mBaseType->getArrayNumElements();
    }
    const auto fieldWidth = mBaseType->getArrayElementType()->getScalarSizeInBits();
    Value * blockCopyBytes = iBuilder->CreateMul(blocksToCopy, iBuilder->getSize(iBuilder->getBitBlockWidth() * numStreams * fieldWidth/8));
    iBuilder->CreateMemMove(iBuilder->CreateBitCast(targetBlockPtr, i8ptr), iBuilder->CreateBitCast(sourceBlockPtr, i8ptr), blockCopyBytes, alignment);
}

void StreamSetBuffer::createBlockAlignedCopy(IDISA::IDISA_Builder * const iBuilder, Value * targetBlockPtr, Value * sourceBlockPtr, Value * itemsToCopy) const {
    Type * const int8PtrTy = iBuilder->getInt8PtrTy();
    const unsigned alignment = iBuilder->getBitBlockWidth() / 8;
    Constant * const blockSize = iBuilder->getSize(iBuilder->getBitBlockWidth());
    uint64_t numStreams = 1;
    if (isa<ArrayType>(mBaseType)) {
        numStreams = mBaseType->getArrayNumElements();
    }
    const auto fieldWidth = mBaseType->getArrayElementType()->getScalarSizeInBits();
    if (numStreams == 1) {
        Value * copyBits = iBuilder->CreateMul(itemsToCopy, iBuilder->getSize(fieldWidth));
        Value * copyBytes = iBuilder->CreateLShr(iBuilder->CreateAdd(copyBits, iBuilder->getSize(7)), iBuilder->getSize(3));
        iBuilder->CreateMemMove(iBuilder->CreateBitCast(targetBlockPtr, int8PtrTy), iBuilder->CreateBitCast(sourceBlockPtr, int8PtrTy), copyBytes, alignment);
    } else {
        Value * blocksToCopy = iBuilder->CreateUDiv(itemsToCopy, blockSize);
        Value * partialItems = iBuilder->CreateURem(itemsToCopy, blockSize);
        Value * partialBlockTargetPtr = iBuilder->CreateGEP(targetBlockPtr, blocksToCopy);
        Value * partialBlockSourcePtr = iBuilder->CreateGEP(sourceBlockPtr, blocksToCopy);
        Value * blockCopyBytes = iBuilder->CreateMul(blocksToCopy, iBuilder->getSize(iBuilder->getBitBlockWidth() * numStreams * fieldWidth/8));
        iBuilder->CreateMemMove(iBuilder->CreateBitCast(targetBlockPtr, int8PtrTy), iBuilder->CreateBitCast(sourceBlockPtr, int8PtrTy), blockCopyBytes, alignment);
        Value * partialCopyBitsPerStream = iBuilder->CreateMul(partialItems, iBuilder->getSize(fieldWidth));
        Value * partialCopyBytesPerStream = iBuilder->CreateLShr(iBuilder->CreateAdd(partialCopyBitsPerStream, iBuilder->getSize(7)), iBuilder->getSize(3));
        for (unsigned strm = 0; strm < numStreams; strm++) {
            Value * strmTargetPtr = iBuilder->CreateGEP(partialBlockTargetPtr, {iBuilder->getInt32(0), iBuilder->getInt32(strm)});
            Value * strmSourcePtr = iBuilder->CreateGEP(partialBlockSourcePtr, {iBuilder->getInt32(0), iBuilder->getInt32(strm)});
            strmTargetPtr = iBuilder->CreateBitCast(strmTargetPtr, int8PtrTy);
            strmSourcePtr = iBuilder->CreateBitCast(strmSourcePtr, int8PtrTy);
            iBuilder->CreateMemMove(strmTargetPtr, strmSourcePtr, partialCopyBytesPerStream, alignment);
        }
    }
}

// Source File Buffer
Value * SourceBuffer::getBufferedSize(IDISA::IDISA_Builder * const iBuilder, Value * self) const {
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(1)});
    return iBuilder->CreateLoad(ptr);
}

void SourceBuffer::setBufferedSize(IDISA::IDISA_Builder * const iBuilder, Value * self, llvm::Value * size) const {
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(1)});
    iBuilder->CreateStore(size, ptr);
}

void SourceBuffer::setBaseAddress(IDISA::IDISA_Builder * const iBuilder, Value * self, Value * addr) const {
    Value * const ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(0)});
    iBuilder->CreateStore(iBuilder->CreatePointerCast(addr, ptr->getType()->getPointerElementType()), ptr);
}

Value * SourceBuffer::getBaseAddress(IDISA::IDISA_Builder * const iBuilder, Value * const self) const {
    iBuilder->CreateAssert(self, "SourceBuffer: instance cannot be null");
    Value * const ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(0)});
    Value * const addr = iBuilder->CreateLoad(ptr);
    iBuilder->CreateAssert(addr, "SourceBuffer: base address cannot be 0");
    return addr;
}

Value * SourceBuffer::getStreamSetBlockPtr(IDISA::IDISA_Builder * const iBuilder, Value * self, Value * blockIndex) const {
    return iBuilder->CreateGEP(getBaseAddress(iBuilder, self), blockIndex);
}

Value * SourceBuffer::getLinearlyAccessibleItems(IDISA::IDISA_Builder * const iBuilder, Value *) const {
    report_fatal_error("External buffers: getLinearlyAccessibleItems is not supported.");
}

// External File Buffer
void ExternalBuffer::allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> &) {
    report_fatal_error("External buffers cannot be allocated.");
}

Value * ExternalBuffer::getStreamSetBlockPtr(IDISA::IDISA_Builder * const iBuilder, Value * self, Value * blockIndex) const {
    return iBuilder->CreateGEP(getBaseAddress(iBuilder, self), blockIndex);
}

Value * ExternalBuffer::getLinearlyAccessibleItems(IDISA::IDISA_Builder * const iBuilder, Value *) const {
    report_fatal_error("External buffers: getLinearlyAccessibleItems is not supported.");
}

// Circular Buffer
Value * CircularBuffer::getStreamSetBlockPtr(IDISA::IDISA_Builder * const iBuilder, Value * const self, Value * const blockIndex) const {
    return iBuilder->CreateGEP(getBaseAddress(iBuilder, self), modByBufferBlocks(iBuilder, blockIndex));
}

Value * CircularBuffer::getRawItemPointer(IDISA::IDISA_Builder * const iBuilder, Value * self, Value * streamIndex, Value * absolutePosition) const {
    Value * ptr = iBuilder->CreateGEP(getBaseAddress(iBuilder, self), {iBuilder->getInt32(0), streamIndex});
    Value * relativePosition = iBuilder->CreateURem(absolutePosition, ConstantInt::get(absolutePosition->getType(), mBufferBlocks * iBuilder->getBitBlockWidth()));
    const auto bw = mBaseType->getArrayElementType()->getScalarSizeInBits();
    if (bw < 8) {
        assert (bw  == 1 || bw == 2 || bw == 4);
        relativePosition = iBuilder->CreateUDiv(relativePosition, ConstantInt::get(relativePosition->getType(), 8 / bw));
        ptr = iBuilder->CreatePointerCast(ptr, iBuilder->getInt8PtrTy());
    } else {
        ptr = iBuilder->CreatePointerCast(ptr, iBuilder->getIntNTy(bw)->getPointerTo());
    }
    return iBuilder->CreateGEP(ptr, relativePosition);
}

// CircularCopybackBuffer Buffer
void CircularCopybackBuffer::allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    mStreamSetBufferPtr = iBuilder->CreateCacheAlignedAlloca(getType(), iBuilder->getSize(mBufferBlocks + mOverflowBlocks));
}

void CircularCopybackBuffer::createCopyBack(IDISA::IDISA_Builder * const iBuilder, Value * self, Value * overFlowItems) const {
    Value * overFlowAreaPtr = iBuilder->CreateGEP(self, iBuilder->getSize(mBufferBlocks));
    createBlockAlignedCopy(iBuilder, self, overFlowAreaPtr, overFlowItems);
}

Value * CircularCopybackBuffer::getLinearlyWritableItems(IDISA::IDISA_Builder * const iBuilder, Value * fromPosition) const {
    return iBuilder->CreateAdd(getLinearlyAccessibleItems(iBuilder, fromPosition), iBuilder->getSize(mOverflowBlocks * iBuilder->getBitBlockWidth()));
}

Value * CircularCopybackBuffer::getLinearlyWritableBlocks(IDISA::IDISA_Builder * const iBuilder, Value * fromBlock) const {
    return iBuilder->CreateAdd(getLinearlyAccessibleBlocks(iBuilder, fromBlock), iBuilder->getSize(mOverflowBlocks));
}

// SwizzledCopybackBuffer Buffer

void SwizzledCopybackBuffer::allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    mStreamSetBufferPtr = iBuilder->CreateCacheAlignedAlloca(getType(), iBuilder->getSize(mBufferBlocks + mOverflowBlocks));
}

void SwizzledCopybackBuffer::createBlockAlignedCopy(IDISA::IDISA_Builder * const iBuilder, Value * targetBlockPtr, Value * sourceBlockPtr, Value * itemsToCopy) const {
    Type * int8PtrTy = iBuilder->getInt8PtrTy();
    DataLayout DL(iBuilder->getModule());
    IntegerType * const intAddrTy = iBuilder->getIntPtrTy(DL);

    Constant * blockSize = iBuilder->getSize(iBuilder->getBitBlockWidth());
    Function * f = iBuilder->GetInsertBlock()->getParent();
    BasicBlock * wholeBlockCopy = BasicBlock::Create(iBuilder->getContext(), "wholeBlockCopy", f, 0);
    BasicBlock * partialBlockCopy = BasicBlock::Create(iBuilder->getContext(), "partialBlockCopy", f, 0);
    BasicBlock * copyDone = BasicBlock::Create(iBuilder->getContext(), "copyDone", f, 0);
    const unsigned numStreams = getType()->getArrayNumElements();
    const unsigned swizzleFactor = iBuilder->getBitBlockWidth()/mFieldWidth;
    const auto elemTy = getType()->getArrayElementType();
    const unsigned fieldWidth = isa<ArrayType>(elemTy) ? elemTy->getArrayNumElements() : 1;
    Value * blocksToCopy = iBuilder->CreateUDiv(itemsToCopy, blockSize);
    Value * partialItems = iBuilder->CreateURem(itemsToCopy, blockSize);
    Value * partialBlockTargetPtr = iBuilder->CreateGEP(targetBlockPtr, blocksToCopy);
    Value * partialBlockSourcePtr = iBuilder->CreateGEP(sourceBlockPtr, blocksToCopy);
    iBuilder->CreateCondBr(iBuilder->CreateICmpUGT(blocksToCopy, iBuilder->getSize(0)), wholeBlockCopy, partialBlockCopy);

    iBuilder->SetInsertPoint(wholeBlockCopy);
    const unsigned alignment = iBuilder->getBitBlockWidth() / 8;
    Value * copyLength = iBuilder->CreateSub(iBuilder->CreatePtrToInt(partialBlockTargetPtr, intAddrTy), iBuilder->CreatePtrToInt(targetBlockPtr, intAddrTy));
    iBuilder->CreateMemMove(iBuilder->CreatePointerCast(targetBlockPtr, int8PtrTy), iBuilder->CreatePointerCast(sourceBlockPtr, int8PtrTy), copyLength, alignment);
    iBuilder->CreateCondBr(iBuilder->CreateICmpUGT(partialItems, iBuilder->getSize(0)), partialBlockCopy, copyDone);
    iBuilder->SetInsertPoint(partialBlockCopy);
    Value * copyBits = iBuilder->CreateMul(itemsToCopy, iBuilder->getSize(fieldWidth * swizzleFactor));
    Value * copyBytes = iBuilder->CreateLShr(iBuilder->CreateAdd(copyBits, iBuilder->getSize(7)), iBuilder->getSize(3));
    for (unsigned strm = 0; strm < numStreams; strm += swizzleFactor) {
        Value * strmTargetPtr = iBuilder->CreateGEP(partialBlockTargetPtr, {iBuilder->getInt32(0), iBuilder->getInt32(strm)});
        Value * strmSourcePtr = iBuilder->CreateGEP(partialBlockSourcePtr, {iBuilder->getInt32(0), iBuilder->getInt32(strm)});
        iBuilder->CreateMemMove(iBuilder->CreatePointerCast(strmTargetPtr, int8PtrTy), iBuilder->CreatePointerCast(strmSourcePtr, int8PtrTy), copyBytes, alignment);
    }
    iBuilder->CreateBr(copyDone);

    iBuilder->SetInsertPoint(copyDone);
}

void SwizzledCopybackBuffer::createCopyBack(IDISA::IDISA_Builder * const iBuilder, Value * self, Value * overFlowItems) const {
    Value * overFlowAreaPtr = iBuilder->CreateGEP(self, iBuilder->getSize(mBufferBlocks));
    createBlockAlignedCopy(iBuilder, self, overFlowAreaPtr, overFlowItems);
}

Value * SwizzledCopybackBuffer::getStreamSetBlockPtr(IDISA::IDISA_Builder * const iBuilder, Value * self, Value * blockIndex) const {
    return iBuilder->CreateGEP(getBaseAddress(iBuilder, self), modByBufferBlocks(iBuilder, blockIndex));
}

Value * SwizzledCopybackBuffer::getLinearlyWritableItems(IDISA::IDISA_Builder * const iBuilder, Value * fromPosition) const {
    return iBuilder->CreateAdd(getLinearlyAccessibleItems(iBuilder, fromPosition), iBuilder->getSize(mOverflowBlocks * iBuilder->getBitBlockWidth()));
}

Value * SwizzledCopybackBuffer::getLinearlyWritableBlocks(IDISA::IDISA_Builder * const iBuilder, Value * fromBlock) const {
    return iBuilder->CreateAdd(getLinearlyAccessibleBlocks(iBuilder, fromBlock), iBuilder->getSize(mOverflowBlocks));
}

// Expandable Buffer

void ExpandableBuffer::allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    mStreamSetBufferPtr = iBuilder->CreateCacheAlignedAlloca(getType());
    Value * const capacityPtr = iBuilder->CreateGEP(mStreamSetBufferPtr, {iBuilder->getInt32(0), iBuilder->getInt32(0)});
    iBuilder->CreateStore(iBuilder->getSize(mInitialCapacity), capacityPtr);
    Type * const bufferType = getType()->getStructElementType(1)->getPointerElementType();
    Constant * const bufferWidth = ConstantExpr::getIntegerCast(ConstantExpr::getSizeOf(bufferType), iBuilder->getSizeTy(), false);
    Constant * const size = ConstantExpr::getMul(iBuilder->getSize(mBufferBlocks * mInitialCapacity), bufferWidth);
    const auto alignment = std::max(iBuilder->getCacheAlignment(), iBuilder->getBitBlockWidth() / 8);
    Value * const ptr = iBuilder->CreateAlignedMalloc(size, alignment);
    iBuilder->CreateMemZero(ptr, size, bufferType->getPrimitiveSizeInBits() / 8);
    Value * const streamSetPtr = iBuilder->CreateGEP(mStreamSetBufferPtr, {iBuilder->getInt32(0), iBuilder->getInt32(1)});
    iBuilder->CreateStore(iBuilder->CreatePointerCast(ptr, bufferType->getPointerTo()), streamSetPtr);
}

std::pair<Value *, Value *> ExpandableBuffer::getInternalStreamBuffer(IDISA::IDISA_Builder * const iBuilder, Value * self, Value * streamIndex, Value * blockIndex, const bool readOnly) const {

    // ENTRY
    Value * const capacityPtr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(0)});
    Value * const capacity = iBuilder->CreateLoad(capacityPtr);
    Value * const streamSetPtr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(1)});
    Value * const streamSet = iBuilder->CreateLoad(streamSetPtr);
    blockIndex = modByBufferBlocks(iBuilder, blockIndex);

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
        const auto memAlign = std::max(iBuilder->getCacheAlignment(), iBuilder->getBitBlockWidth() / 8);

        Value * newStreamSet = iBuilder->CreatePointerCast(iBuilder->CreateAlignedMalloc(iBuilder->CreateMul(size, vectorWidth), memAlign), elementType->getPointerTo());
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

        iBuilder->CreateFree(streamSet);

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

Value * ExpandableBuffer::getStreamBlockPtr(IDISA::IDISA_Builder * const iBuilder, Value * self, Value * streamIndex, Value * blockIndex, const bool readOnly) const {
    Value * ptr, * offset;
    std::tie(ptr, offset) = getInternalStreamBuffer(iBuilder, self, streamIndex, blockIndex, readOnly);
    return iBuilder->CreateGEP(ptr, offset);
}

Value * ExpandableBuffer::getStreamPackPtr(IDISA::IDISA_Builder * const iBuilder, Value * self, Value * streamIndex, Value * blockIndex, Value * packIndex, const bool readOnly) const {
    Value * ptr, * offset;
    std::tie(ptr, offset) = getInternalStreamBuffer(iBuilder, self, streamIndex, blockIndex, readOnly);
    return iBuilder->CreateGEP(ptr, {offset, packIndex});
}

Value * ExpandableBuffer::getStreamSetCount(IDISA::IDISA_Builder * const iBuilder, Value * self) const {
    return iBuilder->CreateLoad(iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(0)}));
}

Value * ExpandableBuffer::getBaseAddress(IDISA::IDISA_Builder * const iBuilder, Value * self) const {
    iBuilder->CreateAssert(self, "ExpandableBuffer: instance cannot be null");
    Value * const baseAddr = iBuilder->CreateLoad(iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(1)}));
    iBuilder->CreateAssert(self, "ExpandableBuffer: base address cannot be 0");
    return baseAddr;
}

void ExpandableBuffer::releaseBuffer(IDISA::IDISA_Builder * const iBuilder, Value * self) const {
    iBuilder->CreateFree(getBaseAddress(iBuilder, self));
}

Value * ExpandableBuffer::getStreamSetBlockPtr(IDISA::IDISA_Builder * const iBuilder, Value *, Value *) const {
    report_fatal_error("Expandable buffers: getStreamSetBlockPtr is not supported.");
}

Value * ExpandableBuffer::getLinearlyAccessibleItems(IDISA::IDISA_Builder * const iBuilder, Value *) const {
    report_fatal_error("Expandable buffers: getLinearlyAccessibleItems is not supported.");
}

SourceBuffer::SourceBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, Type * type, unsigned MemoryAddressSpace, unsigned StructAddressSpace)
: StreamSetBuffer(BufferKind::SourceBuffer, type, StructType::get(resolveStreamSetType(b, type)->getPointerTo(MemoryAddressSpace), b->getSizeTy(), nullptr), 0, StructAddressSpace) {
    mUniqueID = "B";
    if (MemoryAddressSpace != 0 || StructAddressSpace != 0) {
        mUniqueID += "@" + std::to_string(MemoryAddressSpace) + ":" + std::to_string(StructAddressSpace);
    }
}

ExternalBuffer::ExternalBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, Type * type, llvm::Value * addr, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::ExternalBuffer, type, resolveStreamSetType(b, type), 0, AddressSpace) {
    mUniqueID = "E";
    if (AddressSpace > 0) mUniqueID += "@" + std::to_string(AddressSpace);
    mStreamSetBufferPtr = b->CreatePointerBitCastOrAddrSpaceCast(addr, getPointerType());
}

CircularBuffer::CircularBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, Type * type, size_t bufferBlocks, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::CircularBuffer, type, resolveStreamSetType(b, type), bufferBlocks, AddressSpace) {
    mUniqueID = "C" + std::to_string(bufferBlocks);
    if (AddressSpace > 0) mUniqueID += "@" + std::to_string(AddressSpace);
}

CircularBuffer::CircularBuffer(const BufferKind k, const std::unique_ptr<kernel::KernelBuilder> & b, Type * type, size_t bufferBlocks, unsigned AddressSpace)
: StreamSetBuffer(k, type, resolveStreamSetType(b, type), bufferBlocks, AddressSpace) {

}

CircularCopybackBuffer::CircularCopybackBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, Type * type, size_t bufferBlocks, size_t overflowBlocks, unsigned AddressSpace)
: CircularBuffer(BufferKind::CircularCopybackBuffer, b, type, bufferBlocks, AddressSpace)
, mOverflowBlocks(overflowBlocks) {
    mUniqueID = "CC" + std::to_string(bufferBlocks);
    if (mOverflowBlocks != 1) mUniqueID += "_" + std::to_string(mOverflowBlocks);
    if (AddressSpace > 0) mUniqueID += "@" + std::to_string(AddressSpace);
}

ExpandableBuffer::ExpandableBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, Type * type, size_t bufferBlocks, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::ExpandableBuffer, type, resolveExpandableStreamSetType(b, type), bufferBlocks, AddressSpace)
, mInitialCapacity(type->getArrayNumElements()) {
    mUniqueID = "XP" + std::to_string(bufferBlocks);
    if (AddressSpace > 0) mUniqueID += "@" + std::to_string(AddressSpace);
}

SwizzledCopybackBuffer::SwizzledCopybackBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, Type * type, size_t bufferBlocks, size_t overflowBlocks, unsigned fieldwidth, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::SwizzledCopybackBuffer, type, resolveStreamSetType(b, type), bufferBlocks, AddressSpace), mOverflowBlocks(overflowBlocks), mFieldWidth(fieldwidth) {
    mUniqueID = "SW" + std::to_string(fieldwidth) + ":" + std::to_string(bufferBlocks);
    if (mOverflowBlocks != 1) {
        mUniqueID += "_" + std::to_string(mOverflowBlocks);
    }
    if (AddressSpace > 0) {
        mUniqueID += "@" + std::to_string(AddressSpace);
    }
}

inline StreamSetBuffer::StreamSetBuffer(BufferKind k, Type * baseType, Type * resolvedType, unsigned BufferBlocks, unsigned AddressSpace)
: mBufferKind(k)
, mType(resolvedType)
, mBufferBlocks(BufferBlocks)
, mAddressSpace(AddressSpace)
, mStreamSetBufferPtr(nullptr)
, mBaseType(baseType)
, mProducer(nullptr) {

}

StreamSetBuffer::~StreamSetBuffer() { }

// Helper routines
ArrayType * resolveStreamSetType(const std::unique_ptr<kernel::KernelBuilder> & b, Type * type) {
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

StructType * resolveExpandableStreamSetType(const std::unique_ptr<kernel::KernelBuilder> & b, Type * type) {
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
