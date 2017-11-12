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
#include <llvm/Support/Debug.h>
#include <llvm/Support/Format.h>

namespace llvm { class Constant; }
namespace llvm { class Function; }

using namespace parabix;
using namespace llvm;
using namespace IDISA;


Type * StreamSetBuffer::getStreamSetBlockType() const { return mType;}

ArrayType * resolveStreamSetType(const std::unique_ptr<kernel::KernelBuilder> & b, Type * type);

StructType * resolveExpandableStreamSetType(const std::unique_ptr<kernel::KernelBuilder> & b, Type * type);

void StreamSetBuffer::allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    assert (mBufferBlocks > 0);
    if (LLVM_LIKELY(mStreamSetBufferPtr == nullptr)) {
        Type * const ty = getType();
        if (mAddressSpace == 0) {
            Constant * size = ConstantExpr::getSizeOf(ty);
            size = ConstantExpr::getMul(size, ConstantInt::get(size->getType(), mBufferBlocks));
            mStreamSetBufferPtr = iBuilder->CreatePointerCast(iBuilder->CreateCacheAlignedMalloc(size), ty->getPointerTo());
        } else {
            mStreamSetBufferPtr = iBuilder->CreateCacheAlignedAlloca(ty, iBuilder->getSize(mBufferBlocks));
        }
        iBuilder->CreateAlignedStore(Constant::getNullValue(ty), mStreamSetBufferPtr, iBuilder->getCacheAlignment());
    } else {
        report_fatal_error("StreamSetBuffer::allocateBuffer() was called twice on the same stream set");
    }
}

void StreamSetBuffer::releaseBuffer(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) const {
    if (mAddressSpace == 0) {
        iBuilder->CreateFree(mStreamSetBufferPtr);
    }
}

Value * StreamSetBuffer::getStreamBlockPtr(IDISA::IDISA_Builder * const iBuilder, Value * self, Value * addr, Value * streamIndex, const bool /* readOnly */) const {
    if (codegen::DebugOptionIsSet(codegen::EnableAsserts)) {
        Value * const count = getStreamSetCount(iBuilder, self);
        Value * const index = iBuilder->CreateZExtOrTrunc(streamIndex, count->getType());
        Value * const cond = iBuilder->CreateICmpULT(index, count);
        iBuilder->CreateAssert(cond, "StreamSetBuffer: out-of-bounds stream access");
    }
    return iBuilder->CreateGEP(addr, {iBuilder->getInt32(0), streamIndex});
}

Value * StreamSetBuffer::getStreamPackPtr(IDISA::IDISA_Builder * const iBuilder, Value * self, Value * addr, Value * streamIndex, Value * packIndex, const bool /* readOnly */) const {
    if (codegen::DebugOptionIsSet(codegen::EnableAsserts)) {
        Value * const count = getStreamSetCount(iBuilder, self);
        Value * const index = iBuilder->CreateZExtOrTrunc(streamIndex, count->getType());
        Value * const cond = iBuilder->CreateICmpULT(index, count);
        iBuilder->CreateAssert(cond, "StreamSetBuffer: out-of-bounds stream access");
    }
    return iBuilder->CreateGEP(addr, {iBuilder->getInt32(0), streamIndex, packIndex});
}

void StreamSetBuffer::setBaseAddress(IDISA::IDISA_Builder * const /* iBuilder */, Value * /* self */, Value * /* addr */) const {
    report_fatal_error("setBaseAddress is not supported by this buffer type");
}

Value * StreamSetBuffer::getBufferedSize(IDISA::IDISA_Builder * const /* iBuilder */, Value * /* self */) const {
    report_fatal_error("getBufferedSize is not supported by this buffer type");
}

void StreamSetBuffer::setBufferedSize(IDISA::IDISA_Builder * const /* iBuilder */, Value * /* self */, llvm::Value * /* size */) const {
    report_fatal_error("setBufferedSize is not supported by this buffer type");
}

Value * StreamSetBuffer::getCapacity(IDISA::IDISA_Builder * const iBuilder, Value * self) const {
    return getBufferedSize(iBuilder, self);
}

void StreamSetBuffer::setCapacity(IDISA::IDISA_Builder * const /* iBuilder */, Value * /* self */, llvm::Value * /* c */) const {
    report_fatal_error("setCapacity is not supported by this buffer type");
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
    size_t count = 1;
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
Value * StreamSetBuffer::getRawItemPointer(IDISA::IDISA_Builder * const iBuilder, Value * self, Value * absolutePosition) const {
    Value * ptr = getBaseAddress(iBuilder, self);
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

Value * StreamSetBuffer::getLinearlyAccessibleItems(IDISA::IDISA_Builder * const b, Value * self, Value * fromPosition, Value * availItems, bool reverse) const {
    Constant * bufSize = ConstantInt::get(fromPosition->getType(), mBufferBlocks * b->getStride());
    Value * itemsFromBase = b->CreateURem(fromPosition, bufSize);
    if (reverse) {
        Value * bufAvail = b->CreateSelect(b->CreateICmpEQ(itemsFromBase, b->getSize(0)), bufSize, itemsFromBase);
        return b->CreateSelect(b->CreateICmpULT(bufAvail, availItems), bufAvail, availItems);
    } else {
        Value * linearSpace = b->CreateSub(bufSize, itemsFromBase, "linearSpace");
        return b->CreateSelect(b->CreateICmpULT(availItems, linearSpace), availItems, linearSpace);
    }
}

Value * StreamSetBuffer::getLinearlyWritableItems(IDISA::IDISA_Builder * const iBuilder, Value * self, Value * fromPosition, bool reverse) const {
    Constant * bufSize = ConstantInt::get(fromPosition->getType(), mBufferBlocks * iBuilder->getStride());
    Value * bufRem = iBuilder->CreateURem(fromPosition, bufSize);
    if (reverse) {
        return iBuilder->CreateSelect(iBuilder->CreateICmpEQ(bufRem, iBuilder->getSize(0)), bufSize, bufRem);
    }
    return iBuilder->CreateSub(bufSize, bufRem, "linearSpace");
}

Value * StreamSetBuffer::getBaseAddress(IDISA::IDISA_Builder * const iBuilder, Value * self) const {
    iBuilder->CreateAssert(self, "StreamSetBuffer base address cannot be 0");
    return self;
}

void StreamSetBuffer::createBlockCopy(IDISA::IDISA_Builder * const iBuilder, Value * targetBlockPtr, Value * sourceBlockPtr, Value * blocksToCopy) const {
    Type * i8ptr = iBuilder->getInt8PtrTy();
    unsigned alignment = iBuilder->getBitBlockWidth() / 8;
    size_t numStreams = 1;
    if (isa<ArrayType>(mBaseType)) {
        numStreams = mBaseType->getArrayNumElements();
    }
    const auto fieldWidth = mBaseType->getArrayElementType()->getScalarSizeInBits();
    Value * blockCopyBytes = iBuilder->CreateMul(blocksToCopy, iBuilder->getSize(iBuilder->getBitBlockWidth() * numStreams * fieldWidth/8));
    iBuilder->CreateMemMove(iBuilder->CreateBitCast(targetBlockPtr, i8ptr), iBuilder->CreateBitCast(sourceBlockPtr, i8ptr), blockCopyBytes, alignment);
}

Value * StreamSetBuffer::copy(IDISA::IDISA_Builder * const b, Value * self, Value * const target, Value * const source, Value * itemsToCopy, const unsigned alignment) const {
    Type * ty = getBaseType();
    if (LLVM_LIKELY(isa<ArrayType>(ty))) {
        ty = ty->getArrayElementType();
    }
    if (LLVM_LIKELY(isa<VectorType>(ty))) {
        ty = ty->getVectorElementType();
    }
    const auto itemWidth = ty->getScalarSizeInBits();
    assert (itemWidth > 0);
    Value * const m = b->CreateMul(getStreamSetCount(b, self), b->getSize(itemWidth / 8));
    Value * const bytesToCopy = b->CreateMul(itemsToCopy, m);

    // TODO: lz4d s2p reads misaligned data into the source stream. The stream binding should indicate alignment.
    // alignment ? alignment : b->getBitBlockWidth() / 8
    b->CreateMemCpy(target, source, bytesToCopy, 1);
    return bytesToCopy;
}

void StreamSetBuffer::createBlockAlignedCopy(IDISA::IDISA_Builder * const iBuilder, Value * targetBlockPtr, Value * sourceBlockPtr, Value * itemsToCopy) const {
    const unsigned alignment = iBuilder->getBitBlockWidth() / 8;
    Constant * const blockSize = ConstantInt::get(itemsToCopy->getType(), iBuilder->getBitBlockWidth());
    size_t numStreams = 1;
    if (isa<ArrayType>(mBaseType)) {
        numStreams = mBaseType->getArrayNumElements();
    }
    const auto fieldWidth = mBaseType->getArrayElementType()->getScalarSizeInBits();
    if (numStreams == 1) {
        Value * copyBits = iBuilder->CreateMul(itemsToCopy, iBuilder->getSize(fieldWidth));
        Value * copyBytes = iBuilder->CreateLShr(iBuilder->CreateAdd(copyBits, iBuilder->getSize(7)), iBuilder->getSize(3));
        iBuilder->CreateMemMove(targetBlockPtr, sourceBlockPtr, copyBytes, alignment);
    } else {
        Value * blocksToCopy = iBuilder->CreateUDiv(itemsToCopy, blockSize);
        Value * partialItems = iBuilder->CreateURem(itemsToCopy, blockSize);
        Value * partialBlockTargetPtr = iBuilder->CreateGEP(targetBlockPtr, blocksToCopy);
        Value * partialBlockSourcePtr = iBuilder->CreateGEP(sourceBlockPtr, blocksToCopy);
        Value * blockCopyBytes = iBuilder->CreateMul(blocksToCopy, iBuilder->getSize(iBuilder->getBitBlockWidth() * numStreams * fieldWidth/8));
        iBuilder->CreateMemMove(targetBlockPtr, sourceBlockPtr, blockCopyBytes, alignment);
        Value * partialCopyBitsPerStream = iBuilder->CreateMul(partialItems, iBuilder->getSize(fieldWidth));
        Value * partialCopyBytesPerStream = iBuilder->CreateLShr(iBuilder->CreateAdd(partialCopyBitsPerStream, iBuilder->getSize(7)), iBuilder->getSize(3));
        for (unsigned i = 0; i < numStreams; i++) {
            Value * strmTargetPtr = iBuilder->CreateGEP(partialBlockTargetPtr, {iBuilder->getInt32(0), iBuilder->getInt32(i)});
            Value * strmSourcePtr = iBuilder->CreateGEP(partialBlockSourcePtr, {iBuilder->getInt32(0), iBuilder->getInt32(i)});
            iBuilder->CreateMemMove(strmTargetPtr, strmSourcePtr, partialCopyBytesPerStream, alignment);
        }
    }
}

void StreamSetBuffer::genCopyBackLogic(IDISA::IDISA_Builder * const b, Value * handle, Value * priorProduced, Value * newProduced, const std::string Name) const {
    report_fatal_error("Copy back not supported for this buffer type:" + Name);
}

// Source File Buffer

Type * SourceBuffer::getStreamSetBlockType() const {
    return cast<PointerType>(mType->getStructElementType(int(SourceBuffer::Field::BaseAddress)))->getElementType();
}


Value * SourceBuffer::getBufferedSize(IDISA::IDISA_Builder * const iBuilder, Value * self) const {
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(int(SourceBuffer::Field::BufferedSize))});
    return iBuilder->CreateLoad(ptr);
}

void SourceBuffer::setBufferedSize(IDISA::IDISA_Builder * const iBuilder, Value * self, llvm::Value * size) const {
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(int(SourceBuffer::Field::BufferedSize))});
    iBuilder->CreateStore(size, ptr);
}

Value * SourceBuffer::getCapacity(IDISA::IDISA_Builder * const iBuilder, Value * self) const {
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(int(SourceBuffer::Field::Capacity))});
    return iBuilder->CreateLoad(ptr);
}

void SourceBuffer::setCapacity(IDISA::IDISA_Builder * const iBuilder, Value * self, llvm::Value * c) const {
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(int(SourceBuffer::Field::Capacity))});
    iBuilder->CreateStore(c, ptr);
}

void SourceBuffer::setBaseAddress(IDISA::IDISA_Builder * const iBuilder, Value * self, Value * addr) const {
    Value * const ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(int(SourceBuffer::Field::BaseAddress))});
    iBuilder->CreateStore(iBuilder->CreatePointerCast(addr, ptr->getType()->getPointerElementType()), ptr);
}

Value * SourceBuffer::getBaseAddress(IDISA::IDISA_Builder * const iBuilder, Value * const self) const {
    iBuilder->CreateAssert(self, "SourceBuffer: instance cannot be null");
    Value * const ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(int(SourceBuffer::Field::BaseAddress))});
    Value * const addr = iBuilder->CreateLoad(ptr);
    iBuilder->CreateAssert(addr, "SourceBuffer: base address cannot be 0");
    return addr;
}

Value * SourceBuffer::getBlockAddress(IDISA::IDISA_Builder * const iBuilder, Value * self, Value * blockIndex) const {
    return iBuilder->CreateGEP(getBaseAddress(iBuilder, self), blockIndex );
}

Value * SourceBuffer::getLinearlyAccessibleItems(IDISA::IDISA_Builder * const iBuilder, Value * self, Value * fromPosition, Value * availItems, bool reverse) const {
    if (reverse) report_fatal_error("SourceBuffer cannot be accessed in reverse");
    Value * maxAvail = iBuilder->CreateSub(getBufferedSize(iBuilder, self), fromPosition);
    return iBuilder->CreateSelect(iBuilder->CreateICmpULT(availItems, maxAvail), availItems, maxAvail);
}

Value * SourceBuffer::getLinearlyWritableItems(IDISA::IDISA_Builder * const iBuilder, Value * self, Value * fromPosition, bool reverse) const {
    report_fatal_error("SourceBuffers cannot be written");
}

void SourceBuffer::allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    if (LLVM_LIKELY(mStreamSetBufferPtr == nullptr)) {
        Type * const ty = getType();
        mStreamSetBufferPtr = iBuilder->CreateCacheAlignedAlloca(ty, iBuilder->getSize(mBufferBlocks));
        iBuilder->CreateAlignedStore(Constant::getNullValue(ty), mStreamSetBufferPtr, iBuilder->getCacheAlignment());
    } else {
        report_fatal_error("StreamSetBuffer::allocateBuffer() was called twice on the same stream set");
    }
}

void SourceBuffer::releaseBuffer(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) const {

}

// External File Buffer
void ExternalBuffer::allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> &) {
    report_fatal_error("External buffers cannot be allocated.");
}

void ExternalBuffer::releaseBuffer(const std::unique_ptr<kernel::KernelBuilder> &) const {

}

Value * ExternalBuffer::getBlockAddress(IDISA::IDISA_Builder * const iBuilder, Value * self, Value * blockIndex) const {
    return iBuilder->CreateGEP(getBaseAddress(iBuilder, self), blockIndex);
}

Value * ExternalBuffer::getLinearlyAccessibleItems(IDISA::IDISA_Builder * const, Value *, Value *, Value * availItems, const bool reverse) const {
    // All available items can be accessed.
    return reverse ? ConstantInt::getAllOnesValue(availItems->getType()) : availItems;
}

Value * ExternalBuffer::getLinearlyWritableItems(IDISA::IDISA_Builder * const, Value *, Value * fromPosition, const bool reverse) const {
    // Trust that the buffer is large enough to write any amount
    return reverse ? fromPosition : ConstantInt::getAllOnesValue(fromPosition->getType());
}

// Circular Buffer
Value * CircularBuffer::getBlockAddress(IDISA::IDISA_Builder * const iBuilder, Value * const self, Value * const blockIndex) const {
    return iBuilder->CreateGEP(getBaseAddress(iBuilder, self), modByBufferBlocks(iBuilder, blockIndex));
}

Value * CircularBuffer::getRawItemPointer(IDISA::IDISA_Builder * const iBuilder, Value * self, Value * absolutePosition) const {
    Value * ptr = getBaseAddress(iBuilder, self);
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
    Type * const ty = getType();
    Constant * size = ConstantExpr::getSizeOf(ty);
    size = ConstantExpr::getMul(size, ConstantInt::get(size->getType(), mBufferBlocks + mOverflowBlocks));
    mStreamSetBufferPtr = iBuilder->CreatePointerCast(iBuilder->CreateCacheAlignedMalloc(size), ty->getPointerTo());
}

Value * CircularCopybackBuffer::getLinearlyWritableItems(IDISA::IDISA_Builder * const iBuilder, Value * self, Value * fromPosition, bool reverse) const {
    Value * writableProper = StreamSetBuffer::getLinearlyWritableItems(iBuilder, self, fromPosition, reverse);
    if (reverse) return writableProper;
    return iBuilder->CreateAdd(writableProper, iBuilder->getSize(mOverflowBlocks * iBuilder->getBitBlockWidth()));
}

void CircularCopybackBuffer::genCopyBackLogic(IDISA::IDISA_Builder * const b, Value * handle, Value * priorProduced, Value * newProduced, const std::string Name) const {
    assert (priorProduced->getType() == newProduced->getType());
    Constant * bufSize = ConstantInt::get(priorProduced->getType(), mBufferBlocks * b->getBitBlockWidth());
    Value * priorBufPos = b->CreateURem(priorProduced, bufSize);
    Value * newBufPos = b->CreateURem(newProduced, bufSize);
    BasicBlock * copyBack = b->CreateBasicBlock(Name + "_circularCopyBack");
    BasicBlock * done = b->CreateBasicBlock(Name + "_circularCopyBackDone");
    Value * wraparound = b->CreateICmpUGT(priorBufPos, newBufPos);
    b->CreateCondBr(wraparound, copyBack, done);

    b->SetInsertPoint(copyBack);
    Value * overFlowAreaPtr = b->CreateGEP(handle, b->getInt32(mBufferBlocks));
    createBlockAlignedCopy(b, handle, overFlowAreaPtr, newBufPos);
    b->CreateBr(done);

    b->SetInsertPoint(done);
}


// SwizzledCopybackBuffer Buffer

void SwizzledCopybackBuffer::allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    Type * const ty = getType();
    Constant * size = ConstantExpr::getSizeOf(ty);
    size = ConstantExpr::getMul(size, ConstantInt::get(size->getType(), mBufferBlocks + mOverflowBlocks));
    mStreamSetBufferPtr = iBuilder->CreatePointerCast(iBuilder->CreateCacheAlignedMalloc(size), ty->getPointerTo());
}

void SwizzledCopybackBuffer::createBlockAlignedCopy(IDISA::IDISA_Builder * const iBuilder, Value * targetBlockPtr, Value * sourceBlockPtr, Value * itemsToCopy) const {
    Type * int8PtrTy = iBuilder->getInt8PtrTy();
    DataLayout DL(iBuilder->getModule());
    IntegerType * const intAddrTy = iBuilder->getIntPtrTy(DL);

    Constant * blockSize = ConstantInt::get(itemsToCopy->getType(), iBuilder->getBitBlockWidth());
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

Value * SwizzledCopybackBuffer::getBlockAddress(IDISA::IDISA_Builder * const iBuilder, Value * self, Value * blockIndex) const {
    return iBuilder->CreateGEP(getBaseAddress(iBuilder, self), modByBufferBlocks(iBuilder, blockIndex));
}

Value * SwizzledCopybackBuffer::getLinearlyWritableItems(IDISA::IDISA_Builder * const iBuilder, Value * self, Value * fromPosition, bool reverse) const {
    Value * writableProper = StreamSetBuffer::getLinearlyWritableItems(iBuilder, self, fromPosition, reverse);
    if (reverse) return writableProper;
    return iBuilder->CreateAdd(writableProper, iBuilder->getSize(mOverflowBlocks * iBuilder->getBitBlockWidth()));
}

void SwizzledCopybackBuffer::genCopyBackLogic(IDISA::IDISA_Builder * const b, Value * handle, Value * priorProduced, Value * newProduced, const std::string Name) const {
    assert (priorProduced->getType() == newProduced->getType());
    Constant * bufSize = ConstantInt::get(priorProduced->getType(), mBufferBlocks * b->getBitBlockWidth());
    Value * priorBufPos = b->CreateURem(priorProduced, bufSize);
    Value * newBufPos = b->CreateURem(newProduced, bufSize);
    BasicBlock * copyBack = b->CreateBasicBlock(Name + "_swizzledCopyBack");
    BasicBlock * done = b->CreateBasicBlock(Name + "_swizzledCopyBackDone");
    Value * wraparound = b->CreateICmpUGT(priorBufPos, newBufPos);
    b->CreateCondBr(wraparound, copyBack, done);
    b->SetInsertPoint(copyBack);
    Value * overFlowAreaPtr = b->CreateGEP(handle, b->getSize(mBufferBlocks));
    createBlockAlignedCopy(b, handle, overFlowAreaPtr, newBufPos);
    b->CreateBr(done);
    b->SetInsertPoint(done);
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
    report_fatal_error("temporarily not supported");
//    Value * ptr, * offset;
//    std::tie(ptr, offset) = getInternalStreamBuffer(iBuilder, self, streamIndex, blockIndex, readOnly);
//    return iBuilder->CreateGEP(ptr, offset);
}

Value * ExpandableBuffer::getStreamPackPtr(IDISA::IDISA_Builder * const iBuilder, Value * self, Value * streamIndex, Value * blockIndex, Value * packIndex, const bool readOnly) const {
    report_fatal_error("temporarily not supported");
//    Value * ptr, * offset;
//    std::tie(ptr, offset) = getInternalStreamBuffer(iBuilder, self, streamIndex, blockIndex, readOnly);
//    return iBuilder->CreateGEP(ptr, {offset, packIndex});
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

void ExpandableBuffer::releaseBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) const {
    b->CreateFree(getBaseAddress(b.get(), mStreamSetBufferPtr));
}

Value * ExpandableBuffer::getBlockAddress(IDISA::IDISA_Builder * const iBuilder, Value *, Value *) const {
    report_fatal_error("Expandable buffers: getBlockAddress is not supported.");
}

Value * ExpandableBuffer::getLinearlyAccessibleItems(IDISA::IDISA_Builder * const, Value *, Value *, Value *, bool) const {
    report_fatal_error("Expandable buffers: getLinearlyAccessibleItems is not supported.");
}


Value * DynamicBuffer::getBaseAddress(IDISA::IDISA_Builder * const b, Value * const handle) const {
    b->CreateAssert(handle, "DynamicBuffer: instance cannot be null");
    Value * const p = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(int(DynamicBuffer::Field::BaseAddress))});
    Value * const addr = b->CreateLoad(p);
    b->CreateAssert(addr, "DynamicBuffer: base address cannot be 0");
    return addr;
}

Value * DynamicBuffer::getBlockAddress(IDISA::IDISA_Builder * const b, Value * handle, Value * blockIndex) const {
    Value * const wkgBlocks = b->CreateLoad(b->CreateGEP(handle, {b->getInt32(0), b->getInt32(int(DynamicBuffer::Field::WorkingBlocks))}));
    assert (blockIndex->getType() == wkgBlocks->getType());
    return b->CreateGEP(getBaseAddress(b, handle), b->CreateURem(blockIndex, wkgBlocks));
}

Value * DynamicBuffer::getRawItemPointer(IDISA::IDISA_Builder * const b, Value * handle, Value * absolutePosition) const {
    Constant * blockSize = ConstantInt::get(absolutePosition->getType(), b->getBitBlockWidth());
    Value * absBlock = b->CreateUDiv(absolutePosition, blockSize);
    Value * blockPos = b->CreateURem(absolutePosition, blockSize);
    Value * blockPtr = getBlockAddress(b, handle, absBlock);
    const auto bw = mBaseType->getArrayElementType()->getScalarSizeInBits();
    if (bw < 8) {
        assert (bw  == 1 || bw == 2 || bw == 4);
        blockPos = b->CreateUDiv(blockPos, ConstantInt::get(blockPos->getType(), 8 / bw));
        blockPtr = b->CreatePointerCast(blockPtr, b->getInt8PtrTy());
    } else {
        blockPtr = b->CreatePointerCast(blockPtr, b->getIntNTy(bw)->getPointerTo());
    }
    return b->CreateGEP(blockPtr, blockPos);
}


Value * DynamicBuffer::getLinearlyAccessibleItems(IDISA::IDISA_Builder * const b, Value * handle, Value * fromPosition, Value * availItems, bool reverse) const {
    Value * const bufBlocks = b->CreateLoad(b->CreateGEP(handle, {b->getInt32(0), b->getInt32(int(Field::WorkingBlocks))}));
    Constant * blockSize = ConstantInt::get(bufBlocks->getType(), b->getBitBlockWidth());
    Value * bufSize = b->CreateMul(bufBlocks, blockSize);
    assert (bufSize->getType() == fromPosition->getType());
    Value * itemsFromBase = b->CreateURem(fromPosition, bufSize);
    if (reverse) {
        Value * bufAvail = b->CreateSelect(b->CreateICmpEQ(itemsFromBase, b->getSize(0)), bufSize, itemsFromBase);
        return b->CreateSelect(b->CreateICmpULT(bufAvail, availItems), bufAvail, availItems);
    } else {
        Value * linearSpace = b->CreateSub(bufSize, itemsFromBase, "linearSpace");
        return b->CreateSelect(b->CreateICmpULT(availItems, linearSpace), availItems, linearSpace);
    }
}

Value * DynamicBuffer::getLinearlyWritableItems(IDISA::IDISA_Builder * const b, Value * handle, Value * fromPosition, bool reverse) const {    
    Value * bufBlocks = b->CreateLoad(b->CreateGEP(handle, {b->getInt32(0), b->getInt32(int(Field::WorkingBlocks))}));
    Constant * blockSize = ConstantInt::get(bufBlocks->getType(), b->getBitBlockWidth());
    Value * bufSize = b->CreateMul(bufBlocks, blockSize);
    assert (bufSize->getType() == fromPosition->getType());
    Value * bufRem = b->CreateURem(fromPosition, bufSize);
    if (reverse) {
        return b->CreateSelect(b->CreateICmpEQ(bufRem, b->getSize(0)), bufSize, bufRem);
    }
    Constant * overflow = ConstantInt::get(bufBlocks->getType(), mOverflowBlocks);
    bufSize = b->CreateMul(b->CreateAdd(bufBlocks, overflow), blockSize);
    return b->CreateSub(bufSize, bufRem, "linearWritable");
}

Value * DynamicBuffer::getBufferedSize(IDISA::IDISA_Builder * const iBuilder, Value * self) const {
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), iBuilder->getInt32(int(Field::WorkingBlocks))});
    return iBuilder->CreateMul(iBuilder->CreateLoad(ptr), iBuilder->getSize(iBuilder->getBitBlockWidth()));
}

void DynamicBuffer::genCopyBackLogic(IDISA::IDISA_Builder * const b, Value * handle, Value * priorProducedCount, Value * newProducedCount, const std::string Name) const {
    assert (priorProducedCount->getType() == newProducedCount->getType());    
    Value * workingBlocks = b->CreateLoad(b->CreateGEP(handle, {b->getInt32(0), b->getInt32(int(DynamicBuffer::Field::WorkingBlocks))}));
    assert (workingBlocks->getType() == newProducedCount->getType());
    Value * bufSize = b->CreateMul(workingBlocks, ConstantInt::get(workingBlocks->getType(), b->getBitBlockWidth()));
    Value * priorBufPos = b->CreateURem(priorProducedCount, bufSize);
    Value * newBufPos = b->CreateURem(newProducedCount, bufSize);
    BasicBlock * copyBack = b->CreateBasicBlock(Name + "_dynamicCopyBack");
    BasicBlock * done = b->CreateBasicBlock(Name + "_dynamicCopyBackDone");

    Value * wraparound = b->CreateICmpUGT(priorBufPos, newBufPos);
    b->CreateCondBr(wraparound, copyBack, done);

    b->SetInsertPoint(copyBack);
    Value * bufBasePtrField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(int(DynamicBuffer::Field::BaseAddress))});
    Value * bufBasePtr = b->CreateLoad(bufBasePtrField);
    Value * overFlowAreaPtr = b->CreateGEP(bufBasePtr, workingBlocks);
    createBlockAlignedCopy(b, bufBasePtr, overFlowAreaPtr, newBufPos);
    b->CreateBr(done);

    b->SetInsertPoint(done);
}

void DynamicBuffer::allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) {
    Value * handle = b->CreateCacheAlignedAlloca(mBufferStructType);
    size_t numStreams = 1;
    if (isa<ArrayType>(mBaseType)) {
        numStreams = mBaseType->getArrayNumElements();
    }
    const auto fieldWidth = mBaseType->getArrayElementType()->getScalarSizeInBits();
    Value * bufSize = b->getSize((mBufferBlocks + mOverflowBlocks) * b->getBitBlockWidth() * numStreams * fieldWidth/8);
    bufSize = b->CreateRoundUp(bufSize, b->getSize(b->getCacheAlignment()));
    Value * bufBasePtrField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(int(Field::BaseAddress))});
    Type * bufPtrType = bufBasePtrField->getType()->getPointerElementType();
    Value * bufPtr = b->CreatePointerCast(b->CreateCacheAlignedMalloc(bufSize), bufPtrType);
    if (codegen::DebugOptionIsSet(codegen::TraceDynamicBuffers)) {
        b->CallPrintInt("allocated: ", bufPtr);
        b->CallPrintInt("allocated capacity: ", bufSize);
    }
    b->CreateStore(bufPtr, bufBasePtrField);
    b->CreateStore(ConstantPointerNull::getNullValue(bufPtrType), b->CreateGEP(handle, {b->getInt32(0), b->getInt32(int(DynamicBuffer::Field::PriorBaseAddress))}));
    b->CreateStore(bufSize, b->CreateGEP(handle, {b->getInt32(0), b->getInt32(int(Field::AllocatedCapacity))}));
    b->CreateStore(b->getSize(mBufferBlocks), b->CreateGEP(handle, {b->getInt32(0), b->getInt32(int(Field::WorkingBlocks))}));
    b->CreateStore(b->getSize(-1), b->CreateGEP(handle, {b->getInt32(0), b->getInt32(int(Field::Length))}));
    b->CreateStore(b->getSize(0), b->CreateGEP(handle, {b->getInt32(0), b->getInt32(int(Field::ProducedPosition))}));
    b->CreateStore(b->getSize(0), b->CreateGEP(handle, {b->getInt32(0), b->getInt32(int(Field::ConsumedPosition))}));
    mStreamSetBufferPtr = handle;
}

void DynamicBuffer::releaseBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) const {
    Value * handle = mStreamSetBufferPtr;
    /* Free the dynamically allocated buffer, but not the stack-allocated buffer struct. */
    Value * bufBasePtrField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(int(DynamicBuffer::Field::BaseAddress))});
    Type * bufPtrType = bufBasePtrField->getType()->getPointerElementType();
    Value * priorBasePtrField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(int(DynamicBuffer::Field::PriorBaseAddress))});
    BasicBlock * freePrior = b->CreateBasicBlock("freePrior");
    BasicBlock * freeCurrent = b->CreateBasicBlock("freeCurrent");
    Value * priorBuf = b->CreateLoad(priorBasePtrField);
    Value * priorBufIsNonNull = b->CreateICmpNE(priorBuf, ConstantPointerNull::get(cast<PointerType>(bufPtrType)));
    b->CreateCondBr(priorBufIsNonNull, freePrior, freeCurrent);
    b->SetInsertPoint(freePrior);
    if (codegen::DebugOptionIsSet(codegen::TraceDynamicBuffers)) {
        b->CallPrintInt("releasing: ", priorBuf);
    }
    b->CreateFree(priorBuf);
    b->CreateBr(freeCurrent);
    b->SetInsertPoint(freeCurrent);
    b->CreateFree(b->CreateLoad(bufBasePtrField));
}

//
//  Simple capacity doubling.  Use the circular buffer property: duplicating buffer data
//  ensures that we have correct data.   TODO: consider optimizing based on actual
//  consumer and producer positions.
//
void DynamicBuffer::doubleCapacity(IDISA::IDISA_Builder * const b, Value * handle) {
    size_t numStreams = 1;
    if (isa<ArrayType>(mBaseType)) {
        numStreams = mBaseType->getArrayNumElements();
    }
    const auto fieldWidth = mBaseType->getArrayElementType()->getScalarSizeInBits();
    Constant * blockBytes = b->getSize(b->getBitBlockWidth() * numStreams * fieldWidth/8);
    Value * bufBasePtrField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(int(DynamicBuffer::Field::BaseAddress))});
    Type * bufPtrType = bufBasePtrField->getType()->getPointerElementType();
    Value * priorBasePtrField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(int(DynamicBuffer::Field::PriorBaseAddress))});
    Value * workingBlocksField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(int(DynamicBuffer::Field::WorkingBlocks))});
    Value * capacityField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(int(DynamicBuffer::Field::AllocatedCapacity))});

    Value * oldBufPtr = b->CreateLoad(bufBasePtrField);
    Value * currentWorkingBlocks = b->CreateLoad(workingBlocksField);
    Value * workingBytes = b->CreateMul(currentWorkingBlocks, blockBytes);
    Value * const curAllocated = b->CreateLoad(capacityField);
    Value * neededCapacity = b->CreateAdd(workingBytes, workingBytes);
    if (mOverflowBlocks > 0) {
        Constant * overflowBytes = b->getSize(mOverflowBlocks * b->getBitBlockWidth() * numStreams * fieldWidth/8);
        neededCapacity = b->CreateAdd(neededCapacity, overflowBytes);
    }
    neededCapacity = b->CreateRoundUp(neededCapacity, b->getSize(b->getCacheAlignment()));
    BasicBlock * doubleEntry = b->GetInsertBlock();
    BasicBlock * doRealloc = b->CreateBasicBlock("doRealloc");
    BasicBlock * doCopy2 = b->CreateBasicBlock("doCopy2");
    b->CreateCondBr(b->CreateICmpULT(curAllocated, neededCapacity), doRealloc, doCopy2);
    b->SetInsertPoint(doRealloc);
    // If there is a non-null priorBasePtr, free it.
    Value * priorBuf = b->CreateLoad(priorBasePtrField);
    Value * priorBufIsNonNull = b->CreateICmpNE(priorBuf, ConstantPointerNull::get(cast<PointerType>(bufPtrType)));
    BasicBlock * deallocatePrior = b->CreateBasicBlock("deallocatePrior");
    BasicBlock * allocateNew = b->CreateBasicBlock("allocateNew");
    b->CreateCondBr(priorBufIsNonNull, deallocatePrior, allocateNew);
    b->SetInsertPoint(deallocatePrior);
    if (codegen::DebugOptionIsSet(codegen::TraceDynamicBuffers)) {
        b->CallPrintInt("deallocating: ", priorBuf);
    }
    b->CreateFree(priorBuf);
    b->CreateBr(allocateNew);
    b->SetInsertPoint(allocateNew);
    b->CreateStore(oldBufPtr, priorBasePtrField);
    Value * newBufPtr = b->CreatePointerCast(b->CreateCacheAlignedMalloc(neededCapacity), bufPtrType);
    if (codegen::DebugOptionIsSet(codegen::TraceDynamicBuffers)) {
        b->CallPrintInt("re-allocated: ", newBufPtr);
        b->CallPrintInt("allocated capacity: ", neededCapacity);
    }
    b->CreateStore(newBufPtr, bufBasePtrField);
    createBlockCopy(b, newBufPtr, oldBufPtr, currentWorkingBlocks);
    b->CreateStore(neededCapacity, capacityField);
    b->CreateBr(doCopy2);
    b->SetInsertPoint(doCopy2);
    PHINode * bufPtr = b->CreatePHI(oldBufPtr->getType(), 2);
    bufPtr->addIncoming(oldBufPtr, doubleEntry);
    bufPtr->addIncoming(newBufPtr, allocateNew);
    createBlockCopy(b, b->CreateGEP(bufPtr, currentWorkingBlocks), bufPtr, currentWorkingBlocks);
    currentWorkingBlocks = b->CreateAdd(currentWorkingBlocks, currentWorkingBlocks);
    if (codegen::DebugOptionIsSet(codegen::TraceDynamicBuffers)) {
        b->CallPrintInt("currentWorkingBlocks: ", currentWorkingBlocks);
    }
    b->CreateStore(currentWorkingBlocks, workingBlocksField);
}

SourceBuffer::SourceBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, Type * type, unsigned MemoryAddressSpace, unsigned StructAddressSpace)
: StreamSetBuffer(BufferKind::SourceBuffer, type, StructType::get(b->getContext(), {resolveStreamSetType(b, type)->getPointerTo(MemoryAddressSpace), b->getSizeTy(), b->getSizeTy()}), 0, StructAddressSpace) {
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
    if (bufferBlocks < 2 * overflowBlocks) {
        report_fatal_error("CircularCopybackBuffer: bufferBlocks < 2 * overflowBlocks");
    }
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
    if (bufferBlocks < 2 * overflowBlocks) {
        report_fatal_error("SwizzledCopybackBuffer: bufferBlocks < 2 * overflowBlocks");
    }
    if (mOverflowBlocks != 1) {
        mUniqueID += "_" + std::to_string(mOverflowBlocks);
    }
    if (AddressSpace > 0) {
        mUniqueID += "@" + std::to_string(AddressSpace);
    }
}

inline StructType * getDynamicBufferStructType(const std::unique_ptr<kernel::KernelBuilder> & b, Type * baseType, const unsigned addrSpace) {
    IntegerType * sizeTy = b->getSizeTy();
    PointerType * typePtr = baseType->getPointerTo(addrSpace);
    return StructType::get(b->getContext(), {typePtr, typePtr, sizeTy, sizeTy, sizeTy, sizeTy, sizeTy});
}

DynamicBuffer::DynamicBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, Type * type, size_t initialCapacity, size_t overflow, unsigned swizzle, unsigned addrSpace)
: StreamSetBuffer(BufferKind::DynamicBuffer, type, resolveStreamSetType(b, type), initialCapacity, addrSpace)
, mBufferStructType(getDynamicBufferStructType(b, mType, addrSpace))
, mSwizzleFactor(swizzle)
, mOverflowBlocks(overflow)
{
    if (initialCapacity * b->getBitBlockWidth() < 2 * overflow) {
        report_fatal_error("DynamicBuffer: initialCapacity * b->getBitBlockWidth() < 2 * overflow");
    }
    mUniqueID = "DB";
    if (swizzle != 1) {
        mUniqueID += "s" + std::to_string(swizzle);
    }
        if (overflow != 0) {
        mUniqueID += "o" + std::to_string(overflow);
    }
    if (addrSpace != 0) {
        mUniqueID += "@" + std::to_string(addrSpace);
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
    assert(k == BufferKind::SourceBuffer || k == BufferKind::ExternalBuffer || BufferBlocks);
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
            return StructType::get(b->getContext(), {b->getSizeTy(), type->getPointerTo()});
        }
    }
    std::string tmp;
    raw_string_ostream out(tmp);
    type->print(out);
    out << " is an unvalid stream set buffer type.";
    report_fatal_error(out.str());
}
