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

inline static bool is_power_2(const uint64_t n) {
    return ((n & (n - 1)) == 0) && n;
}

Type * StreamSetBuffer::getStreamSetBlockType() const { return mType;}

ArrayType * resolveStreamSetType(const std::unique_ptr<kernel::KernelBuilder> & b, Type * type);

StructType * resolveExpandableStreamSetType(const std::unique_ptr<kernel::KernelBuilder> & b, Type * type);

void StreamSetBuffer::allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) {
    assert (mBufferBlocks > 0);
    assert ("allocate buffer was called twice" && !mStreamSetBufferPtr);
    Type * const ty = getType();
    const auto blocks = (mBufferBlocks + mOverflowBlocks);
    if (mAddressSpace == 0) {
        Constant * size = ConstantExpr::getSizeOf(ty);
        size = ConstantExpr::getMul(size, ConstantInt::get(size->getType(), blocks));
        mStreamSetBufferPtr = b->CreatePointerCast(b->CreateCacheAlignedMalloc(size), ty->getPointerTo());
    } else {
        mStreamSetBufferPtr = b->CreateCacheAlignedAlloca(ty, b->getSize(blocks));
    }
}

void StreamSetBuffer::releaseBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) const {
    if (mAddressSpace == 0) {
        b->CreateFree(mStreamSetBufferPtr);
    }
}

inline bool StreamSetBuffer::isCapacityGuaranteed(const Value * const index, const size_t capacity) const {
    return isa<ConstantInt>(index) ? cast<ConstantInt>(index)->getLimitedValue() < capacity : false;
}

Value * StreamSetBuffer::modBufferSize(IDISA::IDISA_Builder * const b, Value * const offset) const {
    assert (offset->getType()->isIntegerTy());
    if (mBufferBlocks == 0 || isCapacityGuaranteed(offset, mBufferBlocks)) {
        return offset;
    } else if (mBufferBlocks == 1) {
        return ConstantInt::getNullValue(offset->getType());
    } else if (is_power_2(mBufferBlocks)) {
        return b->CreateAnd(offset, ConstantInt::get(offset->getType(), mBufferBlocks - 1));
    } else {
        return b->CreateURem(offset, ConstantInt::get(offset->getType(), mBufferBlocks));
    }
}

Value * StreamSetBuffer::getStreamBlockPtr(IDISA::IDISA_Builder * const b, Value * const handle, Value * streamIndex, Value * blockIndex, const bool /* readOnly */) const {
    if (codegen::DebugOptionIsSet(codegen::EnableAsserts)) {
        Value * const count = getStreamSetCount(b, handle);
        Value * const index = b->CreateZExtOrTrunc(streamIndex, count->getType());
        Value * const cond = b->CreateICmpULT(index, count);
        b->CreateAssert(cond, "out-of-bounds stream access");
    }
    return b->CreateGEP(getBaseAddress(b, handle), {modBufferSize(b, blockIndex), streamIndex});
}

Value * StreamSetBuffer::getStreamPackPtr(IDISA::IDISA_Builder * const b, Value * const handle, Value * streamIndex, Value * blockIndex, Value * packIndex, const bool /* readOnly */) const {
    if (codegen::DebugOptionIsSet(codegen::EnableAsserts)) {
        Value * const count = getStreamSetCount(b, handle);
        Value * const index = b->CreateZExtOrTrunc(streamIndex, count->getType());
        Value * const cond = b->CreateICmpULT(index, count);
        b->CreateAssert(cond, "out-of-bounds stream access");
    }
    return b->CreateGEP(getBaseAddress(b, handle), {modBufferSize(b, blockIndex), streamIndex, packIndex});
}

void StreamSetBuffer::setBaseAddress(IDISA::IDISA_Builder * const /* b */, Value * /* handle */, Value * /* addr */) const {
    report_fatal_error("setBaseAddress is not supported by this buffer type");
}

Value * StreamSetBuffer::getOverflowAddress(IDISA::IDISA_Builder * const b, Value * const handle) const {
    return b->CreateGEP(getBaseAddress(b, handle), b->getSize(mBufferBlocks));
}

Value * StreamSetBuffer::getBufferedSize(IDISA::IDISA_Builder * const b, Value * /* handle */) const {
    return b->getSize(mBufferBlocks * b->getBitBlockWidth());
}

void StreamSetBuffer::setBufferedSize(IDISA::IDISA_Builder * const /* b */, Value * /* handle */, Value * /* size */) const {
    report_fatal_error("setBufferedSize is not supported by this buffer type");
}

Value * StreamSetBuffer::getCapacity(IDISA::IDISA_Builder * const b, Value * const /* handle */) const {
    return b->getSize(mBufferBlocks * b->getBitBlockWidth());
}

void StreamSetBuffer::setCapacity(IDISA::IDISA_Builder * const /* b */, Value * /* handle */, Value * /* c */) const {
    report_fatal_error("setCapacity is not supported by this buffer type");
}

Value * StreamSetBuffer::getStreamSetCount(IDISA::IDISA_Builder * const b, Value *) const {
    size_t count = 1;
    if (isa<ArrayType>(mBaseType)) {
        count = mBaseType->getArrayNumElements();
    }
    return b->getSize(count);
}

void StreamSetBuffer::doubleCapacity(IDISA::IDISA_Builder * const /* b */, Value */* handle */) const {
    report_fatal_error("doubleCapacity is not supported by this buffer type");
}

/**
 * @brief getRawItemPointer
 *
 * get a raw pointer the iN field at position absoluteItemPosition of the stream number streamIndex of the stream set.
 * In the case of a stream whose fields are less than one byte (8 bits) in size, the pointer is to the containing byte.
 * The type of the pointer is i8* for fields of 8 bits or less, otherwise iN* for N-bit fields.
 */
Value * StreamSetBuffer::getRawItemPointer(IDISA::IDISA_Builder * const b, Value * const handle, Value * absolutePosition) const {
    Value * ptr = getBaseAddress(b, handle);
    Value * relativePosition = absolutePosition;
    Type * const elemTy = mBaseType->getArrayElementType()->getVectorElementType();
    const auto bw = elemTy->getPrimitiveSizeInBits();
    assert (is_power_2(bw));
    if (bw < 8) {
        Constant * const fw = ConstantInt::get(relativePosition->getType(), 8 / bw);
        if (codegen::DebugOptionIsSet(codegen::EnableAsserts)) {
            b->CreateAssertZero(b->CreateURem(absolutePosition, fw), "absolutePosition must be byte aligned");
        }
        relativePosition = b->CreateUDiv(relativePosition, fw);
        ptr = b->CreatePointerCast(ptr, b->getInt8PtrTy());
    } else {
        ptr = b->CreatePointerCast(ptr, elemTy->getPointerTo());
    }
    return b->CreateGEP(ptr, relativePosition);
}

Value * StreamSetBuffer::getLinearlyAccessibleItems(IDISA::IDISA_Builder * const b, Value * const /* handle */, Value * fromPosition, Value * availItems, bool reverse) const {
    Type * const ty = fromPosition->getType();
    const auto blockWidth = b->getBitBlockWidth();
    Constant * const bufferSize = ConstantInt::get(ty, mBufferBlocks * blockWidth);
    Value * const itemsFromBase = b->CreateURem(fromPosition, bufferSize);
    if (reverse) {
        Value * const bufAvail = b->CreateSelect(b->CreateIsNull(itemsFromBase), bufferSize, itemsFromBase);
        return b->CreateUMin(availItems, bufAvail);
    } else {
        Constant * capacity = bufferSize;
        if (mOverflowBlocks) {
            capacity = ConstantInt::get(ty, (mBufferBlocks + mOverflowBlocks) * blockWidth - 1);
        }
        Value * const linearSpace = b->CreateSub(capacity, itemsFromBase);
        return b->CreateUMin(availItems, linearSpace);
    }
}

Value * StreamSetBuffer::getLinearlyWritableItems(IDISA::IDISA_Builder * const b, Value * const /* handle */, Value * fromPosition, Value * consumed, bool reverse) const {
    Type * const ty = fromPosition->getType();
    const auto blockWidth = b->getBitBlockWidth();
    Constant * const bufferSize = ConstantInt::get(ty, mBufferBlocks * blockWidth);
    fromPosition = b->CreateURem(fromPosition, bufferSize);
    if (reverse) {
        return b->CreateSelect(b->CreateIsNull(fromPosition), bufferSize, fromPosition);
    }
    consumed = b->CreateURem(consumed, bufferSize);
    Constant * capacity = bufferSize;
    if (mOverflowBlocks) {
        capacity = ConstantInt::get(ty, (mBufferBlocks + mOverflowBlocks) * blockWidth - 1);
    }
    Value * const limit = b->CreateSelect(b->CreateICmpULE(consumed, fromPosition), capacity, consumed);
    return b->CreateSub(limit, fromPosition);
}

Value * StreamSetBuffer::getBaseAddress(IDISA::IDISA_Builder * const b, Value * const handle) const {
    if (codegen::DebugOptionIsSet(codegen::EnableAsserts)) {
        b->CreateAssert(handle, "handle cannot be null");
    }
    return handle;
}

void StreamSetBuffer::createBlockCopy(IDISA::IDISA_Builder * const b, Value * targetBlockPtr, Value * sourceBlockPtr, Value * blocksToCopy) const {
    Type * i8ptr = b->getInt8PtrTy();
    unsigned alignment = b->getBitBlockWidth() / 8;
    size_t numStreams = 1;
    if (isa<ArrayType>(mBaseType)) {
        numStreams = mBaseType->getArrayNumElements();
    }
    const auto fieldWidth = mBaseType->getArrayElementType()->getScalarSizeInBits();
    Value * blockCopyBytes = b->CreateMul(blocksToCopy, b->getSize(b->getBitBlockWidth() * numStreams * fieldWidth/8));
    b->CreateMemMove(b->CreateBitCast(targetBlockPtr, i8ptr), b->CreateBitCast(sourceBlockPtr, i8ptr), blockCopyBytes, alignment);
}

// Source File Buffer
Type * SourceBuffer::getStreamSetBlockType() const {
    return cast<PointerType>(mType->getStructElementType(BaseAddress))->getElementType();
}

Value * SourceBuffer::getBufferedSize(IDISA::IDISA_Builder * const b, Value * const handle) const {
    Value * ptr = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(BufferedSize)});
    return b->CreateLoad(ptr);
}

void SourceBuffer::setBufferedSize(IDISA::IDISA_Builder * const b, Value * const handle, Value * size) const {
    Value * ptr = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(BufferedSize)});
    b->CreateStore(size, ptr);
}

Value * SourceBuffer::getCapacity(IDISA::IDISA_Builder * const b, Value * const handle) const {
    Value * ptr = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(Capacity)});
    return b->CreateLoad(ptr);
}

void SourceBuffer::setCapacity(IDISA::IDISA_Builder * const b, Value * const handle, Value * c) const {
    Value * ptr = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(Capacity)});
    b->CreateStore(c, ptr);
}

void SourceBuffer::setBaseAddress(IDISA::IDISA_Builder * const b, Value * const handle, Value * addr) const {
    if (codegen::DebugOptionIsSet(codegen::EnableAsserts)) {
        b->CreateAssert(handle, "handle cannot be null");
    }
    Value * const ptr = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(BaseAddress)});
    Type * const ptrTy = ptr->getType()->getPointerElementType();
    if (LLVM_LIKELY(isa<PointerType>(addr->getType()))) {
        const auto ptrSpace = cast<PointerType>(ptr->getType())->getAddressSpace();
        const auto addrSpace = cast<PointerType>(ptrTy)->getAddressSpace();
        if (LLVM_UNLIKELY(addrSpace != ptrSpace)) {
            report_fatal_error("SourceBuffer: base address was declared with address space "
                                     + std::to_string(ptrSpace)
                                     + " but given a pointer in address space "
                                     + std::to_string(addrSpace));
        }
    } else {
        report_fatal_error("SourceBuffer: base address is not a pointer type");
    }
    if (codegen::DebugOptionIsSet(codegen::EnableAsserts)) {
        b->CreateAssert(ptr, "SourceBuffer: base address cannot be zero");
        DataLayout DL(b->getModule());
        IntegerType * const intPtrTy = b->getIntPtrTy(DL, cast<PointerType>(ptrTy)->getAddressSpace());
        Value * const notAligned = b->CreateURem(b->CreatePtrToInt(ptr, intPtrTy), ConstantInt::get(intPtrTy, b->getBitBlockWidth() / 8));
        b->CreateAssertZero(notAligned, "SourceBuffer: base address is not aligned with the bit block width");
    }
    b->CreateStore(b->CreatePointerCast(addr, ptrTy), ptr);
}

Value * SourceBuffer::getBaseAddress(IDISA::IDISA_Builder * const b, Value * const handle) const {
    if (codegen::DebugOptionIsSet(codegen::EnableAsserts)) {
        b->CreateAssert(handle, "handle cannot be null");
    }
    Value * const ptr = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(BaseAddress)});
    return b->CreateLoad(ptr);
}

Value * SourceBuffer::getLinearlyAccessibleItems(IDISA::IDISA_Builder * const b, Value * const handle, Value * fromPosition, Value * availItems, bool reverse) const {
    if (reverse) report_fatal_error("SourceBuffer cannot be accessed in reverse");
    Value * maxAvail = b->CreateNUWSub(getBufferedSize(b, handle), fromPosition);
    return b->CreateSelect(b->CreateICmpULT(availItems, maxAvail), availItems, maxAvail);
}

Value * SourceBuffer::getLinearlyWritableItems(IDISA::IDISA_Builder * const b, Value * const handle, Value * fromPosition, Value *consumed, bool reverse) const {
    report_fatal_error("SourceBuffers cannot be written");
}

void SourceBuffer::allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) {
    if (LLVM_LIKELY(mStreamSetBufferPtr == nullptr)) {
        Type * const ty = getType();
        mStreamSetBufferPtr = b->CreateCacheAlignedAlloca(ty, b->getSize(mBufferBlocks));
        b->CreateAlignedStore(Constant::getNullValue(ty), mStreamSetBufferPtr, b->getCacheAlignment());
    } else {
        report_fatal_error("StreamSetBuffer::allocateBuffer() was called twice on the same stream set");
    }
}

void SourceBuffer::releaseBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) const {

}

// External File Buffer
void ExternalBuffer::allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> &) {
    report_fatal_error("External buffers cannot be allocated.");
}

void ExternalBuffer::releaseBuffer(const std::unique_ptr<kernel::KernelBuilder> &) const {

}

Value * ExternalBuffer::getLinearlyAccessibleItems(IDISA::IDISA_Builder * const, Value *, Value *, Value * availItems, const bool reverse) const {
    // All available items can be accessed.
    return reverse ? ConstantInt::getAllOnesValue(availItems->getType()) : availItems;
}

Value * ExternalBuffer::getLinearlyWritableItems(IDISA::IDISA_Builder * const, Value *, Value * fromPosition, Value *consumed, const bool reverse) const {
    // Trust that the buffer is large enough to write any amount
    return reverse ? fromPosition : ConstantInt::getAllOnesValue(fromPosition->getType());
}

Value * ExternalBuffer::getBufferedSize(IDISA::IDISA_Builder * const b, Value * const /* handle */) const {
    return ConstantInt::getAllOnesValue(b->getSizeTy());
}

Value * ExternalBuffer::getCapacity(IDISA::IDISA_Builder * const b, Value * const /* handle */) const {
    return ConstantInt::getAllOnesValue(b->getSizeTy());
}

// Circular Buffer
Value * CircularBuffer::getRawItemPointer(IDISA::IDISA_Builder * const b, Value * const handle, Value * absolutePosition) const {
    Value * ptr = getBaseAddress(b, handle);
    Value * relativePosition = b->CreateURem(absolutePosition, ConstantInt::get(absolutePosition->getType(), mBufferBlocks * b->getBitBlockWidth()));
    Type * const elemTy = mBaseType->getArrayElementType()->getVectorElementType();
    const auto bw = elemTy->getPrimitiveSizeInBits();
    assert (is_power_2(bw));
    if (bw < 8) {
        Constant * const fw = ConstantInt::get(relativePosition->getType(), 8 / bw);
        relativePosition = b->CreateUDiv(relativePosition, fw);
        ptr = b->CreatePointerCast(ptr, b->getInt8PtrTy());
    } else {
        ptr = b->CreatePointerCast(ptr, elemTy->getPointerTo());
    }
    return b->CreateGEP(ptr, relativePosition);
}

// Expandable Buffer

void ExpandableBuffer::allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) {
    mStreamSetBufferPtr = b->CreateCacheAlignedAlloca(getType());
    Value * const capacityPtr = b->CreateGEP(mStreamSetBufferPtr, {b->getInt32(0), b->getInt32(0)});
    b->CreateStore(b->getSize(mInitialCapacity), capacityPtr);
    Type * const bufferType = getType()->getStructElementType(1)->getPointerElementType();
    Constant * const bufferWidth = ConstantExpr::getIntegerCast(ConstantExpr::getSizeOf(bufferType), b->getSizeTy(), false);
    Constant * const size = ConstantExpr::getMul(b->getSize(mBufferBlocks * mInitialCapacity), bufferWidth);
    const auto alignment = std::max(b->getCacheAlignment(), b->getBitBlockWidth() / 8);
    Value * const ptr = b->CreateAlignedMalloc(size, alignment);
    b->CreateMemZero(ptr, size, bufferType->getPrimitiveSizeInBits() / 8);
    Value * const streamSetPtr = b->CreateGEP(mStreamSetBufferPtr, {b->getInt32(0), b->getInt32(1)});
    b->CreateStore(b->CreatePointerCast(ptr, bufferType->getPointerTo()), streamSetPtr);
}

std::pair<Value *, Value *> ExpandableBuffer::getInternalStreamBuffer(IDISA::IDISA_Builder * const b, Value * const handle, Value * streamIndex, Value * blockIndex, const bool readOnly) const {

    // ENTRY
    Value * const capacityPtr = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(0)});
    Value * const capacity = b->CreateLoad(capacityPtr);
    Value * const streamSetPtr = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(1)});
    Value * const streamSet = b->CreateLoad(streamSetPtr);
    blockIndex = modBufferSize(b, blockIndex);

    assert (streamIndex->getType() == capacity->getType());
    Value * const cond = b->CreateICmpULT(streamIndex, capacity);

    // Are we guaranteed that we can access this stream?
    if (readOnly || isCapacityGuaranteed(streamIndex, mInitialCapacity)) {
        if (codegen::DebugOptionIsSet(codegen::EnableAsserts)) {
            b->CreateAssert(cond, "out-of-bounds stream access");
        }
        Value * offset = b->CreateAdd(b->CreateMul(blockIndex, capacity), streamIndex);
        return {streamSet, offset};
    }

    BasicBlock * const entry = b->GetInsertBlock();
    BasicBlock * const expand = BasicBlock::Create(b->getContext(), "expand", entry->getParent());
    BasicBlock * const resume = BasicBlock::Create(b->getContext(), "resume", entry->getParent());

    b->CreateLikelyCondBr(cond, resume, expand);

    // EXPAND
    b->SetInsertPoint(expand);

    Type * elementType = getType()->getStructElementType(1)->getPointerElementType();
    Constant * const vectorWidth = ConstantExpr::getIntegerCast(ConstantExpr::getSizeOf(elementType), capacity->getType(), false);

    Value * newCapacity = b->CreateAdd(streamIndex, b->getSize(1));
    newCapacity = b->CreateCeilLog2(newCapacity);
    newCapacity = b->CreateShl(b->getSize(1), newCapacity, "newCapacity");

    std::string tmp;
    raw_string_ostream out(tmp);
    out << "__expand";
    elementType->print(out);
    std::string name = out.str();

    Module * const m = b->getModule();
    Function * expandFunction = m->getFunction(name);

    if (expandFunction == nullptr) {

        const auto ip = b->saveIP();

        FunctionType * fty = FunctionType::get(elementType->getPointerTo(), {elementType->getPointerTo(), b->getSizeTy(), b->getSizeTy()}, false);
        expandFunction = Function::Create(fty, GlobalValue::PrivateLinkage, name, m);

        auto args = expandFunction->arg_begin();
        Value * streamSet = &*args++;
        Value * capacity = &*args++;
        Value * newCapacity = &*args;

        BasicBlock * entry = BasicBlock::Create(b->getContext(), "entry", expandFunction);
        b->SetInsertPoint(entry);

        Value * size = b->CreateMul(newCapacity, b->getSize(mBufferBlocks));
        const auto memAlign = std::max(b->getCacheAlignment(), b->getBitBlockWidth() / 8);

        Value * newStreamSet = b->CreatePointerCast(b->CreateAlignedMalloc(b->CreateMul(size, vectorWidth), memAlign), elementType->getPointerTo());
        Value * const diffCapacity = b->CreateMul(b->CreateSub(newCapacity, capacity), vectorWidth);

        const auto alignment = elementType->getPrimitiveSizeInBits() / 8;
        for (unsigned i = 0; i < mBufferBlocks; ++i) {
            ConstantInt * const offset = b->getSize(i);
            Value * srcOffset = b->CreateMul(capacity, offset);
            Value * srcPtr = b->CreateGEP(streamSet, srcOffset);
            Value * destOffset = b->CreateMul(newCapacity, offset);
            Value * destPtr = b->CreateGEP(newStreamSet, destOffset);
            b->CreateMemCpy(destPtr, srcPtr, b->CreateMul(capacity, vectorWidth), alignment);
            Value * destZeroOffset = b->CreateAdd(destOffset, capacity);
            Value * destZeroPtr = b->CreateGEP(newStreamSet, destZeroOffset);
            b->CreateMemZero(destZeroPtr, diffCapacity, alignment);
        }

        b->CreateFree(streamSet);

        b->CreateRet(newStreamSet);

        b->restoreIP(ip);
    }

    Value * newStreamSet = b->CreateCall(expandFunction, {streamSet, capacity, newCapacity});
    b->CreateStore(newStreamSet, streamSetPtr);
    b->CreateStore(newCapacity, capacityPtr);

    b->CreateBr(resume);

    // RESUME
    b->SetInsertPoint(resume);

    PHINode * phiStreamSet = b->CreatePHI(streamSet->getType(), 2);
    phiStreamSet->addIncoming(streamSet, entry);
    phiStreamSet->addIncoming(newStreamSet, expand);

    PHINode * phiCapacity = b->CreatePHI(capacity->getType(), 2);
    phiCapacity->addIncoming(capacity, entry);
    phiCapacity->addIncoming(newCapacity, expand);

    Value * offset = b->CreateAdd(b->CreateMul(blockIndex, phiCapacity), streamIndex);

    return {phiStreamSet, offset};
}

Value * ExpandableBuffer::getStreamBlockPtr(IDISA::IDISA_Builder * const b, Value * const handle, Value * streamIndex, Value * blockIndex, const bool readOnly) const {
    report_fatal_error("temporarily not supported");
//    Value * ptr, * offset;
//    std::tie(ptr, offset) = getInternalStreamBuffer(b, handle, streamIndex, blockIndex, readOnly);
//    return b->CreateGEP(ptr, offset);
}

Value * ExpandableBuffer::getStreamPackPtr(IDISA::IDISA_Builder * const b, Value * const handle, Value * streamIndex, Value * blockIndex, Value * packIndex, const bool readOnly) const {
    report_fatal_error("temporarily not supported");
//    Value * ptr, * offset;
//    std::tie(ptr, offset) = getInternalStreamBuffer(b, handle, streamIndex, blockIndex, readOnly);
//    return b->CreateGEP(ptr, {offset, packIndex});
}

Value * ExpandableBuffer::getStreamSetCount(IDISA::IDISA_Builder * const b, Value * const handle) const {
    return b->CreateLoad(b->CreateGEP(handle, {b->getInt32(0), b->getInt32(0)}));
}

Value * ExpandableBuffer::getBaseAddress(IDISA::IDISA_Builder * const b, Value * const handle) const {
    if (codegen::DebugOptionIsSet(codegen::EnableAsserts)) {
        b->CreateAssert(handle, "handle cannot be null");
    }
    Value * const baseAddr = b->CreateLoad(b->CreateGEP(handle, {b->getInt32(0), b->getInt32(1)}));
    if (codegen::DebugOptionIsSet(codegen::EnableAsserts)) {
        b->CreateAssert(handle, "base address cannot be 0");
    }
    return baseAddr;
}

void ExpandableBuffer::releaseBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) const {
    b->CreateFree(getBaseAddress(b.get(), mStreamSetBufferPtr));
}

Value * ExpandableBuffer::getLinearlyAccessibleItems(IDISA::IDISA_Builder * const, Value *, Value *, Value *, bool) const {
    report_fatal_error("Expandable buffers: getLinearlyAccessibleItems is not supported.");
}


Value * DynamicBuffer::getBaseAddress(IDISA::IDISA_Builder * const b, Value * const handle) const {
    if (codegen::DebugOptionIsSet(codegen::EnableAsserts)) {
        b->CreateAssert(handle, "handle cannot be null");
    }
    Value * const p = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(BaseAddress)});
    Value * const addr = b->CreateLoad(p);
    if (codegen::DebugOptionIsSet(codegen::EnableAsserts)) {
        b->CreateAssert(addr, "base address cannot be 0");
    }
    return addr;
}

Value * DynamicBuffer::getBlockAddress(IDISA::IDISA_Builder * const b, Value * const handle, Value * blockIndex) const {
    Value * const workingBlocks = b->CreateLoad(b->CreateGEP(handle, {b->getInt32(0), b->getInt32(WorkingBlocks)}));
    assert (blockIndex->getType() == workingBlocks->getType());
    return b->CreateGEP(getBaseAddress(b, handle), b->CreateURem(blockIndex, workingBlocks));
}

Value * DynamicBuffer::getOverflowAddress(IDISA::IDISA_Builder * const b, Value * const handle) const {
    Value * const workingBlocks = b->CreateLoad(b->CreateGEP(handle, {b->getInt32(0), b->getInt32(WorkingBlocks)}));
    return b->CreateGEP(getBaseAddress(b, handle), workingBlocks);
}

Value * DynamicBuffer::getRawItemPointer(IDISA::IDISA_Builder * const b, Value * const handle, Value * absolutePosition) const {
    Constant * blockSize = ConstantInt::get(absolutePosition->getType(), b->getBitBlockWidth());
    Value * const absBlock = b->CreateUDiv(absolutePosition, blockSize);
    Value * blockPos = b->CreateURem(absolutePosition, blockSize);
    Value * blockPtr = getBlockAddress(b, handle, absBlock);
    Type * const elemTy = mBaseType->getArrayElementType()->getVectorElementType();
    const auto bw = elemTy->getPrimitiveSizeInBits();
    assert (is_power_2(bw));
    if (bw < 8) {
        blockPos = b->CreateUDiv(blockPos, ConstantInt::get(blockPos->getType(), 8 / bw));
        blockPtr = b->CreatePointerCast(blockPtr, b->getInt8PtrTy());
    } else {
        blockPtr = b->CreatePointerCast(blockPtr, elemTy->getPointerTo());
    }
    return b->CreateGEP(blockPtr, blockPos);
}

//Value * DynamicBuffer::getLinearlyAccessibleItems(IDISA::IDISA_Builder * const b, Value * const handle, Value * fromPosition, Value * availItems, bool reverse) const {
//    Value * const bufferSize = getBufferedSize(b, handle);
//    assert (bufferSize->getType() == fromPosition->getType());
//    Value * itemsFromBase = b->CreateURem(fromPosition, bufferSize);
//    if (reverse) {
//        Value * bufAvail = b->CreateSelect(b->CreateIsNull(itemsFromBase), bufferSize, itemsFromBase);
//        return b->CreateSelect(b->CreateICmpULT(bufAvail, availItems), bufAvail, availItems);
//    } else {
//        Constant * const overflow = ConstantInt::get(bufBlocks->getType(), mOverflowBlocks * b->getBitBlockWidth() - 1);
//        Value * const linearSpace = b->CreateAdd(bufferSize, overflow);
//        Value * remaining = b->CreateSub(linearSpace, itemsFromBase);
//        return b->CreateSelect(b->CreateICmpULT(availItems, remaining), availItems, remaining);
//    }
//}

//Value * DynamicBuffer::getLinearlyWritableItems(IDISA::IDISA_Builder * const b, Value * const handle, Value * fromPosition, Value * consumed, bool reverse) const {
//    Value * const bufferSize = getBufferedSize(b, handle);
//    assert (bufferSize->getType() == fromPosition->getType());
//    Value * bufRem = b->CreateURem(fromPosition, bufferSize);
//    if (reverse) {
//        return b->CreateSelect(b->CreateIsNull(bufRem), bufferSize, bufRem);
//    }
//    Constant * const overflow = ConstantInt::get(bufBlocks->getType(), mOverflowBlocks * b->getBitBlockWidth() - 1);
//    Value * const linearSpace = b->CreateAdd(bufferSize, overflow);
//    return b->CreateSub(linearSpace, bufRem);
//}

Value * DynamicBuffer::getLinearlyAccessibleItems(IDISA::IDISA_Builder * const b, Value * const handle, Value * fromPosition, Value * availItems, bool reverse) const {
    Value * const bufferSize = getBufferedSize(b, handle);
    Value * const itemsFromBase = b->CreateURem(fromPosition, bufferSize);
    if (reverse) {
        Value * const bufAvail = b->CreateSelect(b->CreateIsNull(itemsFromBase), bufferSize, itemsFromBase);
        return b->CreateUMin(availItems, bufAvail);
    } else {
        Value * capacity = bufferSize;
        if (mOverflowBlocks) {
            Constant * const overflow = b->getSize(mOverflowBlocks * b->getBitBlockWidth() - 1);
            capacity = b->CreateAdd(bufferSize, overflow);
        }
        Value * const linearSpace = b->CreateSub(capacity, itemsFromBase);
        return b->CreateUMin(availItems, linearSpace);
    }
}

Value * DynamicBuffer::getLinearlyWritableItems(IDISA::IDISA_Builder * const b, Value * const handle, Value * fromPosition, Value * consumed, bool reverse) const {
    Value * const bufferSize = getBufferedSize(b, handle);
    fromPosition = b->CreateURem(fromPosition, bufferSize);
    if (reverse) {
        return b->CreateSelect(b->CreateIsNull(fromPosition), bufferSize, fromPosition);
    }
    consumed = b->CreateURem(consumed, bufferSize);
    Value * capacity = bufferSize;
    if (mOverflowBlocks) {
        Constant * const overflow = b->getSize(mOverflowBlocks * b->getBitBlockWidth() - 1);
        capacity = b->CreateAdd(bufferSize, overflow);
    }
    Value * const limit = b->CreateSelect(b->CreateICmpULE(consumed, fromPosition), capacity, consumed);
    return b->CreateSub(limit, fromPosition);
}



Value * DynamicBuffer::getBufferedSize(IDISA::IDISA_Builder * const b, Value * const handle) const {
    Value * ptr = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(WorkingBlocks)});
    return b->CreateMul(b->CreateLoad(ptr), b->getSize(b->getBitBlockWidth()));
}


inline StructType * getDynamicBufferStructType(const std::unique_ptr<kernel::KernelBuilder> & b, Type * baseType, const unsigned addrSpace) {
    IntegerType * sizeTy = b->getSizeTy();
    PointerType * typePtr = baseType->getPointerTo(addrSpace);
    return StructType::get(b->getContext(), {typePtr, typePtr, sizeTy, sizeTy, sizeTy, sizeTy, sizeTy});
}

void DynamicBuffer::allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) {
    Type * const structTy = getDynamicBufferStructType(b, mType, mAddressSpace);
    Value * const handle = b->CreateCacheAlignedAlloca(structTy);
    size_t numStreams = 1;
    if (isa<ArrayType>(mBaseType)) {
        numStreams = mBaseType->getArrayNumElements();
    }
    const auto fieldWidth = mBaseType->getArrayElementType()->getScalarSizeInBits();
    Value * bufSize = b->getSize((mBufferBlocks + mOverflowBlocks) * b->getBitBlockWidth() * numStreams * fieldWidth/8);
    bufSize = b->CreateRoundUp(bufSize, b->getSize(b->getCacheAlignment()));
    Value * bufBasePtrField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(BaseAddress)});
    Type * bufPtrType = bufBasePtrField->getType()->getPointerElementType();
    Value * bufPtr = b->CreatePointerCast(b->CreateCacheAlignedMalloc(bufSize), bufPtrType);
    if (codegen::DebugOptionIsSet(codegen::TraceDynamicBuffers)) {
        b->CallPrintInt("allocated: ", bufPtr);
        b->CallPrintInt("allocated capacity: ", bufSize);
    }
    b->CreateStore(bufPtr, bufBasePtrField);
    b->CreateStore(ConstantPointerNull::getNullValue(bufPtrType), b->CreateGEP(handle, {b->getInt32(0), b->getInt32(PriorBaseAddress)}));
    b->CreateStore(bufSize, b->CreateGEP(handle, {b->getInt32(0), b->getInt32(AllocatedCapacity)}));
    b->CreateStore(b->getSize(mBufferBlocks), b->CreateGEP(handle, {b->getInt32(0), b->getInt32(WorkingBlocks)}));
    b->CreateStore(b->getSize(-1), b->CreateGEP(handle, {b->getInt32(0), b->getInt32(Length)}));
    b->CreateStore(b->getSize(0), b->CreateGEP(handle, {b->getInt32(0), b->getInt32(ProducedPosition)}));
    b->CreateStore(b->getSize(0), b->CreateGEP(handle, {b->getInt32(0), b->getInt32(ConsumedPosition)}));
    mStreamSetBufferPtr = handle;
}

void DynamicBuffer::releaseBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) const {
    Value * const handle = mStreamSetBufferPtr;
    /* Free the dynamically allocated buffer, but not the stack-allocated buffer struct. */
    Value * bufBasePtrField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(BaseAddress)});
    Type * bufPtrType = bufBasePtrField->getType()->getPointerElementType();
    Value * priorBasePtrField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(PriorBaseAddress)});
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
void DynamicBuffer::doubleCapacity(IDISA::IDISA_Builder * const b, Value * const handle) const {
    size_t numStreams = 1;
    if (isa<ArrayType>(mBaseType)) {
        numStreams = mBaseType->getArrayNumElements();
    }
    const auto fieldWidth = mBaseType->getArrayElementType()->getScalarSizeInBits();
    Constant * blockBytes = b->getSize(b->getBitBlockWidth() * numStreams * fieldWidth/8);
    Value * bufBasePtrField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(BaseAddress)});
    Type * bufPtrType = bufBasePtrField->getType()->getPointerElementType();
    Value * priorBasePtrField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(PriorBaseAddress)});
    Value * workingBlocksField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(WorkingBlocks)});
    Value * capacityField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(AllocatedCapacity)});

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

inline StructType * getSourceBufferType(const std::unique_ptr<kernel::KernelBuilder> & b, Type * const type, const unsigned MemoryAddressSpace) {
    return StructType::get(b->getContext(), {resolveStreamSetType(b, type)->getPointerTo(MemoryAddressSpace), b->getSizeTy(), b->getSizeTy()});
}

SourceBuffer::SourceBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, Type * type, unsigned MemoryAddressSpace, unsigned StructAddressSpace)
: StreamSetBuffer(BufferKind::SourceBuffer, type, getSourceBufferType(b, type, MemoryAddressSpace), 0, 0, StructAddressSpace) {
    mUniqueID = "B";
    if (MemoryAddressSpace != 0 || StructAddressSpace != 0) {
        mUniqueID += "@" + std::to_string(MemoryAddressSpace) + ":" + std::to_string(StructAddressSpace);
    }
}

ExternalBuffer::ExternalBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, Type * type, Value * addr, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::ExternalBuffer, type, resolveStreamSetType(b, type), 0, 0, AddressSpace) {
    mUniqueID = "E";
    if (AddressSpace > 0) mUniqueID += "@" + std::to_string(AddressSpace);
    mStreamSetBufferPtr = b->CreatePointerBitCastOrAddrSpaceCast(addr, getPointerType());
}

CircularBuffer::CircularBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, Type * type, size_t bufferBlocks, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::CircularBuffer, type, resolveStreamSetType(b, type), bufferBlocks, 0, AddressSpace) {
    mUniqueID = "C" + std::to_string(bufferBlocks);
    if (AddressSpace > 0) mUniqueID += "@" + std::to_string(AddressSpace);
}

CircularBuffer::CircularBuffer(const BufferKind k, const std::unique_ptr<kernel::KernelBuilder> & b, Type * type, size_t bufferBlocks, size_t overflowBlocks, unsigned AddressSpace)
: StreamSetBuffer(k, type, resolveStreamSetType(b, type), bufferBlocks, overflowBlocks, AddressSpace) {

}

CircularCopybackBuffer::CircularCopybackBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, Type * type, size_t bufferBlocks, size_t overflowBlocks, unsigned AddressSpace)
: CircularBuffer(BufferKind::CircularCopybackBuffer, b, type, bufferBlocks, overflowBlocks, AddressSpace) {
    if (overflowBlocks < 1) {
        report_fatal_error("CircularCopybackBuffer: overflowBlocks < 1");
    }
    if (bufferBlocks < 2 * overflowBlocks) {
        report_fatal_error("CircularCopybackBuffer: bufferBlocks < 2 * overflowBlocks");
    }
    mUniqueID = "CC" + std::to_string(bufferBlocks);
    if (mOverflowBlocks != 1) mUniqueID += "_" + std::to_string(mOverflowBlocks);
    if (AddressSpace > 0) mUniqueID += "@" + std::to_string(AddressSpace);
}

ExpandableBuffer::ExpandableBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, Type * type, size_t bufferBlocks, unsigned AddressSpace)
: StreamSetBuffer(BufferKind::ExpandableBuffer, type, resolveExpandableStreamSetType(b, type), bufferBlocks, 0, AddressSpace)
, mInitialCapacity(type->getArrayNumElements()) {
    mUniqueID = "XP" + std::to_string(bufferBlocks);
    if (AddressSpace > 0) mUniqueID += "@" + std::to_string(AddressSpace);
}

DynamicBuffer::DynamicBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, Type * type, size_t initialCapacity, size_t overflow, unsigned swizzle, unsigned addrSpace)
: StreamSetBuffer(BufferKind::DynamicBuffer, type, resolveStreamSetType(b, type), initialCapacity, overflow, addrSpace) {
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


inline StreamSetBuffer::StreamSetBuffer(BufferKind k, Type * baseType, Type * resolvedType, unsigned BufferBlocks, unsigned OverflowBlocks, unsigned AddressSpace)
: mBufferKind(k)
, mType(resolvedType)
, mBufferBlocks(BufferBlocks)
, mOverflowBlocks(OverflowBlocks)
, mAddressSpace(AddressSpace)
, mStreamSetBufferPtr(nullptr)
, mBaseType(baseType)
, mProducer(nullptr) {
    assert((k == BufferKind::SourceBuffer || k == BufferKind::ExternalBuffer) ^ (BufferBlocks > 0));
    assert ("A zero length buffer cannot have overflow blocks!" && ((BufferBlocks > 0) || (OverflowBlocks == 0)));
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
