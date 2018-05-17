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

Value * StreamSetBuffer::getStreamBlockPtr(IDISA::IDISA_Builder * const b, Value * const handle, Value * streamIndex, Value * blockIndex, const bool /* readOnly */) const {
    if (codegen::DebugOptionIsSet(codegen::EnableAsserts)) {
        Value * const count = getStreamSetCount(b, handle);
        Value * const index = b->CreateZExtOrTrunc(streamIndex, count->getType());
        Value * const cond = b->CreateICmpULT(index, count);
        b->CreateAssert(cond, "out-of-bounds stream access");
    }
    return b->CreateGEP(getBaseAddress(b, handle), {blockIndex, streamIndex});
}

Value * StreamSetBuffer::getStreamPackPtr(IDISA::IDISA_Builder * const b, Value * const handle, Value * streamIndex, Value * blockIndex, Value * packIndex, const bool /* readOnly */) const {
    if (codegen::DebugOptionIsSet(codegen::EnableAsserts)) {
        Value * const count = getStreamSetCount(b, handle);
        Value * const index = b->CreateZExtOrTrunc(streamIndex, count->getType());
        Value * const cond = b->CreateICmpULT(index, count);
        b->CreateAssert(cond, "out-of-bounds stream access");
    }
    return b->CreateGEP(getBaseAddress(b, handle), {blockIndex, streamIndex, packIndex});
}

Value * StreamSetBuffer::getStreamSetCount(IDISA::IDISA_Builder * const b, Value *) const {
    size_t count = 1;
    if (isa<ArrayType>(mBaseType)) {
        count = mBaseType->getArrayNumElements();
    }
    return b->getSize(count);
}

// External File Buffer
void ExternalBuffer::allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) {
    PointerType * const ptrTy = getPointerType();
    IntegerType * const sizeTy = b->getSizeTy();
    StructType * const structTy = StructType::get(b->getContext(), {ptrTy, sizeTy});
    Value * const handle = b->CreateCacheAlignedAlloca(structTy);
    mStreamSetHandle = handle;
    // If mExternalAddress is null, it must be set by a source kernel.
    Value * ptr = nullptr;
    Constant * size = nullptr;
    if (mExternalAddress) {
        ptr = b->CreatePointerBitCastOrAddrSpaceCast(mExternalAddress, ptrTy);
        size = ConstantInt::getAllOnesValue(sizeTy);
    } else {
        ptr = ConstantPointerNull::get(ptrTy);
        size = ConstantInt::getNullValue(sizeTy);
    }
    b->CreateStore(ptr, b->CreateGEP(handle, {b->getInt32(0), b->getInt32(BaseAddress)}));
    b->CreateStore(size, b->CreateGEP(handle, {b->getInt32(0), b->getInt32(Capacity)}));
}

void ExternalBuffer::setBaseAddress(IDISA::IDISA_Builder * const b, llvm::Value * handle, llvm::Value * addr) const {
    if (codegen::DebugOptionIsSet(codegen::EnableAsserts)) {
        b->CreateAssert(handle, "handle cannot be null");
    }
    Value * const p = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(BaseAddress)});
    Value * const ptr = b->CreatePointerBitCastOrAddrSpaceCast(addr, getPointerType());
    b->CreateStore(ptr, p);
}

Value * ExternalBuffer::getBaseAddress(IDISA::IDISA_Builder * const b, Value * const handle) const {
    if (codegen::DebugOptionIsSet(codegen::EnableAsserts)) {
        b->CreateAssert(handle, "handle cannot be null");
    }
    Value * const p = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(BaseAddress)});
    return b->CreateLoad(p);
}

Value * ExternalBuffer::getOverflowAddress(IDISA::IDISA_Builder * const /* b */, Value * const /* handle */) const {
    report_fatal_error("getOverflowAddress is not supported by this buffer type");
}

void ExternalBuffer::setCapacity(IDISA::IDISA_Builder * const b, Value * const handle, Value * const capacity) const {
    if (codegen::DebugOptionIsSet(codegen::EnableAsserts)) {
        b->CreateAssert(handle, "handle cannot be null");
    }
    Value * const p = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(Capacity)});
    b->CreateStore(capacity, p);
}

Value * ExternalBuffer::getCapacity(IDISA::IDISA_Builder * const b, Value * const handle) const {
    if (codegen::DebugOptionIsSet(codegen::EnableAsserts)) {
        b->CreateAssert(handle, "handle cannot be null");
    }
    Value * const p = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(Capacity)});
    return b->CreateLoad(p);
}

void ExternalBuffer::releaseBuffer(const std::unique_ptr<kernel::KernelBuilder> &) const {
    // this buffer is not responsible for free-ing th data associated with it
}

Value * ExternalBuffer::getLinearlyAccessibleItems(IDISA::IDISA_Builder * const, Value *, Value *, Value * availItems, const bool reverse) const {
    // All available items can be accessed.
    return reverse ? ConstantInt::getAllOnesValue(availItems->getType()) : availItems;
}

Value * ExternalBuffer::getLinearlyWritableItems(IDISA::IDISA_Builder * const, Value *, Value * fromPosition, Value *consumed, const bool reverse) const {
    // Trust that the buffer is large enough to write any amount
    return reverse ? fromPosition : ConstantInt::getAllOnesValue(fromPosition->getType());
}

/**
 * @brief getRawItemPointer
 *
 * get a raw pointer the iN field at position absoluteItemPosition of the stream number streamIndex of the stream set.
 * In the case of a stream whose fields are less than one byte (8 bits) in size, the pointer is to the containing byte.
 * The type of the pointer is i8* for fields of 8 bits or less, otherwise iN* for N-bit fields.
 */
Value * ExternalBuffer::getRawItemPointer(IDISA::IDISA_Builder * const b, Value * const handle, Value * absolutePosition) const {
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

// Circular Buffer
void StaticBuffer::allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) {
    assert (mCapacity > 0);
    assert ("allocate buffer was called twice" && !mStreamSetHandle);
    Type * const ty = getType();
    const auto blocks = (mCapacity + mOverflow);
    if (mAddressSpace == 0) {
        Constant * size = ConstantExpr::getSizeOf(ty);
        size = ConstantExpr::getMul(size, ConstantInt::get(size->getType(), blocks));
        mStreamSetHandle = b->CreatePointerCast(b->CreateCacheAlignedMalloc(size), ty->getPointerTo());
    } else {
        mStreamSetHandle = b->CreateCacheAlignedAlloca(ty, b->getSize(blocks));
    }
}

void StaticBuffer::releaseBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) const {
    if (mAddressSpace == 0) {
        b->CreateFree(mStreamSetHandle);
    }
}

inline bool isCapacityGuaranteed(const Value * const index, const size_t capacity) {
    if (capacity == 0) {
        return true;
    } else if (isa<ConstantInt>(index)) {
        return cast<ConstantInt>(index)->getLimitedValue() < capacity;
    }
    return false;
}

Value * StaticBuffer::modByCapacity(IDISA::IDISA_Builder * const b, Value * const offset) const {
    assert (offset->getType()->isIntegerTy());
    if (isCapacityGuaranteed(offset, mCapacity)) {
        return offset;
    } else if (mCapacity == 1) {
        return ConstantInt::getNullValue(offset->getType());
    } else if (is_power_2(mCapacity)) {
        return b->CreateAnd(offset, ConstantInt::get(offset->getType(), mCapacity - 1));
    } else {
        return b->CreateURem(offset, ConstantInt::get(offset->getType(), mCapacity));
    }
}

Value * StaticBuffer::getCapacity(IDISA::IDISA_Builder * const b, Value * const /* handle */) const {
    return b->getSize(mCapacity * b->getBitBlockWidth());
}

void StaticBuffer::setCapacity(IDISA::IDISA_Builder * const /* b */, Value * /* handle */, Value * /* c */) const {
    report_fatal_error("setCapacity is not supported by this buffer type");
}

Value * StaticBuffer::getBaseAddress(IDISA::IDISA_Builder * const b, Value * const handle) const {
    if (codegen::DebugOptionIsSet(codegen::EnableAsserts)) {
        b->CreateAssert(handle, "handle cannot be null");
    }
    return handle;
}

void StaticBuffer::setBaseAddress(IDISA::IDISA_Builder * const /* b */, Value * /* addr */, Value * /* handle */) const {
    report_fatal_error("setBaseAddress is not supported by this buffer type");
}

Value * StaticBuffer::getOverflowAddress(IDISA::IDISA_Builder * const b, Value * const handle) const {
    return b->CreateGEP(getBaseAddress(b, handle), b->getSize(mCapacity));
}

Value * StaticBuffer::getStreamBlockPtr(IDISA::IDISA_Builder * const b, Value * const handle, Value * streamIndex, Value * blockIndex, const bool /* readOnly */) const {
    if (codegen::DebugOptionIsSet(codegen::EnableAsserts)) {
        Value * const count = getStreamSetCount(b, handle);
        Value * const index = b->CreateZExtOrTrunc(streamIndex, count->getType());
        Value * const cond = b->CreateICmpULT(index, count);
        b->CreateAssert(cond, "out-of-bounds stream access");
    }
    return b->CreateGEP(getBaseAddress(b, handle), {modByCapacity(b, blockIndex), streamIndex});
}

Value * StaticBuffer::getStreamPackPtr(IDISA::IDISA_Builder * const b, Value * const handle, Value * streamIndex, Value * blockIndex, Value * packIndex, const bool /* readOnly */) const {
    if (codegen::DebugOptionIsSet(codegen::EnableAsserts)) {
        Value * const count = getStreamSetCount(b, handle);
        Value * const index = b->CreateZExtOrTrunc(streamIndex, count->getType());
        Value * const cond = b->CreateICmpULT(index, count);
        b->CreateAssert(cond, "out-of-bounds stream access");
    }
    return b->CreateGEP(getBaseAddress(b, handle), {modByCapacity(b, blockIndex), streamIndex, packIndex});
}

Value * StaticBuffer::getRawItemPointer(IDISA::IDISA_Builder * const b, Value * const handle, Value * absolutePosition) const {
    Value * ptr = getBaseAddress(b, handle);
    Value * relativePosition = b->CreateURem(absolutePosition, ConstantInt::get(absolutePosition->getType(), mCapacity * b->getBitBlockWidth()));
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

Value * StaticBuffer::getLinearlyAccessibleItems(IDISA::IDISA_Builder * const b, Value * const handle, Value * fromPosition, Value * availItems, bool reverse) const {
    Type * const ty = fromPosition->getType();
    const auto blockWidth = b->getBitBlockWidth();
    Value * capacity = getCapacity(b, handle);
    Value * const itemsFromBase = b->CreateURem(fromPosition, capacity);
    if (reverse) {
        Value * const bufAvail = b->CreateSelect(b->CreateIsNull(itemsFromBase), capacity, itemsFromBase);
        return b->CreateUMin(availItems, bufAvail);
    } else {
        if (mOverflow) {
            capacity = ConstantInt::get(ty, (mCapacity + mOverflow) * blockWidth - 1);
        }
        Value * const linearSpace = b->CreateSub(capacity, itemsFromBase);
        return b->CreateUMin(availItems, linearSpace);
    }
}

Value * StaticBuffer::getLinearlyWritableItems(IDISA::IDISA_Builder * const b, Value * const handle, Value * fromPosition, Value * consumed, bool reverse) const {
    Type * const ty = fromPosition->getType();
    const auto blockWidth = b->getBitBlockWidth();
    Value * capacity = getCapacity(b, handle);
    fromPosition = b->CreateURem(fromPosition, capacity);
    if (reverse) {
        return b->CreateSelect(b->CreateIsNull(fromPosition), capacity, fromPosition);
    }
    consumed = b->CreateURem(consumed, capacity);
    if (mOverflow) {
        capacity = ConstantInt::get(ty, (mCapacity + mOverflow) * blockWidth - 1);
    }
    Value * const limit = b->CreateSelect(b->CreateICmpULE(consumed, fromPosition), capacity, consumed);
    return b->CreateSub(limit, fromPosition);
}


// Dynamic Buffer

inline StructType * getDynamicBufferStructType(const std::unique_ptr<kernel::KernelBuilder> & b, Type * baseType, const unsigned addrSpace) {
    IntegerType * sizeTy = b->getSizeTy();
    PointerType * typePtr = baseType->getPointerTo(addrSpace);
    return StructType::get(b->getContext(), {typePtr, typePtr, sizeTy, sizeTy});
}

void DynamicBuffer::allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) {
    Type * const structTy = getDynamicBufferStructType(b, mType, mAddressSpace);
    Value * const handle = b->CreateCacheAlignedAlloca(structTy);
    Constant * const capacity = b->getSize(mInitialCapacity * b->getBitBlockWidth());
    // note: when adding extensible stream sets, make sure to set the initial count here.
    Value * const bufferSize = b->CreateRoundUp(getAllocationSize(b.get(), handle, capacity), b->getSize(b->getCacheAlignment()));;
    Value * const baseAddressField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(BaseAddress)});
    Type * const baseAddressPtrTy = baseAddressField->getType()->getPointerElementType();
    Value * const baseAddress = b->CreatePointerCast(b->CreateCacheAlignedMalloc(bufferSize), baseAddressPtrTy);
    if (codegen::DebugOptionIsSet(codegen::TraceDynamicBuffers)) {
        b->CallPrintInt("allocated: ", baseAddress);
        b->CallPrintInt("allocated capacity: ", bufferSize);
    }
    b->CreateStore(baseAddress, baseAddressField);
    Value * const priorAddressField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(PriorBaseAddress)});
    b->CreateStore(ConstantPointerNull::getNullValue(baseAddressPtrTy), priorAddressField);
    Value * const capacityField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(Capacity)});
    b->CreateStore(b->getSize(mInitialCapacity), capacityField);
    mStreamSetHandle = handle;
}

void DynamicBuffer::releaseBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) const {
    Value * const handle = mStreamSetHandle;
    /* Free the dynamically allocated buffer, but not the stack-allocated buffer struct. */
    Value * priorAddressField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(PriorBaseAddress)});
    b->CreateFree(b->CreateLoad(priorAddressField));
    Value * baseAddressField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(BaseAddress)});
    b->CreateFree(b->CreateLoad(baseAddressField));
}

void DynamicBuffer::setBaseAddress(IDISA::IDISA_Builder * const /* b */, Value * /* addr */, Value * /* handle */) const {
    report_fatal_error("setBaseAddress is not supported by this buffer type");
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
    Value * const workingBlocks = b->CreateLoad(b->CreateGEP(handle, {b->getInt32(0), b->getInt32(Capacity)}));
    assert (blockIndex->getType() == workingBlocks->getType());
    return b->CreateGEP(getBaseAddress(b, handle), b->CreateURem(blockIndex, workingBlocks));
}

Value * DynamicBuffer::getOverflowAddress(IDISA::IDISA_Builder * const b, Value * const handle) const {
    Value * const workingBlocks = b->CreateLoad(b->CreateGEP(handle, {b->getInt32(0), b->getInt32(Capacity)}));
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

Value * DynamicBuffer::getLinearlyAccessibleItems(IDISA::IDISA_Builder * const b, Value * const handle, Value * fromPosition, Value * availItems, bool reverse) const {
    Value * const bufferSize = getCapacity(b, handle);
    Value * const itemsFromBase = b->CreateURem(fromPosition, bufferSize);
    if (reverse) {
        Value * const bufAvail = b->CreateSelect(b->CreateIsNull(itemsFromBase), bufferSize, itemsFromBase);
        return b->CreateUMin(availItems, bufAvail);
    } else {
        Value * capacity = bufferSize;
        if (mOverflow) {
            Constant * const overflow = b->getSize(mOverflow * b->getBitBlockWidth() - 1);
            capacity = b->CreateAdd(bufferSize, overflow);
        }
        Value * const linearSpace = b->CreateSub(capacity, itemsFromBase);
        return b->CreateUMin(availItems, linearSpace);
    }
}

Value * DynamicBuffer::getLinearlyWritableItems(IDISA::IDISA_Builder * const b, Value * const handle, Value * fromPosition, Value * consumed, bool reverse) const {
    Value * const bufferSize = getCapacity(b, handle);
    fromPosition = b->CreateURem(fromPosition, bufferSize);
    if (reverse) {
        return b->CreateSelect(b->CreateIsNull(fromPosition), bufferSize, fromPosition);
    }
    consumed = b->CreateURem(consumed, bufferSize);
    Value * capacity = bufferSize;
    if (mOverflow) {
        Constant * const overflow = b->getSize(mOverflow * b->getBitBlockWidth() - 1);
        capacity = b->CreateAdd(bufferSize, overflow);
    }
    Value * const limit = b->CreateSelect(b->CreateICmpULE(consumed, fromPosition), capacity, consumed);
    return b->CreateSub(limit, fromPosition);
}

Value * DynamicBuffer::getCapacity(IDISA::IDISA_Builder * const b, Value * const handle) const {
    Value * ptr = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(Capacity)});
    return b->CreateMul(b->CreateLoad(ptr), b->getSize(b->getBitBlockWidth()));
}

void DynamicBuffer::setCapacity(IDISA::IDISA_Builder * const b, Value * handle, Value * required) const {

    BasicBlock * const entry = b->GetInsertBlock(); assert (entry);
    BasicBlock * const insertBefore = entry->getNextNode();
    BasicBlock * const expand = b->CreateBasicBlock("expandDynamicBuffer", insertBefore);
    BasicBlock * const resume = b->CreateBasicBlock("", insertBefore);

    ConstantInt * const BLOCK_WIDTH = b->getSize(b->getBitBlockWidth());

    Value * const capacityField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(Capacity)});
    Value * const capacity = b->CreateMul(capacityField, BLOCK_WIDTH);
    Value * const needsExpansion = b->CreateICmpULT(capacity, required);
    b->CreateUnlikelyCondBr(needsExpansion, expand, resume);

    b->SetInsertPoint(expand);
    Value * const newCapacity = b->CreateRoundUp(required, capacity);
    Value * const bufferSize = getAllocationSize(b, handle, newCapacity);
    Value * const newBaseAddress = b->CreateCacheAlignedMalloc(bufferSize);
    Value * const baseAddressField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(BaseAddress)});
    Value * const currentBaseAddress = b->CreateLoad(baseAddressField);
    Value * const priorBaseAddressField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(PriorBaseAddress)});
    Value * const priorBaseAddress = b->CreateLoad(priorBaseAddressField);
    b->CreateMemCpy(newBaseAddress, currentBaseAddress, capacity, b->getCacheAlignment());
    b->CreateStore(baseAddressField, newBaseAddress);
    b->CreateStore(capacityField, b->CreateUDiv(newCapacity, BLOCK_WIDTH));
    b->CreateStore(priorBaseAddressField, currentBaseAddress);
    b->CreateFree(priorBaseAddress);

    b->CreateBr(resume);
    b->SetInsertPoint(resume);
}

Value * DynamicBuffer::getAllocationSize(IDISA::IDISA_Builder * const b, Value * handle, Value * const requiredItemCapacity) const {
    Value * itemCapacity = requiredItemCapacity;
    if (mOverflow) {
        Constant * const overflowSize =  b->getSize(mOverflow * b->getBitBlockWidth());
        itemCapacity = b->CreateAdd(requiredItemCapacity, overflowSize);
    }
    Value * const numOfStreams = getStreamSetCount(b, handle);
    Value * bufferSize = b->CreateMul(itemCapacity, numOfStreams);
    const auto fieldWidth = mBaseType->getArrayElementType()->getScalarSizeInBits();
    if (LLVM_LIKELY(fieldWidth < 8)) {
        bufferSize = b->CreateUDiv(bufferSize, b->getSize(8 / fieldWidth));
    } else if (LLVM_UNLIKELY(fieldWidth > 8)) {
        bufferSize = b->CreateMul(bufferSize, b->getSize(fieldWidth / 8));
    }
    return bufferSize;
}

ExternalBuffer::ExternalBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, Type * const type,
                               Value * const externalAddress, const unsigned AddressSpace)
: StreamSetBuffer(BufferKind::ExternalBuffer, b, type, AddressSpace)
, mExternalAddress(externalAddress) {
    mUniqueID = "E";
    if (AddressSpace > 0) mUniqueID += "@" + std::to_string(AddressSpace);
}

StaticBuffer::StaticBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, Type * const type,
                           const size_t capacity, const size_t overflowBlocks, const unsigned AddressSpace)
: StreamSetBuffer(BufferKind::StaticBuffer, b, type, AddressSpace)
, mCapacity(capacity)
, mOverflow(overflowBlocks) {
    mUniqueID = "S" + std::to_string(capacity);
    if (AddressSpace > 0) mUniqueID += "@" + std::to_string(AddressSpace);
}

DynamicBuffer::DynamicBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, Type * const type,
                             const size_t initialCapacity, const size_t overflowBlocks, const unsigned AddressSpace)
: StreamSetBuffer(BufferKind::DynamicBuffer, b, type, AddressSpace)
, mInitialCapacity(initialCapacity)
, mOverflow(overflowBlocks) {
    mUniqueID = "D";
    if (overflowBlocks != 0) mUniqueID += std::to_string(overflowBlocks);
    if (AddressSpace != 0) mUniqueID += "@" + std::to_string(AddressSpace);
}

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


inline StreamSetBuffer::StreamSetBuffer(BufferKind k, const std::unique_ptr<kernel::KernelBuilder> & b, Type * baseType, unsigned AddressSpace)
: mBufferKind(k)
, mType(resolveStreamSetType(b, baseType))
, mAddressSpace(AddressSpace)
, mStreamSetHandle(nullptr)
, mBaseType(baseType)
, mProducer(nullptr) {

}

StreamSetBuffer::~StreamSetBuffer() { }
