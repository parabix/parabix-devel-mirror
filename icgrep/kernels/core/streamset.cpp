/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "streamset.h"
#include "kernel.h"
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_ostream.h>
#include <kernels/kernel_builder.h>
#include <toolchain/toolchain.h>
#include <llvm/Support/Debug.h>
#include <llvm/Support/Format.h>
#include <array>

namespace llvm { class Constant; }
namespace llvm { class Function; }

using namespace llvm;
using IDISA::IDISA_Builder;

namespace kernel {

inline Value * StreamSetBuffer::getHandle(IDISA_Builder * const /* b */) const {
    return mHandle;
}

void StreamSetBuffer::setHandle(const std::unique_ptr<kernel::KernelBuilder> & b, Value * const handle) {
    assert ("handle cannot be null!" && handle);
    assert ("handle is not of the correct type" && handle->getType() == getHandlePointerType(b));
    #ifndef NDEBUG
    const Function * const handleFunction = isa<Argument>(handle) ? cast<Argument>(handle)->getParent() : cast<Instruction>(handle)->getParent()->getParent();
    const Function * const builderFunction = b->GetInsertBlock()->getParent();
    assert ("handle is not from the current function." && (handleFunction == builderFunction));
    #endif
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        b->CreateAssert(handle, "handle cannot be null!");
    }
    mHandle = handle;
}

inline void StreamSetBuffer::assertValidStreamIndex(IDISA_Builder * const b, Value * streamIndex) const {
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        Value * const count = getStreamSetCount(b);
        Value * const withinSet = b->CreateICmpULT(b->CreateZExtOrTrunc(streamIndex, count->getType()), count);
        b->CreateAssert(withinSet, "out-of-bounds stream access");
    }
}

Value * StreamSetBuffer::getStreamBlockPtr(IDISA_Builder * const b, llvm::Value * const baseAddress, Value * const streamIndex, Value * const blockIndex) const {
    assertValidStreamIndex(b, streamIndex);
    return b->CreateGEP(baseAddress, {blockIndex, streamIndex});
}

Value * StreamSetBuffer::getStreamPackPtr(IDISA_Builder * const b, llvm::Value * const baseAddress, Value * const streamIndex, Value * const blockIndex, Value * const packIndex) const {
    assertValidStreamIndex(b, streamIndex);
    return b->CreateGEP(baseAddress, {blockIndex, streamIndex, packIndex});
}

Value * StreamSetBuffer::getStreamSetCount(IDISA_Builder * const b) const {
    size_t count = 1;
    if (isa<ArrayType>(getBaseType())) {
        count = getBaseType()->getArrayNumElements();
    }
    return b->getSize(count);
}

/**
 * @brief getRawItemPointer
 *
 * get a raw pointer the iN field at position absoluteItemPosition of the stream number streamIndex of the stream set.
 * In the case of a stream whose fields are less than one byte (8 bits) in size, the pointer is to the containing byte.
 * The type of the pointer is i8* for fields of 8 bits or less, otherwise iN* for N-bit fields.
 */
Value * StreamSetBuffer::getRawItemPointer(IDISA_Builder * const b, Value * streamIndex, Value * absolutePosition) const {
    Value * ptr = getBaseAddress(b);
    Type * itemTy = mBaseType->getArrayElementType()->getVectorElementType();
    const auto itemWidth = itemTy->getPrimitiveSizeInBits();
    assert (is_power_2(itemWidth));
    IntegerType * const sizeTy = b->getSizeTy();
    streamIndex = b->CreateZExt(streamIndex, sizeTy);
    absolutePosition = b->CreateZExt(absolutePosition, sizeTy);

    if (LLVM_UNLIKELY(itemWidth < 8)) {
        Constant * const itemsPerByte = b->getSize(8 / itemWidth);
        if (codegen::DebugOptionIsSet(codegen::EnableAsserts)) {
            b->CreateAssertZero(b->CreateURem(absolutePosition, itemsPerByte), "absolutePosition must be byte aligned");
        }
        absolutePosition = b->CreateUDiv(absolutePosition, itemsPerByte);
        itemTy = b->getInt8Ty();
    }
    Constant * const itemsPerVector = ConstantExpr::getUDiv(ConstantExpr::getSizeOf(mType), ConstantExpr::getSizeOf(itemTy));
    Value * const blockOffset = b->CreateMul(b->CreateRoundDown(absolutePosition, itemsPerVector), getStreamSetCount(b));
    Value * const streamOffset = b->CreateMul(streamIndex, itemsPerVector);
    Value * const itemOffset = b->CreateURem(absolutePosition, itemsPerVector);
    Value * const position = b->CreateAdd(b->CreateAdd(blockOffset, streamOffset), itemOffset);
    return b->CreateGEP(b->CreatePointerCast(ptr, itemTy->getPointerTo()), position);
}

Value * StreamSetBuffer::addOverflow(const std::unique_ptr<kernel::KernelBuilder> & b, Value * const bufferCapacity, Value * const overflowItems, Value * const consumedOffset) const {
    if (overflowItems) {
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
            Value * const overflowCapacity = b->getSize(getOverflowCapacity(b));
            Value * const valid = b->CreateICmpULE(overflowItems, overflowCapacity);
            b->CreateAssert(valid, "overflow items exceeds overflow capacity");
        }
        // limit the overflow so that we do not overwrite our unconsumed data during a copyback
        Value * const effectiveOverflow = b->CreateUMin(consumedOffset, overflowItems);
        return b->CreateAdd(bufferCapacity, effectiveOverflow);
    } else { // no overflow
        return bufferCapacity;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief resolveType
 ** ------------------------------------------------------------------------------------------------------------- */
Type * StreamSetBuffer::resolveType(const std::unique_ptr<kernel::KernelBuilder> & b, Type * const streamSetType) {
    unsigned numElements = 1;
    Type * type = streamSetType;
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
    streamSetType->print(out);
    out << " is an unvalid stream set buffer type.";
    report_fatal_error(out.str());
}


// External File Buffer
Type * ExternalBuffer::getHandleType(const std::unique_ptr<kernel::KernelBuilder> & b) const {
    PointerType * const ptrTy = getPointerType();
    IntegerType * const sizeTy = b->getSizeTy();
    return StructType::get(b->getContext(), {ptrTy, sizeTy});
}

void ExternalBuffer::allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) {
    report_fatal_error("allocateBuffer is not supported by external buffers");
}

void ExternalBuffer::releaseBuffer(const std::unique_ptr<kernel::KernelBuilder> & /* b */) const {
    // this buffer is not responsible for free-ing th data associated with it
}

void ExternalBuffer::setBaseAddress(IDISA_Builder * const b, Value * const addr) const {
    assert (mHandle && "has not been set prior to calling setBaseAddress");
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        b->CreateAssert(addr, "base address cannot be null");
    }
    Value * const p = b->CreateGEP(getHandle(b), {b->getInt32(0), b->getInt32(BaseAddress)});
    b->CreateStore(b->CreatePointerBitCastOrAddrSpaceCast(addr, getPointerType()), p);
}

Value * ExternalBuffer::getBaseAddress(IDISA_Builder * const b) const {
    assert (mHandle && "has not been set prior to calling getBaseAddress");
    Value * const p = b->CreateGEP(getHandle(b), {b->getInt32(0), b->getInt32(BaseAddress)});
    return b->CreateLoad(p);
}

size_t ExternalBuffer::getOverflowCapacity(const std::unique_ptr<kernel::KernelBuilder> & b) const {
    return 0;
}

Value * ExternalBuffer::getOverflowAddress(IDISA_Builder * const /* b */) const {
    report_fatal_error("getOverflowAddress is not supported by external buffers");
}

void ExternalBuffer::setCapacity(IDISA_Builder * const b, Value * const capacity) const {
    assert (mHandle && "has not been set prior to calling setCapacity");
    Value *  const p = b->CreateGEP(getHandle(b), {b->getInt32(0), b->getInt32(Capacity)});
    b->CreateStore(b->CreateZExt(capacity, b->getSizeTy()), p);
}

Value * ExternalBuffer::getCapacity(IDISA_Builder * const b) const {
    assert (mHandle && "has not been set prior to calling getCapacity");
    Value * const p = b->CreateGEP(getHandle(b), {b->getInt32(0), b->getInt32(Capacity)});
    return b->CreateLoad(p);
}

Value * ExternalBuffer::getLinearlyAccessibleItems(const std::unique_ptr<KernelBuilder> & b, Value * const fromPosition, Value * const totalItems, Value * /* overflowItems */) const {
    return b->CreateSub(totalItems, fromPosition);
}

Value * ExternalBuffer::getLinearlyWritableItems(const std::unique_ptr<KernelBuilder> & b, Value * const fromPosition, Value * const /* consumed */, Value * /* overflowItems */) const {
    assert (fromPosition);
    Value * const capacity = getCapacity(b.get());
    assert (fromPosition->getType() == capacity->getType());
    return b->CreateSub(capacity, fromPosition);
}

inline void ExternalBuffer::assertValidBlockIndex(IDISA_Builder * const b, Value * blockIndex) const {
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        Value * const blockCount = b->CreateCeilUDiv(getCapacity(b), b->getSize(b->getBitBlockWidth()));
        blockIndex = b->CreateZExtOrTrunc(blockIndex, blockCount->getType());
        Value * const withinCapacity = b->CreateICmpULT(blockIndex, blockCount);
        b->CreateAssert(withinCapacity, "blockIndex exceeds buffer capacity");
    }
}

Value * ExternalBuffer::getStreamBlockPtr(IDISA_Builder * const b, llvm::Value * baseAddress, Value * const streamIndex, Value * const blockIndex) const {
    //assertValidBlockIndex(b, blockIndex);
    return StreamSetBuffer::getStreamBlockPtr(b, baseAddress, streamIndex, blockIndex);
}

Value * ExternalBuffer::getStreamPackPtr(IDISA_Builder * const b, llvm::Value * baseAddress, Value * const streamIndex, Value * const blockIndex, Value * const packIndex) const {
    //assertValidBlockIndex(b, blockIndex);
    return StreamSetBuffer::getStreamPackPtr(b, baseAddress, streamIndex, blockIndex, packIndex);
}

Value * ExternalBuffer::getStreamLogicalBasePtr(IDISA_Builder * const b, Value * baseAddress, Value * const streamIndex, Value * /* blockIndex */) const {
    return StreamSetBuffer::getStreamBlockPtr(b, baseAddress, streamIndex, b->getSize(0));
}

// Static Buffer
Type * StaticBuffer::getHandleType(const std::unique_ptr<kernel::KernelBuilder> & /* b */) const {
    return getPointerType();
}

void StaticBuffer::allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) {
    assert (mHandle && "has not been set prior to calling allocateBuffer");
    Value * const buffer = b->CreateCacheAlignedMalloc(getType(), b->getSize(mCapacity + mOverflow), mAddressSpace);
    b->CreateStore(buffer, mHandle);
}

LLVM_READNONE inline ConstantPointerNull * nullPointerFor(Value * ptr) {
    return ConstantPointerNull::get(cast<PointerType>(ptr->getType()));
}

void StaticBuffer::releaseBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) const {
    Value * buffer = b->CreateLoad(mHandle);
    b->CreateFree(buffer);
    b->CreateStore(nullPointerFor(buffer), mHandle);
}

inline bool isCapacityGuaranteed(const Value * const index, const size_t capacity) {
    return isa<ConstantInt>(index) ? cast<ConstantInt>(index)->getLimitedValue() < capacity : false;
}

Value * StaticBuffer::modByCapacity(IDISA_Builder * const b, Value * const offset) const {
    assert (offset->getType()->isIntegerTy());
    if (LLVM_UNLIKELY(isCapacityGuaranteed(offset, mCapacity))) {
        return offset;
    } else if (LLVM_UNLIKELY(mCapacity == 1)) {
        return ConstantInt::getNullValue(offset->getType());
    } else if (LLVM_LIKELY(is_power_2(mCapacity))) {
        return b->CreateAnd(offset, ConstantInt::get(offset->getType(), mCapacity - 1));
    } else {
        return b->CreateURem(offset, ConstantInt::get(offset->getType(), mCapacity));
    }
}

Value * StaticBuffer::getCapacity(IDISA_Builder * const b) const {
    return b->getSize(mCapacity * b->getBitBlockWidth());
}

void StaticBuffer::setCapacity(IDISA_Builder * const /* b */, Value * /* c */) const {
    report_fatal_error("setCapacity is not supported by static buffers");
}

Value * StaticBuffer::getBaseAddress(IDISA_Builder * const b) const {
    return b->CreateLoad(getHandle(b));
}

void StaticBuffer::setBaseAddress(IDISA_Builder * const /* b */, Value * /* addr */) const {
    report_fatal_error("setBaseAddress is not supported by static buffers");
}

size_t StaticBuffer::getOverflowCapacity(const std::unique_ptr<kernel::KernelBuilder> & b) const {
    return mOverflow * b->getBitBlockWidth();
}

Value * StaticBuffer::getOverflowAddress(IDISA_Builder * const b) const {
    return b->CreateGEP(getBaseAddress(b), b->getSize(mCapacity));
}

Value * StaticBuffer::getStreamBlockPtr(IDISA_Builder * const b, llvm::Value * baseAddress, Value * const streamIndex, Value * const blockIndex) const {
    return StreamSetBuffer::getStreamBlockPtr(b, baseAddress, streamIndex, modByCapacity(b, blockIndex));
}

Value * StaticBuffer::getStreamPackPtr(IDISA_Builder * const b, llvm::Value * baseAddress, Value * const streamIndex, Value * const blockIndex, Value * const packIndex) const {
    return StreamSetBuffer::getStreamPackPtr(b, baseAddress, streamIndex, modByCapacity(b, blockIndex), packIndex);
}

Value * StaticBuffer::getStreamLogicalBasePtr(IDISA_Builder * const b, llvm::Value * baseAddress, Value * const streamIndex, Value * const blockIndex) const {
    Value * const baseBlockIndex = b->CreateSub(modByCapacity(b, blockIndex), blockIndex);
    return StreamSetBuffer::getStreamBlockPtr(b, baseAddress, streamIndex, baseBlockIndex);
}

Value * StaticBuffer::getRawItemPointer(IDISA_Builder * const b, Value * streamIndex, Value * absolutePosition) const {
    return StreamSetBuffer::getRawItemPointer(b, streamIndex, b->CreateURem(absolutePosition, getCapacity(b)));
}

Value * StaticBuffer::getLinearlyAccessibleItems(const std::unique_ptr<KernelBuilder> & b, Value * const fromPosition, Value * const totalItems, Value * overflowItems) const {
    Value * const capacity = getCapacity(b.get());
    Value * const availableItems = b->CreateSub(totalItems, fromPosition);
    Value * const fromOffset = b->CreateURem(fromPosition, capacity);
    Value * const capacityWithOverflow = addOverflow(b, capacity, overflowItems);
    Value * const linearSpace = b->CreateSub(capacityWithOverflow, fromOffset);
    return b->CreateUMin(availableItems, linearSpace);
}

Value * StaticBuffer::getLinearlyWritableItems(const std::unique_ptr<KernelBuilder> & b, Value * const fromPosition, Value * const consumedItems, Value * overflowItems) const {
    Value * const capacity = getCapacity(b.get());
    Value * const unconsumedItems = b->CreateSub(fromPosition, consumedItems);
    Value * const full = b->CreateICmpUGE(unconsumedItems, capacity);
    Value * const fromOffset = b->CreateURem(fromPosition, capacity);
    Value * const consumedOffset = b->CreateURem(consumedItems, capacity);
    Value * const toEnd = b->CreateICmpULE(consumedOffset, fromOffset);
    Value * const capacityWithOverflow = addOverflow(b, capacity, overflowItems, consumedOffset);
    Value * const limit = b->CreateSelect(toEnd, capacityWithOverflow, consumedOffset);
    Value * const remaining = b->CreateSub(limit, fromOffset);
    return b->CreateSelect(full, b->getSize(0), remaining);
}

// Dynamic Buffer
Type * DynamicBuffer::getHandleType(const std::unique_ptr<kernel::KernelBuilder> & b) const {
    PointerType * typePtr = getPointerType();
    IntegerType * sizeTy = b->getSizeTy();
    return StructType::get(b->getContext(), {typePtr, typePtr, sizeTy});
}

void DynamicBuffer::allocateBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) {
    assert (mHandle && "has not been set prior to calling allocateBuffer");
    Constant * const capacity = b->getSize(mInitialCapacity * b->getBitBlockWidth());
    // note: when adding extensible stream sets, make sure to set the initial count here.
    Value * const bufferSize = b->CreateRoundUp(getAllocationSize(b.get(), capacity, mOverflow), b->getSize(b->getCacheAlignment()));
    Value * const baseAddressField = b->CreateGEP(mHandle, {b->getInt32(0), b->getInt32(BaseAddress)});
    Type * const baseAddressPtrTy = baseAddressField->getType()->getPointerElementType();
    Value * const baseAddress = b->CreatePointerCast(b->CreateCacheAlignedMalloc(bufferSize), baseAddressPtrTy);
    if (codegen::DebugOptionIsSet(codegen::TraceDynamicBuffers)) {
        b->CallPrintInt("allocated: ", baseAddress);
        b->CallPrintInt("allocated capacity: ", bufferSize);
    }
    b->CreateStore(baseAddress, baseAddressField);
    Value * const priorAddressField = b->CreateGEP(mHandle, {b->getInt32(0), b->getInt32(PriorBaseAddress)});
    b->CreateStore(ConstantPointerNull::getNullValue(baseAddressPtrTy), priorAddressField);
    Value * const capacityField = b->CreateGEP(mHandle, {b->getInt32(0), b->getInt32(Capacity)});
    b->CreateStore(b->getSize(mInitialCapacity), capacityField);
}

void DynamicBuffer::releaseBuffer(const std::unique_ptr<kernel::KernelBuilder> & b) const {
    /* Free the dynamically allocated buffer(s). */
    Value * const handle = getHandle(b.get());
    Value * priorAddressField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(PriorBaseAddress)});
    Value * priorAddress = b->CreateLoad(priorAddressField);
    b->CreateFree(priorAddress);
    b->CreateStore(nullPointerFor(priorAddress), priorAddressField);
    Value * baseAddressField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(BaseAddress)});
    Value * baseAddress = b->CreateLoad(baseAddressField);
    b->CreateFree(baseAddress);
    b->CreateStore(nullPointerFor(baseAddress), baseAddressField);
}

void DynamicBuffer::setBaseAddress(IDISA_Builder * const /* b */, Value * /* addr */) const {
    report_fatal_error("setBaseAddress is not supported by DynamicBuffers");
}

Value * DynamicBuffer::getBaseAddress(IDISA_Builder * const b) const {
    Value * const ptr = b->CreateGEP(getHandle(b), {b->getInt32(0), b->getInt32(BaseAddress)});
    return b->CreateLoad(ptr);
}

size_t DynamicBuffer::getOverflowCapacity(const std::unique_ptr<kernel::KernelBuilder> & b) const {
    return mOverflow * b->getBitBlockWidth();
}

Value * DynamicBuffer::getOverflowAddress(IDISA_Builder * const b) const {
    Value * const capacityPtr = b->CreateGEP(getHandle(b), {b->getInt32(0), b->getInt32(Capacity)});
    Value * const capacity = b->CreateLoad(capacityPtr);
    return b->CreateGEP(getBaseAddress(b), capacity);
}

Value * DynamicBuffer::modByCapacity(IDISA_Builder * const b, Value * const offset) const {
    assert (offset->getType()->isIntegerTy());
    if (isCapacityGuaranteed(offset, mInitialCapacity)) {
        return offset;
    } else {
        Value * const capacityPtr = b->CreateGEP(getHandle(b), {b->getInt32(0), b->getInt32(Capacity)});
        Value * const capacity = b->CreateLoad(capacityPtr);
        return b->CreateURem(b->CreateZExtOrTrunc(offset, capacity->getType()), capacity);
    }
}

Value * DynamicBuffer::getStreamBlockPtr(IDISA_Builder * const b, llvm::Value * baseAddress, Value * const streamIndex, Value * const blockIndex) const {
    return StreamSetBuffer::getStreamBlockPtr(b, baseAddress, streamIndex, modByCapacity(b, blockIndex));
}

Value * DynamicBuffer::getStreamPackPtr(IDISA_Builder * const b, llvm::Value * baseAddress, Value * const streamIndex, Value * const blockIndex, Value * const packIndex) const {
    return StreamSetBuffer::getStreamPackPtr(b, baseAddress, streamIndex, modByCapacity(b, blockIndex), packIndex);
}

Value * DynamicBuffer::getStreamLogicalBasePtr(IDISA_Builder * const b, llvm::Value * baseAddress, Value * const streamIndex, Value * const blockIndex) const {
    Value * const baseBlockIndex = b->CreateSub(modByCapacity(b, blockIndex), blockIndex);
    return StreamSetBuffer::getStreamBlockPtr(b, baseAddress, streamIndex, baseBlockIndex);
}

Value * DynamicBuffer::getRawItemPointer(IDISA_Builder * const b, Value * streamIndex, Value * absolutePosition) const {
    return StreamSetBuffer::getRawItemPointer(b, streamIndex, b->CreateURem(absolutePosition, getCapacity(b)));
}

Value * DynamicBuffer::getLinearlyAccessibleItems(const std::unique_ptr<KernelBuilder> &b, Value * const fromPosition, Value * const totalItems, Value * overflowItems) const {
    Value * const capacity = getCapacity(b.get());
    Value * const availableItems = b->CreateSub(totalItems, fromPosition);
    Value * const fromOffset = b->CreateURem(fromPosition, capacity);
    Value * const capacityWithOverflow = addOverflow(b, capacity, overflowItems);
    Value * const linearSpace = b->CreateSub(capacityWithOverflow, fromOffset);
    return b->CreateUMin(availableItems, linearSpace);
}

Value * DynamicBuffer::getLinearlyWritableItems(const std::unique_ptr<KernelBuilder> & b, Value * const fromPosition, Value * const consumedItems, Value * overflowItems) const {
    Value * const capacity = getCapacity(b.get());
    Value * const unconsumedItems = b->CreateSub(fromPosition, consumedItems);
    Value * const full = b->CreateICmpUGE(unconsumedItems, capacity);
    Value * const fromOffset = b->CreateURem(fromPosition, capacity);
    Value * const consumedOffset = b->CreateURem(consumedItems, capacity);
    Value * const toEnd = b->CreateICmpULE(consumedOffset, fromOffset);
    Value * const capacityWithOverflow = addOverflow(b, capacity, overflowItems, consumedOffset);
    Value * const limit = b->CreateSelect(toEnd, capacityWithOverflow, consumedOffset);
    Value * const remaining = b->CreateSub(limit, fromOffset);
    return b->CreateSelect(full, b->getSize(0), remaining);
}

Value * DynamicBuffer::getCapacity(IDISA_Builder * const b) const {
    Value * ptr = b->CreateGEP(getHandle(b), {b->getInt32(0), b->getInt32(Capacity)});
    return b->CreateMul(b->CreateLoad(ptr), b->getSize(b->getBitBlockWidth()));
}

void DynamicBuffer::setCapacity(IDISA_Builder * const b, Value * required) const {

    std::vector<Value *> indices(2);
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(Capacity);

    Constant * const LOG_2_BIT_BLOCK_WIDTH = b->getSize(std::log2(b->getBitBlockWidth()));
    Value * const handle = getHandle(b);
    Value * const capacityField = b->CreateGEP(handle, indices);
    Value * const capacity = b->CreateShl(b->CreateLoad(capacityField), LOG_2_BIT_BLOCK_WIDTH);
    Value * const newCapacity = b->CreateRoundUp(required, b->CreateShl(capacity, 1));
    Value * const newBufferSize = b->CreateRoundUp(getAllocationSize(b, newCapacity, mOverflow), b->getSize(b->getCacheAlignment()));
    Value * const newBaseAddress = b->CreateCacheAlignedMalloc(newBufferSize);
    indices[1] = b->getInt32(BaseAddress);
    Value * const baseAddressField = b->CreateGEP(handle, indices);
    Value * const currentBaseAddress = b->CreateLoad(baseAddressField);
    indices[1] = b->getInt32(PriorBaseAddress);
    Value * const priorBaseAddressField = b->CreateGEP(handle, indices);
    Value * const priorBaseAddress = b->CreateLoad(priorBaseAddressField);

    // Copy the data twice to handle the potential of a dynamic circular buffer. E.g., suppose p is the processed
    // item count of some kernel. Conceptually, all As after p will be processed before any B, despite the fact
    // that Bs are placed before the As in the buffer. When we double the size of the buffer, we double the modulus
    // of the circular buffer. By copying the data to both halves,

    //                          p
    // Current Buffer   |BBBBBBB|AAAAAAAAAAAAAAAAA|

    //                          p
    // New Buffer       |BBBBBBB|AAAAAAAAAAAAAAAAA|BBBBBBB|AAAAAAAAAAAAAAAAA|

    // TODO: what if this has to be more than doubled? original method just repeated the doubling process.

    Value * const bufferSize = getAllocationSize(b, capacity, 0);
    b->CreateMemCpy(newBaseAddress, currentBaseAddress, bufferSize, b->getCacheAlignment());
    Value * const expandedBaseAddress = b->CreateGEP(newBaseAddress, bufferSize);
    b->CreateMemCpy(expandedBaseAddress, currentBaseAddress, bufferSize, b->getCacheAlignment());

    b->CreateStore(b->CreatePointerCast(newBaseAddress, currentBaseAddress->getType()), baseAddressField);
    b->CreateStore(b->CreateLShr(newCapacity, LOG_2_BIT_BLOCK_WIDTH), capacityField);
    b->CreateStore(currentBaseAddress, priorBaseAddressField);
    b->CreateFree(priorBaseAddress);

}

#if 0

/**
 * @brief expandBuffer
 *
 * Expand the buffer, ensuring that we have at least the required space after the current produced offset and that
 * the total size of the buffer is at least 2x the current capacity.
 */
void DynamicBuffer::expandBuffer(const std::unique_ptr<KernelBuilder> & b, Value * const consumed, Value * const produced, Value * const required) const {

    ConstantInt * const ZERO = b->getSize(0);
    ConstantInt * const LOG_2_BIT_BLOCK_WIDTH = b->getSize(std::log2(b->getBitBlockWidth()));

    Value * const handle = getHandle(b.get());
    Value * const capacityField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(Capacity)});
    Value * const blockCapacity = b->CreateLoad(capacityField);
    Value * const currentCapacity = b->CreateShl(blockCapacity, LOG_2_BIT_BLOCK_WIDTH);

    //                             c       p
    // Current Buffer |CCCCCCCCCCCC|PPPPPPP|...|

    // New Buffer       |..........|PPPPPPP|RRRRRRRRRR|...................|

    Value * const stepSize = b->CreateRoundUp(required, b->getSize(b->getBitBlockWidth()));
    Value * const unconsumed = b->CreateSub(produced, consumed);
    Value * const expansion = b->CreateAdd(unconsumed, stepSize);
    Value * const baseNewCapacity = b->CreateRoundUp(expansion, currentCapacity);
    BasicBlock * const entryBlock = b->GetInsertBlock();
    BasicBlock * const calculate = b->CreateBasicBlock("capacityCalculation");
    BasicBlock * const expand = b->CreateBasicBlock("capacityExpansion");
    b->CreateBr(calculate);

    // ENSURE: (consumed % newCapacity) < (produced % newCapacity) && (produced % newCapacity) + required < newCapacity

    b->SetInsertPoint(calculate);
    PHINode * const newCapacity = b->CreatePHI(baseNewCapacity->getType(), 2);
    newCapacity->addIncoming(baseNewCapacity, entryBlock);
    Value * const consumedOffset = b->CreateURem(consumed, newCapacity);
    Value * const producedOffset = b->CreateURem(produced, newCapacity);
    Value * const dataIsArrangedCorrectly = b->CreateICmpULE(consumedOffset, producedOffset);
    Value * const hasEnoughSpace = b->CreateICmpULE(b->CreateAdd(producedOffset, stepSize), newCapacity);
    Value * const valid = b->CreateOr(dataIsArrangedCorrectly, hasEnoughSpace);
    Value * const newCapacity2 = b->CreateAdd(newCapacity, stepSize);
    newCapacity->addIncoming(newCapacity2, calculate);
    b->CreateLikelyCondBr(valid, expand, calculate);

    b->SetInsertPoint(expand);
    Value * const baseAddressField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(BaseAddress)});
    Value * const currentBaseAddress = b->CreateLoad(baseAddressField);
    Value * const priorBaseAddressField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(PriorBaseAddress)});
    Value * const priorBaseAddress = b->CreateLoad(priorBaseAddressField);

    Value * const allocationSize = getAllocationSize(b.get(), newCapacity, mOverflow);
    Value * const newBufferSize = b->CreateRoundUp(allocationSize, b->getSize(b->getCacheAlignment()));
    Value * const newBaseAddress = b->CreatePointerCast(b->CreateCacheAlignedMalloc(newBufferSize), currentBaseAddress->getType());
    Value * const sourceConsumedOffset = b->CreateURem(consumed, currentCapacity);
    Value * const sourceProducedOffset = b->CreateURem(produced, currentCapacity);

    BasicBlock * const copyLinear = b->CreateBasicBlock("copyLinear");
    BasicBlock * const copyNonLinear = b->CreateBasicBlock("copyNonLinear");
    BasicBlock * const storeNewBuffer = b->CreateBasicBlock("storeNewBuffer");

    Value * const consumedIndex = b->CreateLShr(consumedOffset, LOG_2_BIT_BLOCK_WIDTH);
    Value * const consumedAddr = b->CreateGEP(newBaseAddress, {consumedIndex, ZERO});

    Value * const sourceConsumedIndex = b->CreateLShr(sourceConsumedOffset, LOG_2_BIT_BLOCK_WIDTH);
    Value * const sourceProducedOffsetCeil = b->CreateAdd(sourceProducedOffset, b->getSize(b->getBitBlockWidth() - 1));
    Value * const sourceProducedIndex = b->CreateLShr(sourceProducedOffsetCeil, LOG_2_BIT_BLOCK_WIDTH);

    DataLayout DL(b->getModule());
    Type * const intPtrTy = DL.getIntPtrType(newBaseAddress->getType());

    Value * const sourceConsumedAddr = b->CreateGEP(currentBaseAddress, {sourceConsumedIndex, ZERO});
    Value * const sourceConsumedAddrInt = b->CreatePtrToInt(sourceConsumedAddr, intPtrTy);

    Value * const sourceProducedAddr = b->CreateGEP(currentBaseAddress, {sourceProducedIndex, ZERO});
    Value * const sourceProducedAddrInt = b->CreatePtrToInt(sourceProducedAddr, intPtrTy);

    Value * const initiallyValid = b->CreateICmpULE(sourceConsumedOffset, sourceProducedOffset);
    b->CreateLikelyCondBr(initiallyValid, copyLinear, copyNonLinear);

    b->SetInsertPoint(copyLinear); // consumed <= produced
    Value * const copyLength = b->CreateSub(sourceProducedAddrInt, sourceConsumedAddrInt);
    b->CreateMemCpy(consumedAddr, sourceConsumedAddr, copyLength, b->getCacheAlignment());
    b->CreateBr(storeNewBuffer);

    b->SetInsertPoint(copyNonLinear); // consumed > produced
    Value * const bufferEnd = b->CreateGEP(currentBaseAddress, {blockCapacity, ZERO});
    Value * const bufferEndInt = b->CreatePtrToInt(bufferEnd, intPtrTy);
    Value * const copyLength1 = b->CreateSub(bufferEndInt, sourceConsumedAddrInt);
    b->CreateMemCpy(consumedAddr, sourceConsumedAddr, copyLength1, b->getCacheAlignment());
    Constant * const elementSize = ConstantExpr::getSizeOf(consumedAddr->getType()->getPointerElementType());
    Value * const continuationIndex = b->CreateAdd(consumedIndex, b->CreateExactUDiv(copyLength1, elementSize));
    Value * const continuationAddr = b->CreateGEP(newBaseAddress, {continuationIndex, ZERO});
    Value * const baseAddressInt = b->CreatePtrToInt(currentBaseAddress, intPtrTy);
    Value * const copyLength2 = b->CreateSub(sourceProducedAddrInt, baseAddressInt);
    b->CreateMemCpy(continuationAddr, currentBaseAddress, copyLength2, b->getCacheAlignment());
    b->CreateBr(storeNewBuffer);

    b->SetInsertPoint(storeNewBuffer);
    b->CreateStore(newBaseAddress, baseAddressField);
    b->CreateStore(b->CreateLShr(newCapacity, LOG_2_BIT_BLOCK_WIDTH), capacityField);
    b->CreateStore(currentBaseAddress, priorBaseAddressField);
    b->CreateFree(priorBaseAddress);

}

#endif

Value * DynamicBuffer::getAllocationSize(IDISA_Builder * const b, Value * const requiredItemCapacity, const size_t overflow) const {
    Value * itemCapacity = requiredItemCapacity;
    if (overflow) {
        Constant * const overflowSize =  b->getSize(overflow * b->getBitBlockWidth());
        itemCapacity = b->CreateAdd(requiredItemCapacity, overflowSize);
    }
    Value * const numOfStreams = getStreamSetCount(b);
    Value * bufferSize = b->CreateMul(itemCapacity, numOfStreams);
    const auto fieldWidth = mBaseType->getArrayElementType()->getScalarSizeInBits();
    if (LLVM_LIKELY(fieldWidth < 8)) {
        bufferSize = b->CreateCeilUDiv(bufferSize, b->getSize(8 / fieldWidth));
    } else if (LLVM_UNLIKELY(fieldWidth > 8)) {
        bufferSize = b->CreateMul(bufferSize, b->getSize(fieldWidth / 8));
    }
    return bufferSize;
}

ExternalBuffer::ExternalBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, Type * const type,
                               const unsigned AddressSpace)
: StreamSetBuffer(BufferKind::ExternalBuffer, b, type, AddressSpace) {

}

StaticBuffer::StaticBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, Type * const type,
                           const size_t capacity, const size_t overflowSize, const unsigned AddressSpace)
: StreamSetBuffer(BufferKind::StaticBuffer, b, type, AddressSpace)
, mCapacity(capacity / b->getBitBlockWidth())
, mOverflow(overflowSize / b->getBitBlockWidth()) {
    assert ("static buffer cannot have 0 capacity" && capacity);
    assert ("static buffer capacity must be a multiple of bitblock width" && (capacity % b->getBitBlockWidth()) == 0);
    assert ("static buffer overflow must be a multiple of bitblock width" && (overflowSize % b->getBitBlockWidth()) == 0);
}

DynamicBuffer::DynamicBuffer(const std::unique_ptr<kernel::KernelBuilder> & b, Type * const type,
                             const size_t initialCapacity, const size_t overflowSize, const unsigned AddressSpace)
: StreamSetBuffer(BufferKind::DynamicBuffer, b, type, AddressSpace)
, mInitialCapacity(initialCapacity / b->getBitBlockWidth())
, mOverflow(overflowSize / b->getBitBlockWidth()) {
    assert ("dynamic buffer cannot have 0 initial capacity" && initialCapacity);
    assert ("dynamic buffer capacity must be a multiple of bitblock width" && (initialCapacity % b->getBitBlockWidth()) == 0);
    assert ("dynamic buffer overflow must be a multiple of bitblock width" && (overflowSize % b->getBitBlockWidth()) == 0);
}

StreamSetBuffer::StreamSetBuffer(const BufferKind k, const std::unique_ptr<kernel::KernelBuilder> & b,
                                 Type * const baseType, const unsigned AddressSpace)
: mBufferKind(k)
, mHandle(nullptr)
, mType(resolveType(b, baseType))
, mAddressSpace(AddressSpace)
, mBaseType(baseType) {

}

StreamSetBuffer::~StreamSetBuffer() { }

}
