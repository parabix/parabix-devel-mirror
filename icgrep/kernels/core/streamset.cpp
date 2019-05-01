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
using BuilderRef = const std::unique_ptr<kernel::KernelBuilder> &;

namespace kernel {

LLVM_ATTRIBUTE_NORETURN void unsupported(const char * const function, const char * const bufferType) {
    report_fatal_error(StringRef{function} + " is not supported by " + bufferType + "Buffers");
}

LLVM_READNONE inline Constant * nullPointerFor(BuilderRef b, Type * type, const unsigned underflow) {
    if (LLVM_LIKELY(underflow == 0)) {
        return ConstantPointerNull::get(cast<PointerType>(type));
    } else {
        DataLayout DL(b->getModule());
        Type * const intPtrTy = DL.getIntPtrType(type);
        Constant * const U = ConstantInt::get(intPtrTy, underflow);
        Constant * const P = ConstantExpr::getSizeOf(type->getPointerElementType());
        return ConstantExpr::getIntToPtr(ConstantExpr::getMul(U, P), type);
    }
}

LLVM_READNONE inline Constant * nullPointerFor(BuilderRef b, Value * ptr, const unsigned underflow) {
    return nullPointerFor(b, ptr->getType(), underflow);
}

LLVM_READNONE inline Value * addUnderflow(BuilderRef b, Value * ptr, const unsigned underflow) {
    if (LLVM_LIKELY(underflow == 0)) {
        return ptr;
    } else {
        DataLayout DL(b->getModule());
        Type * const intPtrTy = DL.getIntPtrType(ptr->getType());
        Constant * offset = ConstantInt::get(intPtrTy, underflow);
        return b->CreateGEP(ptr, offset);
    }
}

LLVM_READNONE inline Value * subtractUnderflow(BuilderRef b, Value * ptr, const unsigned underflow) {
    if (LLVM_LIKELY(underflow == 0)) {
        return ptr;
    } else {
        DataLayout DL(b->getModule());
        Type * const intPtrTy = DL.getIntPtrType(ptr->getType());
        Constant * offset = ConstantExpr::getNeg(ConstantInt::get(intPtrTy, underflow));
        return b->CreateGEP(ptr, offset);
    }
}

inline Value * StreamSetBuffer::getHandle(IDISA_Builder * const /* b */) const {
    return mHandle;
}

void StreamSetBuffer::setHandle(BuilderRef b, Value * const handle) const {
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

Value * StreamSetBuffer::getStreamBlockPtr(IDISA_Builder * const b, Value * const baseAddress, Value * const streamIndex, Value * const blockIndex) const {
    assertValidStreamIndex(b, streamIndex);
    return b->CreateGEP(baseAddress, {blockIndex, streamIndex});
}

Value * StreamSetBuffer::getStreamPackPtr(IDISA_Builder * const b, Value * const baseAddress, Value * const streamIndex, Value * blockIndex, Value * const packIndex) const {
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

size_t StreamSetBuffer::getUnderflowCapacity(BuilderRef b) const {
    return mUnderflow * b->getBitBlockWidth();
}

size_t StreamSetBuffer::getOverflowCapacity(BuilderRef b) const {
    return mOverflow * b->getBitBlockWidth();
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

    if (LLVM_LIKELY(itemWidth < 8)) {
        Constant * const itemsPerByte = b->getSize(8 / itemWidth);
        if (codegen::DebugOptionIsSet(codegen::EnableAsserts)) {
            b->CreateAssertZero(b->CreateURem(absolutePosition, itemsPerByte), "absolutePosition must be byte aligned");
        }
        absolutePosition = b->CreateUDiv(absolutePosition, itemsPerByte);
        itemTy = b->getInt8Ty();
    }
    Constant * const itemsPerVector = ConstantExpr::getUDiv(ConstantExpr::getSizeOf(mType), ConstantExpr::getSizeOf(itemTy));
    Value * blockOffset = b->CreateMul(b->CreateRoundDown(absolutePosition, itemsPerVector), getStreamSetCount(b));
    Value * const streamOffset = b->CreateMul(streamIndex, itemsPerVector);
    Value * const itemOffset = b->CreateURem(absolutePosition, itemsPerVector);
    Value * const position = b->CreateAdd(b->CreateAdd(blockOffset, streamOffset), itemOffset);
    return b->CreateGEP(b->CreatePointerCast(ptr, itemTy->getPointerTo()), position);
}

Value * StreamSetBuffer::addOverflow(BuilderRef b, Value * const bufferCapacity, Value * const overflowItems, Value * const consumedOffset) const {
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
Type * StreamSetBuffer::resolveType(BuilderRef b, Type * const streamSetType) {
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
Type * ExternalBuffer::getHandleType(BuilderRef b) const {
    PointerType * const ptrTy = getPointerType();
    IntegerType * const sizeTy = b->getSizeTy();
    return StructType::get(b->getContext(), {ptrTy, sizeTy});
}

void ExternalBuffer::allocateBuffer(BuilderRef /* b */) {
    unsupported("allocateBuffer", "External");
}

void ExternalBuffer::releaseBuffer(BuilderRef /* b */) const {
    // this buffer is not responsible for free-ing th data associated with it
}

void ExternalBuffer::setBaseAddress(IDISA_Builder * const b, Value * const addr) const {
    assert (mHandle && "has not been set prior to calling setBaseAddress");
    Value * const p = b->CreateGEP(getHandle(b), {b->getInt32(0), b->getInt32(BaseAddress)});
    b->CreateStore(b->CreatePointerBitCastOrAddrSpaceCast(addr, getPointerType()), p);
}

Value * ExternalBuffer::getBaseAddress(IDISA_Builder * const b) const {
    assert (mHandle && "has not been set prior to calling getBaseAddress");
    Value * const p = b->CreateGEP(getHandle(b), {b->getInt32(0), b->getInt32(BaseAddress)});
    return b->CreateLoad(p);
}

Value * ExternalBuffer::getOverflowAddress(IDISA_Builder * const /* b */) const {
    unsupported("getOverflowAddress", "External");
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

Value * ExternalBuffer::getLinearlyAccessibleItems(BuilderRef b, Value * const fromPosition, Value * const totalItems, Value * /* overflowItems */) const {
    return b->CreateSub(totalItems, fromPosition);
}

Value * ExternalBuffer::getLinearlyWritableItems(BuilderRef b, Value * const fromPosition, Value * const /* consumed */, Value * /* overflowItems */) const {
    assert (fromPosition);
    Value * const capacity = getCapacity(b.get());
    assert (fromPosition->getType() == capacity->getType());
    return b->CreateSub(capacity, fromPosition);
}

Value * ExternalBuffer::getStreamLogicalBasePtr(IDISA_Builder * const b, Value * baseAddress, Value * const streamIndex, Value * /* blockIndex */) const {
    return StreamSetBuffer::getStreamBlockPtr(b, baseAddress, streamIndex, b->getSize(0));
}

inline void ExternalBuffer::assertValidBlockIndex(IDISA_Builder * const b, Value * blockIndex) const {
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        Value * const blockCount = b->CreateCeilUDiv(getCapacity(b), b->getSize(b->getBitBlockWidth()));
        blockIndex = b->CreateZExtOrTrunc(blockIndex, blockCount->getType());
        Value * const withinCapacity = b->CreateICmpULT(blockIndex, blockCount);
        b->CreateAssert(withinCapacity, "blockIndex exceeds buffer capacity");
    }
}

Value * ExternalBuffer::reserveCapacity(BuilderRef /* b */, Value * /* produced */, Value * /* consumed */, Value * const /* required */, Constant * const /* overflowItems */) const  {
    unsupported("reserveCapacity", "External");
}

// Static Buffer
Type * StaticBuffer::getHandleType(BuilderRef /* b */) const {
    return getPointerType();
}

void StaticBuffer::allocateBuffer(BuilderRef b) {
    Value * const handle = getHandle(b.get());
    assert (handle && "has not been set prior to calling allocateBuffer");
    Constant * size = b->getSize(mCapacity + mUnderflow + mOverflow);
    Value * const buffer = b->CreateCacheAlignedMalloc(mType, size, mAddressSpace);
    b->CreateStore(addUnderflow(b, buffer, mUnderflow), handle);
}

void StaticBuffer::releaseBuffer(BuilderRef b) const {
    Value * const handle = getHandle(b.get());
    Value * buffer = b->CreateLoad(handle);
    b->CreateFree(subtractUnderflow(b, buffer, mUnderflow));
    b->CreateStore(nullPointerFor(b, buffer, mUnderflow), handle);
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
    unsupported("setCapacity", "Static");
}

Value * StaticBuffer::getBaseAddress(IDISA_Builder * const b) const {
    return b->CreateLoad(getHandle(b));
}

void StaticBuffer::setBaseAddress(IDISA_Builder * const /* b */, Value * /* addr */) const {
    unsupported("setBaseAddress", "Static");
}

Value * StaticBuffer::getOverflowAddress(IDISA_Builder * const b) const {
    return b->CreateGEP(getBaseAddress(b), b->getSize(mCapacity));
}

Value * StaticBuffer::getStreamBlockPtr(IDISA_Builder * const b, Value * baseAddress, Value * const streamIndex, Value * const blockIndex) const {
    return StreamSetBuffer::getStreamBlockPtr(b, baseAddress, streamIndex, modByCapacity(b, blockIndex));
}

Value * StaticBuffer::getStreamPackPtr(IDISA_Builder * const b, Value * baseAddress, Value * const streamIndex, Value * const blockIndex, Value * const packIndex) const {
    return StreamSetBuffer::getStreamPackPtr(b, baseAddress, streamIndex, modByCapacity(b, blockIndex), packIndex);
}

Value * StaticBuffer::getStreamLogicalBasePtr(IDISA_Builder * const b, Value * baseAddress, Value * const streamIndex, Value * const blockIndex) const {
    Value * const baseBlockIndex = b->CreateSub(modByCapacity(b, blockIndex), blockIndex);
    return StreamSetBuffer::getStreamBlockPtr(b, baseAddress, streamIndex, baseBlockIndex);
}

Value * StaticBuffer::getRawItemPointer(IDISA_Builder * const b, Value * streamIndex, Value * absolutePosition) const {
    return StreamSetBuffer::getRawItemPointer(b, streamIndex, b->CreateURem(absolutePosition, getCapacity(b)));
}

Value * StaticBuffer::getLinearlyAccessibleItems(BuilderRef b, Value * const fromPosition, Value * const totalItems, Value * overflowItems) const {
    Value * const capacity = getCapacity(b.get());
    Value * const availableItems = b->CreateSub(totalItems, fromPosition);
    Value * const fromOffset = b->CreateURem(fromPosition, capacity);
    Value * const capacityWithOverflow = addOverflow(b, capacity, overflowItems, nullptr);
    Value * const linearSpace = b->CreateSub(capacityWithOverflow, fromOffset);
    return b->CreateUMin(availableItems, linearSpace);
}

Value * StaticBuffer::getLinearlyWritableItems(BuilderRef b, Value * const fromPosition, Value * const consumedItems, Value * overflowItems) const {
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

Value * StaticBuffer::reserveCapacity(BuilderRef /* b */, Value * /* produced */, Value * /* consumed */, Value * const /* required */, Constant * const /* overflowItems */) const  {
    unsupported("reserveCapacity", "Static");
}

// Dynamic Buffer
Type * DynamicBuffer::getHandleType(BuilderRef b) const {
    PointerType * const typePtr = getPointerType();
    IntegerType * const sizeTy = b->getSizeTy();
    FixedArray<Type *, 3> types;
    types[BaseAddress] = typePtr;
    types[Capacity] = sizeTy;
    types[PriorBaseAddress] = typePtr;
    return StructType::get(b->getContext(), types);
}

void DynamicBuffer::allocateBuffer(BuilderRef b) {
    assert (mHandle && "has not been set prior to calling allocateBuffer");
    // note: when adding extensible stream sets, make sure to set the initial count here.
    FixedArray<Value *, 2> indices;
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(BaseAddress);
    Value * const handle = getHandle(b.get());
    Value * const baseAddressField = b->CreateGEP(handle, indices);
    Constant * size = b->getSize(mInitialCapacity + mUnderflow + mOverflow);
    Value * const baseAddress = b->CreateCacheAlignedMalloc(mType, size, mAddressSpace);
    b->CreateStore(addUnderflow(b, baseAddress, mUnderflow), baseAddressField);
    indices[1] = b->getInt32(PriorBaseAddress);
    Value * const priorAddressField = b->CreateGEP(handle, indices);
    b->CreateStore(nullPointerFor(b, baseAddress, mUnderflow), priorAddressField);
    indices[1] = b->getInt32(Capacity);
    Value * const capacityField = b->CreateGEP(handle, indices);
    b->CreateStore(b->getSize(mInitialCapacity), capacityField);
}

void DynamicBuffer::releaseBuffer(BuilderRef b) const {
    /* Free the dynamically allocated buffer(s). */
    Value * const handle = getHandle(b.get());
    FixedArray<Value *, 2> indices;
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(PriorBaseAddress);
    Value * const priorAddressField = b->CreateGEP(handle, indices);
    Value * const priorAddress = b->CreateLoad(priorAddressField);
    b->CreateFree(subtractUnderflow(b, priorAddress, mUnderflow));
    Constant * const nullPtr = nullPointerFor(b, priorAddress, mUnderflow);
    b->CreateStore(nullPtr, priorAddressField);
    indices[1] = b->getInt32(BaseAddress);
    Value * const baseAddressField = b->CreateGEP(handle, indices);
    Value * const baseAddress = b->CreateLoad(baseAddressField);
    b->CreateFree(subtractUnderflow(b, baseAddress, mUnderflow));
    b->CreateStore(nullPtr, baseAddressField);
}

void DynamicBuffer::setBaseAddress(IDISA_Builder * const /* b */, Value * /* addr */) const {
    unsupported("setBaseAddress", "Dynamic");
}

Value * DynamicBuffer::getBaseAddress(IDISA_Builder * const b) const {
    Value * const ptr = b->CreateGEP(getHandle(b), {b->getInt32(0), b->getInt32(BaseAddress)});
    return b->CreateLoad(ptr);
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

Value * DynamicBuffer::getStreamBlockPtr(IDISA_Builder * const b, Value * baseAddress, Value * const streamIndex, Value * const blockIndex) const {
    return StreamSetBuffer::getStreamBlockPtr(b, baseAddress, streamIndex, modByCapacity(b, blockIndex));
}

Value * DynamicBuffer::getStreamPackPtr(IDISA_Builder * const b, Value * baseAddress, Value * const streamIndex, Value * const blockIndex, Value * const packIndex) const {
    return StreamSetBuffer::getStreamPackPtr(b, baseAddress, streamIndex, modByCapacity(b, blockIndex), packIndex);
}

Value * DynamicBuffer::getStreamLogicalBasePtr(IDISA_Builder * const b, Value * baseAddress, Value * const streamIndex, Value * const blockIndex) const {
    Value * const baseBlockIndex = b->CreateSub(modByCapacity(b, blockIndex), blockIndex);
    return StreamSetBuffer::getStreamBlockPtr(b, baseAddress, streamIndex, baseBlockIndex);
}

Value * DynamicBuffer::getRawItemPointer(IDISA_Builder * const b, Value * streamIndex, Value * absolutePosition) const {
    return StreamSetBuffer::getRawItemPointer(b, streamIndex, b->CreateURem(absolutePosition, getCapacity(b)));
}

Value * DynamicBuffer::getLinearlyAccessibleItems(BuilderRef b, Value * const fromPosition, Value * const totalItems, Value * overflowItems) const {
    Value * const capacity = getCapacity(b.get());
    Value * const availableItems = b->CreateSub(totalItems, fromPosition);
    Value * const fromOffset = b->CreateURem(fromPosition, capacity);
    Value * const capacityWithOverflow = addOverflow(b, capacity, overflowItems, nullptr);
    Value * const linearSpace = b->CreateSub(capacityWithOverflow, fromOffset);
    return b->CreateUMin(availableItems, linearSpace);
}

Value * DynamicBuffer::getLinearlyWritableItems(BuilderRef b, Value * const fromPosition, Value * const consumedItems, Value * overflowItems) const {
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

void DynamicBuffer::setCapacity(IDISA_Builder * const /* b */, Value * /* c */) const {
    unsupported("setCapacity", "Dynamic");
}

Value * DynamicBuffer::reserveCapacity(BuilderRef b, Value * const produced, Value * const consumed, Value * const required, Constant * const overflowItems) const {

    SmallVector<char, 200> buf;
    raw_svector_ostream name(buf);

    name << "__DynamicBuffer_reserveCapacity_";

    Type * ty = getBaseType();
    name << ty->getArrayNumElements() << 'x';
    ty = ty->getArrayElementType();
    ty = ty->getVectorElementType();
    name << ty->getIntegerBitWidth()
         << '_' << mUnderflow
         << '_' << mOverflow
         << '_' << mAddressSpace;

    Value * const myHandle = getHandle(b.get());

    Module * const m = b->getModule();
    Function * func = m->getFunction(name.str());
    if (func == nullptr) {

        const auto ip = b->saveIP();

        LLVMContext & C = m->getContext();
        IntegerType * const sizeTy = b->getSizeTy();
        FunctionType * funcTy = FunctionType::get(sizeTy, {myHandle->getType(), sizeTy, sizeTy, sizeTy, sizeTy}, false);
        func = Function::Create(funcTy, Function::InternalLinkage, name.str(), m);

        b->SetInsertPoint(BasicBlock::Create(C, "entry", func));
        BasicBlock * const copyLinear = BasicBlock::Create(C, "copyLinear", func);
        BasicBlock * const copyNonLinear = BasicBlock::Create(C, "copyNonLinear", func);
        BasicBlock * const storeNewBuffer = BasicBlock::Create(C, "storeNewBuffer", func);

        auto arg = func->arg_begin();
        auto nextArg = [&]() {
            assert (arg != func->arg_end());
            Value * const v = &*arg;
            std::advance(arg, 1);
            return v;
        };

        Value * const handle = nextArg();
        handle->setName("handle");
        Value * const produced = nextArg();
        produced->setName("produced");
        Value * const consumed = nextArg();
        consumed->setName("consumed");
        Value * const required = nextArg();
        required->setName("required");
        Value * const overflowItems = nextArg();
        overflowItems->setName("overflowItems");
        assert (arg == func->arg_end());

        setHandle(b, handle);

        const auto blockWidth = b->getBitBlockWidth();
        assert (is_power_2(blockWidth));

        ConstantInt * const BLOCK_WIDTH = b->getSize(blockWidth);
        Constant * const CHUNK_SIZE = ConstantExpr::getSizeOf(mType);

        FixedArray<Value *, 2> indices;
        indices[0] = b->getInt32(0);
        indices[1] = b->getInt32(Capacity);

        Value * const capacityField = b->CreateGEP(handle, indices);
        Value * const capacity = b->CreateLoad(capacityField);
        Value * const consumedChunks = b->CreateUDiv(consumed, BLOCK_WIDTH);
        Value * const producedChunks = b->CreateCeilUDiv(produced, BLOCK_WIDTH);
        Value * const requiredChunks = b->CreateCeilUDiv(required, BLOCK_WIDTH);

        // make sure the new capacity is at least 2x the current capacity and a multiple of it
        Value * const unconsumedChunks = b->CreateSub(producedChunks, consumedChunks);
        Value * newCapacity = b->CreateAdd(unconsumedChunks, requiredChunks);
        #ifdef NDEBUG
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        #endif
            Value * const check = b->CreateICmpUGE(newCapacity, capacity);
            b->CreateAssert(check, "unnecessary buffer expansion occurred");
        #ifdef NDEBUG
        }
        #endif
        newCapacity = b->CreateRoundUp(newCapacity, capacity);

        Value * const totalBytesToCopy = b->CreateMul(unconsumedChunks, CHUNK_SIZE);
        Value * requiredCapacity = newCapacity;
        if (mUnderflow || mOverflow) {
            Constant * const additionalCapacity = b->getSize(mUnderflow + mOverflow);
            requiredCapacity = b->CreateAdd(newCapacity, additionalCapacity);
        }
        Value * newBuffer = b->CreateCacheAlignedMalloc(mType, requiredCapacity, mAddressSpace);
        newBuffer = addUnderflow(b, newBuffer, mUnderflow);

        indices[1] = b->getInt32(BaseAddress);
        Value * const bufferField = b->CreateGEP(handle, indices);
        Value * const buffer = b->CreateLoad(bufferField);

        Value * const consumedOffset = b->CreateURem(consumedChunks, capacity);
        Value * const producedOffset = b->CreateURem(producedChunks, capacity);
        Value * const newConsumedOffset = b->CreateURem(consumedChunks, newCapacity);
        Value * const newProducedOffset = b->CreateURem(producedChunks, newCapacity);
        Value * const sourceLinear = b->CreateICmpULT(consumedOffset, producedOffset);
        Value * const targetLinear = b->CreateICmpULT(newConsumedOffset, newProducedOffset);
        Value * const linearCopy = b->CreateAnd(sourceLinear, targetLinear);

        DataLayout DL(b->getModule());
        Type * const intPtrTy = DL.getIntPtrType(buffer->getType());
        Value * const consumedOffsetPtr = b->CreateGEP(buffer, consumedOffset);
        Value * const newConsumedOffsetPtr = b->CreateGEP(newBuffer, newConsumedOffset);
        Value * const consumedOffsetPtrInt = b->CreatePtrToInt(consumedOffsetPtr, intPtrTy);

        b->CreateCondBr(linearCopy, copyLinear, copyNonLinear);

        b->SetInsertPoint(copyLinear);
        b->CreateMemCpy(newConsumedOffsetPtr, consumedOffsetPtr, totalBytesToCopy, blockWidth / 8);
        b->CreateBr(storeNewBuffer);

        b->SetInsertPoint(copyNonLinear);
        Value * const bufferLength1 = b->CreateSub(capacity, consumedOffset);
        Value * const newBufferLength1 = b->CreateSub(newCapacity, newConsumedOffset);
        Value * const partialLength1 = b->CreateUMin(bufferLength1, newBufferLength1);
        Value * const capacityPtr = b->CreateGEP(buffer, b->CreateAdd(consumedOffset, bufferLength1));
        Value * const capacityPtrInt = b->CreatePtrToInt(capacityPtr, intPtrTy);
        Value * const bytesToCopy1 = b->CreateSub(capacityPtrInt, consumedOffsetPtrInt);
        b->CreateMemCpy(newConsumedOffsetPtr, consumedOffsetPtr, bytesToCopy1, blockWidth / 8);
        Value * const sourceOffset = b->CreateURem(b->CreateAdd(consumedOffset, partialLength1), capacity);
        Value * const sourcePtr = b->CreateGEP(buffer, sourceOffset);
        Value * const targetOffset = b->CreateURem(b->CreateAdd(newConsumedOffset, partialLength1), newCapacity);
        Value * const targetPtr = b->CreateGEP(newBuffer, targetOffset);
        Value * const bytesToCopy2 = b->CreateSub(totalBytesToCopy, bytesToCopy1);
        b->CreateMemCpy(targetPtr, sourcePtr, bytesToCopy2, blockWidth / 8);
        b->CreateBr(storeNewBuffer);

        b->SetInsertPoint(storeNewBuffer);
        indices[1] = b->getInt32(PriorBaseAddress);
        Value * const priorBufferField = b->CreateGEP(handle, indices);
        Value * const priorBuffer = b->CreateLoad(priorBufferField);
        b->CreateStore(buffer, priorBufferField);
        b->CreateStore(newBuffer, bufferField);
        b->CreateStore(newCapacity, capacityField);

        Value * const remainingAfterExpand = getLinearlyWritableItems(b, produced, consumed, overflowItems);
        #ifdef NDEBUG
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        #endif
            Value * const check2 = b->CreateICmpUGE(remainingAfterExpand, requiredChunks);
            b->CreateAssert(check2, "buffer expansion error");
        #ifdef NDEBUG
        }
        #endif
        b->CreateFree(subtractUnderflow(b, priorBuffer, mUnderflow));
        b->CreateRet(remainingAfterExpand);

        b->restoreIP(ip);
        setHandle(b, myHandle);
    }
    return b->CreateCall(func, { myHandle, produced, consumed, required, overflowItems ? overflowItems : b->getSize(0) });
}

// Linear Buffer
Type * LinearBuffer::getHandleType(BuilderRef b) const {
    PointerType * typePtr = getPointerType();
    IntegerType * sizeTy = b->getSizeTy();
    FixedArray<Type *, 5> types;
    types[FirstCapacity] = sizeTy;
    types[ReportedAddress] = typePtr;
    types[FirstBufferAddress] = typePtr;
    types[SecondCapacity] = sizeTy;
    types[SecondBufferAddress] = typePtr;
    return StructType::get(b->getContext(), types);
}

void LinearBuffer::allocateBuffer(BuilderRef b) {
    assert (mHandle && "has not been set prior to calling allocateBuffer");
    Constant * size = b->getSize(mInitialCapacity + mUnderflow + mOverflow);
    // Unlike DynamicBuffers, both buffers are allocated initially. When the first buffer
    // runs out of space, unconsumed data is copied to the second and the reported address
    // is modified to point to the data within the second buffer.

    // If the program is single threaded, we can avoid constructing a second buffer if
    // in the reserveCapacity function, the (produced - consumed + required) < consumed
    // items. However, in a multithreaded scenario, a pathological thread interleaving
    // could result in the producer overwriting a consumer thread's unconsumed data.

    FixedArray<Value *, 2> indices;
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(ReportedAddress);
    Value * const firstBufferField =  b->CreateGEP(mHandle, indices);
    Value * firstBuffer = b->CreateCacheAlignedMalloc(getType(), size, mAddressSpace);
    firstBuffer = addUnderflow(b, firstBuffer, mUnderflow);
    b->CreateStore(firstBuffer, firstBufferField);
    Constant * const initialCapacity = b->getSize(mInitialCapacity);
    indices[1] = b->getInt32(FirstCapacity);
    b->CreateStore(initialCapacity, b->CreateGEP(mHandle, indices));
    indices[1] = b->getInt32(FirstBufferAddress);
    b->CreateStore(firstBuffer, b->CreateGEP(mHandle, indices));
    indices[1] = b->getInt32(SecondCapacity);
    b->CreateStore(initialCapacity, b->CreateGEP(mHandle, indices));
    indices[1] = b->getInt32(SecondBufferAddress);
    Value * secondBuffer = b->CreateCacheAlignedMalloc(getType(), size, mAddressSpace);
    secondBuffer = addUnderflow(b, secondBuffer, mUnderflow);
    b->CreateStore(addUnderflow(b, secondBuffer, mUnderflow), b->CreateGEP(mHandle, indices));
}

void LinearBuffer::releaseBuffer(BuilderRef b) const {
    /* Free the dynamically allocated buffer(s). */
    Value * const handle = getHandle(b.get());
    Value * priorAddressField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(SecondBufferAddress)});
    Value * priorAddress = b->CreateLoad(priorAddressField);
    b->CreateFree(subtractUnderflow(b, priorAddress, mUnderflow));
    Constant * nullPtr = nullPointerFor(b, priorAddress, mUnderflow);
    b->CreateStore(nullPtr, priorAddressField);
    Value * baseAddressField = b->CreateGEP(handle, {b->getInt32(0), b->getInt32(FirstBufferAddress)});
    Value * baseAddress = b->CreateLoad(baseAddressField);
    b->CreateFree(subtractUnderflow(b, baseAddress, mUnderflow));
    b->CreateStore(nullPtr, baseAddressField);
}

void LinearBuffer::setBaseAddress(IDISA_Builder * const /* b */, Value * /* addr */) const {
    unsupported("setBaseAddress", "Linear");
}

Value * LinearBuffer::getBaseAddress(IDISA_Builder * const b) const {
    Value * const ptr = b->CreateGEP(getHandle(b), {b->getInt32(0), b->getInt32(ReportedAddress)});
    return b->CreateLoad(ptr);
}

Value * LinearBuffer::getOverflowAddress(IDISA_Builder * const b) const {
    Value * const capacityPtr = b->CreateGEP(getHandle(b), {b->getInt32(0), b->getInt32(FirstCapacity)});
    Value * const capacity = b->CreateLoad(capacityPtr);
    return b->CreateGEP(getBaseAddress(b), capacity);
}

Value * LinearBuffer::getLinearlyAccessibleItems(BuilderRef b, Value * const fromPosition, Value * const totalItems, Value * /* overflowItems */) const {
    return b->CreateSub(totalItems, fromPosition);
}

Value * LinearBuffer::getLinearlyWritableItems(BuilderRef /* b */, Value * const /* fromPosition */, Value * const /* consumedItems */, Value * /* overflowItems */) const {
    unsupported("getLinearlyWritableItems", "Linear");
}

Value * LinearBuffer::getStreamLogicalBasePtr(IDISA_Builder * const b, Value * baseAddress, Value * const streamIndex, Value * /* blockIndex */) const {
    return StreamSetBuffer::getStreamBlockPtr(b, baseAddress, streamIndex, b->getSize(0));
}

Value * LinearBuffer::getCapacity(IDISA_Builder * const b) const {
    Value * ptr = b->CreateGEP(getHandle(b), {b->getInt32(0), b->getInt32(FirstCapacity)});
    return b->CreateMul(b->CreateLoad(ptr), b->getSize(b->getBitBlockWidth()));
}

void LinearBuffer::setCapacity(IDISA_Builder * const /* b */, Value * /* c */) const {
    unsupported("setCapacity", "Linear");
}

Value * LinearBuffer::reserveCapacity(BuilderRef b, Value * produced, Value * consumed, Value * const required, Constant * const overflowItems) const {

    ConstantInt * const LOG_2_BIT_BLOCK_WIDTH = b->getSize(std::log2(b->getBitBlockWidth()));

    Type * itemTy = mBaseType->getArrayElementType()->getVectorElementType();
    const auto itemWidth = itemTy->getPrimitiveSizeInBits();
    assert (is_power_2(itemWidth));
    const auto blockWidth = b->getBitBlockWidth();
    assert (is_power_2(blockWidth));

    BasicBlock * const copyOrExpand = b->CreateBasicBlock("copyOrExpandBuffer");
    BasicBlock * const expandBuffer = b->CreateBasicBlock("expandBuffer");
    BasicBlock * const copyToBuffer = b->CreateBasicBlock("copyToBuffer");

    FixedArray<Value *, 2> indices;
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(FirstCapacity);

    Value * const handle = getHandle(b.get());
    Value * const firstCapacityField = b->CreateGEP(handle, indices);
    Value * const firstBlockCapacity = b->CreateLoad(firstCapacityField);
    Value * const firstCapacity = b->CreateShl(firstBlockCapacity, LOG_2_BIT_BLOCK_WIDTH);

    if (LLVM_LIKELY(itemWidth < blockWidth)) {
        Constant * const SCALE = b->getSize(blockWidth / itemWidth);
        consumed = b->CreateRoundDown(consumed, SCALE);
        produced = b->CreateRoundUp(produced, SCALE);
    }

    Value * const unconsumed = b->CreateSub(produced, consumed);
    Value * requiredCapacity = b->CreateAdd(required, unconsumed);
    indices[1] = b->getInt32(SecondBufferAddress);
    Value * const secondBufferField = b->CreateGEP(handle, indices);
    Value * const secondBuffer = b->CreateLoad(secondBufferField);
    indices[1] = b->getInt32(SecondCapacity);
    Value * const secondCapacityField = b->CreateGEP(handle, indices);
    Value * const secondBlockCapacity = b->CreateLoad(secondCapacityField);
    Value * const secondCapacity = b->CreateShl(secondBlockCapacity, LOG_2_BIT_BLOCK_WIDTH);

    Value * const canFit = b->CreateICmpULT(requiredCapacity, secondCapacity);
    b->CreateLikelyCondBr(canFit, copyToBuffer, expandBuffer);

    b->SetInsertPoint(expandBuffer);
    // make sure the new capacity is a multiple of the current capacity
    requiredCapacity = b->CreateRoundUp(requiredCapacity, firstCapacity);
    Value * requiredCapacityBlocks = b->CreateLShr(requiredCapacity, LOG_2_BIT_BLOCK_WIDTH);
    Value * size = b->CreateAdd(requiredCapacityBlocks, b->getSize(mUnderflow + mOverflow));
    Value * expandedBuffer = b->CreateCacheAlignedMalloc(getType(), size, mAddressSpace);
    expandedBuffer = addUnderflow(b, expandedBuffer, mUnderflow);
    indices[1] = b->getInt32(SecondBufferAddress);
    b->CreateFree(subtractUnderflow(b, secondBuffer, mUnderflow));
    b->CreateBr(copyToBuffer);

    b->SetInsertPoint(copyToBuffer);
    PHINode * const newBuffer = b->CreatePHI(secondBuffer->getType(), 2);
    newBuffer->addIncoming(secondBuffer, copyOrExpand);
    newBuffer->addIncoming(expandedBuffer, expandBuffer);
    PHINode * const newCapacity = b->CreatePHI(b->getSizeTy(), 2);
    newCapacity->addIncoming(secondCapacity, copyOrExpand);
    newCapacity->addIncoming(requiredCapacity, expandBuffer);

    indices[1] = b->getInt32(ReportedAddress);
    Value * const reportedAddrField = b->CreateGEP(handle, indices);
    Value * const reportedAddr = b->CreateLoad(reportedAddrField);
    Value * const consumedOffset = b->CreateLShr(consumed, LOG_2_BIT_BLOCK_WIDTH);
    Value * const consumedAddr = b->CreateGEP(reportedAddr, consumedOffset);

    Value * const numOfStreams = getStreamSetCount(b.get());

    Value * unconsumedBytes = b->CreateMul(unconsumed, numOfStreams);
    if (LLVM_LIKELY(itemWidth < 8)) {
        unconsumedBytes = b->CreateExactUDiv(unconsumedBytes, b->getSize(8 / itemWidth));
    } else if (LLVM_UNLIKELY(itemWidth > 8)) {
        unconsumedBytes = b->CreateMul(unconsumedBytes, b->getSize(itemWidth / 8));
    }
    b->CreateMemCpy(newBuffer, consumedAddr, unconsumedBytes, blockWidth / 8);
    Value * const newReportedAddr = b->CreateGEP(newBuffer, b->CreateNeg(consumedOffset));
    b->CreateStore(newReportedAddr, reportedAddrField);

    indices[1] = b->getInt32(FirstBufferAddress);
    Value * const firstBufferField = b->CreateGEP(handle, indices);
    Value * const firstBuffer = b->CreateLoad(firstBufferField);
    b->CreateStore(firstBlockCapacity, secondCapacityField);
    b->CreateStore(firstBuffer, secondBufferField);
    b->CreateStore(b->CreateLShr(newCapacity, LOG_2_BIT_BLOCK_WIDTH), firstCapacityField);
    b->CreateStore(newBuffer, firstBufferField);
    Value * const remainingAfterExpand = b->CreateSub(newCapacity, unconsumed);

    return remainingAfterExpand;
}

// Constructors

ExternalBuffer::ExternalBuffer(BuilderRef b, Type * const type,
                               const unsigned AddressSpace)
: StreamSetBuffer(BufferKind::ExternalBuffer, b, type, 0, 0, AddressSpace) {

}

StaticBuffer::StaticBuffer(BuilderRef b, Type * const type,
                           const size_t capacity, const size_t overflowSize, const size_t underflowSize,
                           const unsigned AddressSpace)
: StreamSetBuffer(BufferKind::StaticBuffer, b, type, overflowSize, underflowSize, AddressSpace)
, mCapacity(capacity / b->getBitBlockWidth()) {
    assert ("static buffer cannot have 0 capacity" && capacity);
    assert ("static buffer capacity must be a multiple of bitblock width"
            && (capacity % b->getBitBlockWidth()) == 0);
    assert ("static buffer overflow must be a multiple of bitblock width"
            && (overflowSize % b->getBitBlockWidth()) == 0);
    assert ("static buffer underflow must be a multiple of bitblock width"
            && (underflowSize % b->getBitBlockWidth()) == 0);
    assert ("static buffer capacity must be at least twice its max(underflow, overflow)"
            && (capacity >= (std::max(underflowSize, overflowSize) * 2)));
}

DynamicBuffer::DynamicBuffer(BuilderRef b, Type * const type,
                             const size_t initialCapacity, const size_t overflowSize, const size_t underflowSize,
                             const unsigned AddressSpace)
: StreamSetBuffer(BufferKind::DynamicBuffer, b, type, overflowSize, underflowSize, AddressSpace)
, mInitialCapacity(initialCapacity / b->getBitBlockWidth()) {
    assert ("dynamic buffer cannot have 0 initial capacity" && initialCapacity);
    assert ("dynamic buffer capacity must be a multiple of bitblock width"
            && (initialCapacity % b->getBitBlockWidth()) == 0);
    assert ("dynamic buffer overflow must be a multiple of bitblock width"
            && (overflowSize % b->getBitBlockWidth()) == 0);
    assert ("dynamic buffer underflow must be a multiple of bitblock width"
            && (underflowSize % b->getBitBlockWidth()) == 0);
    assert ("dynamic buffer initial capacity must be at least twice its max(underflow, overflow)"
            && (initialCapacity >= (std::max(underflowSize, overflowSize) * 2)));
}

LinearBuffer::LinearBuffer(BuilderRef b, Type * const type,
                           const size_t initialCapacity, const size_t overflowSize, const size_t underflowSize,
                           const unsigned AddressSpace)
: StreamSetBuffer(BufferKind::LinearBuffer, b, type, overflowSize, underflowSize, AddressSpace)
, mInitialCapacity(initialCapacity / b->getBitBlockWidth()) {
    assert ("linear buffer cannot have 0 initial capacity" && initialCapacity);
    assert ("linear buffer capacity must be a multiple of bitblock width" && (initialCapacity % b->getBitBlockWidth()) == 0);
    assert ("linear buffer overflow must be a multiple of bitblock width" && (overflowSize % b->getBitBlockWidth()) == 0);
    assert ("linear buffer underflow must be a multiple of bitblock width" && (underflowSize % b->getBitBlockWidth()) == 0);
    assert ("linear buffer initial capacity must be at least twice its max(underflow, overflow)"
            && (initialCapacity >= (std::max(underflowSize, overflowSize) * 2)));
}


StreamSetBuffer::StreamSetBuffer(const BufferKind k, BuilderRef b,
                                 Type * const baseType,
                                 const size_t overflowSize, const size_t underflowSize, const unsigned AddressSpace)
: mBufferKind(k)
, mHandle(nullptr)
, mType(resolveType(b, baseType))
, mOverflow(overflowSize / b->getBitBlockWidth())
, mUnderflow(underflowSize / b->getBitBlockWidth())
, mAddressSpace(AddressSpace)
, mBaseType(baseType) {

}

StreamSetBuffer::~StreamSetBuffer() { }

}
