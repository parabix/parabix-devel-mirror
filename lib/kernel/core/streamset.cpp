/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernel/core/streamset.h>

#include <kernel/core/kernel.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_ostream.h>
#include <kernel/core/kernel_builder.h>
#include <toolchain/toolchain.h>
#include <llvm/Support/Debug.h>
#include <llvm/Support/Format.h>
#include <array>

namespace llvm { class Constant; }
namespace llvm { class Function; }

using namespace llvm;
using IDISA::IDISA_Builder;

namespace kernel {

using Rational = KernelBuilder::Rational;

using BuilderPtr = StreamSetBuffer::BuilderPtr;

LLVM_ATTRIBUTE_NORETURN void unsupported(const char * const function, const char * const bufferType) {
    report_fatal_error(StringRef{function} + " is not supported by " + bufferType + "Buffers");
}

LLVM_READNONE inline Constant * nullPointerFor(BuilderPtr & b, Type * type, const unsigned underflow) {
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

LLVM_READNONE inline Constant * nullPointerFor(BuilderPtr & b, Value * ptr, const unsigned underflow) {
    return nullPointerFor(b, ptr->getType(), underflow);
}

LLVM_READNONE inline unsigned getItemWidth(const Type * ty ) {
    if (LLVM_LIKELY(isa<ArrayType>(ty))) {
        ty = ty->getArrayElementType();
    }
    return cast<IntegerType>(ty->getVectorElementType())->getBitWidth();
}

LLVM_READNONE inline Value * addUnderflow(BuilderPtr & b, Value * ptr, const unsigned underflow) {
    if (LLVM_LIKELY(underflow == 0)) {
        return ptr;
    } else {
        assert ("unspecified module" && b.get() && b->getModule());
        DataLayout DL(b->getModule());
        Type * const intPtrTy = DL.getIntPtrType(ptr->getType());
        Constant * offset = ConstantInt::get(intPtrTy, underflow);
        return b->CreateInBoundsGEP(ptr, offset);
    }
}

LLVM_READNONE inline Value * subtractUnderflow(BuilderPtr & b, Value * ptr, const unsigned underflow) {
    if (LLVM_LIKELY(underflow == 0)) {
        return ptr;
    } else {
        DataLayout DL(b->getModule());
        Type * const intPtrTy = DL.getIntPtrType(ptr->getType());
        Constant * offset = ConstantExpr::getNeg(ConstantInt::get(intPtrTy, underflow));
        return b->CreateInBoundsGEP(ptr, offset);
    }
}

void StreamSetBuffer::assertValidStreamIndex(BuilderPtr b, Value * streamIndex) const {
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        Value * const count = getStreamSetCount(b);
        Value * const index = b->CreateZExtOrTrunc(streamIndex, count->getType());
        Value * const withinSet = b->CreateICmpULT(index, count);
        b->CreateAssert(withinSet, "out-of-bounds stream access: %i of %i", index, count);
    }
}

Value * StreamSetBuffer::getStreamBlockPtr(BuilderPtr b, Value * const baseAddress, Value * const streamIndex, Value * const blockIndex) const {
    assertValidStreamIndex(b, streamIndex);
    return b->CreateInBoundsGEP(baseAddress, {blockIndex, streamIndex});
}

Value * StreamSetBuffer::getStreamPackPtr(BuilderPtr b, Value * const baseAddress, Value * const streamIndex, Value * blockIndex, Value * const packIndex) const {
    assertValidStreamIndex(b, streamIndex);
    return b->CreateInBoundsGEP(baseAddress, {blockIndex, streamIndex, packIndex});
}

Value * StreamSetBuffer::getStreamSetCount(BuilderPtr b) const {
    size_t count = 1;
    if (isa<ArrayType>(getBaseType())) {
        count = getBaseType()->getArrayNumElements();
    }
    return b->getSize(count);
}

size_t StreamSetBuffer::getUnderflowCapacity(BuilderPtr b) const {
    return mUnderflow * b->getBitBlockWidth();
}

size_t StreamSetBuffer::getOverflowCapacity(BuilderPtr b) const {
    return mOverflow * b->getBitBlockWidth();
}


/**
 * @brief getRawItemPointer
 *
 * get a raw pointer the iN field at position absoluteItemPosition of the stream number streamIndex of the stream set.
 * In the case of a stream whose fields are less than one byte (8 bits) in size, the pointer is to the containing byte.
 * The type of the pointer is i8* for fields of 8 bits or less, otherwise iN* for N-bit fields.
 */
Value * StreamSetBuffer::getRawItemPointer(BuilderPtr b, Value * streamIndex, Value * absolutePosition) const {
    Type * const itemTy = mBaseType->getArrayElementType()->getVectorElementType();
    const auto itemWidth = itemTy->getPrimitiveSizeInBits();
    IntegerType * const sizeTy = b->getSizeTy();
    streamIndex = b->CreateZExt(streamIndex, sizeTy);
    absolutePosition = b->CreateZExt(absolutePosition, sizeTy);

    const auto blockWidth = b->getBitBlockWidth();
    Constant * const BLOCK_WIDTH = b->getSize(blockWidth);
    Value * blockIndex = b->CreateUDiv(absolutePosition, BLOCK_WIDTH);
    Value * positionInBlock = b->CreateURem(absolutePosition, BLOCK_WIDTH);
    Value * blockPtr = getStreamBlockPtr(b, getBaseAddress(b), streamIndex, blockIndex);
    if (LLVM_UNLIKELY(itemWidth < 8)) {
        const Rational itemsPerByte{8, itemWidth};
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
            b->CreateAssertZero(b->CreateURemRate(absolutePosition, itemsPerByte),
                                "absolutePosition must be byte aligned");
        }
        positionInBlock = b->CreateUDivRate(positionInBlock, itemsPerByte);
        PointerType * const itemPtrTy = b->getInt8Ty()->getPointerTo(mAddressSpace);
        blockPtr = b->CreatePointerCast(blockPtr, itemPtrTy);
        return b->CreateInBoundsGEP(blockPtr, positionInBlock);
    }
    PointerType * const itemPtrTy = itemTy->getPointerTo(mAddressSpace);
    blockPtr = b->CreatePointerCast(blockPtr, itemPtrTy);
    return b->CreateInBoundsGEP(blockPtr, positionInBlock);
}

Value * StreamSetBuffer::addOverflow(BuilderPtr b, Value * const bufferCapacity, Value * const overflowItems, Value * const consumedOffset) const {
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
Type * StreamSetBuffer::resolveType(BuilderPtr b, Type * const streamSetType) {
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

// External Buffer

Type * ExternalBuffer::getHandleType(BuilderPtr b) const {
    PointerType * const ptrTy = getPointerType();
    IntegerType * const sizeTy = b->getSizeTy();
    return StructType::get(b->getContext(), {ptrTy, sizeTy});
}

void ExternalBuffer::allocateBuffer(BuilderPtr /* b */) {
    unsupported("allocateBuffer", "External");
}

void ExternalBuffer::releaseBuffer(BuilderPtr /* b */) const {
    // this buffer is not responsible for free-ing th data associated with it
}

void ExternalBuffer::setBaseAddress(BuilderPtr b, Value * const addr) const {
    assert (mHandle && "has not been set prior to calling setBaseAddress");
    Value * const p = b->CreateInBoundsGEP(mHandle, {b->getInt32(0), b->getInt32(BaseAddress)});
    b->CreateStore(b->CreatePointerBitCastOrAddrSpaceCast(addr, getPointerType()), p);
}

Value * ExternalBuffer::getBaseAddress(BuilderPtr b) const {
    assert (mHandle && "has not been set prior to calling getBaseAddress");
    Value * const p = b->CreateInBoundsGEP(mHandle, {b->getInt32(0), b->getInt32(BaseAddress)});
    return b->CreateLoad(p);
}

Value * ExternalBuffer::getOverflowAddress(BuilderPtr b) const {
    assert (mHandle && "has not been set prior to calling getBaseAddress");
    Value * const p = b->CreateInBoundsGEP(mHandle, {b->getInt32(0), b->getInt32(Capacity)});
    return b->CreateLoad(p);
}

void ExternalBuffer::setCapacity(BuilderPtr b, Value * const capacity) const {
    assert (mHandle && "has not been set prior to calling setCapacity");
    Value *  const p = b->CreateInBoundsGEP(mHandle, {b->getInt32(0), b->getInt32(Capacity)});
    b->CreateStore(b->CreateZExt(capacity, b->getSizeTy()), p);
}

Value * ExternalBuffer::getCapacity(BuilderPtr b) const {
    assert (mHandle && "has not been set prior to calling getCapacity");
    Value * const p = b->CreateInBoundsGEP(mHandle, {b->getInt32(0), b->getInt32(Capacity)});
    return b->CreateLoad(p);
}

Value * ExternalBuffer::modByCapacity(BuilderPtr /* b */, Value * const offset) const {
    assert (offset->getType()->isIntegerTy());
    return offset;
}

Value * ExternalBuffer::getLinearlyAccessibleItems(BuilderPtr b, Value * const fromPosition, Value * const totalItems, Value * /* overflowItems */) const {
    return b->CreateSub(totalItems, fromPosition);
}

Value * ExternalBuffer::getLinearlyWritableItems(BuilderPtr b, Value * const fromPosition, Value * const /* consumed */, Value * /* overflowItems */) const {
    assert (fromPosition);
    Value * const capacity = getCapacity(b);
    assert (fromPosition->getType() == capacity->getType());
    return b->CreateSub(capacity, fromPosition);
}

Value * ExternalBuffer::getStreamLogicalBasePtr(BuilderPtr b, Value * baseAddress, Value * const streamIndex, Value * /* blockIndex */) const {
    return StreamSetBuffer::getStreamBlockPtr(b, baseAddress, streamIndex, b->getSize(0));
}

inline void ExternalBuffer::assertValidBlockIndex(BuilderPtr b, Value * blockIndex) const {
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        Value * const blockCount = b->CreateCeilUDiv(getCapacity(b), b->getSize(b->getBitBlockWidth()));
        blockIndex = b->CreateZExtOrTrunc(blockIndex, blockCount->getType());
        Value * const withinCapacity = b->CreateICmpULT(blockIndex, blockCount);
        b->CreateAssert(withinCapacity, "blockIndex exceeds buffer capacity");
    }
}

void ExternalBuffer::reserveCapacity(BuilderPtr /* b */, Value * /* produced */, Value * /* consumed */, Value * const /* required */, Constant * const /* overflowItems */) const  {
    unsupported("reserveCapacity", "External");
}

// Internal Buffer

Value * InternalBuffer::getStreamBlockPtr(BuilderPtr b, Value * const baseAddress, Value * const streamIndex, Value * const blockIndex) const {
    return StreamSetBuffer::getStreamBlockPtr(b, baseAddress, streamIndex, LLVM_UNLIKELY(mLinear) ? blockIndex : modByCapacity(b, blockIndex));
}

Value * InternalBuffer::getStreamPackPtr(BuilderPtr b, Value * const baseAddress, Value * const streamIndex, Value * const blockIndex, Value * const packIndex) const {
    return StreamSetBuffer::getStreamPackPtr(b, baseAddress, streamIndex, LLVM_UNLIKELY(mLinear) ? blockIndex : modByCapacity(b, blockIndex), packIndex);
}

Value * InternalBuffer::getStreamLogicalBasePtr(BuilderPtr b, Value * const baseAddress, Value * const streamIndex, Value * const blockIndex) const {
    Value * baseBlockIndex = nullptr;
    if (LLVM_UNLIKELY(mLinear)) {
        baseBlockIndex = b->getSize(0);
    } else {
        baseBlockIndex = b->CreateSub(modByCapacity(b, blockIndex), blockIndex);
    }
    return StreamSetBuffer::getStreamBlockPtr(b, baseAddress, streamIndex, baseBlockIndex);
}

Value * InternalBuffer::getRawItemPointer(BuilderPtr b, Value * const streamIndex, Value * absolutePosition) const {
    return StreamSetBuffer::getRawItemPointer(b, streamIndex, b->CreateURem(absolutePosition, getCapacity(b)));
}

Value * InternalBuffer::getLinearlyAccessibleItems(BuilderPtr b, Value * const fromPosition, Value * const totalItems, Value * overflowItems) const {
    if (mLinear) {
        return b->CreateSub(totalItems, fromPosition);
    } else {
        Value * const capacity = getCapacity(b);
        Value * const fromOffset = b->CreateURem(fromPosition, capacity);
        Value * const capacityWithOverflow = addOverflow(b, capacity, overflowItems, nullptr);
        Value * const linearSpace = b->CreateSub(capacityWithOverflow, fromOffset);
        Value * const availableItems = b->CreateSub(totalItems, fromPosition);
        return b->CreateUMin(availableItems, linearSpace);
    }
}

Value * InternalBuffer::getLinearlyWritableItems(BuilderPtr b, Value * const fromPosition, Value * const consumedItems, Value * overflowItems) const {
    Value * const capacity = getCapacity(b);
    if (LLVM_UNLIKELY(mLinear)) {
        return b->CreateSub(capacity, fromPosition);
    } else {
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
}

// Static Buffer

Type * StaticBuffer::getHandleType(BuilderPtr b) const {
    PointerType * const typePtr = getPointerType();
    FixedArray<Type *, 3> types;
    types[BaseAddress] = typePtr;
    auto & C = b->getContext();
    if (LLVM_UNLIKELY(mLinear)) {
        types[EffectiveCapacity] = b->getSizeTy();
        types[ConcreteAddress] = typePtr;
    } else {
        Type * const emptyTy = StructType::get(C);
        types[EffectiveCapacity] = emptyTy;
        types[ConcreteAddress] = emptyTy;
    }
    return StructType::get(C, types);
}

void StaticBuffer::allocateBuffer(BuilderPtr b) {
    FixedArray<Value *, 2> indices;
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(BaseAddress);
    Value * const handle = getHandle();
    assert (handle && "has not been set prior to calling allocateBuffer");
    Constant * const size = b->getSize(mCapacity + mUnderflow + mOverflow);
    Value * const buffer = addUnderflow(b, b->CreateCacheAlignedMalloc(mType, size, mAddressSpace), mUnderflow);
    Value * const baseAddressField = b->CreateInBoundsGEP(handle, indices);
    b->CreateStore(buffer, baseAddressField);
    if (mLinear) {
        indices[1] = b->getInt32(EffectiveCapacity);
        Value * const capacityField = b->CreateInBoundsGEP(handle, indices);
        b->CreateStore(b->getSize(mCapacity), capacityField);
        indices[1] = b->getInt32(ConcreteAddress);
        Value * const concreteAddrField = b->CreateInBoundsGEP(handle, indices);
        b->CreateStore(buffer, concreteAddrField);
    }
}

void StaticBuffer::releaseBuffer(BuilderPtr b) const {
    Value * const handle = getHandle();
    FixedArray<Value *, 2> indices;
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(mLinear ? ConcreteAddress : BaseAddress);
    Value * const addressField = b->CreateInBoundsGEP(handle, indices);
    Value * buffer = b->CreateLoad(addressField);
    b->CreateFree(subtractUnderflow(b, buffer, mUnderflow));
    b->CreateStore(nullPointerFor(b, buffer, mUnderflow), addressField);
}

inline bool isCapacityGuaranteed(const Value * const index, const size_t capacity) {
    return isa<ConstantInt>(index) ? cast<ConstantInt>(index)->getLimitedValue() < capacity : false;
}

Value * StaticBuffer::modByCapacity(BuilderPtr b, Value * const offset) const {
    assert (offset->getType()->isIntegerTy());
    if (LLVM_UNLIKELY(mLinear || isCapacityGuaranteed(offset, mCapacity))) {
        return offset;
    } else if (LLVM_UNLIKELY(mCapacity == 1)) {
        return ConstantInt::getNullValue(offset->getType());
    } else if (LLVM_LIKELY(is_power_2(mCapacity))) {
        return b->CreateAnd(offset, ConstantInt::get(offset->getType(), mCapacity - 1));
    } else {
        return b->CreateURem(offset, ConstantInt::get(offset->getType(), mCapacity));
    }
}

Value * StaticBuffer::getCapacity(BuilderPtr b) const {
    if (mLinear) {
        assert (getHandle());
        Value * ptr = b->CreateInBoundsGEP(getHandle(), {b->getInt32(0), b->getInt32(EffectiveCapacity)});
        return b->CreateMul(b->CreateLoad(ptr), b->getSize(b->getBitBlockWidth()));
    } else {
        return b->getSize(mCapacity * b->getBitBlockWidth());
    }
}

void StaticBuffer::setCapacity(BuilderPtr /* b */, Value * /* c */) const {
    unsupported("setCapacity", "Static");
}

Value * StaticBuffer::getBaseAddress(BuilderPtr b) const {
    Value * const ptr = b->CreateInBoundsGEP(getHandle(), {b->getInt32(0), b->getInt32(BaseAddress)});
    Value * const addr = b->CreateLoad(ptr);
    return addr;
}

void StaticBuffer::setBaseAddress(BuilderPtr /* b */, Value * /* addr */) const {
    unsupported("setBaseAddress", "Static");
}

Value * StaticBuffer::getOverflowAddress(BuilderPtr b) const {
    return b->CreateInBoundsGEP(getBaseAddress(b), b->getSize(mCapacity));
}

void StaticBuffer::reserveCapacity(BuilderPtr b, Value * produced, Value * consumed, Value * const required, Constant * const overflowItems) const  {
#if 1
    if (mLinear) {
        SmallVector<char, 200> buf;
        raw_svector_ostream name(buf);

        assert ("unspecified module" && b.get() && b->getModule());

        name << "__StaticLinearBuffer_reserveCapacity_";

        Type * ty = getBaseType();
        const auto streamCount = ty->getArrayNumElements();
        name << streamCount << 'x';
        ty = ty->getArrayElementType();
        ty = ty->getVectorElementType();
        const auto itemWidth = ty->getIntegerBitWidth();
        name << itemWidth
             << '_' << mUnderflow
             << '_' << mOverflow
             << '_' << mAddressSpace;

        Value * const myHandle = getHandle();

        Module * const m = b->getModule();
        Function * func = m->getFunction(name.str());
        if (func == nullptr) {

            const auto ip = b->saveIP();

            LLVMContext & C = m->getContext();
            IntegerType * const sizeTy = b->getSizeTy();
            FunctionType * funcTy = FunctionType::get(b->getVoidTy(), {myHandle->getType(), sizeTy, sizeTy, sizeTy}, false);
            func = Function::Create(funcTy, Function::InternalLinkage, name.str(), m);

            b->SetInsertPoint(BasicBlock::Create(C, "entry", func));

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
            assert (arg == func->arg_end());

            setHandle(handle);

            const auto blockWidth = b->getBitBlockWidth();
            assert (is_power_2(blockWidth));
            const auto blockSize = blockWidth / 8;

            ConstantInt * const BLOCK_WIDTH = b->getSize(blockWidth);
            Constant * const CHUNK_SIZE = ConstantExpr::getSizeOf(mType);

            FixedArray<Value *, 2> indices;
            indices[0] = b->getInt32(0);
            indices[1] = b->getInt32(EffectiveCapacity);
            Value * const capacityField = b->CreateInBoundsGEP(handle, indices);
            Value * const capacity = b->CreateLoad(capacityField);
            Value * const consumedChunks = b->CreateUDiv(consumed, BLOCK_WIDTH);

            indices[1] = b->getInt32(BaseAddress);
            Value * const virtualBaseField = b->CreateInBoundsGEP(handle, indices);
            Value * const virtualBase = b->CreateLoad(virtualBaseField);
            assert (virtualBase->getType()->getPointerElementType() == mType);

            DataLayout DL(b->getModule());
            Type * const intPtrTy = DL.getIntPtrType(virtualBase->getType());

            indices[1] = b->getInt32(ConcreteAddress);
            Value * const actualBufferStartField = b->CreateInBoundsGEP(handle, indices);
            Value * const bufferStart = b->CreateLoad(actualBufferStartField);
            Value * const producedChunks = b->CreateCeilUDiv(produced, BLOCK_WIDTH);
            BasicBlock * const copyBack = BasicBlock::Create(C, "copyBack", func);
            BasicBlock * const updateBaseAddress = BasicBlock::Create(C, "updateBaseAddress", func);
            Value * const noCopy = b->CreateICmpEQ(producedChunks, consumedChunks);
            b->CreateLikelyCondBr(noCopy, updateBaseAddress, copyBack);

            b->SetInsertPoint(copyBack);
            Value * const unreadDataPtr = b->CreateInBoundsGEP(virtualBase, consumedChunks);
            Value * const unconsumedChunks = b->CreateSub(producedChunks, consumedChunks);

            //if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
                Value * const requiredCapacity = b->CreateAdd(produced, required);
                Value * const requiredChunks = b->CreateCeilUDiv(requiredCapacity, BLOCK_WIDTH);
                Value * const chunksToOverwrite = b->CreateSub(requiredChunks, consumedChunks);
                Value * const overwriteUpToPtr = b->CreateInBoundsGEP(bufferStart, chunksToOverwrite);
                Value * const canCopy = b->CreateICmpULE(overwriteUpToPtr, unreadDataPtr);
                // unlike linear dynamic buffers, a linear static buffer requires that we can always copy
                // the data back within the same buffer.
                b->CreateAssert(canCopy, "Static linear buffer cannot copy the unconsumed items without expansion.");
            //}

            Value * const bytesToCopy = b->CreateMul(unconsumedChunks, CHUNK_SIZE);
            b->CreateMemCpy(bufferStart, unreadDataPtr, bytesToCopy, blockSize);
            b->CreateBr(updateBaseAddress);

            b->SetInsertPoint(updateBaseAddress);
            Value * const newBaseAddress = b->CreateGEP(bufferStart, b->CreateNeg(consumedChunks));
            b->CreateStore(newBaseAddress, virtualBaseField);

            // how many item chunks between the virtual base address and the actual buffer start address?
            Value * const bufferInt = b->CreatePtrToInt(virtualBase, intPtrTy);
            Value * const dataStartInt = b->CreatePtrToInt(bufferStart, intPtrTy);
            Value * firstItemOffset = b->CreateSub(dataStartInt, bufferInt);
            firstItemOffset = b->CreateUDiv(firstItemOffset, CHUNK_SIZE);
            Value * const discarded = b->CreateSub(consumedChunks, firstItemOffset);
            Value * const effectiveCapacity = b->CreateAdd(capacity, discarded);

            b->CreateStore(effectiveCapacity, capacityField);
            b->CreateRetVoid();

            b->restoreIP(ip);
            setHandle(myHandle);
        }

        b->CreateCall(func, { myHandle, produced, consumed, required });
    }
#endif
}

// Dynamic Buffer

Type * DynamicBuffer::getHandleType(BuilderPtr b) const {
    PointerType * const typePtr = getPointerType();
    IntegerType * const sizeTy = b->getSizeTy();
    FixedArray<Type *, 4> types;
    types[BaseAddress] = typePtr;
    types[Capacity] = sizeTy;
    types[PriorAddress] = typePtr;
    auto & C = b->getContext();
    if (LLVM_UNLIKELY(mLinear)) {
        types[ConcreteAddress] = typePtr;
    } else {
        Type * const emptyTy = StructType::get(C);
        types[ConcreteAddress] = emptyTy;
    }
    return StructType::get(C, types);
}

void DynamicBuffer::allocateBuffer(BuilderPtr b) {
    assert (mHandle && "has not been set prior to calling allocateBuffer");
    // note: when adding extensible stream sets, make sure to set the initial count here.
    FixedArray<Value *, 2> indices;
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(BaseAddress);
    Value * const handle = getHandle();
    Value * const baseAddressField = b->CreateInBoundsGEP(handle, indices);
    Constant * size = b->getSize(mInitialCapacity + mUnderflow + mOverflow);
    Value * const baseAddress = b->CreateCacheAlignedMalloc(mType, size, mAddressSpace);
    Value * const adjBaseAddress = addUnderflow(b, baseAddress, mUnderflow);
    b->CreateStore(adjBaseAddress, baseAddressField);
    indices[1] = b->getInt32(PriorAddress);
    Value * const priorAddressField = b->CreateInBoundsGEP(handle, indices);
    b->CreateStore(nullPointerFor(b, baseAddress, mUnderflow), priorAddressField);
    indices[1] = b->getInt32(Capacity);
    Value * const capacityField = b->CreateInBoundsGEP(handle, indices);
    Constant * const capacity = b->getSize(mInitialCapacity);
    b->CreateStore(capacity, capacityField);
    if (LLVM_UNLIKELY(mLinear)) {
        indices[1] = b->getInt32(ConcreteAddress);
        Value * const initialField = b->CreateInBoundsGEP(handle, indices);
        b->CreateStore(adjBaseAddress, initialField);
    }
}

void DynamicBuffer::releaseBuffer(BuilderPtr b) const {
    /* Free the dynamically allocated buffer(s). */
    Value * const handle = getHandle();
    FixedArray<Value *, 2> indices;
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(PriorAddress);
    Value * const priorAddressField = b->CreateInBoundsGEP(handle, indices);
    Value * const priorAddress = b->CreateLoad(priorAddressField);
    b->CreateFree(subtractUnderflow(b, priorAddress, mUnderflow));
    Constant * const nullPtr = nullPointerFor(b, priorAddress, mUnderflow);
    b->CreateStore(nullPtr, priorAddressField);
    indices[1] = b->getInt32(mLinear ? ConcreteAddress : BaseAddress);
    Value * const baseAddressField = b->CreateInBoundsGEP(handle, indices);
    Value * const baseAddress = b->CreateLoad(baseAddressField);
    b->CreateFree(subtractUnderflow(b, baseAddress, mUnderflow));
    b->CreateStore(nullPtr, baseAddressField);
}

void DynamicBuffer::setBaseAddress(BuilderPtr /* b */, Value * /* addr */) const {
    unsupported("setBaseAddress", "Dynamic");
}

Value * DynamicBuffer::getBaseAddress(BuilderPtr b) const {
    assert (getHandle());
    Value * const ptr = b->CreateInBoundsGEP(getHandle(), {b->getInt32(0), b->getInt32(BaseAddress)});
    return b->CreateLoad(ptr);
}

Value * DynamicBuffer::getOverflowAddress(BuilderPtr b) const {
    assert (getHandle());
    Value * const capacityPtr = b->CreateInBoundsGEP(getHandle(), {b->getInt32(0), b->getInt32(Capacity)});
    Value * const capacity = b->CreateLoad(capacityPtr);
    return b->CreateInBoundsGEP(getBaseAddress(b), capacity);
}

Value * DynamicBuffer::modByCapacity(BuilderPtr b, Value * const offset) const {
    assert (offset->getType()->isIntegerTy());
    if (mLinear || isCapacityGuaranteed(offset, mInitialCapacity)) {
        return offset;
    } else {
        assert (getHandle());
        Value * const capacityPtr = b->CreateInBoundsGEP(getHandle(), {b->getInt32(0), b->getInt32(Capacity)});
        Value * const capacity = b->CreateLoad(capacityPtr);
        return b->CreateURem(b->CreateZExtOrTrunc(offset, capacity->getType()), capacity);
    }
}

Value * DynamicBuffer::getCapacity(BuilderPtr b) const {
    assert (getHandle());
    Value * ptr = b->CreateInBoundsGEP(getHandle(), {b->getInt32(0), b->getInt32(Capacity)});
    return b->CreateMul(b->CreateLoad(ptr), b->getSize(b->getBitBlockWidth()));
}

void DynamicBuffer::setCapacity(BuilderPtr /* b */, Value * /* c */) const {
    unsupported("setCapacity", "Dynamic");
}

void DynamicBuffer::reserveCapacity(BuilderPtr b, Value * const produced, Value * const consumed, Value * const required, Constant * const overflowItems) const {

    SmallVector<char, 200> buf;
    raw_svector_ostream name(buf);

    assert ("unspecified module" && b.get() && b->getModule());

    name << "__Dynamic";
    if (mLinear) {
        name << "Linear";
    } else {
        name << "Circular";
    }
    name << "Buffer_reserveCapacity_";

    Type * ty = getBaseType();
    const auto streamCount = ty->getArrayNumElements();
    name << streamCount << 'x';
    ty = ty->getArrayElementType();
    ty = ty->getVectorElementType();
    const auto itemWidth = ty->getIntegerBitWidth();
    name << itemWidth
         << '_' << mUnderflow
         << '_' << mOverflow
         << '_' << mAddressSpace;

    Value * const myHandle = getHandle();

    Module * const m = b->getModule();
    Function * func = m->getFunction(name.str());
    if (func == nullptr) {

        const auto ip = b->saveIP();

        LLVMContext & C = m->getContext();
        IntegerType * const sizeTy = b->getSizeTy();
        FunctionType * funcTy = FunctionType::get(b->getVoidTy(), {myHandle->getType(), sizeTy, sizeTy, sizeTy, sizeTy}, false);
        func = Function::Create(funcTy, Function::InternalLinkage, name.str(), m);

        b->SetInsertPoint(BasicBlock::Create(C, "entry", func));

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

        setHandle(handle);

        const auto blockWidth = b->getBitBlockWidth();
        assert (is_power_2(blockWidth));
        const auto blockSize = blockWidth / 8;

        ConstantInt * const BLOCK_WIDTH = b->getSize(blockWidth);
        Constant * const CHUNK_SIZE = ConstantExpr::getSizeOf(mType);

        FixedArray<Value *, 2> indices;
        indices[0] = b->getInt32(0);
        indices[1] = b->getInt32(Capacity);

        Value * const capacityField = b->CreateInBoundsGEP(handle, indices);
        Value * const capacity = b->CreateLoad(capacityField);
        Value * consumedChunks = b->CreateUDiv(consumed, BLOCK_WIDTH);
        Value * const producedChunks = b->CreateCeilUDiv(produced, BLOCK_WIDTH);
        Value * const requiredCapacity = b->CreateAdd(produced, required);
        Value * const requiredChunks = b->CreateCeilUDiv(requiredCapacity, BLOCK_WIDTH);
        Value * const unconsumedChunks = b->CreateSub(producedChunks, consumedChunks);

        indices[1] = b->getInt32(BaseAddress);
        Value * const virtualBaseField = b->CreateInBoundsGEP(handle, indices);
        Value * const virtualBase = b->CreateLoad(virtualBaseField);
        assert (virtualBase->getType()->getPointerElementType() == mType);

        DataLayout DL(b->getModule());
        Type * const intPtrTy = DL.getIntPtrType(virtualBase->getType());

        if (mLinear) {

            indices[1] = b->getInt32(ConcreteAddress);
            Value * const concreteAddrField = b->CreateInBoundsGEP(handle, indices);
            Value * const concreteAddress = b->CreateLoad(concreteAddrField);
            //Value * const requiredSpacePtr = b->CreateInBoundsGEP(concreteAddress, requiredChunks);
            //Value * const unreadDataPtr = b->CreateInBoundsGEP(buffer, consumedChunks);

            Value * const bytesToCopy = b->CreateMul(unconsumedChunks, CHUNK_SIZE);

            BasicBlock * const copyBack = BasicBlock::Create(C, "copyBack", func);
            BasicBlock * const expandAndCopyBack = BasicBlock::Create(C, "expandAndCopyBack", func);
            BasicBlock * const updateBaseAddress = BasicBlock::Create(C, "updateBaseAddress", func);

            Value * const unreadDataPtr = b->CreateInBoundsGEP(virtualBase, consumedChunks);
            // Value * const unconsumedChunks = b->CreateSub(producedChunks, consumedChunks);

            Value * const chunksToOverwrite = b->CreateSub(requiredChunks, consumedChunks);
            Value * const overwriteUpToPtr = b->CreateInBoundsGEP(concreteAddress, chunksToOverwrite);
            Value * const canCopy = b->CreateICmpULE(overwriteUpToPtr, unreadDataPtr);

            b->CreateLikelyCondBr(canCopy, copyBack, expandAndCopyBack);

            b->SetInsertPoint(copyBack);
            b->CreateMemCpy(concreteAddress, unreadDataPtr, bytesToCopy, blockSize);
            BasicBlock * const copyBackExit = b->GetInsertBlock();
            b->CreateBr(updateBaseAddress);

            b->SetInsertPoint(expandAndCopyBack);
            Value * const initialCapacity = b->getSize(mInitialCapacity);
            Value * const newCapacity = b->CreateRoundUp(requiredChunks, initialCapacity);
            Value * requiredCapacity = newCapacity;
            if (mUnderflow || mOverflow) {
                Constant * const additionalCapacity = b->getSize(mUnderflow + mOverflow);
                requiredCapacity = b->CreateAdd(newCapacity, additionalCapacity);
            }
            Value * expandedBuffer = b->CreateCacheAlignedMalloc(mType, requiredCapacity, mAddressSpace);
            expandedBuffer = addUnderflow(b, expandedBuffer, mUnderflow);

            indices[1] = b->getInt32(PriorAddress);
            Value * const priorBufferField = b->CreateInBoundsGEP(handle, indices);
            Value * const priorBuffer = b->CreateLoad(priorBufferField);
            b->CreateFree(subtractUnderflow(b, priorBuffer, mUnderflow));
            b->CreateStore(concreteAddress, priorBufferField);
            b->CreateStore(expandedBuffer, concreteAddrField);
            b->CreateMemCpy(expandedBuffer, unreadDataPtr, bytesToCopy, blockSize);
            BasicBlock * const expandAndCopyBackExit = b->GetInsertBlock();
            b->CreateBr(updateBaseAddress);

            b->SetInsertPoint(updateBaseAddress);
            PHINode * const newBaseBuffer = b->CreatePHI(virtualBase->getType(), 2);
            newBaseBuffer->addIncoming(concreteAddress, copyBackExit);
            newBaseBuffer->addIncoming(expandedBuffer, expandAndCopyBackExit);
            PHINode * const updatedCapacity = b->CreatePHI(sizeTy, 2);
            updatedCapacity->addIncoming(capacity, copyBackExit);
            updatedCapacity->addIncoming(newCapacity, expandAndCopyBackExit);
            Value * const newBaseAddress = b->CreateGEP(newBaseBuffer, b->CreateNeg(consumedChunks));
            b->CreateStore(newBaseAddress, virtualBaseField);
            // how many item chunks between the virtual base address and the actual buffer start address?
            Value * const virtualAddrInt = b->CreatePtrToInt(virtualBase, intPtrTy);
            Value * const concreteAddrInt = b->CreatePtrToInt(concreteAddress, intPtrTy);
            Value * firstItemOffset = b->CreateSub(concreteAddrInt, virtualAddrInt);
            firstItemOffset = b->CreateUDiv(firstItemOffset, CHUNK_SIZE);
            Value * const discarded = b->CreateSub(consumedChunks, firstItemOffset);
            Value * const effectiveCapacity = b->CreateAdd(updatedCapacity, discarded);
            b->CreateStore(effectiveCapacity, capacityField);
            b->CreateRetVoid();

        } else { // Circular

            // make sure the new capacity is at least 2x the current capacity and a multiple of it
            if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
                Value * const check = b->CreateICmpUGE(requiredChunks, capacity);
                b->CreateAssert(check, "unnecessary buffer expansion occurred");
            }
            Value * const newCapacity = b->CreateRoundUp(requiredChunks, capacity);
            Value * requiredCapacity = newCapacity;
            if (mUnderflow || mOverflow) {
                Constant * const additionalCapacity = b->getSize(mUnderflow + mOverflow);
                requiredCapacity = b->CreateAdd(newCapacity, additionalCapacity);
            }

            Value * const totalBytesToCopy = b->CreateMul(unconsumedChunks, CHUNK_SIZE);
            Value * newBuffer = b->CreateCacheAlignedMalloc(mType, requiredCapacity, mAddressSpace);
            newBuffer = addUnderflow(b, newBuffer, mUnderflow);

            Value * const consumedOffset = b->CreateURem(consumedChunks, capacity);
            Value * const producedOffset = b->CreateURem(producedChunks, capacity);
            Value * const newConsumedOffset = b->CreateURem(consumedChunks, newCapacity);
            Value * const newProducedOffset = b->CreateURem(producedChunks, newCapacity);
            Value * const consumedOffsetEnd = b->CreateAdd(consumedOffset, unconsumedChunks);
            Value * const sourceLinear = b->CreateICmpULE(consumedOffsetEnd, producedOffset);
            Value * const newConsumedOffsetEnd = b->CreateAdd(newConsumedOffset, unconsumedChunks);
            Value * const targetLinear = b->CreateICmpULE(newConsumedOffsetEnd, newProducedOffset);
            Value * const linearCopy = b->CreateAnd(sourceLinear, targetLinear);

            Value * const consumedOffsetPtr = b->CreateInBoundsGEP(virtualBase, consumedOffset);
            Value * const newConsumedOffsetPtr = b->CreateInBoundsGEP(newBuffer, newConsumedOffset);

            BasicBlock * const copyLinear = BasicBlock::Create(C, "copyLinear", func);
            BasicBlock * const copyNonLinear = BasicBlock::Create(C, "copyNonLinear", func);
            BasicBlock * const storeNewBuffer = BasicBlock::Create(C, "storeNewBuffer", func);
            b->CreateCondBr(linearCopy, copyLinear, copyNonLinear);

            b->SetInsertPoint(copyLinear);
            b->CreateMemCpy(newConsumedOffsetPtr, consumedOffsetPtr, totalBytesToCopy, blockSize);
            b->CreateBr(storeNewBuffer);

            b->SetInsertPoint(copyNonLinear);
            Value * const bufferLength1 = b->CreateSub(capacity, consumedOffset);
            Value * const newBufferLength1 = b->CreateSub(newCapacity, newConsumedOffset);
            Value * const partialLength1 = b->CreateUMin(bufferLength1, newBufferLength1);
            Value * const copyEndPtr = b->CreateInBoundsGEP(virtualBase, b->CreateAdd(consumedOffset, partialLength1));
            Value * const copyEndPtrInt = b->CreatePtrToInt(copyEndPtr, intPtrTy);
            Value * const consumedOffsetPtrInt = b->CreatePtrToInt(consumedOffsetPtr, intPtrTy);
            Value * const bytesToCopy1 = b->CreateSub(copyEndPtrInt, consumedOffsetPtrInt);
            b->CreateMemCpy(newConsumedOffsetPtr, consumedOffsetPtr, bytesToCopy1, blockSize);
            Value * const sourceOffset = b->CreateURem(b->CreateAdd(consumedOffset, partialLength1), capacity);
            Value * const sourcePtr = b->CreateInBoundsGEP(virtualBase, sourceOffset);
            Value * const targetOffset = b->CreateURem(b->CreateAdd(newConsumedOffset, partialLength1), newCapacity);
            Value * const targetPtr = b->CreateInBoundsGEP(newBuffer, targetOffset);
            Value * const bytesToCopy2 = b->CreateSub(totalBytesToCopy, bytesToCopy1);
            b->CreateMemCpy(targetPtr, sourcePtr, bytesToCopy2, blockSize);
            b->CreateBr(storeNewBuffer);

            b->SetInsertPoint(storeNewBuffer);
            if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
                BasicBlock * const entryBlock = b->GetInsertBlock();
                BasicBlock * const checkNewBuffer = b->CreateBasicBlock();
                BasicBlock * const checkNewBufferExit = b->CreateBasicBlock();
                b->CreateBr(checkNewBuffer);

                b->SetInsertPoint(checkNewBuffer);
                PHINode * const index = b->CreatePHI(sizeTy, 2);
                index->addIncoming(consumedChunks, entryBlock);
                Value * const sourceOffset = b->CreateURem(index, capacity);
                Value * const sourcePtr = b->CreateInBoundsGEP(virtualBase, sourceOffset);
                Value * const targetOffset = b->CreateURem(index, newCapacity);
                Value * const targetPtr = b->CreateInBoundsGEP(newBuffer, targetOffset);
                assert (sourcePtr->getType() == targetPtr->getType());
                Value * const valid = b->CreateMemCmp(sourcePtr, targetPtr, CHUNK_SIZE);
                b->CreateAssertZero(valid, "dynamic buffer expansion failed to correctly copy the data");
                Value * const nextIndex = b->CreateAdd(index, b->getSize(1));
                index->addIncoming(nextIndex, checkNewBuffer);
                Value * const notDone = b->CreateICmpNE(nextIndex, producedChunks);
                b->CreateCondBr(notDone, checkNewBuffer, checkNewBufferExit);

                b->SetInsertPoint(checkNewBufferExit);
            }
            indices[1] = b->getInt32(PriorAddress);
            Value * const priorBufferField = b->CreateInBoundsGEP(handle, indices);
            Value * const priorBuffer = b->CreateLoad(priorBufferField);
            b->CreateStore(virtualBase, priorBufferField);
            b->CreateStore(newBuffer, virtualBaseField);
            b->CreateStore(newCapacity, capacityField);
            b->CreateFree(subtractUnderflow(b, priorBuffer, mUnderflow));
            b->CreateRetVoid();
        }

        b->restoreIP(ip);
        setHandle(myHandle);
    }
    b->CreateCall(func, { myHandle, produced, consumed, required, overflowItems ? overflowItems : b->getSize(0) });
}

// Constructors

ExternalBuffer::ExternalBuffer(BuilderPtr b, Type * const type,
                               const bool linear,
                               const unsigned AddressSpace)
: StreamSetBuffer(BufferKind::ExternalBuffer, b, type, 0, 0, linear, AddressSpace) {

}

StaticBuffer::StaticBuffer(BuilderPtr b, Type * const type,
                           const size_t capacity, const size_t overflowSize, const size_t underflowSize,
                           const bool linear, const unsigned AddressSpace)
: InternalBuffer(BufferKind::StaticBuffer, b, type, overflowSize, underflowSize, linear, AddressSpace)
, mCapacity(capacity) {
    #ifndef NDEBUG
    assert ("static buffer cannot have 0 capacity" && capacity);
    assert ("static buffer capacity must be at least twice its max(underflow, overflow)"
            && (capacity >= (std::max(underflowSize, overflowSize) * 2)));
    #endif
}

DynamicBuffer::DynamicBuffer(BuilderPtr b, Type * const type,
                             const size_t initialCapacity, const size_t overflowSize, const size_t underflowSize,
                             const bool linear, const unsigned AddressSpace)
: InternalBuffer(BufferKind::DynamicBuffer, b, type, overflowSize, underflowSize, linear, AddressSpace)
, mInitialCapacity(initialCapacity) {
    #ifndef NDEBUG
    assert ("dynamic buffer cannot have 0 initial capacity" && initialCapacity);
    assert ("dynamic buffer initial capacity must be at least twice its max(underflow, overflow)"
            && (initialCapacity >= (std::max(underflowSize, overflowSize) * 2)));
    #endif
}

inline InternalBuffer::InternalBuffer(const BufferKind k, BuilderPtr b, Type * const baseType,
                                      const size_t overflowSize, const size_t underflowSize,
                                      const bool linear, const unsigned AddressSpace)
: StreamSetBuffer(k, b, baseType, overflowSize, underflowSize, linear, AddressSpace) {

}

inline StreamSetBuffer::StreamSetBuffer(const BufferKind k, BuilderPtr b, Type * const baseType,
                                        const size_t overflowSize, const size_t underflowSize,
                                        const bool linear, const unsigned AddressSpace)
: mBufferKind(k)
, mHandle(nullptr)
, mType(resolveType(b, baseType))
, mOverflow(overflowSize)
, mUnderflow(underflowSize)
, mAddressSpace(AddressSpace)
, mBaseType(baseType)
, mLinear(linear) {

}

StreamSetBuffer::~StreamSetBuffer() { }

}
