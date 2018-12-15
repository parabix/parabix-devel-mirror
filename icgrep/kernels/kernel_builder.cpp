#include "kernel_builder.h"
#include <toolchain/toolchain.h>
#include <kernels/kernel.h>
#include <kernels/streamset.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/Module.h>

using namespace llvm;

inline static bool is_power_2(const uint64_t n) {
    return ((n & (n - 1)) == 0) && n;
}

namespace kernel {

using Port = Kernel::Port;

inline Value * KernelBuilder::getScalarFieldPtr(Value * const handle, Value * const index) {
    assert ("handle cannot be null" && handle);
    assert ("index cannot be null" && index);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        CreateAssert(handle, "getScalarFieldPtr: handle cannot be null!");
    }
    #ifndef NDEBUG
    const Function * const handleFunction = isa<Argument>(handle) ? cast<Argument>(handle)->getParent() : cast<Instruction>(handle)->getParent()->getParent();
    const Function * const builderFunction = GetInsertBlock()->getParent();
    assert ("handle is not from the current function." && handleFunction == builderFunction);
    #endif
    return CreateGEP(handle, {getInt32(0), index});
}

#warning TODO: make get scalar field able to get I/O scalars

inline Value * KernelBuilder::getScalarFieldPtr(Value * const handle, const std::string & fieldName) {
    ConstantInt * const index = getInt32(mKernel->getScalarIndex(fieldName));
    return getScalarFieldPtr(handle, index);
}

Value * KernelBuilder::getScalarFieldPtr(Value * const index) {
    return getScalarFieldPtr(mKernel->getHandle(), index);
}

Value * KernelBuilder::getScalarFieldPtr(const std::string & fieldName) {
    return getScalarFieldPtr(mKernel->getHandle(), fieldName);
}

Value * KernelBuilder::getScalarField(const std::string & fieldName) {
    Value * const ptr = getScalarFieldPtr(fieldName);
    return CreateLoad(ptr, fieldName);
}

void KernelBuilder::setScalarField(const std::string & fieldName, Value * const value) {
    Value * const ptr = getScalarFieldPtr(fieldName);
    CreateStore(value, ptr);
}

Value * KernelBuilder::getCycleCountPtr() {
    return getScalarFieldPtr(CYCLECOUNT_SCALAR);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNamedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getNamedItemCount(const std::string & name, const std::string & suffix) {
    const ProcessingRate & rate = mKernel->getStreamBinding(name).getRate();
    Value * itemCount = nullptr;
    if (LLVM_UNLIKELY(rate.isRelative())) {
        Port port; unsigned index;
        std::tie(port, index) = mKernel->getStreamPort(rate.getReference());
        if (port == Port::Input) {
            itemCount = getProcessedItemCount(rate.getReference());
        } else {
            itemCount = getProducedItemCount(rate.getReference());
        }
        itemCount = CreateMul2(itemCount, rate.getRate());
    } else {
        itemCount = getScalarField(name + suffix);
    }
    return itemCount;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setNamedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::setNamedItemCount(const std::string & name, const std::string & suffix, Value * const value) {
    const ProcessingRate & rate = mKernel->getStreamBinding(name).getRate();
    if (LLVM_UNLIKELY(rate.isDerived())) {
        report_fatal_error("cannot set item count: " + name + " is a derived rate stream");
    }
    if (codegen::DebugOptionIsSet(codegen::TraceCounts)) {
        CallPrintInt(mKernel->getName() + ": " + name + suffix, value);
    }
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        Value * const current = getScalarField(name + suffix);
        CreateAssert(CreateICmpUGE(value, current), name + suffix + " must be monotonically non-decreasing");
    }
    setScalarField(name + suffix, value);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getAvailableItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getAvailableItemCount(const std::string & name) {
    return mKernel->getAvailableInputItems(name);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getAccessibleItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getAccessibleItemCount(const std::string & name) {
    return mKernel->getAccessibleInputItems(name);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getTerminationSignal
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getTerminationSignal() {
    Value * const ptr = mKernel->getTerminationSignalPtr();
    if (ptr) {
        return CreateLoad(ptr);
    } else {
        return getFalse();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setTerminationSignal
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::setTerminationSignal(Value * const value) {
    assert (value);
    assert (value->getType() == getInt1Ty());
    if (codegen::DebugOptionIsSet(codegen::TraceCounts)) {
        CallPrintInt(mKernel->getName() + ": setTerminationSignal", value);
    }
    Value * const ptr = mKernel->getTerminationSignalPtr();
    if (LLVM_UNLIKELY(ptr == nullptr)) {
        llvm::report_fatal_error(mKernel->getName() + " does not have CanTerminateEarly or MustExplicitlyTerminate set.");
    }
    CreateStore(value, ptr);
}

Value * KernelBuilder::getInputStreamBlockPtr(const std::string & name, Value * const streamIndex, Value * const blockOffset) {
    const StreamSetBuffer * const buf = mKernel->getInputStreamSetBuffer(name);
    Value * const processed = getProcessedItemCount(name);
    Value * blockIndex = CreateLShr(processed, std::log2(getBitBlockWidth()));
    if (blockOffset) {
        blockIndex = CreateAdd(blockIndex, CreateZExtOrTrunc(blockOffset, blockIndex->getType()));
    }
    return buf->getStreamBlockPtr(this, streamIndex, blockIndex);
}

Value * KernelBuilder::getInputStreamPackPtr(const std::string & name, Value * const streamIndex, Value * const packIndex, Value * const blockOffset) {
    const StreamSetBuffer * const buf = mKernel->getInputStreamSetBuffer(name);
    Value * const processed = getProcessedItemCount(name);
    Value * blockIndex = CreateLShr(processed, std::log2(getBitBlockWidth()));
    if (blockOffset) {
        blockIndex = CreateAdd(blockIndex, CreateZExtOrTrunc(blockOffset, blockIndex->getType()));
    }
    return buf->getStreamPackPtr(this, streamIndex, blockIndex, packIndex);
}

Value * KernelBuilder::loadInputStreamBlock(const std::string & name, Value * const streamIndex, Value * const blockOffset) {
    return CreateBlockAlignedLoad(getInputStreamBlockPtr(name, streamIndex, blockOffset));
}

Value * KernelBuilder::loadInputStreamPack(const std::string & name, Value * const streamIndex, Value * const packIndex, Value * const blockOffset) {
    return CreateBlockAlignedLoad(getInputStreamPackPtr(name, streamIndex, packIndex, blockOffset));
}

Value * KernelBuilder::getInputStreamSetCount(const std::string & name) {
    const StreamSetBuffer * const buf = mKernel->getInputStreamSetBuffer(name);
    return buf->getStreamSetCount(this);
}

Value * KernelBuilder::getOutputStreamBlockPtr(const std::string & name, Value * streamIndex, Value * const blockOffset) {
    const StreamSetBuffer * const buf = mKernel->getOutputStreamSetBuffer(name);
    Value * const produced = getProducedItemCount(name);
    Value * blockIndex = CreateLShr(produced, std::log2(getBitBlockWidth()));
    if (blockOffset) {
        blockIndex = CreateAdd(blockIndex, CreateZExtOrTrunc(blockOffset, blockIndex->getType()));
    }
    return buf->getStreamBlockPtr(this, streamIndex, blockIndex);
}

Value * KernelBuilder::getOutputStreamPackPtr(const std::string & name, Value * streamIndex, Value * packIndex, Value * blockOffset) {
    const StreamSetBuffer * const buf = mKernel->getOutputStreamSetBuffer(name);
    Value * const produced = getProducedItemCount(name);
    Value * blockIndex = CreateLShr(produced, std::log2(getBitBlockWidth()));
    if (blockOffset) {
        blockIndex = CreateAdd(blockIndex, CreateZExtOrTrunc(blockOffset, blockIndex->getType()));
    }
    return buf->getStreamPackPtr(this, streamIndex, blockIndex, packIndex);
}

StoreInst * KernelBuilder::storeOutputStreamBlock(const std::string & name, Value * streamIndex, Value * blockOffset, Value * toStore) {
    Value * const ptr = getOutputStreamBlockPtr(name, streamIndex, blockOffset);
    Type * const storeTy = toStore->getType();
    Type * const ptrElemTy = ptr->getType()->getPointerElementType();
    if (LLVM_UNLIKELY(storeTy != ptrElemTy)) {
        if (LLVM_LIKELY(storeTy->canLosslesslyBitCastTo(ptrElemTy))) {
            toStore = CreateBitCast(toStore, ptrElemTy);
        } else {
            std::string tmp;
            raw_string_ostream out(tmp);
            out << "invalid type conversion when calling storeOutputStreamBlock on " <<  name << ": ";
            ptrElemTy->print(out);
            out << " vs. ";
            storeTy->print(out);
        }
    }
    return CreateBlockAlignedStore(toStore, ptr);
}

StoreInst * KernelBuilder::storeOutputStreamPack(const std::string & name, Value * streamIndex, Value * packIndex, Value * blockOffset, Value * toStore) {
    Value * const ptr = getOutputStreamPackPtr(name, streamIndex, packIndex, blockOffset);
    Type * const storeTy = toStore->getType();
    Type * const ptrElemTy = ptr->getType()->getPointerElementType();
    if (LLVM_UNLIKELY(storeTy != ptrElemTy)) {
        if (LLVM_LIKELY(storeTy->canLosslesslyBitCastTo(ptrElemTy))) {
            toStore = CreateBitCast(toStore, ptrElemTy);
        } else {
            std::string tmp;
            raw_string_ostream out(tmp);
            out << "invalid type conversion when calling storeOutputStreamPack on " <<  name << ": ";
            ptrElemTy->print(out);
            out << " vs. ";
            storeTy->print(out);
        }
    }
    return CreateBlockAlignedStore(toStore, ptr);
}

Value * KernelBuilder::getOutputStreamSetCount(const std::string & name) {
    const StreamSetBuffer * const buf = mKernel->getOutputStreamSetBuffer(name);
    return buf->getStreamSetCount(this);
}

Value * KernelBuilder::getRawInputPointer(const std::string & name, Value * absolutePosition) {
    const StreamSetBuffer * const buf = mKernel->getInputStreamSetBuffer(name);
    return buf->getRawItemPointer(this, absolutePosition);
}

Value * KernelBuilder::getRawOutputPointer(const std::string & name, Value * absolutePosition) {
    const StreamSetBuffer * const buf = mKernel->getOutputStreamSetBuffer(name);
    return buf->getRawItemPointer(this, absolutePosition);
}

Value * KernelBuilder::getBaseAddress(const std::string & name) {
    return mKernel->getStreamSetBuffer(name)->getBaseAddress(this);
}

void KernelBuilder::setBaseAddress(const std::string & name, Value * const addr) {
    return mKernel->getStreamSetBuffer(name)->setBaseAddress(this, addr);
}

Value * KernelBuilder::getCapacity(const std::string & name) {
    return mKernel->getStreamSetBuffer(name)->getCapacity(this);
}

void KernelBuilder::setCapacity(const std::string & name, Value * capacity) {
    mKernel->getStreamSetBuffer(name)->setCapacity(this, capacity);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief CreateUDiv2
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::CreateUDiv2(Value * const number, const ProcessingRate::RateValue & divisor, const Twine & Name) {
    if (divisor.numerator() == 1 && divisor.denominator() == 1) {
        return number;
    }
    Constant * const n = ConstantInt::get(number->getType(), divisor.numerator());
    if (LLVM_LIKELY(divisor.denominator() == 1)) {
        return CreateUDiv(number, n, Name);
    } else {
        Constant * const d = ConstantInt::get(number->getType(), divisor.denominator());
        return CreateUDiv(CreateMul(number, d), n);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief CreateCeilUDiv2
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::CreateCeilUDiv2(Value * const number, const ProcessingRate::RateValue & divisor, const Twine & Name) {
    if (divisor.numerator() == 1 && divisor.denominator() == 1) {
        return number;
    }
    Constant * const n = ConstantInt::get(number->getType(), divisor.numerator());
    if (LLVM_LIKELY(divisor.denominator() == 1)) {
        return CreateCeilUDiv(number, n, Name);
    } else {
        //   ⌊(num + ratio - 1) / ratio⌋
        // = ⌊(num - 1) / (n/d)⌋ + (ratio/ratio)
        // = ⌊(d * (num - 1)) / n⌋ + 1
        Constant * const ONE = ConstantInt::get(number->getType(), 1);
        Constant * const d = ConstantInt::get(number->getType(), divisor.denominator());
        return CreateAdd(CreateUDiv(CreateMul(CreateSub(number, ONE), d), n), ONE, Name);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief CreateMul2
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::CreateMul2(Value * const number, const ProcessingRate::RateValue & factor, const Twine & Name) {
    if (factor.numerator() == 1 && factor.denominator() == 1) {
        return number;
    }
    Constant * const n = ConstantInt::get(number->getType(), factor.numerator());
    if (LLVM_LIKELY(factor.denominator() == 1)) {
        return CreateMul(number, n, Name);
    } else {
        Constant * const d = ConstantInt::get(number->getType(), factor.denominator());
        return CreateUDiv(CreateMul(number, n), d, Name);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief CreateMulCeil2
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::CreateCeilUMul2(Value * const number, const ProcessingRate::RateValue & factor, const Twine & Name) {
    if (factor.denominator() == 1) {
        return CreateMul2(number, factor, Name);
    }
    Constant * const n = ConstantInt::get(number->getType(), factor.numerator());
    Constant * const d = ConstantInt::get(number->getType(), factor.denominator());
    return CreateCeilUDiv(CreateMul(number, n), d, Name);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief resolveStreamSetType
 ** ------------------------------------------------------------------------------------------------------------- */
Type * KernelBuilder::resolveStreamSetType(Type * const streamSetType) {
    // TODO: Should this function be here? in StreamSetBuffer? or in Binding?
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
            type = getBitBlockType();
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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getKernelName
 ** ------------------------------------------------------------------------------------------------------------- */
std::string KernelBuilder::getKernelName() const {
    return mKernel->getName();
}

}
