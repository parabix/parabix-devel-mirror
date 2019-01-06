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

Value * KernelBuilder::getScalarFieldPtr(const StringRef fieldName) {
    return mKernel->getScalarFieldPtr(*this, fieldName);
}

Value * KernelBuilder::getScalarField(const StringRef fieldName) {
    return CreateLoad(getScalarFieldPtr(fieldName));
}

void KernelBuilder::setScalarField(const StringRef fieldName, Value * const value) {
    CreateStore(value, getScalarFieldPtr(fieldName));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getProcessedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getProcessedItemCount(const StringRef name) {
    return CreateLoad(mKernel->getProcessedInputItemsPtr(name));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setProcessedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::setProcessedItemCount(const StringRef name, Value * value) {
    CreateStore(value, mKernel->getProcessedInputItemsPtr(name));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getProducedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getProducedItemCount(const StringRef name) {
    return CreateLoad(mKernel->getProducedOutputItemsPtr(name));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setProducedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::setProducedItemCount(const StringRef name, Value * value) {
    CreateStore(value, mKernel->getProducedOutputItemsPtr(name));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getConsumedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getConsumedItemCount(const StringRef name) const {
    return mKernel->getConsumedOutputItems(name);
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
        report_fatal_error(mKernel->getName() + " does not have CanTerminateEarly or MustExplicitlyTerminate set.");
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
 * @brief getKernelName
 ** ------------------------------------------------------------------------------------------------------------- */
std::string KernelBuilder::getKernelName() const {
    return mKernel->getName();
}

}
