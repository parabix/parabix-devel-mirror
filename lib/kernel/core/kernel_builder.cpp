#include <kernel/core/kernel_builder.h>
#include <kernel/core/kernel_compiler.h>
#include <toolchain/toolchain.h>
#include <kernel/core/streamset.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/Module.h>

using namespace llvm;

namespace kernel {

using PortType = Kernel::PortType;

#define COMPILER (not_null<KernelCompiler *>(mCompiler))

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getHandle
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getHandle() const noexcept {
    return COMPILER->getHandle();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getThreadLocalHandle
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getThreadLocalHandle() const noexcept {
    return COMPILER->getThreadLocalHandle();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getScalarFieldPtr
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getScalarFieldPtr(const StringRef fieldName) {
    return COMPILER->getScalarFieldPtr(this, fieldName);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getScalarField
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getScalarField(const StringRef fieldName) {
    return CreateLoad(getScalarFieldPtr(fieldName));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setScalarField
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::setScalarField(const StringRef fieldName, Value * const value) {
    CreateStore(value, getScalarFieldPtr(fieldName));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief CreateMonitoredScalarFieldLoad
 ** ------------------------------------------------------------------------------------------------------------- */
LoadInst * KernelBuilder::CreateMonitoredScalarFieldLoad(const StringRef fieldName, Value * internalPtr) {
    Value * scalarPtr = getScalarFieldPtr(fieldName);
    Value * scalarEndPtr = CreateGEP(scalarPtr, getInt32(1));
    Value * internalEndPtr = CreateGEP(internalPtr, getInt32(1));
    Value * scalarAddr = CreatePtrToInt(scalarPtr, getSizeTy());
    Value * scalarEndAddr = CreatePtrToInt(scalarEndPtr, getSizeTy());
    Value * internalAddr = CreatePtrToInt(internalPtr, getSizeTy());
    Value * internalEndAddr = CreatePtrToInt(internalEndPtr, getSizeTy());
    Value * inBounds = CreateAnd(CreateICmpULE(scalarAddr, internalAddr), CreateICmpUGE(scalarEndAddr, internalEndAddr));
    __CreateAssert(inBounds, "Access (%" PRIx64 ",%" PRIx64 ") to scalar " + fieldName + " out of bounds (%" PRIx64 ",%" PRIx64 ").",
                   {scalarAddr, scalarEndAddr, internalAddr, internalEndAddr});
    return CreateLoad(internalPtr);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief CreateMonitoredScalarFieldStore
 ** ------------------------------------------------------------------------------------------------------------- */
StoreInst * KernelBuilder::CreateMonitoredScalarFieldStore(const StringRef fieldName, Value * toStore, Value * internalPtr) {
    DataLayout DL(getModule());
    Value * scalarPtr = getScalarFieldPtr(fieldName);
    Value * scalarEndPtr = CreateGEP(scalarPtr, getInt32(1));
    Value * scalarAddr = CreatePtrToInt(scalarPtr, getSizeTy());
    Value * scalarEndAddr = CreatePtrToInt(scalarEndPtr, getSizeTy());
    Value * internalAddr = CreatePtrToInt(internalPtr, getSizeTy());
    Value * internalEndAddr = CreateAdd(internalAddr, getSize(DL.getTypeAllocSize(toStore->getType())));
    Value * inBounds = CreateAnd(CreateICmpULE(scalarAddr, internalAddr), CreateICmpUGE(scalarEndAddr, internalEndAddr));
    __CreateAssert(inBounds, "Store (%" PRIx64 ",%" PRIx64 ") to scalar " + fieldName + " out of bounds (%" PRIx64 ",%" PRIx64 ").",
                   {scalarAddr, scalarEndAddr, internalAddr, internalEndAddr});
    return CreateStore(toStore, internalPtr);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getTerminationSignal
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::getTerminationSignal() {
    Value * const ptr = COMPILER->getTerminationSignalPtr();
    if (ptr) {
        return CreateIsNotNull(CreateLoad(ptr));
    } else {
        return getFalse();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setTerminationSignal
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::setTerminationSignal(Value * const value) {
    Value * const ptr = COMPILER->getTerminationSignalPtr();
    if (LLVM_UNLIKELY(ptr == nullptr)) {
        report_fatal_error(COMPILER->getName() + " does not have CanTerminateEarly or MustExplicitlyTerminate set.");
    }
    CreateStore(value, ptr);    
}

Value * KernelBuilder::getInputStreamBlockPtr(const StringRef name, Value * const streamIndex, Value * const blockOffset) {
    const StreamSetBuffer * const buf = COMPILER->getInputStreamSetBuffer(name);
    assert ("buffer is not accessible in this context!" && buf->getHandle());
    Value * const processed = getProcessedItemCount(name);
    Value * blockIndex = CreateLShr(processed, std::log2(getBitBlockWidth()));
    if (blockOffset) {
        blockIndex = CreateAdd(blockIndex, CreateZExtOrTrunc(blockOffset, blockIndex->getType()));
    }
    return buf->getStreamBlockPtr(this, buf->getBaseAddress(this), streamIndex, blockIndex);
}

Value * KernelBuilder::getInputStreamPackPtr(const StringRef name, Value * const streamIndex, Value * const packIndex, Value * const blockOffset) {
    const StreamSetBuffer * const buf = COMPILER->getInputStreamSetBuffer(name);
    assert ("buffer is not accessible in this context!" && buf->getHandle());
    Value * const processed = getProcessedItemCount(name);
    Value * blockIndex = CreateLShr(processed, std::log2(getBitBlockWidth()));
    if (blockOffset) {
        blockIndex = CreateAdd(blockIndex, CreateZExtOrTrunc(blockOffset, blockIndex->getType()));
    }
    return buf->getStreamPackPtr(this, buf->getBaseAddress(this), streamIndex, blockIndex, packIndex);
}

Value * KernelBuilder::loadInputStreamBlock(const StringRef name, Value * const streamIndex, Value * const blockOffset) {
    return CreateBlockAlignedLoad(getInputStreamBlockPtr(name, streamIndex, blockOffset));
}

Value * KernelBuilder::loadInputStreamPack(const StringRef name, Value * const streamIndex, Value * const packIndex, Value * const blockOffset) {
    return CreateBlockAlignedLoad(getInputStreamPackPtr(name, streamIndex, packIndex, blockOffset));
}

Value * KernelBuilder::getInputStreamSetCount(const StringRef name) {
    const StreamSetBuffer * const buf = COMPILER->getInputStreamSetBuffer(name);
    return buf->getStreamSetCount(this);
}

Value * KernelBuilder::getOutputStreamBlockPtr(const StringRef name, Value * streamIndex, Value * const blockOffset) {
    const StreamSetBuffer * const buf = COMPILER->getOutputStreamSetBuffer(name);
    assert ("buffer is not accessible in this context!" && buf->getHandle());
    Value * const produced = getProducedItemCount(name);
    Value * blockIndex = CreateLShr(produced, std::log2(getBitBlockWidth()));
    if (blockOffset) {
        blockIndex = CreateAdd(blockIndex, CreateZExtOrTrunc(blockOffset, blockIndex->getType()));
    }
    return buf->getStreamBlockPtr(this, buf->getBaseAddress(this), streamIndex, blockIndex);
}

Value * KernelBuilder::getOutputStreamPackPtr(const StringRef name, Value * streamIndex, Value * packIndex, Value * blockOffset) {
    const StreamSetBuffer * const buf = COMPILER->getOutputStreamSetBuffer(name);
    assert ("buffer is not accessible in this context!" && buf->getHandle());
    Value * const produced = getProducedItemCount(name);
    Value * blockIndex = CreateLShr(produced, std::log2(getBitBlockWidth()));
    if (blockOffset) {
        blockIndex = CreateAdd(blockIndex, CreateZExtOrTrunc(blockOffset, blockIndex->getType()));
    }
    return buf->getStreamPackPtr(this, buf->getBaseAddress(this), streamIndex, blockIndex, packIndex);
}

StoreInst * KernelBuilder::storeOutputStreamBlock(const StringRef name, Value * streamIndex, Value * blockOffset, Value * toStore) {
    SmallVector<char, 256> tmp;
    raw_svector_ostream out(tmp);
    out << COMPILER->getName() << '.' << name;
    if (ConstantInt * c = dyn_cast<ConstantInt>(streamIndex)) {
        out << "[" << c->getZExtValue() << "]";
    }
    toStore->setName(out.str());
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

StoreInst * KernelBuilder::storeOutputStreamPack(const StringRef name, Value * streamIndex, Value * packIndex, Value * blockOffset, Value * toStore) {
    SmallVector<char, 256> tmp;
    raw_svector_ostream out(tmp);
    out << COMPILER->getName() << '.' << name;
    if (ConstantInt * c = dyn_cast<ConstantInt>(streamIndex)) {
        out << "[" << c->getZExtValue() << "]";
    }
    toStore->setName(out.str());
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

Value * KernelBuilder::getOutputStreamSetCount(const StringRef name) {
    const StreamSetBuffer * const buf = COMPILER->getOutputStreamSetBuffer(name);
    return buf->getStreamSetCount(this);
}

Value * KernelBuilder::getRawInputPointer(const StringRef name, Value * absolutePosition) {
    const StreamSetBuffer * const buf = COMPILER->getInputStreamSetBuffer(name);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        Value * const sanityCheck = CreateICmpEQ(buf->getStreamSetCount(this), getSize(1));
        CreateAssert(sanityCheck, "stream index must be explicit");
    }
    return buf->getRawItemPointer(this, getSize(0), absolutePosition);
}

Value * KernelBuilder::getRawInputPointer(const StringRef name, Value * const streamIndex, Value * absolutePosition) {
    const StreamSetBuffer * const buf = COMPILER->getInputStreamSetBuffer(name);
    return buf->getRawItemPointer(this, streamIndex, absolutePosition);
}

Value * KernelBuilder::getRawOutputPointer(const StringRef name, Value * absolutePosition) {
    const StreamSetBuffer * const buf = COMPILER->getOutputStreamSetBuffer(name);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        Value * const sanityCheck = CreateICmpEQ(buf->getStreamSetCount(this), getSize(1));
        CreateAssert(sanityCheck, "stream index must be explicit");
    }
    return buf->getRawItemPointer(this, getSize(0), absolutePosition);
}

Value * KernelBuilder::getRawOutputPointer(const StringRef name, Value * const streamIndex, Value * absolutePosition) {
    const StreamSetBuffer * const buf = COMPILER->getOutputStreamSetBuffer(name);
    return buf->getRawItemPointer(this, streamIndex, absolutePosition);
}

Value * KernelBuilder::getBaseAddress(const StringRef name) {
    return COMPILER->getStreamSetBuffer(name)->getBaseAddress(this);
}

void KernelBuilder::setBaseAddress(const StringRef name, Value * const addr) {
    return COMPILER->getStreamSetBuffer(name)->setBaseAddress(this, addr);
}

Value * KernelBuilder::getCapacity(const StringRef name) {
    return COMPILER->getStreamSetBuffer(name)->getCapacity(this);
}

void KernelBuilder::setCapacity(const StringRef name, Value * capacity) {
    COMPILER->getStreamSetBuffer(name)->setCapacity(this, capacity);
}


Value * KernelBuilder::getAvailableItemCount(const StringRef name) const noexcept {
    return COMPILER->getAvailableInputItems(name);
}

Value * KernelBuilder::getAccessibleItemCount(const StringRef name) const noexcept {
    return COMPILER->getAccessibleInputItems(name);
}

Value * KernelBuilder::getProcessedItemCount(const StringRef name) {
    return CreateLoad(COMPILER->getProcessedInputItemsPtr(name));
}

void KernelBuilder::setProcessedItemCount(const StringRef name, Value * value) {
    CreateStore(value, COMPILER->getProcessedInputItemsPtr(name));
}

Value * KernelBuilder::getProducedItemCount(const StringRef name) {
    return CreateLoad(COMPILER->getProducedOutputItemsPtr(name));
}

void KernelBuilder::setProducedItemCount(const StringRef name, Value * value) {
    CreateStore(value, COMPILER->getProducedOutputItemsPtr(name));
}

Value * KernelBuilder::getWritableOutputItems(const StringRef name) const noexcept {
    return COMPILER->getWritableOutputItems(name);
}

Value * KernelBuilder::getConsumedItemCount(const StringRef name) const noexcept {
    return COMPILER->getConsumedOutputItems(name);
}

// internal state

Value * KernelBuilder::getNumOfStrides() const noexcept {
    return COMPILER->getNumOfStrides();
}

Value * KernelBuilder::getExternalSegNo() const noexcept {
    return COMPILER->getExternalSegNo();
}

Value * KernelBuilder::isFinal() const noexcept {
    return COMPILER->isFinal();
}

// input streamset bindings

const Bindings & KernelBuilder::getInputStreamSetBindings() const noexcept {
    return COMPILER->getInputScalarBindings();
}

const Binding & KernelBuilder::getInputStreamSetBinding(const unsigned i) const noexcept {
    return COMPILER->getInputStreamSetBinding(i);
}

const Binding & KernelBuilder::getInputStreamSetBinding(const StringRef name) const noexcept {
    return COMPILER->getInputStreamSetBinding(name);
}

StreamSet * KernelBuilder::getInputStreamSet(const unsigned i) const noexcept {
    return COMPILER->getInputStreamSet(i);
}

StreamSet * KernelBuilder::getInputStreamSet(const StringRef name) const noexcept {
    return COMPILER->getInputStreamSet(name);
}

void KernelBuilder::setInputStreamSet(const StringRef name, StreamSet * value) noexcept {
    return COMPILER->setInputStreamSet(name, value);
}

unsigned KernelBuilder::getNumOfStreamInputs() const noexcept {
    return COMPILER->getNumOfStreamInputs();
}

// input streamsets

StreamSetBuffer * KernelBuilder::getInputStreamSetBuffer(const unsigned i) const noexcept {
    return COMPILER->getInputStreamSetBuffer(i);
}

StreamSetBuffer * KernelBuilder::getInputStreamSetBuffer(const StringRef name) const noexcept {
    return COMPILER->getInputStreamSetBuffer(name);
}

// output streamset bindings

const Bindings & KernelBuilder::getOutputStreamSetBindings() const noexcept {
    return COMPILER->getOutputStreamSetBindings();
}

const Binding & KernelBuilder::getOutputStreamSetBinding(const unsigned i) const noexcept {
    return COMPILER->getOutputStreamSetBinding(i);
}

const Binding & KernelBuilder::getOutputStreamSetBinding(const StringRef name) const noexcept {
    return COMPILER->getOutputStreamSetBinding(name);
}

StreamSet * KernelBuilder::getOutputStreamSet(const unsigned i) const noexcept {
    return COMPILER->getOutputStreamSet(i);
}

StreamSet * KernelBuilder::getOutputStreamSet(const StringRef name) const noexcept {
    return COMPILER->getOutputStreamSet(name);
}

void KernelBuilder::setOutputStreamSet(const StringRef name, StreamSet * value) noexcept {
    return COMPILER->setOutputStreamSet(name, value);
}

unsigned KernelBuilder::getNumOfStreamOutputs() const noexcept {
    return COMPILER->getNumOfStreamOutputs();
}

// output streamsets

StreamSetBuffer * KernelBuilder::getOutputStreamSetBuffer(const unsigned i) const noexcept {
    return COMPILER->getOutputStreamSetBuffer(i);
}

StreamSetBuffer * KernelBuilder::getOutputStreamSetBuffer(const StringRef name) const noexcept {
    return COMPILER->getOutputStreamSetBuffer(name);
}

// input scalar bindings

const Bindings & KernelBuilder::getInputScalarBindings() const noexcept {
    return COMPILER->getInputScalarBindings();
}

const Binding & KernelBuilder::getInputScalarBinding(const unsigned i) const noexcept {
    return COMPILER->getInputScalarBinding(i);
}

const Binding & KernelBuilder::getInputScalarBinding(const StringRef name) const noexcept {
    return COMPILER->getInputScalarBinding(name);
}

unsigned KernelBuilder::getNumOfScalarInputs() const noexcept {
    return COMPILER->getNumOfScalarInputs();
}

// input scalars

Scalar * KernelBuilder::getInputScalar(const unsigned i) noexcept {
    return COMPILER->getInputScalar(i);
}

Scalar * KernelBuilder::getInputScalar(const StringRef name) noexcept {
    return COMPILER->getInputScalar(name);
}

// output scalar bindings

const Bindings & KernelBuilder::getOutputScalarBindings() const noexcept {
    return COMPILER->getOutputScalarBindings();
}

const Binding & KernelBuilder::getOutputScalarBinding(const unsigned i) const noexcept {
    return COMPILER->getOutputScalarBinding(i);
}

const Binding & KernelBuilder::getOutputScalarBinding(const StringRef name) const noexcept {
    return COMPILER->getOutputScalarBinding(name);
}

unsigned KernelBuilder::getNumOfScalarOutputs() const noexcept {
    return COMPILER->getNumOfScalarOutputs();
}

// output scalars

Scalar * KernelBuilder::getOutputScalar(const unsigned i) noexcept {
    return COMPILER->getOutputScalar(i);
}

Scalar * KernelBuilder::getOutputScalar(const StringRef name) noexcept {
    return COMPILER->getOutputScalar(name);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief CreateUDivRate
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::CreateUDivRate(Value * const number, const Rational divisor, const Twine & Name) {
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
 * @brief CreateCeilUDivRate
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::CreateCeilUDivRate(Value * number, const Rational divisor, const Twine & Name) {
    if (LLVM_UNLIKELY(divisor.numerator() == 1 && divisor.denominator() == 1)) {
        return number;
    }
    Constant * const n = ConstantInt::get(number->getType(), divisor.numerator());
    if (LLVM_UNLIKELY(divisor.denominator() != 1)) {
        Constant * const d = ConstantInt::get(number->getType(), divisor.denominator());
        number = CreateMul(number, d);
    }
    return CreateCeilUDiv(number, n, Name);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief CreateMulRate
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::CreateMulRate(Value * const number, const Rational factor, const Twine & Name) {
    if (LLVM_UNLIKELY(factor.numerator() == 1 && factor.denominator() == 1)) {
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
 * @brief CreateCeilUMulRate
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::CreateCeilUMulRate(Value * const number, const Rational factor, const Twine & Name) {
    if (LLVM_LIKELY(factor.denominator() == 1)) {
        return CreateMulRate(number, factor, Name);
    }
    Constant * const n = ConstantInt::get(number->getType(), factor.numerator());
    Constant * const d = ConstantInt::get(number->getType(), factor.denominator());
    return CreateCeilUDiv(CreateMul(number, n), d, Name);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief CreateURemRate
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::CreateURemRate(Value * const number, const Rational factor, const Twine & Name) {
    Constant * const n = ConstantInt::get(number->getType(), factor.numerator());
    if (LLVM_LIKELY(factor.denominator() == 1)) {
        return CreateURem(number, n, Name);
    }
    return CreateSub(number, CreateMulRate(CreateUDivRate(number, factor), factor), Name);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief CreateRoundDownRate
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::CreateRoundDownRate(Value * const number, const Rational divisor, const Twine & Name) {
    Constant * const n = ConstantInt::get(number->getType(), divisor.numerator());
    if (divisor.denominator() == 1) {
        return CBuilder::CreateRoundDown(number, n, Name);
    }
    Constant * const d = ConstantInt::get(number->getType(), divisor.denominator());
    return CreateUDiv(CBuilder::CreateRoundDown(CreateMul(number, d), n, Name), d);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief CreateRoundUpRate
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelBuilder::CreateRoundUpRate(Value * const number, const Rational divisor, const Twine & Name) {
    Constant * const n = ConstantInt::get(number->getType(), divisor.numerator());
    if (divisor.denominator() == 1) {
        return CBuilder::CreateRoundUp(number, n, Name);
    }
    Constant * const d = ConstantInt::get(number->getType(), divisor.denominator());
    return CreateUDiv(CBuilder::CreateRoundUp(CreateMul(number, d), n, Name), d);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getKernelName
 ** ------------------------------------------------------------------------------------------------------------- */
std::string KernelBuilder::getKernelName() const noexcept {
    if (LLVM_UNLIKELY(mCompiler == nullptr)) {
        return "";
    }
    return mCompiler->getName();
}

}
