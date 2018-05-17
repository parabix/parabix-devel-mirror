#include "kernel_builder.h"
#include <toolchain/toolchain.h>
#include <kernels/kernel.h>
#include <kernels/streamset.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/Module.h>

using namespace llvm;
using namespace parabix;

inline static bool is_power_2(const uint64_t n) {
    return ((n & (n - 1)) == 0) && n;
}

namespace kernel {

using Port = Kernel::Port;

Value * KernelBuilder::getScalarFieldPtr(llvm::Value * const instance, Value * const index) {
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        CreateAssert(instance, "getScalarFieldPtr: instance cannot be null!");
    }
    return CreateGEP(instance, {getInt32(0), index});
}

Value * KernelBuilder::getScalarFieldPtr(llvm::Value * const handle, const std::string & fieldName) {
    return getScalarFieldPtr(handle, getInt32(mKernel->getScalarIndex(fieldName)));
}

llvm::Value * KernelBuilder::getScalarFieldPtr(llvm::Value * const index) {
    return getScalarFieldPtr(mKernel->getInstance(), index);
}

llvm::Value *KernelBuilder:: getScalarFieldPtr(const std::string & fieldName) {
    return getScalarFieldPtr(mKernel->getInstance(), fieldName);
}

Value * KernelBuilder::getScalarField(const std::string & fieldName) {
    return CreateLoad(getScalarFieldPtr(fieldName), fieldName);
}

void KernelBuilder::setScalarField(const std::string & fieldName, Value * value) {
    CreateStore(value, getScalarFieldPtr(fieldName));
}

Value * KernelBuilder::getStreamHandle(const std::string & name) {
    Value * const ptr = getScalarField(name + BUFFER_SUFFIX);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        CreateAssert(ptr, name + " handle cannot be null!");
    }
    return ptr;
}

LoadInst * KernelBuilder::acquireLogicalSegmentNo() {
    return CreateAtomicLoadAcquire(getScalarFieldPtr(LOGICAL_SEGMENT_NO_SCALAR));
}

void KernelBuilder::releaseLogicalSegmentNo(Value * const nextSegNo) {
    CreateAtomicStoreRelease(nextSegNo, getScalarFieldPtr(LOGICAL_SEGMENT_NO_SCALAR));
}

Value * KernelBuilder::getCycleCountPtr() {
    return getScalarFieldPtr(CYCLECOUNT_SCALAR);
}

Value * KernelBuilder::getNamedItemCount(const std::string & name, const std::string & suffix) {
    const ProcessingRate & rate = mKernel->getBinding(name).getRate();
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

void KernelBuilder::setNamedItemCount(const std::string & name, const std::string & suffix, llvm::Value * const value) {
    const ProcessingRate & rate = mKernel->getBinding(name).getRate();
    const auto safetyCheck = mKernel->treatUnsafeKernelOperationsAsErrors();
    if (LLVM_UNLIKELY(rate.isDerived() && safetyCheck)) {
        report_fatal_error("Cannot set item count: " + name + " is a derived rate stream");
    }
    if (codegen::DebugOptionIsSet(codegen::TraceCounts)) {
        CallPrintIntToStderr(mKernel->getName() + ": " + name + suffix, value);
    }
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts) && safetyCheck)) {
        Value * const current = getScalarField(name + suffix);
        CreateAssert(CreateICmpUGE(value, current), name + " " + suffix + " must be monotonically non-decreasing");
    }
    setScalarField(name + suffix, value);
}


Value * KernelBuilder::getAvailableItemCount(const std::string & name) {
    const auto & inputs = mKernel->getStreamInputs();
    for (unsigned i = 0; i < inputs.size(); ++i) {
        if (inputs[i].getName() == name) {
            return mKernel->getAvailableItemCount(i);
        }
    }
    return nullptr;
}

Value * KernelBuilder::getTerminationSignal() {
    return CreateICmpNE(getScalarField(TERMINATION_SIGNAL), getSize(0));
}

void KernelBuilder::setTerminationSignal(llvm::Value * const value) {
    assert (value->getType() == getInt1Ty());
    if (codegen::DebugOptionIsSet(codegen::TraceCounts)) {
        CallPrintIntToStderr(mKernel->getName() + ": setTerminationSignal", value);
    }
    setScalarField(TERMINATION_SIGNAL, CreateZExt(value, getSizeTy()));
}

Value * KernelBuilder::getLinearlyAccessibleItems(const std::string & name, Value * fromPosition, Value * avail, bool reverse) {
    const StreamSetBuffer * const buf = mKernel->getInputStreamSetBuffer(name);
    return buf->getLinearlyAccessibleItems(this, getStreamHandle(name), fromPosition, avail, reverse);
}

Value * KernelBuilder::getLinearlyWritableItems(const std::string & name, Value * fromPosition, bool reverse) {
    const StreamSetBuffer * const buf = mKernel->getOutputStreamSetBuffer(name);
    return buf->getLinearlyWritableItems(this, getStreamHandle(name), fromPosition, getConsumedItemCount(name), reverse);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief CreatePrepareOverflow
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::CreatePrepareOverflow(const std::string & name) {
    const StreamSetBuffer * const buf = mKernel->getOutputStreamSetBuffer(name);
    assert (buf->supportsCopyBack());
    Constant * const overflowSize = ConstantExpr::getSizeOf(buf->getType());
    Value * const handle = getStreamHandle(name);
    // TODO: handle non constant stream set counts
    assert (isa<Constant>(buf->getStreamSetCount(this, handle)));
    Value * const base = buf->getBaseAddress(this, handle);
    Value * const overflow = buf->getOverflowAddress(this, handle);
    const auto blockSize = getBitBlockWidth() / 8;
    CreateMemZero(overflow, overflowSize, blockSize);
    CreateMemZero(base, overflowSize, blockSize);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getItemWidth
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned LLVM_READNONE getItemWidth(const Type * ty ) {
    if (LLVM_LIKELY(isa<ArrayType>(ty))) {
        ty = ty->getArrayElementType();
    }
    return cast<IntegerType>(ty->getVectorElementType())->getBitWidth();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief CreateNonLinearCopyFromOverflow
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::CreateNonLinearCopyFromOverflow(const Binding & output, llvm::Value * const itemsToCopy, Value * overflowOffset) {

    Value * const handle = getStreamHandle(output.getName());
    Type * const bitBlockPtrTy = getBitBlockType()->getPointerTo();
    const StreamSetBuffer * const buf = mKernel->getOutputStreamSetBuffer(output.getName());
    assert (buf->supportsCopyBack());
    Value * const target = CreatePointerCast(buf->getBaseAddress(this, handle), bitBlockPtrTy);
    Value * const source = CreatePointerCast(buf->getOverflowAddress(this, handle), bitBlockPtrTy);
    const auto blockSize = getBitBlockWidth() / 8;
    Constant * const BLOCK_WIDTH = getSize(getBitBlockWidth());
    Constant * const ITEM_WIDTH = getSize(getItemWidth(buf->getBaseType()));
    Value * const streamCount = buf->getStreamSetCount(this, handle);

    // If we have a computed overflow position, the base and overflow regions were not speculatively zeroed out prior
    // to the kernel writing over them. To handle them, we compute a mask of valid items and exclude any bit not in
    // them before OR-ing together the streams.
    if (overflowOffset) {

        overflowOffset = CreateMul(overflowOffset, ITEM_WIDTH);
        Value * targetMask = bitblock_mask_from(CreateURem(overflowOffset, BLOCK_WIDTH));
        Value * sourceMask = CreateNot(targetMask);
        Value * const overflowBlockCount = CreateUDiv(overflowOffset, BLOCK_WIDTH);
        Value * const blockOffset = CreateMul(overflowBlockCount, streamCount);
        Value * const fullCopyLength = CreateMul(blockOffset, getSize(blockSize));
        CreateMemCpy(target, source, fullCopyLength, blockSize);

        BasicBlock * const partialCopyEntry = GetInsertBlock();
        BasicBlock * const partialCopyLoop = CreateBasicBlock();
        BasicBlock * const partialCopyExit = CreateBasicBlock();

        Value * const partialBlockCount = CreateAdd(blockOffset, streamCount);
        CreateBr(partialCopyLoop);

        SetInsertPoint(partialCopyLoop);
        PHINode * const blockIndex = CreatePHI(getSizeTy(), 2);
        blockIndex->addIncoming(blockOffset, partialCopyEntry);
        Value * const sourcePtr = CreateGEP(source, blockIndex);
        Value * sourceValue = CreateBlockAlignedLoad(sourcePtr);
        sourceValue = CreateAnd(sourceValue, sourceMask);
        Value * const targetPtr = CreateGEP(target, blockIndex);
        Value * targetValue = CreateBlockAlignedLoad(targetPtr);
        targetValue = CreateAnd(targetValue, targetMask);
        targetValue = CreateOr(targetValue, sourceValue);
        CreateBlockAlignedStore(targetValue, targetPtr);
        Value * const nextBlockIndex = CreateAdd(blockIndex, getSize(1));
        blockIndex->addIncoming(nextBlockIndex, partialCopyLoop);
        CreateCondBr(CreateICmpNE(nextBlockIndex, partialBlockCount), partialCopyLoop, partialCopyExit);

        SetInsertPoint(partialCopyExit);

    } else {

        BasicBlock * const mergeCopyEntry = GetInsertBlock();
        BasicBlock * const mergeCopyLoop = CreateBasicBlock();
        BasicBlock * const mergeCopyExit = CreateBasicBlock();

        Value * blocksToCopy = CreateCeilUDiv(itemsToCopy, BLOCK_WIDTH);
        blocksToCopy = CreateMul(blocksToCopy, ITEM_WIDTH);
        blocksToCopy = CreateMul(blocksToCopy, streamCount);

        CreateBr(mergeCopyLoop);

        SetInsertPoint(mergeCopyLoop);
        PHINode * const blockIndex = CreatePHI(getSizeTy(), 2);
        blockIndex->addIncoming(getSize(0), mergeCopyEntry);
        Value * const sourcePtr = CreateGEP(source, blockIndex);
        Value * const sourceValue = CreateBlockAlignedLoad(sourcePtr);
        Value * const targetPtr = CreateGEP(target, blockIndex);
        Value * targetValue = CreateBlockAlignedLoad(targetPtr);
        targetValue = CreateOr(targetValue, sourceValue);
        CreateBlockAlignedStore(targetValue, targetPtr);
        Value * const nextBlockIndex = CreateAdd(blockIndex, getSize(1));
        blockIndex->addIncoming(nextBlockIndex, mergeCopyLoop);
        CreateCondBr(CreateICmpNE(nextBlockIndex, blocksToCopy), mergeCopyLoop, mergeCopyExit);

        SetInsertPoint(mergeCopyExit);
    }



}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief CreateCopyFromOverflow
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::CreateCopyFromOverflow(const Binding & output, llvm::Value * const itemsToCopy) {

    Value * const handle = getStreamHandle(output.getName());
    Type * const bitBlockPtrTy = getBitBlockType()->getPointerTo();
    const StreamSetBuffer * const buf = mKernel->getOutputStreamSetBuffer(output.getName());
    assert (buf->supportsCopyBack());
    Value * const target = CreatePointerCast(buf->getBaseAddress(this, handle), bitBlockPtrTy);
    Value * const source = CreatePointerCast(buf->getOverflowAddress(this, handle), bitBlockPtrTy);
    Constant * const BLOCK_WIDTH = getSize(getBitBlockWidth());
    Constant * const ITEM_WIDTH = getSize(getItemWidth(buf->getBaseType()));
    Value * const streamCount = buf->getStreamSetCount(this, handle);

    BasicBlock * const mergeCopyEntry = GetInsertBlock();
    BasicBlock * const mergeCopyLoop = CreateBasicBlock();
    BasicBlock * const mergeCopyExit = CreateBasicBlock();

    Value * blocksToCopy = CreateCeilUDiv(itemsToCopy, BLOCK_WIDTH);
    blocksToCopy = CreateMul(blocksToCopy, ITEM_WIDTH);
    blocksToCopy = CreateMul(blocksToCopy, streamCount);

    CreateCondBr(CreateICmpEQ(blocksToCopy, getSize(0)), mergeCopyExit, mergeCopyLoop);

    SetInsertPoint(mergeCopyLoop);
    PHINode * const blockIndex = CreatePHI(getSizeTy(), 2);
    blockIndex->addIncoming(getSize(0), mergeCopyEntry);
    Value * const sourcePtr = CreateGEP(source, blockIndex);
    Value * const sourceValue = CreateBlockAlignedLoad(sourcePtr);
    Value * const targetPtr = CreateGEP(target, blockIndex);
    CreateBlockAlignedStore(sourceValue, targetPtr);
    Value * const nextBlockIndex = CreateAdd(blockIndex, getSize(1));
    blockIndex->addIncoming(nextBlockIndex, mergeCopyLoop);
    CreateCondBr(CreateICmpNE(nextBlockIndex, blocksToCopy), mergeCopyLoop, mergeCopyExit);

    SetInsertPoint(mergeCopyExit);
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief CreateCopyToOverflow
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::CreateCopyToOverflow(const std::string & name) {
    const StreamSetBuffer * const buf = mKernel->getOutputStreamSetBuffer(name);
    assert (buf->supportsCopyBack());
    Value * const handle = getStreamHandle(name);
    // TODO: handle non constant stream set counts
    assert (isa<Constant>(buf->getStreamSetCount(this, handle)));
    Value * const target = buf->getBaseAddress(this, handle);
    Value * const source = buf->getOverflowAddress(this, handle);
    Constant * const overflowSize = ConstantExpr::getSizeOf(buf->getType());
    CreateMemCpy(target, source, overflowSize, getBitBlockWidth() / 8);
}

Value * KernelBuilder::getConsumerLock(const std::string & name) {
    return getScalarField(name + CONSUMER_SUFFIX);
}

void KernelBuilder::setConsumerLock(const std::string & name, Value * const value) {
    setScalarField(name + CONSUMER_SUFFIX, value);
}

Value * KernelBuilder::getInputStreamBlockPtr(const std::string & name, Value * const streamIndex, Value * const blockOffset) {
    const StreamSetBuffer * const buf = mKernel->getInputStreamSetBuffer(name);
    Value * blockIndex = CreateLShr(getProcessedItemCount(name), std::log2(getBitBlockWidth()));
    if (blockOffset) {
        assert (blockOffset->getType() == blockIndex->getType());
        blockIndex = CreateAdd(blockIndex, blockOffset);
    }
    return buf->getStreamBlockPtr(this, getStreamHandle(name), streamIndex, blockIndex, true);
}

Value * KernelBuilder::getInputStreamPackPtr(const std::string & name, Value * const streamIndex, Value * const packIndex, Value * const blockOffset) {
    const StreamSetBuffer * const buf = mKernel->getInputStreamSetBuffer(name);
    Value * blockIndex = CreateLShr(getProcessedItemCount(name), std::log2(getBitBlockWidth()));
    if (blockOffset) {
        assert (blockOffset->getType() == blockIndex->getType());
        blockIndex = CreateAdd(blockIndex, blockOffset);
    }
    return buf->getStreamPackPtr(this, getStreamHandle(name), streamIndex, blockIndex, packIndex, true);
}

Value * KernelBuilder::loadInputStreamBlock(const std::string & name, Value * const streamIndex, Value * const blockOffset) {
    return CreateBlockAlignedLoad(getInputStreamBlockPtr(name, streamIndex, blockOffset));
}

Value * KernelBuilder::loadInputStreamPack(const std::string & name, Value * const streamIndex, Value * const packIndex, Value * const blockOffset) {
    return CreateBlockAlignedLoad(getInputStreamPackPtr(name, streamIndex, packIndex, blockOffset));
}

Value * KernelBuilder::getInputStreamSetCount(const std::string & name) {
    const StreamSetBuffer * const buf = mKernel->getInputStreamSetBuffer(name);
    return buf->getStreamSetCount(this, getStreamHandle(name));
}

Value * KernelBuilder::getOutputStreamBlockPtr(const std::string & name, Value * streamIndex, Value * const blockOffset) {
    const StreamSetBuffer * const buf = mKernel->getOutputStreamSetBuffer(name);
    Value * blockIndex = CreateLShr(getProducedItemCount(name), std::log2(getBitBlockWidth()));
    if (blockOffset) {
        assert (blockOffset->getType() == blockIndex->getType());
        blockIndex = CreateAdd(blockIndex, blockOffset);
    }
    return buf->getStreamBlockPtr(this, getStreamHandle(name), streamIndex, blockIndex, false);
}

Value * KernelBuilder::getOutputStreamPackPtr(const std::string & name, Value * streamIndex, Value * packIndex, llvm::Value * blockOffset) {
    const StreamSetBuffer * const buf = mKernel->getOutputStreamSetBuffer(name);
    Value * blockIndex = CreateLShr(getProducedItemCount(name), std::log2(getBitBlockWidth()));
    if (blockOffset) {
        assert (blockOffset->getType() == blockIndex->getType());
        blockIndex = CreateAdd(blockIndex, blockOffset);
    }
    return buf->getStreamPackPtr(this, getStreamHandle(name), streamIndex, blockIndex, packIndex, false);
}


StoreInst * KernelBuilder::storeOutputStreamBlock(const std::string & name, Value * streamIndex, llvm::Value * blockOffset, Value * toStore) {
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

StoreInst * KernelBuilder::storeOutputStreamPack(const std::string & name, Value * streamIndex, Value * packIndex, llvm::Value * blockOffset, Value * toStore) {
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
    return buf->getStreamSetCount(this, getStreamHandle(name));
}

Value * KernelBuilder::getRawInputPointer(const std::string & name, Value * absolutePosition) {
    const StreamSetBuffer * const buf = mKernel->getInputStreamSetBuffer(name);
    return buf->getRawItemPointer(this, getStreamHandle(name), absolutePosition);
}

Value * KernelBuilder::getRawOutputPointer(const std::string & name, Value * absolutePosition) {
    const StreamSetBuffer * const buf = mKernel->getOutputStreamSetBuffer(name);
    return buf->getRawItemPointer(this, getStreamHandle(name), absolutePosition);
}

Value * KernelBuilder::getBaseAddress(const std::string & name) {
    return mKernel->getAnyStreamSetBuffer(name)->getBaseAddress(this, getStreamHandle(name));
}

void KernelBuilder::setBaseAddress(const std::string & name, Value * const addr) {
    return mKernel->getAnyStreamSetBuffer(name)->setBaseAddress(this, getStreamHandle(name), addr);
}

Value * KernelBuilder::getCapacity(const std::string & name) {
    return mKernel->getAnyStreamSetBuffer(name)->getCapacity(this, getStreamHandle(name));
}

void KernelBuilder::setCapacity(const std::string & name, Value * c) {
    mKernel->getAnyStreamSetBuffer(name)->setCapacity(this, getStreamHandle(name), c);
}
    
CallInst * KernelBuilder::createDoSegmentCall(const std::vector<Value *> & args) {
    return mKernel->makeDoSegmentCall(*this, args);
}

Value * KernelBuilder::getAccumulator(const std::string & accumName) {
    auto results = mKernel->mOutputScalarResult;
    if (LLVM_UNLIKELY(results == nullptr)) {
        report_fatal_error("Cannot get accumulator " + accumName + " until " + mKernel->getName() + " has terminated.");
    }
    const auto & outputs = mKernel->getScalarOutputs();
    const auto n = outputs.size();
    if (LLVM_UNLIKELY(n == 0)) {
        report_fatal_error(mKernel->getName() + " has no output scalars.");
    } else {
        for (unsigned i = 0; i < n; ++i) {
            const Binding & b = outputs[i];
            if (b.getName() == accumName) {
                if (n == 1) {
                    return results;
                } else {
                    return CreateExtractValue(results, {i});
                }
            }
        }
        report_fatal_error(mKernel->getName() + " has no output scalar named " + accumName);
    }
}

BasicBlock * KernelBuilder::CreateConsumerWait() {
    const auto consumers = mKernel->getStreamOutputs();
    BasicBlock * const entry = GetInsertBlock();
    if (consumers.empty()) {
        return entry;
    } else {
        Function * const parent = entry->getParent();
        IntegerType * const sizeTy = getSizeTy();
        ConstantInt * const zero = getInt32(0);
        ConstantInt * const one = getInt32(1);
        ConstantInt * const size0 = getSize(0);

        Value * const segNo = acquireLogicalSegmentNo();
        const auto n = consumers.size();
        BasicBlock * load[n + 1];
        BasicBlock * wait[n];
        for (unsigned i = 0; i < n; ++i) {
            load[i] = BasicBlock::Create(getContext(), consumers[i].getName() + "Load", parent);
            wait[i] = BasicBlock::Create(getContext(), consumers[i].getName() + "Wait", parent);
        }
        load[n] = BasicBlock::Create(getContext(), "Resume", parent);
        CreateBr(load[0]);
        for (unsigned i = 0; i < n; ++i) {

            SetInsertPoint(load[i]);
            Value * const outputConsumers = getConsumerLock(consumers[i].getName());

            Value * const consumerCount = CreateLoad(CreateGEP(outputConsumers, {zero, zero}));
            Value * const consumerPtr = CreateLoad(CreateGEP(outputConsumers, {zero, one}));
            Value * const noConsumers = CreateICmpEQ(consumerCount, size0);
            CreateUnlikelyCondBr(noConsumers, load[i + 1], wait[i]);

            SetInsertPoint(wait[i]);
            PHINode * const consumerPhi = CreatePHI(sizeTy, 2);
            consumerPhi->addIncoming(size0, load[i]);

            Value * const conSegPtr = CreateLoad(CreateGEP(consumerPtr, consumerPhi));
            Value * const processedSegmentCount = CreateAtomicLoadAcquire(conSegPtr);
            Value * const ready = CreateICmpEQ(segNo, processedSegmentCount);
            assert (ready->getType() == getInt1Ty());
            Value * const nextConsumerIdx = CreateAdd(consumerPhi, CreateZExt(ready, sizeTy));
            consumerPhi->addIncoming(nextConsumerIdx, wait[i]);
            Value * const next = CreateICmpEQ(nextConsumerIdx, consumerCount);
            CreateCondBr(next, load[i + 1], wait[i]);
        }

        BasicBlock * const exit = load[n];
        SetInsertPoint(exit);
        return exit;
    }
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

}
