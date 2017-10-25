#include "kernel_builder.h"
#include <toolchain/toolchain.h>
#include <kernels/kernel.h>
#include <kernels/streamset.h>
#include <llvm/Support/raw_ostream.h>

using namespace llvm;
using namespace parabix;

using Value = Value;

namespace kernel {

using Port = Kernel::Port;

Value * KernelBuilder::getScalarFieldPtr(llvm::Value * instance, Value * const index) {
    assert (instance);
    CreateAssert(instance, "getScalarFieldPtr: instance cannot be null!");
    return CreateGEP(instance, {getInt32(0), index});
}

Value * KernelBuilder::getScalarFieldPtr(llvm::Value * instance, const std::string & fieldName) {
    return getScalarFieldPtr(instance, getInt32(mKernel->getScalarIndex(fieldName)));
}

llvm::Value * KernelBuilder::getScalarFieldPtr(llvm::Value * index) {
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
    Value * const ptr = getScalarField(name + Kernel::BUFFER_PTR_SUFFIX);
    CreateAssert(ptr, name + " cannot be null!");
    return ptr;
}

LoadInst * KernelBuilder::acquireLogicalSegmentNo() {
    return CreateAtomicLoadAcquire(getScalarFieldPtr(Kernel::LOGICAL_SEGMENT_NO_SCALAR));
}

void KernelBuilder::releaseLogicalSegmentNo(Value * nextSegNo) {
    CreateAtomicStoreRelease(nextSegNo, getScalarFieldPtr(Kernel::LOGICAL_SEGMENT_NO_SCALAR));
}

Value * KernelBuilder::getCycleCountPtr() {
    return getScalarFieldPtr(Kernel::CYCLECOUNT_SCALAR);
}

inline const Binding & getBinding(const Kernel * k, const std::string & name) {
    Port port; unsigned index;
    std::tie(port, index) = k->getStreamPort(name);
    if (port == Port::Input) {
        return k->getStreamInput(index);
    } else {
        return k->getStreamOutput(index);
    }
}

Value * KernelBuilder::getInternalItemCount(const std::string & name, const std::string & suffix) {
    const ProcessingRate & rate = getBinding(mKernel, name).getRate();
    Value * itemCount = nullptr;
    if (rate.isExactlyRelative()) {
        Port port; unsigned index;
        std::tie(port, index) = mKernel->getStreamPort(rate.getReference());
        if (port == Port::Input) {
            itemCount = getProcessedItemCount(rate.getReference());
        } else {
            itemCount = getProducedItemCount(rate.getReference());
        }
        if (rate.getNumerator() != 1) {
            itemCount = CreateMul(itemCount, ConstantInt::get(itemCount->getType(), rate.getNumerator()));
        }
        if (rate.getDenominator() != 1) {
            itemCount = CreateExactUDiv(itemCount, ConstantInt::get(itemCount->getType(), rate.getDenominator()));
        }
    } else {
        itemCount = getScalarField(name + suffix);
    }
    return itemCount;
}

void KernelBuilder::setInternalItemCount(const std::string & name, const std::string & suffix, llvm::Value * const value) {
    const ProcessingRate & rate = getBinding(mKernel, name).getRate();
    if (LLVM_UNLIKELY(rate.isDerived())) {
        report_fatal_error("Cannot set item count: " + name + " is a Derived rate");
    }
    if (codegen::DebugOptionIsSet(codegen::TraceCounts)) {
        CallPrintIntToStderr(mKernel->getName() + ": " + name + suffix, value);
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
    if (mKernel->hasNoTerminateAttribute()) {
        return getFalse();
    }
    return getScalarField(Kernel::TERMINATION_SIGNAL);
}

void KernelBuilder::setTerminationSignal(llvm::Value * const value) {
    assert (!mKernel->hasNoTerminateAttribute());
    assert (value->getType() == getInt1Ty());
    if (codegen::DebugOptionIsSet(codegen::TraceCounts)) {
        CallPrintIntToStderr(mKernel->getName() + ": setTerminationSignal", value);
    }
    setScalarField(Kernel::TERMINATION_SIGNAL, value);
}

Value * KernelBuilder::getLinearlyAccessibleItems(const std::string & name, Value * fromPosition, Value * avail, bool reverse) {
    const StreamSetBuffer * const buf = mKernel->getInputStreamSetBuffer(name);
    return buf->getLinearlyAccessibleItems(this, getStreamHandle(name), fromPosition, avail, reverse);
}

Value * KernelBuilder::getLinearlyWritableItems(const std::string & name, Value * fromPosition, bool reverse) {
    const StreamSetBuffer * const buf = mKernel->getOutputStreamSetBuffer(name);
    return buf->getLinearlyWritableItems(this, getStreamHandle(name), fromPosition, reverse);
}

Value * KernelBuilder::copy(const std::string & name, Value * target, Value * source, Value * itemsToCopy, const unsigned alignment) {
    const StreamSetBuffer * const buf = mKernel->getAnyStreamSetBuffer(name);
    return buf->copy(this, getStreamHandle(name), target, source, itemsToCopy, alignment);
}

void KernelBuilder::CreateCopyBack(const std::string & name, llvm::Value * from, llvm::Value * to) {
    const StreamSetBuffer * const buf = mKernel->getAnyStreamSetBuffer(name);
    return buf->genCopyBackLogic(this, getStreamHandle(name), from, to, name);
}

Value * KernelBuilder::getConsumerLock(const std::string & name) {
    return getScalarField(name + Kernel::CONSUMER_SUFFIX);
}

void KernelBuilder::setConsumerLock(const std::string & name, Value * value) {
    setScalarField(name + Kernel::CONSUMER_SUFFIX, value);
}

inline Value * KernelBuilder::computeBlockIndex(Value * itemCount) {
    const auto divisor = getBitBlockWidth();
    if (LLVM_LIKELY((divisor & (divisor - 1)) == 0)) {
        return CreateLShr(itemCount, std::log2(divisor));
    } else {
        return CreateUDiv(itemCount, getSize(divisor));
    }
}

Value * KernelBuilder::getInputStreamPtr(const std::string & name, Value * const blockIndex) {
//    Value * const blockIndex = computeBlockIndex(getProcessedItemCount(name));
    const StreamSetBuffer * const buf = mKernel->getInputStreamSetBuffer(name);
    return buf->getBlockAddress(this, getStreamHandle(name), blockIndex);
}

Value * KernelBuilder::getInputStreamBlockPtr(const std::string & name, Value * streamIndex) {
    const Kernel::StreamPort p = mKernel->getStreamPort(name);
    if (LLVM_UNLIKELY(p.first == Port::Output)) {
        report_fatal_error(name + " is not an input stream set");
    }
    Value * const addr = mKernel->getStreamSetInputBufferPtr(p.second);
    const StreamSetBuffer * const buf = mKernel->getInputStreamSetBuffer(name);
    return buf->getStreamBlockPtr(this, getStreamHandle(name), addr, streamIndex, true);
}

Value * KernelBuilder::loadInputStreamBlock(const std::string & name, Value * streamIndex) {
    return CreateBlockAlignedLoad(getInputStreamBlockPtr(name, streamIndex));
}

Value * KernelBuilder::getInputStreamPackPtr(const std::string & name, Value * streamIndex, Value * packIndex) {
    const Kernel::StreamPort p = mKernel->getStreamPort(name);
    if (LLVM_UNLIKELY(p.first == Port::Output)) {
        report_fatal_error(name + " is not an input stream set");
    }
    Value * const addr = mKernel->getStreamSetInputBufferPtr(p.second);
    const StreamSetBuffer * const buf = mKernel->getInputStreamSetBuffer(name);
    return buf->getStreamPackPtr(this, getStreamHandle(name), addr, streamIndex, packIndex, true);
}

Value * KernelBuilder::loadInputStreamPack(const std::string & name, Value * streamIndex, Value * packIndex) {
    return CreateBlockAlignedLoad(getInputStreamPackPtr(name, streamIndex, packIndex));
}

Value * KernelBuilder::getInputStreamSetCount(const std::string & name) {
    const StreamSetBuffer * const buf = mKernel->getInputStreamSetBuffer(name);
    return buf->getStreamSetCount(this, getStreamHandle(name));
}

Value * KernelBuilder::getAdjustedInputStreamBlockPtr(Value * blockAdjustment, const std::string & name, Value * streamIndex) {
    const Kernel::StreamPort p = mKernel->getStreamPort(name);
    if (LLVM_UNLIKELY(p.first == Port::Output)) {
        report_fatal_error(name + " is not an input stream set");
    }
    Value * const addr = mKernel->getStreamSetInputBufferPtr(p.second);
    const StreamSetBuffer * const buf = mKernel->getInputStreamSetBuffer(name);
    return buf->getStreamBlockPtr(this, getStreamHandle(name), CreateGEP(addr, blockAdjustment), streamIndex, true);
}

Value * KernelBuilder::getOutputStreamPtr(const std::string & name, Value * const blockIndex) {
//    Value * const blockIndex = computeBlockIndex(getProducedItemCount(name));
    const StreamSetBuffer * const buf = mKernel->getOutputStreamSetBuffer(name);
    return buf->getBlockAddress(this, getStreamHandle(name), blockIndex);
}

Value * KernelBuilder::getOutputStreamBlockPtr(const std::string & name, Value * streamIndex) {
    const Kernel::StreamPort p = mKernel->getStreamPort(name);
    if (LLVM_UNLIKELY(p.first == Port::Input)) {
        report_fatal_error(name + " is not an output stream set");
    }
    Value * addr = mKernel->getStreamSetOutputBufferPtr(p.second);
    const StreamSetBuffer * const buf = mKernel->getOutputStreamSetBuffer(name);
    return buf->getStreamBlockPtr(this, getStreamHandle(name), addr, streamIndex, true);
}

StoreInst * KernelBuilder::storeOutputStreamBlock(const std::string & name, Value * streamIndex, Value * toStore) {
    return CreateBlockAlignedStore(toStore, getOutputStreamBlockPtr(name, streamIndex));
}

Value * KernelBuilder::getOutputStreamPackPtr(const std::string & name, Value * streamIndex, Value * packIndex) {
    const Kernel::StreamPort p = mKernel->getStreamPort(name);
    if (LLVM_UNLIKELY(p.first == Port::Input)) {
        report_fatal_error(name + " is not an output stream set");
    }
    Value * addr = mKernel->getStreamSetOutputBufferPtr(p.second);
    const StreamSetBuffer * const buf = mKernel->getOutputStreamSetBuffer(name);
    return buf->getStreamPackPtr(this, getStreamHandle(name), addr, streamIndex, packIndex, false);
}

StoreInst * KernelBuilder::storeOutputStreamPack(const std::string & name, Value * streamIndex, Value * packIndex, Value * toStore) {
    return CreateBlockAlignedStore(toStore, getOutputStreamPackPtr(name, streamIndex, packIndex));
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

Value * KernelBuilder::getBufferedSize(const std::string & name) {
    return mKernel->getAnyStreamSetBuffer(name)->getBufferedSize(this, getStreamHandle(name));
}

void KernelBuilder::setBufferedSize(const std::string & name, Value * size) {
    mKernel->getAnyStreamSetBuffer(name)->setBufferedSize(this, getStreamHandle(name), size);
}


Value * KernelBuilder::getCapacity(const std::string & name) {
    return mKernel->getAnyStreamSetBuffer(name)->getCapacity(this, getStreamHandle(name));
}

void KernelBuilder::setCapacity(const std::string & name, Value * c) {
    mKernel->getAnyStreamSetBuffer(name)->setCapacity(this, getStreamHandle(name), c);
}

    
CallInst * KernelBuilder::createDoSegmentCall(const std::vector<Value *> & args) {
//    Function * const doSegment = mKernel->getDoSegmentFunction(getModule());
//    assert (doSegment->getArgumentList().size() == args.size());
//    return CreateCall(doSegment, args);
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

}
