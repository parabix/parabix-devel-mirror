/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "kernel.h"
#include <toolchain/toolchain.h>
#include <kernels/streamset.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/MDBuilder.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Bitcode/ReaderWriter.h>
#include <llvm/Transforms/Utils/Local.h>
#include <kernels/streamset.h>
#include <sstream>
#include <kernels/kernel_builder.h>

using namespace llvm;
using namespace parabix;

namespace kernel {

const std::string Kernel::DO_BLOCK_SUFFIX = "_DoBlock";
const std::string Kernel::FINAL_BLOCK_SUFFIX = "_FinalBlock";
const std::string Kernel::MULTI_BLOCK_SUFFIX = "_MultiBlock";
const std::string Kernel::LOGICAL_SEGMENT_NO_SCALAR = "logicalSegNo";
const std::string Kernel::PROCESSED_ITEM_COUNT_SUFFIX = "_processedItemCount";
const std::string Kernel::CONSUMED_ITEM_COUNT_SUFFIX = "_consumedItemCount";
const std::string Kernel::PRODUCED_ITEM_COUNT_SUFFIX = "_producedItemCount";
const std::string Kernel::TERMINATION_SIGNAL = "terminationSignal";
const std::string Kernel::BUFFER_PTR_SUFFIX = "_bufferPtr";
const std::string Kernel::CONSUMER_SUFFIX = "_consumerLocks";

unsigned Kernel::addScalar(Type * const type, const std::string & name) {
    if (LLVM_UNLIKELY(mKernelStateType != nullptr)) {
        report_fatal_error("Cannot add field " + name + " to " + getName() + " after kernel state finalized");
    }
    if (LLVM_UNLIKELY(mKernelMap.count(name))) {
        report_fatal_error(getName() + " already contains scalar field " + name);
    }
    const auto index = mKernelFields.size();
    mKernelMap.emplace(name, index);
    mKernelFields.push_back(type);
    return index;
}

unsigned Kernel::addUnnamedScalar(Type * const type) {
    if (LLVM_UNLIKELY(mKernelStateType != nullptr)) {
        report_fatal_error("Cannot add unnamed field  to " + getName() + " after kernel state finalized");
    }
    const auto index = mKernelFields.size();
    mKernelFields.push_back(type);
    return index;
}

// Get the value of a scalar field for the current instance.
llvm::Value * Kernel::getScalarFieldPtr(llvm::Value * index) const {
    return iBuilder->CreateGEP(getInstance(), {iBuilder->getInt32(0), index});
}

llvm::Value * Kernel::getScalarFieldPtr(const std::string & fieldName) const {
    return getScalarFieldPtr(iBuilder->getInt32(getScalarIndex(fieldName)));
}

llvm::Value * Kernel::getScalarField(const std::string & fieldName) const {
    return iBuilder->CreateLoad(getScalarFieldPtr(fieldName), fieldName);
}

// Set the value of a scalar field for the current instance.
void Kernel::setScalarField(const std::string & fieldName, llvm::Value * value) const {
    iBuilder->CreateStore(value, getScalarFieldPtr(fieldName));
}

void Kernel::prepareStreamSetNameMap() {
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        mStreamMap.emplace(mStreamSetInputs[i].name, std::make_pair(Port::Input, i));
    }
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        mStreamMap.emplace(mStreamSetOutputs[i].name, std::make_pair(Port::Output, i));
    }
}
    
void Kernel::prepareKernel() {
    assert ("KernelBuilder does not have a valid IDISA Builder" && iBuilder);
    if (LLVM_UNLIKELY(mKernelStateType != nullptr)) {
        report_fatal_error("Cannot prepare kernel after kernel state finalized");
    }
    if (mStreamSetInputs.size() != mStreamSetInputBuffers.size()) {
        std::string tmp;
        raw_string_ostream out(tmp);
        out << "kernel contains " << mStreamSetInputBuffers.size() << " input buffers for "
            << mStreamSetInputs.size() << " input stream sets.";
        report_fatal_error(out.str());
    }
    if (mStreamSetOutputs.size() != mStreamSetOutputBuffers.size()) {
        std::string tmp;
        raw_string_ostream out(tmp);
        out << "kernel contains " << mStreamSetOutputBuffers.size() << " output buffers for "
            << mStreamSetOutputs.size() << " output stream sets.";
        report_fatal_error(out.str());
    }
    const auto blockSize = iBuilder->getBitBlockWidth();
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        if ((mStreamSetInputBuffers[i]->getBufferBlocks() > 0) && (mStreamSetInputBuffers[i]->getBufferBlocks() < codegen::SegmentSize + (blockSize + mLookAheadPositions - 1)/blockSize)) {
            report_fatal_error("Kernel preparation: Buffer size too small " + mStreamSetInputs[i].name);
        }
        mScalarInputs.emplace_back(mStreamSetInputBuffers[i]->getPointerType(), mStreamSetInputs[i].name + BUFFER_PTR_SUFFIX);
        if ((i == 0) || !mStreamSetInputs[i].rate.isExact()) {
            addScalar(iBuilder->getSizeTy(), mStreamSetInputs[i].name + PROCESSED_ITEM_COUNT_SUFFIX);
        }        
    }

    IntegerType * const sizeTy = iBuilder->getSizeTy();
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        mScalarInputs.emplace_back(mStreamSetOutputBuffers[i]->getPointerType(), mStreamSetOutputs[i].name + BUFFER_PTR_SUFFIX);
        if ((mStreamSetInputs.empty() && (i == 0)) || !mStreamSetOutputs[i].rate.isExact()) {
            addScalar(sizeTy, mStreamSetOutputs[i].name + PRODUCED_ITEM_COUNT_SUFFIX);
        }
    }
    for (const auto binding : mScalarInputs) {
        addScalar(binding.type, binding.name);
    }
    for (const auto binding : mScalarOutputs) {
        addScalar(binding.type, binding.name);
    }
    if (mStreamMap.empty()) {
        prepareStreamSetNameMap();
    }
    for (auto binding : mInternalScalars) {
        addScalar(binding.type, binding.name);
    }

    Type * const consumerSetTy = StructType::get(sizeTy, sizeTy->getPointerTo()->getPointerTo(), nullptr)->getPointerTo();
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        addScalar(consumerSetTy, mStreamSetOutputs[i].name + CONSUMER_SUFFIX);
    }

    addScalar(sizeTy, LOGICAL_SEGMENT_NO_SCALAR);
    addScalar(iBuilder->getInt1Ty(), TERMINATION_SIGNAL);

    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        addScalar(sizeTy, mStreamSetOutputs[i].name + CONSUMED_ITEM_COUNT_SUFFIX);
    }

    mKernelStateType = StructType::create(iBuilder->getContext(), mKernelFields, getName());
}

void Kernel::createKernelStub(const StreamSetBuffers & inputs, const StreamSetBuffers & outputs) {
    assert ("KernelBuilder does not have a valid IDISA Builder" && iBuilder);
    assert ("IDISA Builder does not have a valid Module" && iBuilder->getModule());
    std::stringstream cacheName;   
    cacheName << getName() << '_' << iBuilder->getBuilderUniqueName();
    for (const StreamSetBuffer * b: inputs) {
        cacheName <<  ':' <<  b->getUniqueID();
    }
    for (const StreamSetBuffer * b: outputs) {
        cacheName <<  ':' <<  b->getUniqueID();
    }
    Module * const kernelModule = new Module(cacheName.str(), iBuilder->getContext());
    kernelModule->setTargetTriple(iBuilder->getModule()->getTargetTriple());
    createKernelStub(inputs, outputs, kernelModule);
}

void Kernel::createKernelStub(const StreamSetBuffers & inputs, const StreamSetBuffers & outputs, Module * const kernelModule) {
    assert (mModule == nullptr);
    assert ("KernelBuilder does not have a valid IDISA Builder" && iBuilder);
    assert (mStreamSetInputBuffers.empty());
    assert (mStreamSetOutputBuffers.empty());

    if (LLVM_UNLIKELY(mStreamSetInputs.size() != inputs.size())) {
        report_fatal_error(getName() + ": expected " + std::to_string(mStreamSetInputs.size()) +
                           " input stream sets but was given "
                           + std::to_string(inputs.size()));
    }

    for (unsigned i = 0; i < inputs.size(); ++i) {
        StreamSetBuffer * const buf = inputs[i];
        if (LLVM_UNLIKELY(buf == nullptr)) {
            report_fatal_error(getName() + ": input stream set " + std::to_string(i)
                               + " cannot be null");
        }
        buf->addConsumer(this);
    }

    if (LLVM_UNLIKELY(mStreamSetOutputs.size() != outputs.size())) {
        report_fatal_error(getName() + ": expected " + std::to_string(mStreamSetOutputs.size())
                           + " output stream sets but was given "
                           + std::to_string(outputs.size()));
    }

    for (unsigned i = 0; i < outputs.size(); ++i) {
        StreamSetBuffer * const buf = outputs[i];
        if (LLVM_UNLIKELY(buf == nullptr)) {
            report_fatal_error(getName() + ": output stream set " + std::to_string(i) + " cannot be null");
        }
        if (LLVM_LIKELY(buf->getProducer() == nullptr)) {
            buf->setProducer(this);
        } else {
            report_fatal_error(getName() + ": output stream set " + std::to_string(i)
                               + " is already produced by kernel " + buf->getProducer()->getName());
        }
    }

    mModule = kernelModule;

    mStreamSetInputBuffers.assign(inputs.begin(), inputs.end());
    mStreamSetOutputBuffers.assign(outputs.begin(), outputs.end());

    prepareKernel();
}


// Default kernel signature: generate the IR and emit as byte code.
std::string Kernel::makeSignature() {
    assert ("KernelBuilder does not have a valid IDISA Builder" && iBuilder);
    if (LLVM_LIKELY(moduleIDisSignature())) {
        return getModule()->getModuleIdentifier();
    } else {
        generateKernel();
        std::string signature;
        raw_string_ostream OS(signature);
        WriteBitcodeToFile(getModule(), OS);
        return signature;
    }
}

void Kernel::generateKernel() {
    assert ("KernelBuilder does not have a valid IDISA Builder" && iBuilder);
    // If the module id cannot uniquely identify this kernel, "generateKernelSignature()" will have already
    // generated the unoptimized IR.
    if (!mIsGenerated) {
        auto ip = iBuilder->saveIP();
        auto saveInstance = getInstance();
        addKernelDeclarations();
        callGenerateInitializeMethod();
        callGenerateDoSegmentMethod();        
        callGenerateFinalizeMethod();
        setInstance(saveInstance);
        iBuilder->restoreIP(ip);
        mIsGenerated = true;
    }
}

inline void Kernel::callGenerateInitializeMethod() {
    mCurrentMethod = getInitFunction(iBuilder->getModule());
    iBuilder->SetInsertPoint(CreateBasicBlock("entry"));
    Function::arg_iterator args = mCurrentMethod->arg_begin();
    setInstance(&*(args++));
    iBuilder->CreateStore(ConstantAggregateZero::get(mKernelStateType), getInstance());
    for (const auto & binding : mScalarInputs) {
        setScalarField(binding.name, &*(args++));
    }
    for (const auto & binding : mStreamSetOutputs) {
        setConsumerLock(binding.name, &*(args++));
    }
    generateInitializeMethod();
    iBuilder->CreateRetVoid();
}

inline void Kernel::callGenerateDoSegmentMethod() {
    mCurrentMethod = getDoSegmentFunction(iBuilder->getModule());
    BasicBlock * const entry = CreateBasicBlock(getName() + "_entry");
    iBuilder->SetInsertPoint(entry);
    auto args = mCurrentMethod->arg_begin();
    setInstance(&*(args++));
    mIsFinal = &*(args++);
    const auto n = mStreamSetInputs.size();
    mAvailableItemCount.resize(n, nullptr);
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        mAvailableItemCount[i] = &*(args++);
    }
    generateDoSegmentMethod(); // must be overridden by the KernelBuilder subtype
    mIsFinal = nullptr;
    mAvailableItemCount.clear();
    iBuilder->CreateRetVoid();
}

inline void Kernel::callGenerateFinalizeMethod() {
    mCurrentMethod = getTerminateFunction(iBuilder->getModule());
    iBuilder->SetInsertPoint(CreateBasicBlock("entry"));
    auto args = mCurrentMethod->arg_begin();
    setInstance(&*(args++));
    generateFinalizeMethod(); // may be overridden by the KernelBuilder subtype
    const auto n = mScalarOutputs.size();
    if (n == 0) {
        iBuilder->CreateRetVoid();
    } else {
        Value * outputs[n];
        for (unsigned i = 0; i < n; ++i) {
            outputs[i] = getScalarField(mScalarOutputs[i].name);
        }
        if (n == 1) {
            iBuilder->CreateRet(outputs[0]);
        } else {
            iBuilder->CreateAggregateRet(outputs, n);
        }
    }
}

unsigned Kernel::getScalarIndex(const std::string & name) const {
    assert ("getScalarIndex was given a null IDISA Builder" && iBuilder);
    const auto f = mKernelMap.find(name);
    if (LLVM_UNLIKELY(f == mKernelMap.end())) {
        report_fatal_error(getName() + " does not contain scalar: " + name);
    }
    return f->second;
}

Value * Kernel::getProducedItemCount(const std::string & name, Value * doFinal) const {
    Port port; unsigned ssIdx;
    std::tie(port, ssIdx) = getStreamPort(name);
    assert (port == Port::Output);
    if (mStreamSetOutputs[ssIdx].rate.isExact()) {
        std::string refSet = mStreamSetOutputs[ssIdx].rate.referenceStreamSet();
        std::string principalField;
        if (refSet.empty()) {
            if (mStreamSetInputs.empty()) {
                principalField = mStreamSetOutputs[0].name + PRODUCED_ITEM_COUNT_SUFFIX;
            } else {
                principalField = mStreamSetInputs[0].name + PROCESSED_ITEM_COUNT_SUFFIX;
            }
        } else {
            Port port; unsigned pfIndex;
            std::tie(port, pfIndex) = getStreamPort(refSet);
            if (port == Port::Input) {
               principalField = refSet + PROCESSED_ITEM_COUNT_SUFFIX;
            } else {
               principalField = refSet + PRODUCED_ITEM_COUNT_SUFFIX;
            }
        }
        Value * principalItemsProcessed = getScalarField(principalField);
        return mStreamSetOutputs[ssIdx].rate.CreateRatioCalculation(iBuilder, principalItemsProcessed, doFinal);
    }
    return getScalarField(name + PRODUCED_ITEM_COUNT_SUFFIX);
}

llvm::Value * Kernel::getAvailableItemCount(const std::string & name) const {
    for (unsigned i = 0; i < mStreamSetInputs.size(); ++i) {
        if (mStreamSetInputs[i].name == name) {
            return mAvailableItemCount[i];
        }
    }
    return nullptr;
}

Value * Kernel::getProcessedItemCount(const std::string & name) const {
    Port port; unsigned ssIdx;
    std::tie(port, ssIdx) = getStreamPort(name);
    assert (port == Port::Input);
    if (mStreamSetInputs[ssIdx].rate.isExact()) {
        std::string refSet = mStreamSetInputs[ssIdx].rate.referenceStreamSet();
        if (refSet.empty()) {
            refSet = mStreamSetInputs[0].name;
        }
        Value * principalItemsProcessed = getScalarField(refSet + PROCESSED_ITEM_COUNT_SUFFIX);
        return mStreamSetInputs[ssIdx].rate.CreateRatioCalculation(iBuilder, principalItemsProcessed);
    }
    return getScalarField(name + PROCESSED_ITEM_COUNT_SUFFIX);
}

Value * Kernel::getConsumedItemCount(const std::string & name) const {
    return getScalarField(name + CONSUMED_ITEM_COUNT_SUFFIX);
}

void Kernel::setProducedItemCount(const std::string & name, Value * value) const {
    setScalarField(name + PRODUCED_ITEM_COUNT_SUFFIX, value);
}

void Kernel::setProcessedItemCount(const std::string & name, Value * value) const {
    setScalarField(name + PROCESSED_ITEM_COUNT_SUFFIX, value);
}

void Kernel::setConsumedItemCount(const std::string & name, Value * value) const {
    setScalarField(name + CONSUMED_ITEM_COUNT_SUFFIX, value);
}

Value * Kernel::getTerminationSignal() const {
    return getScalarField(TERMINATION_SIGNAL);
}

void Kernel::setTerminationSignal() const {
    setScalarField(TERMINATION_SIGNAL, iBuilder->getTrue());
}

LoadInst * Kernel::acquireLogicalSegmentNo() const {
    assert (iBuilder);
    return iBuilder->CreateAtomicLoadAcquire(getScalarFieldPtr(LOGICAL_SEGMENT_NO_SCALAR));
}

void Kernel::releaseLogicalSegmentNo(Value * nextSegNo) const {
    iBuilder->CreateAtomicStoreRelease(nextSegNo, getScalarFieldPtr(LOGICAL_SEGMENT_NO_SCALAR));
}

llvm::Value * Kernel::getLinearlyAccessibleItems(const std::string & name, llvm::Value * fromPosition) const {
    const StreamSetBuffer * const buf = getInputStreamSetBuffer(name);
    return buf->getLinearlyAccessibleItems(iBuilder, fromPosition);
}

llvm::Value * Kernel::getConsumerLock(const std::string & name) const {
    return getScalarField(name + CONSUMER_SUFFIX);
}

void Kernel::setConsumerLock(const std::string & name, llvm::Value * value) const {
    setScalarField(name + CONSUMER_SUFFIX, value);
}

inline Value * Kernel::computeBlockIndex(const std::vector<Binding> & bindings, const std::string & name, Value * itemCount) const {
    for (const Binding & b : bindings) {
        if (b.name == name) {
            const auto divisor = iBuilder->getBitBlockWidth();
            if (LLVM_LIKELY((divisor & (divisor - 1)) == 0)) {
                return iBuilder->CreateLShr(itemCount, std::log2(divisor));
            } else {
                return iBuilder->CreateUDiv(itemCount, iBuilder->getSize(divisor));
            }
        }
    }
    report_fatal_error("Error: no binding in " + getName() + " for " + name);
}

Value * Kernel::getInputStreamBlockPtr(const std::string & name, Value * streamIndex) const {
    Value * const blockIndex = computeBlockIndex(mStreamSetInputs, name, getProcessedItemCount(name));
    const StreamSetBuffer * const buf = getInputStreamSetBuffer(name);
    return buf->getStreamBlockPtr(iBuilder, getStreamSetBufferPtr(name), streamIndex, blockIndex, true);
}

Value * Kernel::loadInputStreamBlock(const std::string & name, Value * streamIndex) const {
    return iBuilder->CreateBlockAlignedLoad(getInputStreamBlockPtr(name, streamIndex));
}

Value * Kernel::getInputStreamPackPtr(const std::string & name, Value * streamIndex, Value * packIndex) const {
    Value * const blockIndex = computeBlockIndex(mStreamSetInputs, name, getProcessedItemCount(name));
    const StreamSetBuffer * const buf = getInputStreamSetBuffer(name);
    return buf->getStreamPackPtr(iBuilder, getStreamSetBufferPtr(name), streamIndex, blockIndex, packIndex, true);
}

Value * Kernel::loadInputStreamPack(const std::string & name, Value * streamIndex, Value * packIndex) const {
    return iBuilder->CreateBlockAlignedLoad(getInputStreamPackPtr(name, streamIndex, packIndex));
}

llvm::Value * Kernel::getInputStreamSetCount(const std::string & name) const {
    return getInputStreamSetBuffer(name)->getStreamSetCount(iBuilder, getStreamSetBufferPtr(name));
}

llvm::Value * Kernel::getAdjustedInputStreamBlockPtr(Value * blockAdjustment, const std::string & name, llvm::Value * streamIndex) const {
    Value * blockIndex = computeBlockIndex(mStreamSetInputs, name, getProcessedItemCount(name));
    blockIndex = iBuilder->CreateAdd(blockIndex, blockAdjustment);
    const StreamSetBuffer * const buf = getInputStreamSetBuffer(name);
    return buf->getStreamBlockPtr(iBuilder, getStreamSetBufferPtr(name), streamIndex, blockIndex, true);
}

Value * Kernel::getOutputStreamBlockPtr(const std::string & name, Value * streamIndex) const {
    Value * const blockIndex = computeBlockIndex(mStreamSetOutputs, name, getProducedItemCount(name));
    const StreamSetBuffer * const buf = getOutputStreamSetBuffer(name);
    return buf->getStreamBlockPtr(iBuilder, getStreamSetBufferPtr(name), streamIndex, blockIndex, false);
}

void Kernel::storeOutputStreamBlock(const std::string & name, Value * streamIndex, Value * toStore) const {
    return iBuilder->CreateBlockAlignedStore(toStore, getOutputStreamBlockPtr(name, streamIndex));
}

Value * Kernel::getOutputStreamPackPtr(const std::string & name, Value * streamIndex, Value * packIndex) const {
    Value * const blockIndex = computeBlockIndex(mStreamSetOutputs, name, getProducedItemCount(name));
    const StreamSetBuffer * const buf = getOutputStreamSetBuffer(name);
    return buf->getStreamPackPtr(iBuilder, getStreamSetBufferPtr(name), streamIndex, blockIndex, packIndex, false);
}

void Kernel::storeOutputStreamPack(const std::string & name, Value * streamIndex, Value * packIndex, Value * toStore) const {
    return iBuilder->CreateBlockAlignedStore(toStore, getOutputStreamPackPtr(name, streamIndex, packIndex));
}

llvm::Value * Kernel::getOutputStreamSetCount(const std::string & name) const {
    return getOutputStreamSetBuffer(name)->getStreamSetCount(iBuilder, getStreamSetBufferPtr(name));
}

Value * Kernel::getRawInputPointer(const std::string & name, Value * streamIndex, Value * absolutePosition) const {
    return getInputStreamSetBuffer(name)->getRawItemPointer(iBuilder, getStreamSetBufferPtr(name), streamIndex, absolutePosition);
}

Value * Kernel::getRawOutputPointer(const std::string & name, Value * streamIndex, Value * absolutePosition) const {
    return getOutputStreamSetBuffer(name)->getRawItemPointer(iBuilder, getStreamSetBufferPtr(name), streamIndex, absolutePosition);
}

Value * Kernel::getBaseAddress(const std::string & name) const {
    return getAnyStreamSetBuffer(name)->getBaseAddress(iBuilder, getStreamSetBufferPtr(name));
}

void Kernel::setBaseAddress(const std::string & name, Value * const addr) const {
    return getAnyStreamSetBuffer(name)->setBaseAddress(iBuilder, getStreamSetBufferPtr(name), addr);
}

Value * Kernel::getBufferedSize(const std::string & name) const {
    return getAnyStreamSetBuffer(name)->getBufferedSize(iBuilder, getStreamSetBufferPtr(name));
}

void Kernel::setBufferedSize(const std::string & name, Value * size) const {
    unsigned index; Port port;
    std::tie(port, index) = getStreamPort(name);
    const StreamSetBuffer * buf = nullptr;
    if (port == Port::Input) {
        assert (index < mStreamSetInputBuffers.size());
        buf = mStreamSetInputBuffers[index];
    } else {
        assert (index < mStreamSetOutputBuffers.size());
        buf = mStreamSetOutputBuffers[index];
    }
    buf->setBufferedSize(iBuilder, getStreamSetBufferPtr(name), size);
}

BasicBlock * Kernel::CreateWaitForConsumers() const {

    const auto consumers = getStreamOutputs();
    BasicBlock * const entry = iBuilder->GetInsertBlock();
    if (consumers.empty()) {
        return entry;
    } else {
        Function * const parent = entry->getParent();
        IntegerType * const sizeTy = iBuilder->getSizeTy();
        ConstantInt * const zero = iBuilder->getInt32(0);
        ConstantInt * const one = iBuilder->getInt32(1);
        ConstantInt * const size0 = iBuilder->getSize(0);

        Value * const segNo = acquireLogicalSegmentNo();
        const auto n = consumers.size();
        BasicBlock * load[n + 1];
        BasicBlock * wait[n];
        for (unsigned i = 0; i < n; ++i) {
            load[i] = BasicBlock::Create(iBuilder->getContext(), consumers[i].name + "Load", parent);
            wait[i] = BasicBlock::Create(iBuilder->getContext(), consumers[i].name + "Wait", parent);
        }
        load[n] = BasicBlock::Create(iBuilder->getContext(), "Resume", parent);
        iBuilder->CreateBr(load[0]);
        for (unsigned i = 0; i < n; ++i) {

            iBuilder->SetInsertPoint(load[i]);
            Value * const outputConsumers = getConsumerLock(consumers[i].name);

            Value * const consumerCount = iBuilder->CreateLoad(iBuilder->CreateGEP(outputConsumers, {zero, zero}));
            Value * const consumerPtr = iBuilder->CreateLoad(iBuilder->CreateGEP(outputConsumers, {zero, one}));
            Value * const noConsumers = iBuilder->CreateICmpEQ(consumerCount, size0);
            iBuilder->CreateUnlikelyCondBr(noConsumers, load[i + 1], wait[i]);

            iBuilder->SetInsertPoint(wait[i]);
            PHINode * const consumerPhi = iBuilder->CreatePHI(sizeTy, 2);
            consumerPhi->addIncoming(size0, load[i]);

            Value * const conSegPtr = iBuilder->CreateLoad(iBuilder->CreateGEP(consumerPtr, consumerPhi));
            Value * const processedSegmentCount = iBuilder->CreateAtomicLoadAcquire(conSegPtr);
            Value * const ready = iBuilder->CreateICmpEQ(segNo, processedSegmentCount);
            assert (ready->getType() == iBuilder->getInt1Ty());
            Value * const nextConsumerIdx = iBuilder->CreateAdd(consumerPhi, iBuilder->CreateZExt(ready, sizeTy));
            consumerPhi->addIncoming(nextConsumerIdx, wait[i]);
            Value * const next = iBuilder->CreateICmpEQ(nextConsumerIdx, consumerCount);
            iBuilder->CreateCondBr(next, load[i + 1], wait[i]);
        }

        BasicBlock * const exit = load[n];
        iBuilder->SetInsertPoint(exit);
        return exit;
    }

}

Value * Kernel::getStreamSetBufferPtr(const std::string & name) const {
    return getScalarField(name + BUFFER_PTR_SUFFIX);
}

//Argument * Kernel::getParameter(Function * const f, const std::string & name) const {
//    for (auto & arg : f->getArgumentList()) {
//        if (arg.getName().equals(name)) {
//            return &arg;
//        }
//    }
//    report_fatal_error(getName() + " does not have parameter " + name);
//}

CallInst * Kernel::createDoSegmentCall(const std::vector<Value *> & args) const {
    Function * const doSegment = getDoSegmentFunction(iBuilder->getModule());
    assert (doSegment->getArgumentList().size() == args.size());
    return iBuilder->CreateCall(doSegment, args);
}

Value * Kernel::getAccumulator(const std::string & accumName) const {
    assert ("KernelBuilder does not have a valid IDISA Builder" && iBuilder);
    if (LLVM_UNLIKELY(mOutputScalarResult == nullptr)) {
        report_fatal_error("Cannot get accumulator " + accumName + " until " + getName() + " has terminated.");
    }
    const auto n = mScalarOutputs.size();
    if (LLVM_UNLIKELY(n == 0)) {
        report_fatal_error(getName() + " has no output scalars.");
    } else {
        for (unsigned i = 0; i < n; ++i) {
            const Binding & b = mScalarOutputs[i];
            if (b.name == accumName) {
                if (n == 1) {
                    return mOutputScalarResult;
                } else {
                    return iBuilder->CreateExtractValue(mOutputScalarResult, {i});
                }
            }
        }
        report_fatal_error(getName() + " has no output scalar named " + accumName);
    }
}

BasicBlock * Kernel::CreateBasicBlock(std::string && name) const {
    return BasicBlock::Create(iBuilder->getContext(), name, mCurrentMethod);
}

Value * Kernel::createInstance() {
    assert ("KernelBuilder does not have a valid IDISA Builder" && iBuilder);
    if (LLVM_UNLIKELY(mKernelStateType == nullptr)) {
        report_fatal_error("Cannot instantiate " + getName() + " before calling prepareKernel()");
    }
    setInstance(iBuilder->CreateCacheAlignedAlloca(mKernelStateType));
    return getInstance();
}

void Kernel::initializeInstance() {
    assert ("KernelBuilder does not have a valid IDISA Builder" && iBuilder);
    if (LLVM_UNLIKELY(getInstance() == nullptr)) {
        report_fatal_error("Cannot initialize " + getName() + " before calling createInstance()");
    }
    std::vector<Value *> args;
    args.reserve(1 + mInitialArguments.size() + mStreamSetInputBuffers.size() + (mStreamSetOutputBuffers.size() * 2));
    args.push_back(getInstance());
    for (unsigned i = 0; i < mInitialArguments.size(); ++i) {
        Value * arg = mInitialArguments[i];
        if (LLVM_UNLIKELY(arg == nullptr)) {
            report_fatal_error(getName() + ": initial argument " + std::to_string(i)
                               + " cannot be null when calling createInstance()");
        }
        args.push_back(arg);
    }
    for (unsigned i = 0; i < mStreamSetInputBuffers.size(); ++i) {
        assert (mStreamSetInputBuffers[i]);
        Value * arg = mStreamSetInputBuffers[i]->getStreamSetBasePtr();
        if (LLVM_UNLIKELY(arg == nullptr)) {
            report_fatal_error(getName() + ": input stream set " + std::to_string(i)
                               + " was not allocated prior to calling createInstance()");
        }
        args.push_back(arg);
    }
    assert (mStreamSetInputs.size() == mStreamSetInputBuffers.size());
    for (unsigned i = 0; i < mStreamSetOutputBuffers.size(); ++i) {
        assert (mStreamSetOutputBuffers[i]);
        Value * arg = mStreamSetOutputBuffers[i]->getStreamSetBasePtr();
        if (LLVM_UNLIKELY(arg == nullptr)) {
            report_fatal_error(getName() + ": output stream set " + std::to_string(i)
                               + " was not allocated prior to calling createInstance()");
        }
        args.push_back(arg);
    }
    assert (mStreamSetOutputs.size() == mStreamSetOutputBuffers.size());
    IntegerType * const sizeTy = iBuilder->getSizeTy();
    PointerType * const sizePtrTy = sizeTy->getPointerTo();
    PointerType * const sizePtrPtrTy = sizePtrTy->getPointerTo();
    StructType * const consumerTy = StructType::get(sizeTy, sizePtrPtrTy, nullptr);
    for (unsigned i = 0; i < mStreamSetOutputBuffers.size(); ++i) {
        const auto output = mStreamSetOutputBuffers[i];
        const auto & consumers = output->getConsumers();
        const auto n = consumers.size();
        AllocaInst * const outputConsumers = iBuilder->CreateAlloca(consumerTy);
        Value * const consumerSegNoArray = iBuilder->CreateAlloca(ArrayType::get(sizePtrTy, n));
        for (unsigned i = 0; i < n; ++i) {
            Kernel * const consumer = consumers[i];
            assert ("all instances must be created prior to initialization of any instance" && consumer->getInstance());
            Value * const segmentNoPtr = consumer->getScalarFieldPtr(LOGICAL_SEGMENT_NO_SCALAR);
            iBuilder->CreateStore(segmentNoPtr, iBuilder->CreateGEP(consumerSegNoArray, { iBuilder->getInt32(0), iBuilder->getInt32(i) }));
        }
        Value * const consumerCountPtr = iBuilder->CreateGEP(outputConsumers, {iBuilder->getInt32(0), iBuilder->getInt32(0)});
        iBuilder->CreateStore(iBuilder->getSize(n), consumerCountPtr);
        Value * const consumerSegNoArrayPtr = iBuilder->CreateGEP(outputConsumers, {iBuilder->getInt32(0), iBuilder->getInt32(1)});
        iBuilder->CreateStore(iBuilder->CreatePointerCast(consumerSegNoArray, sizePtrPtrTy), consumerSegNoArrayPtr);
        args.push_back(outputConsumers);
    }

    iBuilder->CreateCall(getInitFunction(iBuilder->getModule()), args);
}

//  The default doSegment method dispatches to the doBlock routine for
//  each block of the given number of blocksToDo, and then updates counts.

void BlockOrientedKernel::generateDoSegmentMethod() {    
    BasicBlock * const entryBlock = iBuilder->GetInsertBlock();
    BasicBlock * const strideLoopCond = CreateBasicBlock(getName() + "_strideLoopCond");
    mStrideLoopBody = CreateBasicBlock(getName() + "_strideLoopBody");
    BasicBlock * const stridesDone = CreateBasicBlock(getName() + "_stridesDone");
    BasicBlock * const doFinalBlock = CreateBasicBlock(getName() + "_doFinalBlock");
    BasicBlock * const segmentDone = CreateBasicBlock(getName() + "_segmentDone");

    Value * baseTarget = nullptr;
    if (useIndirectBr()) {
        baseTarget = iBuilder->CreateSelect(mIsFinal, BlockAddress::get(doFinalBlock), BlockAddress::get(segmentDone));
    }

    ConstantInt * stride = iBuilder->getSize(iBuilder->getStride());
    Value * availablePos = mAvailableItemCount[0];
    Value * processed = getProcessedItemCount(mStreamSetInputs[0].name);
    Value * itemsAvail = iBuilder->CreateSub(availablePos, processed);
    Value * stridesToDo = iBuilder->CreateUDiv(itemsAvail, stride);

    iBuilder->CreateBr(strideLoopCond);

    iBuilder->SetInsertPoint(strideLoopCond);

    PHINode * branchTarget = nullptr;
    if (useIndirectBr()) {
        branchTarget = iBuilder->CreatePHI(baseTarget->getType(), 2, "branchTarget");
        branchTarget->addIncoming(baseTarget, entryBlock);
    }

    PHINode * const stridesRemaining = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2, "stridesRemaining");
    stridesRemaining->addIncoming(stridesToDo, entryBlock);
    // NOTE: stridesRemaining may go to a negative number in the final block if the generateFinalBlockMethod(...)
    // calls CreateDoBlockMethodCall(). Do *not* replace the comparator with an unsigned one!
    Value * notDone = iBuilder->CreateICmpSGT(stridesRemaining, iBuilder->getSize(0));
    iBuilder->CreateLikelyCondBr(notDone, mStrideLoopBody, stridesDone);

    iBuilder->SetInsertPoint(mStrideLoopBody);

    if (useIndirectBr()) {
        mStrideLoopTarget = iBuilder->CreatePHI(baseTarget->getType(), 2, "strideTarget");
        mStrideLoopTarget->addIncoming(branchTarget, strideLoopCond);
    }

    /// GENERATE DO BLOCK METHOD

    writeDoBlockMethod();

    /// UPDATE PROCESSED COUNTS

    processed = getProcessedItemCount(mStreamSetInputs[0].name);
    Value * itemsDone = iBuilder->CreateAdd(processed, stride);
    setProcessedItemCount(mStreamSetInputs[0].name, itemsDone);

    stridesRemaining->addIncoming(iBuilder->CreateSub(stridesRemaining, iBuilder->getSize(1)), iBuilder->GetInsertBlock());

    BasicBlock * bodyEnd = iBuilder->GetInsertBlock();
    if (useIndirectBr()) {
        branchTarget->addIncoming(mStrideLoopTarget, bodyEnd);
    }
    iBuilder->CreateBr(strideLoopCond);

    stridesDone->moveAfter(bodyEnd);

    iBuilder->SetInsertPoint(stridesDone);

    // Now conditionally perform the final block processing depending on the doFinal parameter.
    if (useIndirectBr()) {
        mStrideLoopBranch = iBuilder->CreateIndirectBr(branchTarget, 3);
        mStrideLoopBranch->addDestination(doFinalBlock);
        mStrideLoopBranch->addDestination(segmentDone);
    } else {
        iBuilder->CreateUnlikelyCondBr(mIsFinal, doFinalBlock, segmentDone);
    }

    doFinalBlock->moveAfter(stridesDone);

    iBuilder->SetInsertPoint(doFinalBlock);

    Value * remainingItems = iBuilder->CreateSub(mAvailableItemCount[0], getProcessedItemCount(mStreamSetInputs[0].name));
    writeFinalBlockMethod(remainingItems);

    itemsDone = mAvailableItemCount[0];
    setProcessedItemCount(mStreamSetInputs[0].name, itemsDone);
    setTerminationSignal();
    iBuilder->CreateBr(segmentDone);

    segmentDone->moveAfter(iBuilder->GetInsertBlock());

    iBuilder->SetInsertPoint(segmentDone);

    // Update the branch prediction metadata to indicate that the likely target will be segmentDone
    if (useIndirectBr()) {
        MDBuilder mdb(iBuilder->getContext());
        const auto destinations = mStrideLoopBranch->getNumDestinations();
        uint32_t weights[destinations];
        for (unsigned i = 0; i < destinations; ++i) {
            weights[i] = (mStrideLoopBranch->getDestination(i) == segmentDone) ? 100 : 1;
        }
        ArrayRef<uint32_t> bw(weights, destinations);
        mStrideLoopBranch->setMetadata(LLVMContext::MD_prof, mdb.createBranchWeights(bw));
    }

}

inline void BlockOrientedKernel::writeDoBlockMethod() {

    Value * const self = getInstance();
    Function * const cp = mCurrentMethod;
    auto ip = iBuilder->saveIP();

    /// Check if the do block method is called and create the function if necessary    
    if (!useIndirectBr()) {
        FunctionType * const type = FunctionType::get(iBuilder->getVoidTy(), {self->getType()}, false);
        mCurrentMethod = Function::Create(type, GlobalValue::InternalLinkage, getName() + DO_BLOCK_SUFFIX, iBuilder->getModule());
        mCurrentMethod->setCallingConv(CallingConv::C);
        mCurrentMethod->setDoesNotThrow();
        mCurrentMethod->setDoesNotCapture(1);
        auto args = mCurrentMethod->arg_begin();
        args->setName("self");
        setInstance(&*args);
        iBuilder->SetInsertPoint(CreateBasicBlock("entry"));
    }

    std::vector<Value *> priorProduced;
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        if (isa<CircularCopybackBuffer>(mStreamSetOutputBuffers[i]) || isa<SwizzledCopybackBuffer>(mStreamSetOutputBuffers[i]))  {
            priorProduced.push_back(getProducedItemCount(mStreamSetOutputs[i].name));
        }
    }

    generateDoBlockMethod(); // must be implemented by the BlockOrientedKernelBuilder subtype

    unsigned priorIdx = 0;
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        Value * log2BlockSize = iBuilder->getSize(std::log2(iBuilder->getBitBlockWidth()));
        if (SwizzledCopybackBuffer * const cb = dyn_cast<SwizzledCopybackBuffer>(mStreamSetOutputBuffers[i]))  {
            BasicBlock * copyBack = CreateBasicBlock(mStreamSetOutputs[i].name + "_copyBack");
            BasicBlock * done = CreateBasicBlock(mStreamSetOutputs[i].name + "_copyBackDone");
            Value * newlyProduced = iBuilder->CreateSub(getProducedItemCount(mStreamSetOutputs[i].name), priorProduced[priorIdx]);
            Value * priorBlock = iBuilder->CreateLShr(priorProduced[priorIdx], log2BlockSize);
            Value * priorOffset = iBuilder->CreateAnd(priorProduced[priorIdx], iBuilder->getSize(iBuilder->getBitBlockWidth() - 1));
            Value * instance = getStreamSetBufferPtr(mStreamSetOutputs[i].name);
            Value * accessibleBlocks = cb->getLinearlyAccessibleBlocks(iBuilder, priorBlock);
            Value * accessible = iBuilder->CreateSub(iBuilder->CreateShl(accessibleBlocks, log2BlockSize), priorOffset);
            Value * wraparound = iBuilder->CreateICmpULT(accessible, newlyProduced);
            iBuilder->CreateCondBr(wraparound, copyBack, done);
            iBuilder->SetInsertPoint(copyBack);
            Value * copyItems = iBuilder->CreateSub(newlyProduced, accessible);
            cb->createCopyBack(iBuilder, instance, copyItems);
            iBuilder->CreateBr(done);
            iBuilder->SetInsertPoint(done);
            priorIdx++;
        }
        if (CircularCopybackBuffer * const cb = dyn_cast<CircularCopybackBuffer>(mStreamSetOutputBuffers[i]))  {
            BasicBlock * copyBack = CreateBasicBlock(mStreamSetOutputs[i].name + "_copyBack");
            BasicBlock * done = CreateBasicBlock(mStreamSetOutputs[i].name + "_copyBackDone");
            Value * instance = getStreamSetBufferPtr(mStreamSetOutputs[i].name);
            Value * newlyProduced = iBuilder->CreateSub(getProducedItemCount(mStreamSetOutputs[i].name), priorProduced[priorIdx]);
            Value * accessible = cb->getLinearlyAccessibleItems(iBuilder, priorProduced[priorIdx]);
            Value * wraparound = iBuilder->CreateICmpULT(accessible, newlyProduced);
            iBuilder->CreateCondBr(wraparound, copyBack, done);
            iBuilder->SetInsertPoint(copyBack);
            Value * copyItems = iBuilder->CreateSub(newlyProduced, accessible);
            cb->createCopyBack(iBuilder, instance, copyItems);
            iBuilder->CreateBr(done);
            iBuilder->SetInsertPoint(done);
            priorIdx++;
        }
    }


    /// Call the do block method if necessary then restore the current function state to the do segement method
    if (!useIndirectBr()) {
        iBuilder->CreateRetVoid();
        mDoBlockMethod = mCurrentMethod;
        iBuilder->restoreIP(ip);
        iBuilder->CreateCall(mCurrentMethod, self);
        setInstance(self);
        mCurrentMethod = cp;
    }

}

inline void BlockOrientedKernel::writeFinalBlockMethod(Value * remainingItems) {

    Value * const self = getInstance();
    Function * const cp = mCurrentMethod;
    Value * const remainingItemCount = remainingItems;
    auto ip = iBuilder->saveIP();

    if (!useIndirectBr()) {
        FunctionType * const type = FunctionType::get(iBuilder->getVoidTy(), {self->getType(), iBuilder->getSizeTy()}, false);
        mCurrentMethod = Function::Create(type, GlobalValue::InternalLinkage, getName() + FINAL_BLOCK_SUFFIX, iBuilder->getModule());
        mCurrentMethod->setCallingConv(CallingConv::C);
        mCurrentMethod->setDoesNotThrow();
        mCurrentMethod->setDoesNotCapture(1);
        auto args = mCurrentMethod->arg_begin();
        args->setName("self");
        setInstance(&*args);
        remainingItems = &*(++args);
        remainingItems->setName("remainingItems");
        iBuilder->SetInsertPoint(CreateBasicBlock("entry"));
    }

    generateFinalBlockMethod(remainingItems); // may be implemented by the BlockOrientedKernel subtype

    RecursivelyDeleteTriviallyDeadInstructions(remainingItems); // if remainingItems was not used, this will eliminate it.

    if (!useIndirectBr()) {
        iBuilder->CreateRetVoid();        
        iBuilder->restoreIP(ip);
        iBuilder->CreateCall(mCurrentMethod, {self, remainingItemCount});
        mCurrentMethod = cp;
        setInstance(self);
    }

}

//  The default finalBlock method simply dispatches to the doBlock routine.
void BlockOrientedKernel::generateFinalBlockMethod(Value * /* remainingItems */) {
    CreateDoBlockMethodCall();
}

bool BlockOrientedKernel::useIndirectBr() const {
    return iBuilder->supportsIndirectBr();
}

void BlockOrientedKernel::CreateDoBlockMethodCall() {
    if (useIndirectBr()) {
        BasicBlock * bb = CreateBasicBlock("resume");
        mStrideLoopBranch->addDestination(bb);
        mStrideLoopTarget->addIncoming(BlockAddress::get(bb), iBuilder->GetInsertBlock());
        iBuilder->CreateBr(mStrideLoopBody);
        bb->moveAfter(iBuilder->GetInsertBlock());
        iBuilder->SetInsertPoint(bb);
    } else {
        iBuilder->CreateCall(mDoBlockMethod, getInstance());
    }
}

void Kernel::finalizeInstance() {
    assert ("KernelBuilder does not have a valid IDISA Builder" && iBuilder);
    mOutputScalarResult = iBuilder->CreateCall(getTerminateFunction(iBuilder->getModule()), { getInstance() });
}

Kernel::StreamPort Kernel::getStreamPort(const std::string & name) const {
    const auto f = mStreamMap.find(name);
    if (LLVM_UNLIKELY(f == mStreamMap.end())) {
        report_fatal_error(getName() + " does not contain stream set " + name);
    }
    return f->second;
}

    
void MultiBlockKernel::generateDoSegmentMethod() {
    
    // First prepare the multi-block method that will be used.
    
    std::vector<Type *> multiBlockParmTypes;
    multiBlockParmTypes.push_back(mKernelStateType->getPointerTo());
    for (auto buffer : mStreamSetInputBuffers) {
        multiBlockParmTypes.push_back(buffer->getPointerType());
    }
    for (auto buffer : mStreamSetOutputBuffers) {
        multiBlockParmTypes.push_back(buffer->getPointerType());
    }
    FunctionType * const type = FunctionType::get(iBuilder->getVoidTy(), multiBlockParmTypes, false);
    Function * multiBlockFunction = Function::Create(type, GlobalValue::InternalLinkage, getName() + MULTI_BLOCK_SUFFIX, iBuilder->getModule());
    multiBlockFunction->setCallingConv(CallingConv::C);
    multiBlockFunction->setDoesNotThrow();
    auto args = multiBlockFunction->arg_begin();
    args->setName("self");
    for (auto binding : mStreamSetInputs) {
        (++args)->setName(binding.name + "BufPtr");
    }
    for (auto binding : mStreamSetOutputs) {
        (args++)->setName(binding.name + "BufPtr");
    }
    
    // Now use the generateMultiBlockLogic method of the MultiBlockKernelBuilder subtype to 
    // provide the required multi-block kernel logic.
    auto ip = iBuilder->saveIP();
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "multiBlockEntry", multiBlockFunction, 0));
    generateMultiBlockLogic(); 
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(ip);
    
    // Now proceed with creation of the doSegment method.
    
    BasicBlock * const entry = iBuilder->GetInsertBlock();
    BasicBlock * const doSegmentOuterLoop = CreateBasicBlock(getName() + "_doSegmentOuterLoop");
    BasicBlock * const doMultiBlockCall = CreateBasicBlock(getName() + "_doMultiBlockCall");
    BasicBlock * const finalBlockCheck = CreateBasicBlock(getName() + "_finalBlockCheck");
    BasicBlock * const doTempBufferBlock = CreateBasicBlock(getName() + "_doTempBufferBlock");
    BasicBlock * const segmentDone = CreateBasicBlock(getName() + "_segmentDone");
    
    Value * blockBaseMask = iBuilder->CreateNot(iBuilder->getSize(iBuilder->getBitBlockWidth() - 1));
    
    //
    //  A. Temporary Buffer Area Determination
    //
    // For final block processing and for processing near the end of physical buffer
    // boundaries, we need to allocate temporary space for processing a full block of input.
    // Compute the size requirements to store stream set data at the declared processing
    // rates in reference to one block of the principal input stream.  
    // 

    unsigned bitBlockWidth = iBuilder->getBitBlockWidth();
    std::vector<Type *> tempBuffers;
    std::vector<unsigned> itemsPerPrincipalBlock;
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        auto & rate = mStreamSetInputs[i].rate;
        std::string refSet = mStreamSetInputs[i].rate.referenceStreamSet();
        if (refSet.empty()) {
            itemsPerPrincipalBlock.push_back(rate.calculateRatio(bitBlockWidth));
        }
        else {
            Port port; unsigned ssIdx;
            std::tie(port, ssIdx) = getStreamPort(mStreamSetInputs[i].name);
            assert (port == Port::Input && ssIdx < i);
            itemsPerPrincipalBlock.push_back(rate.calculateRatio(itemsPerPrincipalBlock[ssIdx]));
        }
        unsigned blocks = (itemsPerPrincipalBlock.back() + bitBlockWidth - 1)/bitBlockWidth;
        if (blocks > 1) {
            tempBuffers.push_back(ArrayType::get(mStreamSetInputBuffers[i]->getType(), blocks));
        }
        else {
            tempBuffers.push_back(mStreamSetInputBuffers[i]->getType());
        }
    }
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        auto & rate = mStreamSetOutputs[i].rate;
        std::string refSet = mStreamSetOutputs[i].rate.referenceStreamSet();
        if (refSet.empty()) {
            itemsPerPrincipalBlock.push_back(rate.calculateRatio(bitBlockWidth));
        }
        else {
            Port port; unsigned ssIdx;
            std::tie(port, ssIdx) = getStreamPort(mStreamSetOutputs[i].name);
            if (port == Port::Output) ssIdx += mStreamSetInputs.size();
            itemsPerPrincipalBlock.push_back(rate.calculateRatio(itemsPerPrincipalBlock[ssIdx]));
        }
        unsigned blocks = (itemsPerPrincipalBlock.back() + bitBlockWidth - 1)/bitBlockWidth;
        if (blocks > 1) {
            tempBuffers.push_back(ArrayType::get(mStreamSetOutputBuffers[i]->getType(), blocks));
        }
        else {
            tempBuffers.push_back(mStreamSetOutputBuffers[i]->getType());
        }
    }
    Type * tempParameterStructType = StructType::create(iBuilder->getContext(), tempBuffers);
    Value * tempParameterArea = iBuilder->CreateCacheAlignedAlloca(tempParameterStructType);
    
    ConstantInt * blockSize = iBuilder->getSize(iBuilder->getBitBlockWidth());
    Value * availablePos = mAvailableItemCount[0];
    Value * itemsAvail = availablePos;
    //  Make sure that corresponding data is available depending on processing rate
    //  for all input stream sets.
    for (unsigned i = 1; i < mStreamSetInputs.size(); i++) {
        Value * a = mAvailableItemCount[i];
        auto & rate = mStreamSetInputs[i].rate;
        assert (((rate.referenceStreamSet() == "") || (rate.referenceStreamSet() == mStreamSetInputs[0].name)) && "Multiblock kernel input rate not with respect to principal stream.");
        Value * maxItems = rate.CreateMaxReferenceItemsCalculation(iBuilder, a);
        itemsAvail = iBuilder->CreateSelect(iBuilder->CreateICmpULT(itemsAvail, maxItems), itemsAvail, maxItems);
    }
    
    Value * processed = getProcessedItemCount(mStreamSetInputs[0].name);
    Value * itemsToDo = iBuilder->CreateSub(itemsAvail, processed);
    Value * fullBlocksToDo = iBuilder->CreateUDiv(itemsToDo, blockSize);
    Value * excessItems = iBuilder->CreateURem(itemsToDo, blockSize);
    
    //  Now we iteratively process these blocks using the doMultiBlock method.  
    //  In each iteration, we process the maximum number of linearly accessible
    //  blocks on the principal input, reduced to ensure that the corresponding
    //  data is linearly available at the specified processing rates for the other inputs,
    //  and that each of the output buffers has sufficient linearly available space
    //  (using overflow areas, if necessary) for the maximum output that can be
    //  produced.
    
    //iBuilder->CreateCondBr(iBuilder->CreateICmpUGT(fullBlocksToDo, iBuilder->getSize(0)), doSegmentOuterLoop, finalBlockCheck);
    iBuilder->CreateBr(doSegmentOuterLoop);
    
    iBuilder->SetInsertPoint(doSegmentOuterLoop);
    PHINode * const blocksRemaining = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2, "blocksRemaining");
    blocksRemaining->addIncoming(fullBlocksToDo, entry);
    
    
    // For each input buffer, determine the processedItemCount, the block pointer for the
    // buffer block containing the next item, and the number of linearly available items.
    // 
    std::vector<Value *> processedItemCount;
    std::vector<Value *> inputBlockPtr;
    std::vector<Value *> producedItemCount;
    std::vector<Value *> outputBlockPtr;
    
    //  Calculate linearly available blocks for all input stream sets.
    Value * linearlyAvailBlocks = nullptr;
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        Value * p = getProcessedItemCount(mStreamSetInputs[i].name);
        Value * blkNo = iBuilder->CreateUDiv(p, blockSize);
        Value * b = getInputStreamBlockPtr(mStreamSetInputs[i].name, iBuilder->getInt32(0));
        processedItemCount.push_back(p);
        inputBlockPtr.push_back(b);
        auto & rate = mStreamSetInputs[i].rate;
        Value * blocks = nullptr;
        if ((rate.isFixedRatio()) && (rate.getRatioNumerator() == rate.getRatioDenominator()) && (rate.referenceStreamSet() == "")) {
            blocks = mStreamSetInputBuffers[i]->getLinearlyAccessibleBlocks(iBuilder, blkNo);
        }
        else {
            Value * linearlyAvailItems = mStreamSetInputBuffers[i]->getLinearlyAccessibleItems(iBuilder, p);
            Value * items = rate.CreateMaxReferenceItemsCalculation(iBuilder, linearlyAvailItems);
            blocks = iBuilder->CreateUDiv(items, blockSize);
        }
        if (i == 0) {
            linearlyAvailBlocks = blocks;
        }
        else {
            linearlyAvailBlocks = iBuilder->CreateSelect(iBuilder->CreateICmpULT(blocks, linearlyAvailBlocks), blocks, linearlyAvailBlocks);
        }
    }
    
    //  Now determine the linearly writeable blocks, based on available blocks reduced
    //  by limitations of output buffer space.
    Value * linearlyWritableBlocks = linearlyAvailBlocks;
    
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        Value * p = getProducedItemCount(mStreamSetOutputs[i].name);
        Value * blkNo = iBuilder->CreateUDiv(p, blockSize);
        Value * b = getOutputStreamBlockPtr(mStreamSetOutputs[i].name, iBuilder->getInt32(0));
        producedItemCount.push_back(p);
        outputBlockPtr.push_back(b);
        auto & rate = mStreamSetOutputs[i].rate;
        Value * blocks = nullptr;
        if ((rate.isFixedRatio()) && (rate.getRatioNumerator() == rate.getRatioDenominator())) {
            blocks = mStreamSetOutputBuffers[0]->getLinearlyWritableBlocks(iBuilder, blkNo);
        }
        else {
            Value * writableItems = mStreamSetOutputBuffers[0]->getLinearlyWritableItems(iBuilder, p);
            blocks = iBuilder->CreateUDiv(writableItems, blockSize);
        }
        linearlyWritableBlocks = iBuilder->CreateSelect(iBuilder->CreateICmpULT(blocks, linearlyWritableBlocks), blocks, linearlyWritableBlocks);
    }
    Value * haveBlocks = iBuilder->CreateICmpUGT(linearlyWritableBlocks, iBuilder->getSize(0));
    
    iBuilder->CreateCondBr(haveBlocks, doMultiBlockCall, doTempBufferBlock);
    
    //  At this point we have verified the availability of one or more blocks of input data and output buffer space for all stream sets.
    //  Now prepare the doMultiBlock call.
    iBuilder->SetInsertPoint(doMultiBlockCall);
    
    Value * linearlyAvailItems = iBuilder->CreateMul(linearlyWritableBlocks, blockSize);
    
    std::vector<Value *> doMultiBlockArgs;
    doMultiBlockArgs.push_back(linearlyAvailItems);
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        doMultiBlockArgs.push_back(getRawInputPointer(mStreamSetInputs[i].name, iBuilder->getInt32(0), processedItemCount[i]));
    }
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        doMultiBlockArgs.push_back(getRawOutputPointer(mStreamSetOutputs[i].name, iBuilder->getInt32(0), producedItemCount[i]));
    }
        
    iBuilder->CreateCall(multiBlockFunction, doMultiBlockArgs);
    
    // Do copybacks if necessary.
    unsigned priorIdx = 0;
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        Value * log2BlockSize = iBuilder->getSize(std::log2(iBuilder->getBitBlockWidth()));
        if (auto cb = dyn_cast<SwizzledCopybackBuffer>(mStreamSetOutputBuffers[i]))  {
            BasicBlock * copyBack = CreateBasicBlock(mStreamSetOutputs[i].name + "_copyBack");
            BasicBlock * done = CreateBasicBlock(mStreamSetOutputs[i].name + "_copyBackDone");
            Value * newlyProduced = iBuilder->CreateSub(getProducedItemCount(mStreamSetOutputs[i].name), producedItemCount[i]);
            Value * priorBlock = iBuilder->CreateLShr(producedItemCount[i], log2BlockSize);
            Value * priorOffset = iBuilder->CreateAnd(producedItemCount[i], iBuilder->getSize(iBuilder->getBitBlockWidth() - 1));
            Value * instance = getStreamSetBufferPtr(mStreamSetOutputs[i].name);
            Value * accessibleBlocks = cb->getLinearlyAccessibleBlocks(iBuilder, priorBlock);
            Value * accessible = iBuilder->CreateSub(iBuilder->CreateShl(accessibleBlocks, log2BlockSize), priorOffset);
            Value * wraparound = iBuilder->CreateICmpULT(accessible, newlyProduced);
            iBuilder->CreateCondBr(wraparound, copyBack, done);
            iBuilder->SetInsertPoint(copyBack);
            Value * copyItems = iBuilder->CreateSub(newlyProduced, accessible);
            cb->createCopyBack(iBuilder, instance, copyItems);
            iBuilder->CreateBr(done);
            iBuilder->SetInsertPoint(done);
            priorIdx++;
        }
        if (auto cb = dyn_cast<CircularCopybackBuffer>(mStreamSetOutputBuffers[i]))  {
            BasicBlock * copyBack = CreateBasicBlock(mStreamSetOutputs[i].name + "_copyBack");
            BasicBlock * done = CreateBasicBlock(mStreamSetOutputs[i].name + "_copyBackDone");
            Value * instance = getStreamSetBufferPtr(mStreamSetOutputs[i].name);
            Value * newlyProduced = iBuilder->CreateSub(getProducedItemCount(mStreamSetOutputs[i].name), producedItemCount[i]);
            Value * accessible = cb->getLinearlyAccessibleItems(iBuilder, producedItemCount[i]);
            Value * wraparound = iBuilder->CreateICmpULT(accessible, newlyProduced);
            iBuilder->CreateCondBr(wraparound, copyBack, done);
            iBuilder->SetInsertPoint(copyBack);
            Value * copyItems = iBuilder->CreateSub(newlyProduced, accessible);
            cb->createCopyBack(iBuilder, instance, copyItems);
            iBuilder->CreateBr(done);
            iBuilder->SetInsertPoint(done);
            priorIdx++;
        }
    }
    setProcessedItemCount(mStreamSetInputs[0].name, iBuilder->CreateAdd(processed, linearlyAvailItems));
    Value * reducedBlocksToDo = iBuilder->CreateSub(blocksRemaining, linearlyWritableBlocks);
    Value * fullBlocksRemain = iBuilder->CreateICmpUGT(reducedBlocksToDo, iBuilder->getSize(0));
    BasicBlock * multiBlockFinal = iBuilder->GetInsertBlock();
    blocksRemaining->addIncoming(reducedBlocksToDo, multiBlockFinal);
    iBuilder->CreateCondBr(fullBlocksRemain, doSegmentOuterLoop, finalBlockCheck);
    
    // All the full blocks of input have been processed.  If mIsFinal is true,
    // we should process the remaining partial block (i.e., excessItems as determined at entry).
    iBuilder->SetInsertPoint(finalBlockCheck);
    iBuilder->CreateCondBr(mIsFinal, doTempBufferBlock, segmentDone);
    
    //  
    // We use temporary buffers in 3 different cases that preclude full block processing.
    // (a) One or more input buffers does not have a sufficient number of input items linearly available.
    // (b) One or more output buffers does not have sufficient linearly available buffer space.
    // (c) We have processed all the full blocks of input and only the excessItems remain.
    // In each case we set up temporary buffers for input and output and then
    // call the Multiblock routine.
    //
    iBuilder->SetInsertPoint(doTempBufferBlock);
    PHINode * const tempBlockItems = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2, "tempBlockItems");
    tempBlockItems->addIncoming(blockSize, doSegmentOuterLoop);
    tempBlockItems->addIncoming(excessItems, finalBlockCheck);
    
    // Will this be the final block processing?
    Value * doFinal = iBuilder->CreateICmpULT(tempBlockItems, blockSize);
    
    // Begin constructing the doMultiBlock args.
    std::vector<Value *> tempArgs;
    tempArgs.push_back(tempBlockItems);
    
    // Prepare the temporary buffer area.
    //
    // First zero it out.
    Constant * const tempAreaSize = ConstantExpr::getIntegerCast(ConstantExpr::getSizeOf(tempParameterStructType), iBuilder->getSizeTy(), false);
    iBuilder->CreateMemZero(tempParameterArea, tempAreaSize);
    
    // For each input and output buffer, copy over necessary data starting from the last
    // block boundary.
    std::vector<Value *> finalItemPos;
    finalItemPos.push_back(iBuilder->CreateAdd(processedItemCount[0], tempBlockItems));

    for (unsigned i = 0; i < mStreamSetInputBuffers.size(); i++) {
        Value * tempBufPtr = iBuilder->CreateGEP(tempParameterArea, iBuilder->getInt32(i));
        tempBufPtr = iBuilder->CreatePointerCast(tempBufPtr, mStreamSetInputBuffers[i]->getPointerType());
        
        auto & rate = mStreamSetInputs[i].rate;
        Value * blockItemPos = iBuilder->CreateAnd(processedItemCount[i], blockBaseMask);
        
        // The number of items to copy is determined by the processing rate requirements.
        if (i > 1) {
            std::string refSet = mStreamSetInputs[i].rate.referenceStreamSet();
            if (refSet.empty()) {
                finalItemPos.push_back(rate.CreateRatioCalculation(iBuilder, finalItemPos[0], doFinal));
            }
            else {
                Port port; unsigned ssIdx;
                std::tie(port, ssIdx) = getStreamPort(mStreamSetInputs[i].name);
                assert (port == Port::Input && ssIdx < i);
                finalItemPos.push_back(rate.CreateRatioCalculation(iBuilder, finalItemPos[ssIdx], doFinal));
            }
        }
        Value * neededItems = iBuilder->CreateSub(finalItemPos[i], blockItemPos);
        Value * availFromBase = mStreamSetInputBuffers[i]->getLinearlyAccessibleItems(iBuilder, blockItemPos);
        Value * copyItems1 = iBuilder->CreateSelect(iBuilder->CreateICmpULT(neededItems, availFromBase), neededItems, availFromBase);
        Value * copyItems2 = iBuilder->CreateSub(neededItems, copyItems1);
        mStreamSetInputBuffers[i]->createBlockAlignedCopy(iBuilder, tempBufPtr, inputBlockPtr[i], copyItems1);
        Value * nextBufPtr = iBuilder->CreateGEP(tempBufPtr, iBuilder->CreateUDiv(availFromBase, blockSize));
        mStreamSetInputBuffers[i]->createBlockAlignedCopy(iBuilder, nextBufPtr, getStreamSetBufferPtr(mStreamSetInputs[i].name), copyItems2);
        Value * itemAddress = iBuilder->CreatePtrToInt(getRawOutputPointer(mStreamSetInputs[i].name, iBuilder->getInt32(0), processedItemCount[i]), iBuilder->getSizeTy());
        Value * baseAddress = iBuilder->CreatePtrToInt(inputBlockPtr[i], iBuilder->getSizeTy());
        Value * tempAddress = iBuilder->CreateAdd(iBuilder->CreatePtrToInt(tempBufPtr, iBuilder->getSizeTy()), iBuilder->CreateSub(itemAddress, baseAddress));
        tempArgs.push_back(iBuilder->CreateBitCast(tempAddress, mStreamSetInputBuffers[i]->getPointerType()));
    }

    std::vector<Value *> blockItemPos;
    for (unsigned i = 0; i < mStreamSetOutputBuffers.size(); i++) {
        Value * tempBufPtr = iBuilder->CreateGEP(tempParameterArea, iBuilder->getInt32(mStreamSetInputs.size() + i));
        tempBufPtr = iBuilder->CreatePointerCast(tempBufPtr, mStreamSetOutputBuffers[i]->getPointerType());
        blockItemPos.push_back(iBuilder->CreateAnd(producedItemCount[i], blockBaseMask));
        mStreamSetOutputBuffers[i]->createBlockAlignedCopy(iBuilder, tempBufPtr, outputBlockPtr[i], iBuilder->CreateSub(producedItemCount[i], blockItemPos[i]));
        Value * itemAddress = iBuilder->CreatePtrToInt(getRawOutputPointer(mStreamSetInputs[i].name, iBuilder->getInt32(0), producedItemCount[i]), iBuilder->getSizeTy());
        Value * baseAddress = iBuilder->CreatePtrToInt(outputBlockPtr[i], iBuilder->getSizeTy());
        Value * tempAddress = iBuilder->CreateAdd(iBuilder->CreatePtrToInt(tempBufPtr, iBuilder->getSizeTy()), iBuilder->CreateSub(itemAddress, baseAddress));
        tempArgs.push_back(iBuilder->CreateBitCast(tempAddress, mStreamSetOutputBuffers[i]->getPointerType()));
    }

    iBuilder->CreateCall(multiBlockFunction, tempArgs);

    // Copy back data to the actual output buffers.
    
    for (unsigned i = 0; i < mStreamSetOutputBuffers.size(); i++) {
        Value * tempBufPtr = iBuilder->CreateGEP(tempParameterArea, iBuilder->getInt32(mStreamSetInputs.size() + i));
        tempBufPtr = iBuilder->CreatePointerCast(tempBufPtr, mStreamSetOutputBuffers[i]->getPointerType());
        Value * final_items = getProducedItemCount(mStreamSetOutputs[i].name);
        Value * copyItems = iBuilder->CreateSub(final_items, blockItemPos[i]);
        Value * copyItems1 = mStreamSetOutputBuffers[i]->getLinearlyWritableItems(iBuilder, blockItemPos[i]); // must be a whole number of blocks.
        mStreamSetOutputBuffers[i]->createBlockAlignedCopy(iBuilder, outputBlockPtr[i], tempBufPtr, copyItems1);
        Value * copyItems2 = iBuilder->CreateSelect(iBuilder->CreateICmpULT(copyItems, copyItems), iBuilder->getSize(0), iBuilder->CreateSub(copyItems, copyItems1));
        tempBufPtr = iBuilder->CreateGEP(tempBufPtr, iBuilder->CreateUDiv(copyItems1, blockSize));
        mStreamSetOutputBuffers[i]->createBlockAlignedCopy(iBuilder, getStreamSetBufferPtr(mStreamSetOutputs[i].name), tempBufPtr, copyItems2);
    }

    setProcessedItemCount(mStreamSetInputs[0].name, finalItemPos[0]);

    //  We've dealt with the partial block processing and copied information back into the
    //  actual buffers.  If this isn't the final block, loop back for more multiblock processing.
    //
    iBuilder->CreateCondBr(doFinal, segmentDone, doSegmentOuterLoop);
    iBuilder->SetInsertPoint(segmentDone);
}
                                                           
// CONSTRUCTOR
Kernel::Kernel(std::string && kernelName,
                             std::vector<Binding> && stream_inputs,
                             std::vector<Binding> && stream_outputs,
                             std::vector<Binding> && scalar_parameters,
                             std::vector<Binding> && scalar_outputs,
                             std::vector<Binding> && internal_scalars)
: KernelInterface(std::move(kernelName), std::move(stream_inputs), std::move(stream_outputs), std::move(scalar_parameters), std::move(scalar_outputs), std::move(internal_scalars))
, mCurrentMethod(nullptr)
, mNoTerminateAttribute(false)
, mIsGenerated(false)
, mIsFinal(nullptr)
, mOutputScalarResult(nullptr) {

}

Kernel::~Kernel() {

}

// CONSTRUCTOR
BlockOrientedKernel::BlockOrientedKernel(std::string && kernelName,
                                         std::vector<Binding> && stream_inputs,
                                         std::vector<Binding> && stream_outputs,
                                         std::vector<Binding> && scalar_parameters,
                                         std::vector<Binding> && scalar_outputs,
                                         std::vector<Binding> && internal_scalars)
: Kernel(std::move(kernelName), std::move(stream_inputs), std::move(stream_outputs), std::move(scalar_parameters), std::move(scalar_outputs), std::move(internal_scalars))
, mDoBlockMethod(nullptr)
, mStrideLoopBody(nullptr)
, mStrideLoopBranch(nullptr)
, mStrideLoopTarget(nullptr) {

}

// CONSTRUCTOR
SegmentOrientedKernel::SegmentOrientedKernel(std::string && kernelName,
                                             std::vector<Binding> && stream_inputs,
                                             std::vector<Binding> && stream_outputs,
                                             std::vector<Binding> && scalar_parameters,
                                             std::vector<Binding> && scalar_outputs,
                                             std::vector<Binding> && internal_scalars)
: Kernel(std::move(kernelName), std::move(stream_inputs), std::move(stream_outputs), std::move(scalar_parameters), std::move(scalar_outputs), std::move(internal_scalars)) {

}

// CONSTRUCTOR
MultiBlockKernel::MultiBlockKernel(std::string && kernelName,
                                   std::vector<Binding> && stream_inputs,
                                   std::vector<Binding> && stream_outputs,
                                   std::vector<Binding> && scalar_parameters,
                                   std::vector<Binding> && scalar_outputs,
                                   std::vector<Binding> && internal_scalars)
: Kernel(std::move(kernelName), std::move(stream_inputs), std::move(stream_outputs), std::move(scalar_parameters), std::move(scalar_outputs), std::move(internal_scalars)) {
    
}
}
