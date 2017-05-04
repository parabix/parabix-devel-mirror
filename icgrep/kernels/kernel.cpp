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

static const std::string DO_BLOCK_SUFFIX = "_DoBlock";

static const std::string FINAL_BLOCK_SUFFIX = "_FinalBlock";

static const std::string LOGICAL_SEGMENT_NO_SCALAR = "logicalSegNo";

static const std::string PROCESSED_ITEM_COUNT_SUFFIX = "_processedItemCount";

static const std::string CONSUMED_ITEM_COUNT_SUFFIX = "_consumedItemCount";

static const std::string PRODUCED_ITEM_COUNT_SUFFIX = "_producedItemCount";

static const std::string TERMINATION_SIGNAL = "terminationSignal";

static const std::string BUFFER_PTR_SUFFIX = "_bufferPtr";

static const std::string CONSUMER_SUFFIX = "_consumerLocks";

using namespace llvm;
using namespace kernel;
using namespace parabix;

unsigned KernelBuilder::addScalar(Type * const type, const std::string & name) {
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

unsigned KernelBuilder::addUnnamedScalar(Type * const type) {
    if (LLVM_UNLIKELY(mKernelStateType != nullptr)) {
        report_fatal_error("Cannot add unnamed field  to " + getName() + " after kernel state finalized");
    }
    const auto index = mKernelFields.size();
    mKernelFields.push_back(type);
    return index;
}

void KernelBuilder::prepareStreamSetNameMap() {
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        mStreamMap.emplace(mStreamSetInputs[i].name, std::make_pair(Port::Input, i));
    }
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        mStreamMap.emplace(mStreamSetOutputs[i].name, std::make_pair(Port::Output, i));
    }
}
    
void KernelBuilder::prepareKernel() {
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

void KernelBuilder::createKernelStub(const StreamSetBuffers & inputs, const StreamSetBuffers & outputs) {
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

void KernelBuilder::createKernelStub(const StreamSetBuffers & inputs, const StreamSetBuffers & outputs, Module * const kernelModule) {
    assert (mModule == nullptr);
    assert (mStreamSetInputBuffers.empty());
    assert (mStreamSetOutputBuffers.empty());

    if (LLVM_UNLIKELY(mStreamSetInputs.size() != inputs.size())) {
        report_fatal_error(getName() + ": expected " + std::to_string(mStreamSetInputs.size()) +
                           " input stream sets but was given "
                           + std::to_string(mStreamSetInputBuffers.size()));
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
                           + std::to_string(mStreamSetOutputBuffers.size()));
    }

    for (unsigned i = 0; i < outputs.size(); ++i) {
        StreamSetBuffer * const buf = outputs[i];
        if (LLVM_UNLIKELY(buf == nullptr)) {
            report_fatal_error(getName() + ": output stream set " + std::to_string(i)
                               + " cannot be null");
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
std::string KernelBuilder::makeSignature() {
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

void KernelBuilder::generateKernel() {
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

inline void KernelBuilder::callGenerateInitializeMethod() {
    mCurrentMethod = getInitFunction(iBuilder->getModule());
    iBuilder->SetInsertPoint(CreateBasicBlock("entry"));
    Function::arg_iterator args = mCurrentMethod->arg_begin();
    setInstance(&*(args++));
    iBuilder->CreateStore(ConstantAggregateZero::get(mKernelStateType), getInstance());
    for (auto binding : mScalarInputs) {
        setScalarField(binding.name, &*(args++));
    }
    for (auto binding : mStreamSetOutputs) {
        setConsumerLock(binding.name, &*(args++));
    }
    generateInitializeMethod();
    iBuilder->CreateRetVoid();
}

inline void KernelBuilder::callGenerateDoSegmentMethod() {
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

inline void KernelBuilder::callGenerateFinalizeMethod() {
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

ConstantInt * KernelBuilder::getScalarIndex(const std::string & name) const {
    const auto f = mKernelMap.find(name);
    if (LLVM_UNLIKELY(f == mKernelMap.end())) {
        report_fatal_error(getName() + " does not contain scalar: " + name);
    }
    return iBuilder->getInt32(f->second);
}

Value * KernelBuilder::getProducedItemCount(const std::string & name, Value * doFinal) const {
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

llvm::Value * KernelBuilder::getAvailableItemCount(const std::string & name) const {
    for (unsigned i = 0; i < mStreamSetInputs.size(); ++i) {
        if (mStreamSetInputs[i].name == name) {
            return mAvailableItemCount[i];
        }
    }
    return nullptr;
}

Value * KernelBuilder::getProcessedItemCount(const std::string & name) const {
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

Value * KernelBuilder::getConsumedItemCount(const std::string & name) const {
    return getScalarField(name + CONSUMED_ITEM_COUNT_SUFFIX);
}

void KernelBuilder::setProducedItemCount(const std::string & name, Value * value) const {
    setScalarField(name + PRODUCED_ITEM_COUNT_SUFFIX, value);
}

void KernelBuilder::setProcessedItemCount(const std::string & name, Value * value) const {
    setScalarField(name + PROCESSED_ITEM_COUNT_SUFFIX, value);
}

void KernelBuilder::setConsumedItemCount(const std::string & name, Value * value) const {
    setScalarField(name + CONSUMED_ITEM_COUNT_SUFFIX, value);
}

Value * KernelBuilder::getTerminationSignal() const {
    return getScalarField(TERMINATION_SIGNAL);
}

void KernelBuilder::setTerminationSignal() const {
    setScalarField(TERMINATION_SIGNAL, iBuilder->getTrue());
}

LoadInst * KernelBuilder::acquireLogicalSegmentNo() const {
    return iBuilder->CreateAtomicLoadAcquire(getScalarFieldPtr(LOGICAL_SEGMENT_NO_SCALAR));
}

void KernelBuilder::releaseLogicalSegmentNo(Value * nextSegNo) const {
    iBuilder->CreateAtomicStoreRelease(nextSegNo, getScalarFieldPtr(LOGICAL_SEGMENT_NO_SCALAR));
}

llvm::Value * KernelBuilder::getLinearlyAccessibleItems(const std::string & name, llvm::Value * fromPosition) const {
    llvm::Value * instance = getStreamSetBufferPtr(name);
    const StreamSetBuffer * const buf = getInputStreamSetBuffer(name);
    return buf->getLinearlyAccessibleItems(iBuilder, instance, fromPosition);
}

llvm::Value * KernelBuilder::getConsumerLock(const std::string & name) const {
    return getScalarField(name + CONSUMER_SUFFIX);
}

void KernelBuilder::setConsumerLock(const std::string & name, llvm::Value * value) const {
    setScalarField(name + CONSUMER_SUFFIX, value);
}

inline Value * KernelBuilder::computeBlockIndex(const std::vector<Binding> & bindings, const std::string & name, Value * itemCount) const {
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

Value * KernelBuilder::getInputStreamBlockPtr(const std::string & name, Value * streamIndex) const {
    Value * const blockIndex = computeBlockIndex(mStreamSetInputs, name, getProcessedItemCount(name));
    const StreamSetBuffer * const buf = getInputStreamSetBuffer(name);
    return buf->getStreamBlockPtr(iBuilder, getStreamSetBufferPtr(name), streamIndex, blockIndex, true);
}

Value * KernelBuilder::loadInputStreamBlock(const std::string & name, Value * streamIndex) const {
    return iBuilder->CreateBlockAlignedLoad(getInputStreamBlockPtr(name, streamIndex));
}

Value * KernelBuilder::getInputStreamPackPtr(const std::string & name, Value * streamIndex, Value * packIndex) const {
    Value * const blockIndex = computeBlockIndex(mStreamSetInputs, name, getProcessedItemCount(name));
    const StreamSetBuffer * const buf = getInputStreamSetBuffer(name);
    return buf->getStreamPackPtr(iBuilder, getStreamSetBufferPtr(name), streamIndex, blockIndex, packIndex, true);
}

Value * KernelBuilder::loadInputStreamPack(const std::string & name, Value * streamIndex, Value * packIndex) const {
    return iBuilder->CreateBlockAlignedLoad(getInputStreamPackPtr(name, streamIndex, packIndex));
}

llvm::Value * KernelBuilder::getInputStreamSetCount(const std::string & name) const {
    return getInputStreamSetBuffer(name)->getStreamSetCount(iBuilder, getStreamSetBufferPtr(name));
}

llvm::Value * KernelBuilder::getAdjustedInputStreamBlockPtr(Value * blockAdjustment, const std::string & name, llvm::Value * streamIndex) const {
    Value * blockIndex = computeBlockIndex(mStreamSetInputs, name, getProcessedItemCount(name));
    blockIndex = iBuilder->CreateAdd(blockIndex, blockAdjustment);
    const StreamSetBuffer * const buf = getInputStreamSetBuffer(name);
    return buf->getStreamBlockPtr(iBuilder, getStreamSetBufferPtr(name), streamIndex, blockIndex, true);
}

Value * KernelBuilder::getOutputStreamBlockPtr(const std::string & name, Value * streamIndex) const {
    Value * const blockIndex = computeBlockIndex(mStreamSetOutputs, name, getProducedItemCount(name));
    const StreamSetBuffer * const buf = getOutputStreamSetBuffer(name);
    return buf->getStreamBlockPtr(iBuilder, getStreamSetBufferPtr(name), streamIndex, blockIndex, false);
}

void KernelBuilder::storeOutputStreamBlock(const std::string & name, Value * streamIndex, Value * toStore) const {
    return iBuilder->CreateBlockAlignedStore(toStore, getOutputStreamBlockPtr(name, streamIndex));
}

Value * KernelBuilder::getOutputStreamPackPtr(const std::string & name, Value * streamIndex, Value * packIndex) const {
    Value * const blockIndex = computeBlockIndex(mStreamSetOutputs, name, getProducedItemCount(name));
    const StreamSetBuffer * const buf = getOutputStreamSetBuffer(name);
    return buf->getStreamPackPtr(iBuilder, getStreamSetBufferPtr(name), streamIndex, blockIndex, packIndex, false);
}

void KernelBuilder::storeOutputStreamPack(const std::string & name, Value * streamIndex, Value * packIndex, Value * toStore) const {
    return iBuilder->CreateBlockAlignedStore(toStore, getOutputStreamPackPtr(name, streamIndex, packIndex));
}

llvm::Value * KernelBuilder::getOutputStreamSetCount(const std::string & name) const {
    return getOutputStreamSetBuffer(name)->getStreamSetCount(iBuilder, getStreamSetBufferPtr(name));
}

Value * KernelBuilder::getRawInputPointer(const std::string & name, Value * streamIndex, Value * absolutePosition) const {
    return getInputStreamSetBuffer(name)->getRawItemPointer(iBuilder, getStreamSetBufferPtr(name), streamIndex, absolutePosition);
}

Value * KernelBuilder::getRawOutputPointer(const std::string & name, Value * streamIndex, Value * absolutePosition) const {
    return getOutputStreamSetBuffer(name)->getRawItemPointer(iBuilder, getStreamSetBufferPtr(name), streamIndex, absolutePosition);
}

Value * KernelBuilder::getBaseAddress(const std::string & name) const {
    return getAnyStreamSetBuffer(name)->getBaseAddress(iBuilder, getStreamSetBufferPtr(name));
}

void KernelBuilder::setBaseAddress(const std::string & name, Value * const addr) const {
    return getAnyStreamSetBuffer(name)->setBaseAddress(iBuilder, getStreamSetBufferPtr(name), addr);
}

Value * KernelBuilder::getBufferedSize(const std::string & name) const {
    return getAnyStreamSetBuffer(name)->getBufferedSize(iBuilder, getStreamSetBufferPtr(name));
}

void KernelBuilder::setBufferedSize(const std::string & name, Value * size) const {
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

void KernelBuilder::reserveBytes(const std::string & name, llvm::Value * value) const {
    Value * itemCount = getProducedItemCount(name);
    const StreamSetBuffer * const buf = getOutputStreamSetBuffer(name);
    buf->reserveBytes(iBuilder, getStreamSetBufferPtr(name), iBuilder->CreateAdd(itemCount, value));
}

BasicBlock * KernelBuilder::CreateWaitForConsumers() const {

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

Value * KernelBuilder::getStreamSetBufferPtr(const std::string & name) const {
    return getScalarField(name + BUFFER_PTR_SUFFIX);
}

Argument * KernelBuilder::getParameter(Function * const f, const std::string & name) const {
    for (auto & arg : f->getArgumentList()) {
        if (arg.getName().equals(name)) {
            return &arg;
        }
    }
    report_fatal_error(getName() + " does not have parameter " + name);
}

CallInst * KernelBuilder::createDoSegmentCall(const std::vector<Value *> & args) const {
    Function * const doSegment = getDoSegmentFunction(iBuilder->getModule());
    assert (doSegment->getArgumentList().size() == args.size());
    return iBuilder->CreateCall(doSegment, args);
}

Value * KernelBuilder::getAccumulator(const std::string & accumName) const {
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

BasicBlock * KernelBuilder::CreateBasicBlock(std::string && name) const {
    return BasicBlock::Create(iBuilder->getContext(), name, mCurrentMethod);
}

Value * KernelBuilder::createInstance() {
    if (LLVM_UNLIKELY(mKernelStateType == nullptr)) {
        report_fatal_error("Cannot instantiate " + getName() + " before calling prepareKernel()");
    }
    setInstance(iBuilder->CreateCacheAlignedAlloca(mKernelStateType));
    return getInstance();
}

void KernelBuilder::initializeInstance() {

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
            KernelBuilder * const consumer = consumers[i];
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
            Value * accessibleBlocks = cb->getLinearlyAccessibleBlocks(iBuilder, instance, priorBlock);
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
            Value * accessible = cb->getLinearlyAccessibleItems(iBuilder, instance, priorProduced[priorIdx]);
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

void KernelBuilder::finalizeInstance() {
    mOutputScalarResult = iBuilder->CreateCall(getTerminateFunction(iBuilder->getModule()), { getInstance() });
}

KernelBuilder::StreamPort KernelBuilder::getStreamPort(const std::string & name) const {
    const auto f = mStreamMap.find(name);
    if (LLVM_UNLIKELY(f == mStreamMap.end())) {
        report_fatal_error(getName() + " does not contain stream set " + name);
    }
    return f->second;
}

// CONSTRUCTOR
KernelBuilder::KernelBuilder(IDISA::IDISA_Builder * builder,
                             std::string && kernelName,
                             std::vector<Binding> && stream_inputs,
                             std::vector<Binding> && stream_outputs,
                             std::vector<Binding> && scalar_parameters,
                             std::vector<Binding> && scalar_outputs,
                             std::vector<Binding> && internal_scalars)
: KernelInterface(builder, std::move(kernelName), std::move(stream_inputs), std::move(stream_outputs), std::move(scalar_parameters), std::move(scalar_outputs), std::move(internal_scalars))
, mModule(nullptr)
, mCurrentMethod(nullptr)
, mNoTerminateAttribute(false)
, mIsGenerated(false)
, mIsFinal(nullptr)
, mOutputScalarResult(nullptr) {

}

KernelBuilder::~KernelBuilder() {

}

// CONSTRUCTOR
BlockOrientedKernel::BlockOrientedKernel(IDISA::IDISA_Builder * builder,
                                         std::string && kernelName,
                                         std::vector<Binding> && stream_inputs,
                                         std::vector<Binding> && stream_outputs,
                                         std::vector<Binding> && scalar_parameters,
                                         std::vector<Binding> && scalar_outputs,
                                         std::vector<Binding> && internal_scalars)
: KernelBuilder(builder, std::move(kernelName), std::move(stream_inputs), std::move(stream_outputs), std::move(scalar_parameters), std::move(scalar_outputs), std::move(internal_scalars))
, mDoBlockMethod(nullptr)
, mStrideLoopBody(nullptr)
, mStrideLoopBranch(nullptr)
, mStrideLoopTarget(nullptr) {

}

// CONSTRUCTOR
SegmentOrientedKernel::SegmentOrientedKernel(IDISA::IDISA_Builder * builder,
                                             std::string && kernelName,
                                             std::vector<Binding> && stream_inputs,
                                             std::vector<Binding> && stream_outputs,
                                             std::vector<Binding> && scalar_parameters,
                                             std::vector<Binding> && scalar_outputs,
                                             std::vector<Binding> && internal_scalars)
: KernelBuilder(builder, std::move(kernelName), std::move(stream_inputs), std::move(stream_outputs), std::move(scalar_parameters), std::move(scalar_outputs), std::move(internal_scalars)) {

}

