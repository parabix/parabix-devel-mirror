/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "kernel.h"
#include <toolchain.h>
#include <kernels/streamset.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_ostream.h>

static const auto BLOCK_NO_SCALAR = "blockNo";

static const auto DO_BLOCK_SUFFIX = "_DoBlock";

static const auto FINAL_BLOCK_SUFFIX = "_FinalBlock";

static const auto LOGICAL_SEGMENT_NO_SCALAR = "logicalSegNo";

static const auto PROCESSED_ITEM_COUNT_SUFFIX = "_processedItemCount";

static const auto PRODUCED_ITEM_COUNT_SUFFIX = "_producedItemCount";

static const auto TERMINATION_SIGNAL = "terminationSignal";

static const auto BUFFER_PTR_SUFFIX = "_bufferPtr";

static const auto BLOCK_MASK_SUFFIX = "_blkMask";

using namespace llvm;
using namespace kernel;
using namespace parabix;

unsigned KernelBuilder::addScalar(Type * const type, const std::string & name) {
    if (LLVM_UNLIKELY(mKernelStateType != nullptr)) {
        report_fatal_error("Cannot add kernel field " + name + " after kernel state finalized");
    }
    if (LLVM_UNLIKELY(mKernelMap.count(name))) {
        report_fatal_error("Kernel already contains field " + name);
    }
    const auto index = mKernelFields.size();
    mKernelMap.emplace(name, index);
    mKernelFields.push_back(type);
    return index;
}

unsigned KernelBuilder::addUnnamedScalar(Type * const type) {
    if (LLVM_UNLIKELY(mKernelStateType != nullptr)) {
        report_fatal_error("Cannot add unnamed kernel field after kernel state finalized");
    }
    const auto index = mKernelFields.size();
    mKernelFields.push_back(type);
    return index;
}

void KernelBuilder::prepareKernelSignature() {
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        mStreamSetNameMap.emplace(mStreamSetInputs[i].name, i);
    }
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        mStreamSetNameMap.emplace(mStreamSetOutputs[i].name, i);
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
        addScalar(iBuilder->getSizeTy(), mStreamSetInputs[i].name + PROCESSED_ITEM_COUNT_SUFFIX);
    }
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        mScalarInputs.emplace_back(mStreamSetOutputBuffers[i]->getPointerType(), mStreamSetOutputs[i].name + BUFFER_PTR_SUFFIX);
        addScalar(iBuilder->getSizeTy(), mStreamSetOutputs[i].name + PRODUCED_ITEM_COUNT_SUFFIX);
    }
    for (const auto binding : mScalarInputs) {
        addScalar(binding.type, binding.name);
    }
    for (const auto binding : mScalarOutputs) {
        addScalar(binding.type, binding.name);
    }
    if (mStreamSetNameMap.empty()) {
        prepareKernelSignature();
    }
    for (auto binding : mInternalScalars) {
        addScalar(binding.type, binding.name);
    }
    addScalar(iBuilder->getSizeTy(), BLOCK_NO_SCALAR);
    addScalar(iBuilder->getSizeTy(), LOGICAL_SEGMENT_NO_SCALAR);
    addScalar(iBuilder->getInt1Ty(), TERMINATION_SIGNAL);
    mKernelStateType = StructType::create(iBuilder->getContext(), mKernelFields, getName());
}

std::unique_ptr<Module> KernelBuilder::createKernelModule(const std::vector<StreamSetBuffer *> & inputs, const std::vector<StreamSetBuffer *> & outputs) {
    auto saveModule = iBuilder->getModule();
    auto savePoint = iBuilder->saveIP();
    auto module = make_unique<Module>(getName() + "_" + iBuilder->getBitBlockTypeName(), iBuilder->getContext());
    iBuilder->setModule(module.get());
    generateKernel(inputs, outputs);
    iBuilder->setModule(saveModule);
    iBuilder->restoreIP(savePoint);
    return module;
}

void KernelBuilder::generateKernel(const std::vector<StreamSetBuffer *> & inputs, const std::vector<StreamSetBuffer *> & outputs) {
    auto savePoint = iBuilder->saveIP();
    Module * const m = iBuilder->getModule();
    mStreamSetInputBuffers.assign(inputs.begin(), inputs.end());
    mStreamSetOutputBuffers.assign(outputs.begin(), outputs.end());
    prepareKernel(); // possibly overridden by the KernelBuilder subtype
    addKernelDeclarations(m);
    callGenerateInitMethod();
    generateInternalMethods();
    callGenerateDoSegmentMethod();
    // Implement the accumulator get functions
    for (auto binding : mScalarOutputs) {
        Function * f = getAccumulatorFunction(binding.name);
        iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "get_" + binding.name, f));
        Value * self = &*(f->arg_begin());
        Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), getScalarIndex(binding.name)});
        Value * retVal = iBuilder->CreateLoad(ptr);
        iBuilder->CreateRet(retVal);
    }
    iBuilder->restoreIP(savePoint);
}

void KernelBuilder::callGenerateDoSegmentMethod() {
    mCurrentFunction = getDoSegmentFunction();
    iBuilder->SetInsertPoint(CreateBasicBlock(getName() + "_entry"));
    auto args = mCurrentFunction->arg_begin();
    mSelf = &*(args++);
    Value * doFinal = &*(args++);
    std::vector<Value *> producerPos;
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        producerPos.push_back(&*(args++));
    }
    assert (args == mCurrentFunction->arg_end());
    generateDoSegmentMethod(doFinal, producerPos); // must be overridden by the KernelBuilder subtype
    iBuilder->CreateRetVoid();
}

void KernelBuilder::callGenerateInitMethod() {
    mCurrentFunction = getInitFunction();
    iBuilder->SetInsertPoint(CreateBasicBlock("Init_entry"));
    Function::arg_iterator args = mCurrentFunction->arg_begin();
    mSelf = &*(args++);
    iBuilder->CreateStore(ConstantAggregateZero::get(mKernelStateType), mSelf);
    for (auto binding : mScalarInputs) {
        Value * param = &*(args++);
        Value * ptr = iBuilder->CreateGEP(mSelf, {iBuilder->getInt32(0), getScalarIndex(binding.name)});
        iBuilder->CreateStore(param, ptr);
    }
    generateInitMethod();
    iBuilder->CreateRetVoid();
}

ConstantInt * KernelBuilder::getScalarIndex(const std::string & name) const {
    const auto f = mKernelMap.find(name);
    if (LLVM_UNLIKELY(f == mKernelMap.end())) {
        report_fatal_error("Kernel does not contain scalar: " + name);
    }
    return iBuilder->getInt32(f->second);
}

unsigned KernelBuilder::getScalarCount() const {
    return mKernelFields.size();
}

Value * KernelBuilder::getScalarFieldPtr(Value * instance, Value * index) const {
    return iBuilder->CreateGEP(instance, {iBuilder->getInt32(0), index});
}

Value * KernelBuilder::getScalarFieldPtr(Value * instance, const std::string & fieldName) const {
    return getScalarFieldPtr(instance, getScalarIndex(fieldName));
}

Value * KernelBuilder::getScalarField(Value * instance, const std::string & fieldName) const {
    return iBuilder->CreateLoad(getScalarFieldPtr(instance, fieldName));
}

Value * KernelBuilder::getScalarField(Value * instance, Value * index) const {
    return iBuilder->CreateLoad(getScalarFieldPtr(instance, index));
}

void KernelBuilder::setScalarField(Value * instance, const std::string & fieldName, Value * value) const {
    iBuilder->CreateStore(value, getScalarFieldPtr(instance, fieldName));
}

void KernelBuilder::setScalarField(Value * instance, Value * index, Value * value) const {
    iBuilder->CreateStore(value, getScalarFieldPtr(instance, index));
}

Value * KernelBuilder::getProcessedItemCount(Value * instance, const std::string & name) const {
    return getScalarField(instance, name + PROCESSED_ITEM_COUNT_SUFFIX);
}

Value * KernelBuilder::getProducedItemCount(Value * instance, const std::string & name) const {
    return getScalarField(instance, name + PRODUCED_ITEM_COUNT_SUFFIX);
}

void KernelBuilder::setProcessedItemCount(Value * instance, const std::string & name, Value * value) const {
    setScalarField(instance, name + PROCESSED_ITEM_COUNT_SUFFIX, value);
}

void KernelBuilder::setProducedItemCount(Value * instance, const std::string & name, Value * value) const {
    setScalarField(instance, name + PRODUCED_ITEM_COUNT_SUFFIX, value);
}

Value * KernelBuilder::getTerminationSignal(Value * instance) const {
    return getScalarField(instance, TERMINATION_SIGNAL);
}

void KernelBuilder::setTerminationSignal(Value * instance) const {
    setScalarField(instance, TERMINATION_SIGNAL, iBuilder->getInt1(true));
}

LoadInst * KernelBuilder::acquireLogicalSegmentNo(Value * instance) const {
    return iBuilder->CreateAtomicLoadAcquire(getScalarFieldPtr(instance, LOGICAL_SEGMENT_NO_SCALAR));
}

void KernelBuilder::releaseLogicalSegmentNo(Value * instance, Value * newCount) const {
    iBuilder->CreateAtomicStoreRelease(newCount, getScalarFieldPtr(instance, LOGICAL_SEGMENT_NO_SCALAR));
}

Value * KernelBuilder::getBlockNo() const {
    return getScalarField(mSelf, BLOCK_NO_SCALAR);
}

void KernelBuilder::setBlockNo(Value * value) const {
    setScalarField(mSelf, BLOCK_NO_SCALAR, value);
}

inline Value * KernelBuilder::computeBlockIndex(const std::vector<Binding> & bindings, const std::string & name, Value * itemCount) const {
    for (const Binding & b : bindings) {
        if (b.name == name) {
            const auto divisor = (b.step == 0) ? iBuilder->getBitBlockWidth() : b.step;
            if (LLVM_LIKELY((divisor & (divisor - 1)) == 0)) {
                return iBuilder->CreateLShr(itemCount, std::log2(divisor));
            } else {
                return iBuilder->CreateUDiv(itemCount, iBuilder->getSize(divisor));
            }
        }
    }
    report_fatal_error("Error: no binding in " + getName() + " for " + name);
}

Value * KernelBuilder::getInputStream(const std::string & name, Value * streamIndex) const {
    Value * const blockIndex = computeBlockIndex(mStreamSetInputs, name, getProcessedItemCount(name));
    const StreamSetBuffer * const buf = getInputStreamSetBuffer(name);
    return buf->getStream(getStreamSetBufferPtr(name), streamIndex, blockIndex);
}

Value * KernelBuilder::getInputStream(const std::string & name, Value * streamIndex, Value * packIndex) const {
    Value * const blockIndex = computeBlockIndex(mStreamSetInputs, name, getProcessedItemCount(name));
    const StreamSetBuffer * const buf = getInputStreamSetBuffer(name);
    return buf->getStream(getStreamSetBufferPtr(name), streamIndex, blockIndex, packIndex);
}

Value * KernelBuilder::getOutputStream(const std::string & name, Value * streamIndex) const {
    Value * const blockIndex = computeBlockIndex(mStreamSetOutputs, name, getProducedItemCount(name));
    const StreamSetBuffer * const buf = getOutputStreamSetBuffer(name);
    return buf->getStream(getStreamSetBufferPtr(name), streamIndex, blockIndex);
}

Value * KernelBuilder::getOutputStream(const std::string & name, Value * streamIndex, Value * packIndex) const {
    Value * const blockIndex = computeBlockIndex(mStreamSetOutputs, name, getProducedItemCount(name));
    const StreamSetBuffer * const buf = getOutputStreamSetBuffer(name);
    return buf->getStream(getStreamSetBufferPtr(name), streamIndex, blockIndex, packIndex);
}

Value * KernelBuilder::getRawInputPointer(const std::string & name, Value * streamIndex, Value * absolutePosition) const {
    return getInputStreamSetBuffer(name)->getRawItemPointer(getStreamSetBufferPtr(name), streamIndex, absolutePosition);
}

Value * KernelBuilder::getRawOutputPointer(const std::string & name, Value * streamIndex, Value * absolutePosition) const {
    return getOutputStreamSetBuffer(name)->getRawItemPointer(getStreamSetBufferPtr(name), streamIndex, absolutePosition);
}

unsigned KernelBuilder::getStreamSetIndex(const std::string & name) const {
    const auto f = mStreamSetNameMap.find(name);
    if (LLVM_UNLIKELY(f == mStreamSetNameMap.end())) {
        throw std::runtime_error("Kernel " + getName() + " does not contain stream set: " + name);
    }
    return f->second;
}

Value * KernelBuilder::getStreamSetBufferPtr(const std::string & name) const {
    return getScalarField(getSelf(), name + BUFFER_PTR_SUFFIX);
}

Argument * KernelBuilder::getParameter(Function * const f, const std::string & name) const {
    for (auto & arg : f->getArgumentList()) {
        if (arg.getName().equals(name)) {
            return &arg;
        }
    }
    report_fatal_error(f->getName() + " does not have parameter " + name);
}

Value * KernelBuilder::createDoSegmentCall(const std::vector<Value *> & args) const {
    return iBuilder->CreateCall(getDoSegmentFunction(), args);
}

Value * KernelBuilder::createGetAccumulatorCall(Value * self, const std::string & accumName) const {
    return iBuilder->CreateCall(getAccumulatorFunction(accumName), {self});
}

Value * KernelBuilder::getInputStreamSetPtr(const std::string & name, Value * blockNo) const {
    return getInputStreamSetBuffer(name)->getStreamSetPtr(getStreamSetBufferPtr(name), blockNo);
}

BasicBlock * KernelBuilder::CreateBasicBlock(std::string && name) const {
    return BasicBlock::Create(iBuilder->getContext(), name, mCurrentFunction);
}

void KernelBuilder::createInstance() {
    if (LLVM_UNLIKELY(mKernelStateType == nullptr)) {
        report_fatal_error("Cannot create kernel instance before calling prepareKernel()");
    }
    mKernelInstance = iBuilder->CreateCacheAlignedAlloca(mKernelStateType);
    std::vector<Value *> init_args = {mKernelInstance};
    for (auto a : mInitialArguments) {
        init_args.push_back(a);
    }
    for (auto b : mStreamSetInputBuffers) {
        init_args.push_back(b->getStreamSetBasePtr());
    }
    for (auto b : mStreamSetOutputBuffers) {
        init_args.push_back(b->getStreamSetBasePtr());
    }
    Function * initMethod = getInitFunction();
    iBuilder->CreateCall(initMethod, init_args);
}

//  The default finalBlock method simply dispatches to the doBlock routine.
void BlockOrientedKernel::generateFinalBlockMethod(Value * remainingBytes) {
//    std::vector<Value *> args = {self};
//    for (Argument & arg : function->getArgumentList()){
//        args.push_back(&arg);
//    }
    CreateDoBlockMethodCall();
}

//Value * BlockOrientedKernel::loadBlock(const std::string & inputName, Value * const streamIndex) const {

//}

//Value * BlockOrientedKernel::loadPack(const std::string & inputName, Value * const streamIndex, Value * const packIndex) const {

//}


//  The default doSegment method dispatches to the doBlock routine for
//  each block of the given number of blocksToDo, and then updates counts.
void BlockOrientedKernel::generateDoSegmentMethod(Value * doFinal, const std::vector<Value *> & producerPos) {

    BasicBlock * const entryBlock = iBuilder->GetInsertBlock();
    BasicBlock * const strideLoopCond = CreateBasicBlock(getName() + "_strideLoopCond");
    BasicBlock * const strideLoopBody = CreateBasicBlock(getName() + "_strideLoopBody");
    BasicBlock * const stridesDone = CreateBasicBlock(getName() + "_stridesDone");
    BasicBlock * const doFinalBlock = CreateBasicBlock(getName() + "_doFinalBlock");
    BasicBlock * const segmentDone = CreateBasicBlock(getName() + "_segmentDone");

    ConstantInt * stride = iBuilder->getSize(iBuilder->getStride());
    ConstantInt * strideBlocks = iBuilder->getSize(iBuilder->getStride() / iBuilder->getBitBlockWidth());

    Value * availablePos = producerPos[0];
    for (unsigned i = 1; i < mStreamSetInputs.size(); i++) {
        Value * p = producerPos[i];
        availablePos = iBuilder->CreateSelect(iBuilder->CreateICmpULT(availablePos, p), availablePos, p);
    }

    Value * processed = getProcessedItemCount(mStreamSetInputs[0].name);
    Value * itemsAvail = iBuilder->CreateSub(availablePos, processed);
    Value * stridesToDo = iBuilder->CreateUDiv(itemsAvail, stride);
    iBuilder->CreateBr(strideLoopCond);

    iBuilder->SetInsertPoint(strideLoopCond);
    PHINode * stridesRemaining = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2, "stridesRemaining");
    stridesRemaining->addIncoming(stridesToDo, entryBlock);
    Value * notDone = iBuilder->CreateICmpNE(stridesRemaining, iBuilder->getSize(0));
    iBuilder->CreateCondBr(notDone, strideLoopBody, stridesDone);

    iBuilder->SetInsertPoint(strideLoopBody);
    Value * blockNo = getBlockNo();

    CreateDoBlockMethodCall();

    setBlockNo(iBuilder->CreateAdd(blockNo, strideBlocks));

    // Update counts

    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        Value * processed = getProcessedItemCount(mStreamSetInputs[i].name);
        processed = iBuilder->CreateAdd(processed, stride);
        setProcessedItemCount(mStreamSetInputs[i].name, processed);
    }

    if (!mDoBlockUpdatesProducedItemCountsAttribute) {
        for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
            Value * produced = getProducedItemCount(mStreamSetOutputs[i].name);
            produced = iBuilder->CreateAdd(produced, stride);
            setProducedItemCount(mStreamSetOutputs[i].name, produced);
        }
    }

    stridesRemaining->addIncoming(iBuilder->CreateSub(stridesRemaining, iBuilder->getSize(1)), strideLoopBody);
    iBuilder->CreateBr(strideLoopCond);

    iBuilder->SetInsertPoint(stridesDone);

    // Now conditionally perform the final block processing depending on the doFinal parameter.
    iBuilder->CreateCondBr(doFinal, doFinalBlock, segmentDone);
    iBuilder->SetInsertPoint(doFinalBlock);

    Value * remainingItems = iBuilder->CreateSub(producerPos[0], getProcessedItemCount(mStreamSetInputs[0].name));

    CreateDoFinalBlockMethodCall(remainingItems);

    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        Value * preProcessed = getProcessedItemCount(mStreamSetInputs[i].name);
        setProcessedItemCount(mStreamSetInputs[i].name, iBuilder->CreateAdd(preProcessed, remainingItems));
    }
    if (!mDoBlockUpdatesProducedItemCountsAttribute) {
        for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
            Value * preProduced = getProducedItemCount(mStreamSetOutputs[i].name);
            setProducedItemCount(mStreamSetOutputs[i].name, iBuilder->CreateAdd(preProduced, remainingItems));
        }
    }
    setTerminationSignal();
    iBuilder->CreateBr(segmentDone);

    iBuilder->SetInsertPoint(segmentDone);

}

void BlockOrientedKernel::generateInternalMethods() {

    callGenerateDoBlockMethod();

    callGenerateDoFinalBlockMethod();
}

void BlockOrientedKernel::callGenerateDoBlockMethod() {
    mCurrentFunction = getDoBlockFunction();
    auto args = mCurrentFunction->arg_begin();
    mSelf = &(*args);
    iBuilder->SetInsertPoint(CreateBasicBlock("entry"));
    generateDoBlockMethod(); // must be implemented by the KernelBuilder subtype
    iBuilder->CreateRetVoid();
}


void BlockOrientedKernel::callGenerateDoFinalBlockMethod() {
    mCurrentFunction = getDoFinalBlockFunction();
    auto args = mCurrentFunction->arg_begin();
    mSelf = &(*args++);
    Value * const remainingBytes = &(*args);
    iBuilder->SetInsertPoint(CreateBasicBlock("entry"));
    generateFinalBlockMethod(remainingBytes); // possibly overridden by the KernelBuilder subtype
    iBuilder->CreateRetVoid();
}

Function * BlockOrientedKernel::getDoBlockFunction() const {
    const auto name = getName() + DO_BLOCK_SUFFIX;
    Function * const f = iBuilder->getModule()->getFunction(name);
    if (LLVM_UNLIKELY(f == nullptr)) {
        report_fatal_error("Cannot find " + name);
    }
    return f;
}

CallInst * BlockOrientedKernel::CreateDoBlockMethodCall() const {
    return iBuilder->CreateCall(getDoBlockFunction(), mSelf);
}

Function * BlockOrientedKernel::getDoFinalBlockFunction() const {
    const auto name = getName() + FINAL_BLOCK_SUFFIX;
    Function * const f = iBuilder->getModule()->getFunction(name);
    if (LLVM_UNLIKELY(f == nullptr)) {
        report_fatal_error("Cannot find " + name);
    }
    return f;
}

CallInst * BlockOrientedKernel::CreateDoFinalBlockMethodCall(Value * remainingItems) const {
    return iBuilder->CreateCall(getDoFinalBlockFunction(), {mSelf, remainingItems});
}

void BlockOrientedKernel::addAdditionalKernelDeclarations(Module * m, PointerType * selfType) {
    // Create the doBlock and finalBlock function prototypes
    FunctionType * const doBlockType = FunctionType::get(iBuilder->getVoidTy(), {selfType}, false);
    Function * const doBlock = Function::Create(doBlockType, GlobalValue::ExternalLinkage, getName() + DO_BLOCK_SUFFIX, m);
    doBlock->setCallingConv(CallingConv::C);
    doBlock->setDoesNotThrow();
    doBlock->setDoesNotCapture(1);
    auto args = doBlock->arg_begin();
    args->setName("self");
    assert ((++args) == doBlock->arg_end());

    FunctionType * const finalBlockType = FunctionType::get(iBuilder->getVoidTy(), {selfType, iBuilder->getSizeTy()}, false);
    Function * const finalBlock = Function::Create(finalBlockType, GlobalValue::ExternalLinkage, getName() + FINAL_BLOCK_SUFFIX, m);
    finalBlock->setCallingConv(CallingConv::C);
    finalBlock->setDoesNotThrow();
    finalBlock->setDoesNotCapture(1);
    args = finalBlock->arg_begin();
    args->setName("self");
    (++args)->setName("remainingBytes");
    assert ((++args) == finalBlock->arg_end());
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
, mNoTerminateAttribute(false)
, mDoBlockUpdatesProducedItemCountsAttribute(false) {

}

KernelBuilder::~KernelBuilder() { }

// CONSTRUCTOR
BlockOrientedKernel::BlockOrientedKernel(IDISA::IDISA_Builder * builder,
                                         std::string && kernelName,
                                         std::vector<Binding> && stream_inputs,
                                         std::vector<Binding> && stream_outputs,
                                         std::vector<Binding> && scalar_parameters,
                                         std::vector<Binding> && scalar_outputs,
                                         std::vector<Binding> && internal_scalars)
: KernelBuilder(builder, std::move(kernelName), std::move(stream_inputs), std::move(stream_outputs), std::move(scalar_parameters), std::move(scalar_outputs), std::move(internal_scalars)) {

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
