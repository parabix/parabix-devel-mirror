/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "kernel.h"
#include <llvm/IR/Value.h>               // for Value
#include <llvm/Support/ErrorHandling.h>  // for report_fatal_error
#include <toolchain.h>                   // for BufferSegments, SegmentSize
#include "IR_Gen/idisa_builder.h"        // for IDISA_Builder
#include "kernels/streamset.h"           // for StreamSetBuffer
#include "llvm/ADT/StringRef.h"          // for StringRef, operator==
#include "llvm/IR/CallingConv.h"         // for ::C
#include "llvm/IR/Constant.h"            // for Constant
#include "llvm/IR/Constants.h"           // for ConstantInt
#include "llvm/IR/Function.h"            // for Function, Function::arg_iter...
#include "llvm/IR/Instructions.h"        // for LoadInst (ptr only), PHINode
#include "llvm/Support/Compiler.h"       // for LLVM_UNLIKELY
namespace llvm { class BasicBlock; }
namespace llvm { class Module; }
namespace llvm { class Type; }

using namespace llvm;
using namespace kernel;
using namespace parabix;

KernelBuilder::KernelBuilder(IDISA::IDISA_Builder * builder,
                             std::string kernelName,
                             std::vector<Binding> stream_inputs,
                             std::vector<Binding> stream_outputs,
                             std::vector<Binding> scalar_parameters,
                             std::vector<Binding> scalar_outputs,
                             std::vector<Binding> internal_scalars)
: KernelInterface(builder, kernelName, stream_inputs, stream_outputs, scalar_parameters, scalar_outputs, internal_scalars),
mNoTerminateAttribute(false),
mDoBlockUpdatesProducedItemCountsAttribute(false) {

}

unsigned KernelBuilder::addScalar(Type * type, const std::string & name) {
    if (LLVM_UNLIKELY(mKernelStateType != nullptr)) {
        llvm::report_fatal_error("Cannot add kernel field " + name + " after kernel state finalized");
    }
    const auto index = mKernelFields.size();
    mKernelMap.emplace(name, index);
    mKernelFields.push_back(type);
    return index;
}

void KernelBuilder::prepareKernel() {
    if (LLVM_UNLIKELY(mKernelStateType != nullptr)) {
        llvm::report_fatal_error("Cannot prepare kernel after kernel state finalized");
    }
    unsigned blockSize = iBuilder->getBitBlockWidth();
    if (mStreamSetInputs.size() != mStreamSetInputBuffers.size()) {
        std::string tmp;
        raw_string_ostream out(tmp);
        out << "kernel contains " << mStreamSetInputBuffers.size() << " input buffers for "
            << mStreamSetInputs.size() << " input stream sets.";
        throw std::runtime_error(out.str());
    }
    if (mStreamSetOutputs.size() != mStreamSetOutputBuffers.size()) {
        std::string tmp;
        raw_string_ostream out(tmp);
        out << "kernel contains " << mStreamSetOutputBuffers.size() << " output buffers for "
            << mStreamSetOutputs.size() << " output stream sets.";
        throw std::runtime_error(out.str());
    }
    int streamSetNo = 0;
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        if ((mStreamSetInputBuffers[i]->getBufferSize() > 0) && (mStreamSetInputBuffers[i]->getBufferSize() < codegen::SegmentSize + (blockSize + mLookAheadPositions - 1)/blockSize)) {
             llvm::report_fatal_error("Kernel preparation: Buffer size too small " + mStreamSetInputs[i].name);
        }
        mScalarInputs.push_back(Binding{mStreamSetInputBuffers[i]->getStreamSetStructPointerType(), mStreamSetInputs[i].name + structPtrSuffix});
        mStreamSetNameMap.emplace(mStreamSetInputs[i].name, streamSetNo);
        addScalar(iBuilder->getSizeTy(), mStreamSetInputs[i].name + processedItemCountSuffix);
        streamSetNo++;
    }
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        mScalarInputs.push_back(Binding{mStreamSetOutputBuffers[i]->getStreamSetStructPointerType(), mStreamSetOutputs[i].name + structPtrSuffix});
        mStreamSetNameMap.emplace(mStreamSetOutputs[i].name, streamSetNo);
        addScalar(iBuilder->getSizeTy(), mStreamSetOutputs[i].name + producedItemCountSuffix);
        streamSetNo++;
    }
    for (auto binding : mScalarInputs) {
        addScalar(binding.type, binding.name);
    }
    for (auto binding : mScalarOutputs) {
        addScalar(binding.type, binding.name);
    }
    for (auto binding : mInternalScalars) {
        addScalar(binding.type, binding.name);
    }
    addScalar(iBuilder->getSizeTy(), blockNoScalar);
    addScalar(iBuilder->getSizeTy(), logicalSegmentNoScalar);
    addScalar(iBuilder->getInt1Ty(), terminationSignal);
    mKernelStateType = StructType::create(iBuilder->getContext(), mKernelFields, mKernelName);
}

std::unique_ptr<Module> KernelBuilder::createKernelModule(const std::vector<StreamSetBuffer *> & inputs, const std::vector<StreamSetBuffer *> & outputs) {
    auto saveModule = iBuilder->getModule();
    auto savePoint = iBuilder->saveIP();
    auto module = make_unique<Module>(mKernelName + "_" + iBuilder->getBitBlockTypeName(), iBuilder->getContext());
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
    prepareKernel();            // possibly overridden by the KernelBuilder subtype
    addKernelDeclarations(m);
    generateInitMethod();       // possibly overridden by the KernelBuilder subtype
    generateDoSegmentMethod();

    // Implement the accumulator get functions
    for (auto binding : mScalarOutputs) {
        auto fnName = mKernelName + accumulator_infix + binding.name;
        Function * accumFn = m->getFunction(fnName);
        iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "get_" + binding.name, accumFn, 0));
        Value * self = &*(accumFn->arg_begin());
        Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), getScalarIndex(binding.name)});
        Value * retVal = iBuilder->CreateLoad(ptr);
        iBuilder->CreateRet(retVal);
    }
    iBuilder->restoreIP(savePoint);
}

// Default init method, possibly overridden if special init actions required. 
void KernelBuilder::generateInitMethod() const {
    auto savePoint = iBuilder->saveIP();
    Module * const m = iBuilder->getModule();
    Function * initFunction = m->getFunction(mKernelName + init_suffix);
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "Init_entry", initFunction, 0));    
    Function::arg_iterator args = initFunction->arg_begin();
    Value * self = &*(args++);
    iBuilder->CreateStore(ConstantAggregateZero::get(mKernelStateType), self);
    for (auto binding : mScalarInputs) {
        Value * param = &*(args++);
        Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), getScalarIndex(binding.name)});
        iBuilder->CreateStore(param, ptr);
    }
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}

//  The default finalBlock method simply dispatches to the doBlock routine.
void KernelBuilder::generateFinalBlockMethod() const {
    auto savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Function * doBlockFunction = m->getFunction(mKernelName + doBlock_suffix);
    Function * finalBlockFunction = m->getFunction(mKernelName + finalBlock_suffix);
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "fb_entry", finalBlockFunction, 0));
    // Final Block arguments: self, remaining, then the standard DoBlock args.
    Function::arg_iterator args = finalBlockFunction->arg_begin();
    Value * self = &*(args++);
    /* Skip "remaining" arg */ args++;
    std::vector<Value *> doBlockArgs = {self};
    iBuilder->CreateCall(doBlockFunction, doBlockArgs);
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}

// Note: this may be overridden to incorporate doBlock logic directly into
// the doSegment function.
void KernelBuilder::generateDoBlockLogic(Value * self, Value * /* blockNo */) const {
    Function * doBlockFunction = iBuilder->getModule()->getFunction(mKernelName + doBlock_suffix);
    iBuilder->CreateCall(doBlockFunction, self);
}

// Note: this may be overridden to incorporate doBlock logic directly into
// the doSegment function.
void KernelBuilder::generateDoBlockMethod() const {
    llvm::report_fatal_error(mKernelName + " DoBlock method called but not implemented");
}


//  The default doSegment method dispatches to the doBlock routine for
//  each block of the given number of blocksToDo, and then updates counts.
void KernelBuilder::generateDoSegmentMethod() const {
    generateDoBlockMethod();    // must be implemented by the KernelBuilder subtype
    generateFinalBlockMethod(); // possibly overridden by the KernelBuilder subtype

  
    auto savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Function * doSegmentFunction = m->getFunction(mKernelName + doSegment_suffix);
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), mKernelName + "_entry", doSegmentFunction, 0));
    BasicBlock * entryBlock = iBuilder->GetInsertBlock();
    BasicBlock * strideLoopCond = BasicBlock::Create(iBuilder->getContext(), mKernelName + "_strideLoopCond", doSegmentFunction, 0);
    BasicBlock * strideLoopBody = BasicBlock::Create(iBuilder->getContext(), mKernelName + "_strideLoopBody", doSegmentFunction, 0);
    BasicBlock * stridesDone = BasicBlock::Create(iBuilder->getContext(), mKernelName + "_stridesDone", doSegmentFunction, 0);
    BasicBlock * doFinalBlock = BasicBlock::Create(iBuilder->getContext(), mKernelName + "_doFinalBlock", doSegmentFunction, 0);
    BasicBlock * segmentDone = BasicBlock::Create(iBuilder->getContext(), mKernelName + "_segmentDone", doSegmentFunction, 0);
    BasicBlock * finalExit = BasicBlock::Create(iBuilder->getContext(), mKernelName + "_finalExit", doSegmentFunction, 0);
    Type * const size_ty = iBuilder->getSizeTy();
    Constant * stride = ConstantInt::get(size_ty, iBuilder->getStride());
    Value * strideBlocks = ConstantInt::get(size_ty, iBuilder->getStride() / iBuilder->getBitBlockWidth());
    
    Function::arg_iterator args = doSegmentFunction->arg_begin();
    Value * self = &*(args++);
    Value * doFinal = &*(args++);
    
    std::vector<Value *> producerPos;
    producerPos.push_back(&*(args++));
    Value * availablePos = producerPos[0];
    for (unsigned i = 1; i < mStreamSetInputs.size(); i++) {
        Value * p = &*(args++);
        producerPos.push_back(p);
        availablePos = iBuilder->CreateSelect(iBuilder->CreateICmpULT(availablePos, p), availablePos, p);
    }
    Value * processed = getProcessedItemCount(self, mStreamSetInputs[0].name);
    Value * itemsAvail = iBuilder->CreateSub(availablePos, processed);
    Value * stridesToDo = iBuilder->CreateUDiv(itemsAvail, stride);
    iBuilder->CreateBr(strideLoopCond);

    iBuilder->SetInsertPoint(strideLoopCond);
    PHINode * stridesRemaining = iBuilder->CreatePHI(size_ty, 2, "stridesRemaining");
    stridesRemaining->addIncoming(stridesToDo, entryBlock);
    Value * notDone = iBuilder->CreateICmpUGT(stridesRemaining, ConstantInt::get(size_ty, 0));
    iBuilder->CreateCondBr(notDone, strideLoopBody, stridesDone);

    iBuilder->SetInsertPoint(strideLoopBody);
    Value * blockNo = getScalarField(self, blockNoScalar);   

    generateDoBlockLogic(self, blockNo);
    setBlockNo(self, iBuilder->CreateAdd(blockNo, strideBlocks));
    stridesRemaining->addIncoming(iBuilder->CreateSub(stridesRemaining, ConstantInt::get(size_ty, 1)), strideLoopBody);
    iBuilder->CreateBr(strideLoopCond);
    
    iBuilder->SetInsertPoint(stridesDone);
    // Update counts for the full strides processed.
    Value * segmentItemsProcessed = iBuilder->CreateMul(stridesToDo, stride);
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        Value * preProcessed = getProcessedItemCount(self, mStreamSetInputs[i].name);
        setProcessedItemCount(self, mStreamSetInputs[i].name, iBuilder->CreateAdd(preProcessed, segmentItemsProcessed));
    }
    if (!mDoBlockUpdatesProducedItemCountsAttribute) {
        for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
            Value * preProduced = getProducedItemCount(self, mStreamSetOutputs[i].name);
            setProducedItemCount(self, mStreamSetOutputs[i].name, iBuilder->CreateAdd(preProduced, segmentItemsProcessed));
        }
    }
    
    // Now conditionally perform the final block processing depending on the doFinal parameter.
    iBuilder->CreateCondBr(doFinal, doFinalBlock, segmentDone);
    iBuilder->SetInsertPoint(doFinalBlock);

    Value * remainingItems = iBuilder->CreateSub(producerPos[0], processed);
    //iBuilder->CallPrintInt(mKernelName + " remainingItems", remainingItems);
    
    createFinalBlockCall(self, remainingItems);
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        Value * preProcessed = getProcessedItemCount(self, mStreamSetInputs[i].name);
        setProcessedItemCount(self, mStreamSetInputs[i].name, iBuilder->CreateAdd(preProcessed, remainingItems));
    }
    if (!mDoBlockUpdatesProducedItemCountsAttribute) {
        for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
            Value * preProduced = getProducedItemCount(self, mStreamSetOutputs[i].name);
            setProducedItemCount(self, mStreamSetOutputs[i].name, iBuilder->CreateAdd(preProduced, remainingItems));
        }
    }
    iBuilder->CreateBr(segmentDone);
    
    iBuilder->SetInsertPoint(segmentDone);
//#ifndef NDEBUG
//    iBuilder->CallPrintInt(mKernelName + "_processed", processed);
//#endif
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        Value * produced = getProducedItemCount(self, mStreamSetOutputs[i].name);
        Value * ssStructPtr = getStreamSetStructPtr(self, mStreamSetOutputs[i].name);
        Value * producerPosPtr = mStreamSetOutputBuffers[i]->getProducerPosPtr(ssStructPtr);
        iBuilder->CreateAtomicStoreRelease(produced, producerPosPtr);
    }
    iBuilder->CreateBr(finalExit);
    iBuilder->SetInsertPoint(finalExit);

    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}


ConstantInt * KernelBuilder::getScalarIndex(const std::string & name) const {
    const auto f = mKernelMap.find(name);
    if (LLVM_UNLIKELY(f == mKernelMap.end())) {
        llvm::report_fatal_error("Kernel does not contain scalar: " + name);
    }
    return iBuilder->getInt32(f->second);
}

unsigned KernelBuilder::getScalarCount() const {
    return mKernelFields.size();
}

Value * KernelBuilder::getScalarFieldPtr(Value * self, const std::string & fieldName) const {
    return iBuilder->CreateGEP(self, {iBuilder->getInt32(0), getScalarIndex(fieldName)});
}

Value * KernelBuilder::getScalarField(Value * self, const std::string & fieldName) const {
    return iBuilder->CreateLoad(getScalarFieldPtr(self, fieldName));
}

void KernelBuilder::setScalarField(Value * self, const std::string & fieldName, Value * newFieldVal) const {
    iBuilder->CreateStore(newFieldVal, getScalarFieldPtr(self, fieldName));
}

LoadInst * KernelBuilder::acquireLogicalSegmentNo(Value * self) const {
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), getScalarIndex(logicalSegmentNoScalar)});
    return iBuilder->CreateAtomicLoadAcquire(ptr);
}

Value * KernelBuilder::getProcessedItemCount(Value * self, const std::string & ssName) const {
    return getScalarField(self, ssName + processedItemCountSuffix);
}

Value * KernelBuilder::getProducedItemCount(Value * self, const std::string & ssName) const {
    return getScalarField(self, ssName + producedItemCountSuffix);
}

Value * KernelBuilder::getTerminationSignal(Value * self) const {
    return getScalarField(self, terminationSignal);
}

void KernelBuilder::releaseLogicalSegmentNo(Value * self, Value * newCount) const {
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), getScalarIndex(logicalSegmentNoScalar)});
    iBuilder->CreateAtomicStoreRelease(newCount, ptr);
}

void KernelBuilder::setProcessedItemCount(Value * self, const std::string & ssName, Value * newCount) const {
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), getScalarIndex(ssName + processedItemCountSuffix)});
    iBuilder->CreateStore(newCount, ptr);
}

void KernelBuilder::setProducedItemCount(Value * self, const std::string & ssName, Value * newCount) const {
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), getScalarIndex(ssName + producedItemCountSuffix)});
    iBuilder->CreateStore(newCount, ptr);
}

void KernelBuilder::setTerminationSignal(Value * self) const {
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), getScalarIndex(terminationSignal)});
    iBuilder->CreateStore(ConstantInt::get(iBuilder->getInt1Ty(), 1), ptr);
}

Value * KernelBuilder::getBlockNo(Value * self) const {
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), getScalarIndex(blockNoScalar)});
    return iBuilder->CreateLoad(ptr);
}

void KernelBuilder::setBlockNo(Value * self, Value * value) const {
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), getScalarIndex(blockNoScalar)});
    iBuilder->CreateStore(value, ptr);
}


Value * KernelBuilder::getParameter(Function * f, const std::string & paramName) const {
    for (Function::arg_iterator argIter = f->arg_begin(), end = f->arg_end(); argIter != end; argIter++) {
        Value * arg = &*argIter;
        if (arg->getName() == paramName) return arg;
    }
    llvm::report_fatal_error("Method does not have parameter: " + paramName);
}

unsigned KernelBuilder::getStreamSetIndex(const std::string & name) const {
    const auto f = mStreamSetNameMap.find(name);
    if (LLVM_UNLIKELY(f == mStreamSetNameMap.end())) {
        llvm::report_fatal_error("Kernel does not contain stream set: " + name);
    }
    return f->second;
}

Value * KernelBuilder::getStreamSetStructPtr(Value * self, const std::string & name) const {
    return getScalarField(self, name + structPtrSuffix);
}

inline const StreamSetBuffer * KernelBuilder::getStreamSetBuffer(const std::string & name) const {
    const unsigned structIdx = getStreamSetIndex(name);
    if (structIdx < mStreamSetInputs.size()) {
        return mStreamSetInputBuffers[structIdx];
    } else {
        return mStreamSetOutputBuffers[structIdx - mStreamSetInputs.size()];
    }
}

Value * KernelBuilder::getStreamSetPtr(Value * self, const std::string & name, Value * blockNo) const {
    return getStreamSetBuffer(name)->getStreamSetPtr(getStreamSetStructPtr(self, name), blockNo);
}

Value * KernelBuilder::getStream(Value * self, const std::string & name, Value * blockNo, Value * index) const {
    return getStreamSetBuffer(name)->getStream(getStreamSetStructPtr(self, name), blockNo, index);
}

Value * KernelBuilder::getStream(Value * self, const std::string & name, Value * blockNo, Value * index1, Value * index2) const {
    assert (index1->getType() == index2->getType());
    return getStreamSetBuffer(name)->getStream(getStreamSetStructPtr(self, name), blockNo, index1, index2);
}

Value * KernelBuilder::getStreamView(Value * self, const std::string & name, Value * blockNo, Value * index) const {
    return getStreamSetBuffer(name)->getStreamView(getStreamSetStructPtr(self, name), blockNo, index);
}

Value * KernelBuilder::getStreamView(llvm::Type * type, Value * self, const std::string & name, Value * blockNo, Value * index) const {
    return getStreamSetBuffer(name)->getStreamView(type, getStreamSetStructPtr(self, name), blockNo, index);
}

void KernelBuilder::createInstance() {
    if (LLVM_UNLIKELY(mKernelStateType == nullptr)) {
        llvm::report_fatal_error("Cannot create kernel instance before calling prepareKernel()");
    }
    mKernelInstance = iBuilder->CreateCacheAlignedAlloca(mKernelStateType);
    Module * m = iBuilder->getModule();
    std::vector<Value *> init_args = {mKernelInstance};
    for (auto a : mInitialArguments) {
        init_args.push_back(a);
    }
    for (auto b : mStreamSetInputBuffers) {
        init_args.push_back(b->getStreamSetStructPtr());
    }
    for (auto b : mStreamSetOutputBuffers) {
        init_args.push_back(b->getStreamSetStructPtr());
    }
    std::string initFnName = mKernelName + init_suffix;
    Function * initMethod = m->getFunction(initFnName);
    if (!initMethod) {
        llvm::report_fatal_error("Cannot find " + initFnName);
    }
    iBuilder->CreateCall(initMethod, init_args);
}

Function * KernelBuilder::generateThreadFunction(const std::string & name) const {
    if (LLVM_UNLIKELY(mKernelStateType == nullptr)) {
        llvm::report_fatal_error("Cannot generate thread function before calling prepareKernel()");
    }
    Module * m = iBuilder->getModule();
    Type * const voidTy = iBuilder->getVoidTy();
    Type * const voidPtrTy = iBuilder->getVoidPtrTy();
    Type * const int8PtrTy = iBuilder->getInt8PtrTy();
    Type * const int1ty = iBuilder->getInt1Ty();
    
    Function * const threadFunc = cast<Function>(m->getOrInsertFunction(name, voidTy, int8PtrTy, nullptr));
    threadFunc->setCallingConv(CallingConv::C);
    Function::arg_iterator args = threadFunc->arg_begin();
    
    Value * const arg = &*(args++);
    arg->setName("args");
    
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", threadFunc,0));
    
    Value * self = iBuilder->CreateBitCast(arg, PointerType::get(mKernelStateType, 0));
    
    std::vector<Value *> inbufProducerPtrs;
    std::vector<Value *> inbufConsumerPtrs;
    std::vector<Value *> outbufProducerPtrs;
    std::vector<Value *> outbufConsumerPtrs;   
    std::vector<Value *> endSignalPtrs;
    
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        Value * ssStructPtr = getStreamSetStructPtr(self, mStreamSetInputs[i].name);
        inbufProducerPtrs.push_back(mStreamSetInputBuffers[i]->getProducerPosPtr(ssStructPtr));
        inbufConsumerPtrs.push_back(mStreamSetInputBuffers[i]->getConsumerPosPtr(ssStructPtr));
        endSignalPtrs.push_back(mStreamSetInputBuffers[i]->getEndOfInputPtr(ssStructPtr));
    }
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        Value * ssStructPtr = getStreamSetStructPtr(self, mStreamSetOutputs[i].name);
        outbufProducerPtrs.push_back(mStreamSetOutputBuffers[i]->getProducerPosPtr(ssStructPtr));
        outbufConsumerPtrs.push_back(mStreamSetOutputBuffers[i]->getConsumerPosPtr(ssStructPtr));
    }
    
    const unsigned segmentBlocks = codegen::SegmentSize;
    const unsigned bufferSegments = codegen::BufferSegments;
    const unsigned segmentSize = segmentBlocks * iBuilder->getBitBlockWidth();
    Type * const size_ty = iBuilder->getSizeTy();
    
    Value * segSize = ConstantInt::get(size_ty, segmentSize);
    Value * bufferSize = ConstantInt::get(size_ty, segmentSize * (bufferSegments - 1));
    
    BasicBlock * outputCheckBlock = BasicBlock::Create(iBuilder->getContext(), "outputCheck", threadFunc, 0);
    BasicBlock * inputCheckBlock = BasicBlock::Create(iBuilder->getContext(), "inputCheck", threadFunc, 0);
    
    BasicBlock * endSignalCheckBlock = BasicBlock::Create(iBuilder->getContext(), "endSignalCheck", threadFunc, 0);
    BasicBlock * doSegmentBlock = BasicBlock::Create(iBuilder->getContext(), "doSegment", threadFunc, 0);
    BasicBlock * endBlock = BasicBlock::Create(iBuilder->getContext(), "end", threadFunc, 0);
    
    iBuilder->CreateBr(outputCheckBlock);
    
    iBuilder->SetInsertPoint(outputCheckBlock);
    
    Value * waitCondTest = ConstantInt::get(int1ty, 1);   
    for (unsigned i = 0; i < outbufProducerPtrs.size(); i++) {
        LoadInst * producerPos = iBuilder->CreateAtomicLoadAcquire(outbufProducerPtrs[i]);
        // iBuilder->CallPrintInt(name + ":output producerPos", producerPos);
        LoadInst * consumerPos = iBuilder->CreateAtomicLoadAcquire(outbufConsumerPtrs[i]);
        // iBuilder->CallPrintInt(name + ":output consumerPos", consumerPos);
        waitCondTest = iBuilder->CreateAnd(waitCondTest, iBuilder->CreateICmpULE(producerPos, iBuilder->CreateAdd(consumerPos, bufferSize)));
    }
    
    iBuilder->CreateCondBr(waitCondTest, inputCheckBlock, outputCheckBlock); 
    
    iBuilder->SetInsertPoint(inputCheckBlock); 
    
    Value * requiredSize = segSize;
    if (mLookAheadPositions > 0) {
        requiredSize = iBuilder->CreateAdd(segSize, ConstantInt::get(size_ty, mLookAheadPositions));
    }
    waitCondTest = ConstantInt::get(int1ty, 1); 
    for (unsigned i = 0; i < inbufProducerPtrs.size(); i++) {
        LoadInst * producerPos = iBuilder->CreateAtomicLoadAcquire(inbufProducerPtrs[i]);
        // iBuilder->CallPrintInt(name + ":input producerPos", producerPos);
        LoadInst * consumerPos = iBuilder->CreateAtomicLoadAcquire(inbufConsumerPtrs[i]);
        // iBuilder->CallPrintInt(name + ":input consumerPos", consumerPos);
        waitCondTest = iBuilder->CreateAnd(waitCondTest, iBuilder->CreateICmpULE(iBuilder->CreateAdd(consumerPos, requiredSize), producerPos));
    }
    
    iBuilder->CreateCondBr(waitCondTest, doSegmentBlock, endSignalCheckBlock);
    
    iBuilder->SetInsertPoint(endSignalCheckBlock);
    
    LoadInst * endSignal = iBuilder->CreateLoad(endSignalPtrs[0]);
    for (unsigned i = 1; i < endSignalPtrs.size(); i++){
        LoadInst * endSignal_next = iBuilder->CreateLoad(endSignalPtrs[i]);
        iBuilder->CreateAnd(endSignal, endSignal_next);
    }
    
    iBuilder->CreateCondBr(endSignal, endBlock, inputCheckBlock);
    
    iBuilder->SetInsertPoint(doSegmentBlock);
    
    // needs positions 
    createDoSegmentCall({self, ConstantInt::getNullValue(iBuilder->getInt1Ty())});
    
    for (unsigned i = 0; i < inbufConsumerPtrs.size(); i++) {
        Value * consumerPos = iBuilder->CreateAdd(iBuilder->CreateLoad(inbufConsumerPtrs[i]), segSize);
        iBuilder->CreateAtomicStoreRelease(consumerPos, inbufConsumerPtrs[i]);
    }
    
    for (unsigned i = 0; i < outbufProducerPtrs.size(); i++) {
        Value * produced = getProducedItemCount(self, mStreamSetOutputs[i].name);
        iBuilder->CreateAtomicStoreRelease(produced, outbufProducerPtrs[i]);
    }
    
    Value * earlyEndSignal = getTerminationSignal(self);
    if (earlyEndSignal != ConstantInt::getNullValue(iBuilder->getInt1Ty())) {
        BasicBlock * earlyEndBlock = BasicBlock::Create(iBuilder->getContext(), "earlyEndSignal", threadFunc, 0);
        iBuilder->CreateCondBr(earlyEndSignal, earlyEndBlock, outputCheckBlock);
        
        iBuilder->SetInsertPoint(earlyEndBlock);
        for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
            Value * ssStructPtr = getStreamSetStructPtr(self, mStreamSetOutputs[i].name);
            mStreamSetOutputBuffers[i]->setEndOfInput(ssStructPtr);
        }        
    }
    iBuilder->CreateBr(outputCheckBlock);
    
    iBuilder->SetInsertPoint(endBlock);
    LoadInst * producerPos = iBuilder->CreateLoad(inbufProducerPtrs[0]);
    LoadInst * consumerPos = iBuilder->CreateLoad(inbufConsumerPtrs[0]);
    Value * remainingBytes = iBuilder->CreateSub(producerPos, consumerPos);
    
        // needs positions 
    createDoSegmentCall({self, ConstantInt::getAllOnesValue(iBuilder->getInt1Ty())});
    
    
    for (unsigned i = 0; i < inbufConsumerPtrs.size(); i++) {
        Value * consumerPos = iBuilder->CreateAdd(iBuilder->CreateLoad(inbufConsumerPtrs[i]), remainingBytes);
        iBuilder->CreateAtomicStoreRelease(consumerPos, inbufConsumerPtrs[i]);
    }
    for (unsigned i = 0; i < outbufProducerPtrs.size(); i++) {
        iBuilder->CreateAtomicStoreRelease(producerPos, outbufProducerPtrs[i]);
    }
    
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        Value * ssStructPtr = getStreamSetStructPtr(self, mStreamSetOutputs[i].name);
        mStreamSetOutputBuffers[i]->setEndOfInput(ssStructPtr);
    }
    
    iBuilder->CreatePThreadExitCall(Constant::getNullValue(voidPtrTy));
    iBuilder->CreateRetVoid();
    
    return threadFunc;
    
}

KernelBuilder::~KernelBuilder() {
}
