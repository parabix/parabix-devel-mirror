/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "kernel.h"
#include <llvm/IR/Module.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Value.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/ErrorHandling.h>
#include <toolchain.h>

using namespace llvm;
using namespace kernel;

KernelBuilder::KernelBuilder(IDISA::IDISA_Builder * builder,
                             std::string kernelName,
                             std::vector<Binding> stream_inputs,
                             std::vector<Binding> stream_outputs,
                             std::vector<Binding> scalar_parameters,
                             std::vector<Binding> scalar_outputs,
                             std::vector<Binding> internal_scalars)
: KernelInterface(builder, kernelName, stream_inputs, stream_outputs, scalar_parameters, scalar_outputs, internal_scalars) {

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
    generateDoBlockMethod();    // must be implemented by the KernelBuilder subtype
    generateFinalBlockMethod(); // possibly overridden by the KernelBuilder subtype
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
    generateInitMethod();
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
    while (args != finalBlockFunction->arg_end()){
        doBlockArgs.push_back(&*args++);
    }
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

//  The default doSegment method dispatches to the doBlock routine for
//  each block of the given number of blocksToDo, and then updates counts.
void KernelBuilder::generateDoSegmentMethod() const {
    auto savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Function * doSegmentFunction = m->getFunction(mKernelName + doSegment_suffix);
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", doSegmentFunction, 0));
    BasicBlock * entryBlock = iBuilder->GetInsertBlock();
    BasicBlock * strideLoopCond = BasicBlock::Create(iBuilder->getContext(), "strideLoopCond", doSegmentFunction, 0);
    BasicBlock * strideLoopBody = BasicBlock::Create(iBuilder->getContext(), "strideLoopBody", doSegmentFunction, 0);
    BasicBlock * stridesDone = BasicBlock::Create(iBuilder->getContext(), "stridesDone", doSegmentFunction, 0);
    BasicBlock * checkFinalStride = BasicBlock::Create(iBuilder->getContext(), "checkFinalStride", doSegmentFunction, 0);
    BasicBlock * checkEndSignals = BasicBlock::Create(iBuilder->getContext(), "checkEndSignals", doSegmentFunction, 0);
    BasicBlock * callFinalBlock = BasicBlock::Create(iBuilder->getContext(), "callFinalBlock", doSegmentFunction, 0);
    BasicBlock * segmentDone = BasicBlock::Create(iBuilder->getContext(), "segmentDone", doSegmentFunction, 0);
    BasicBlock * finalExit = BasicBlock::Create(iBuilder->getContext(), "finalExit", doSegmentFunction, 0);
    Type * const size_ty = iBuilder->getSizeTy();
    Constant * stride = ConstantInt::get(size_ty, iBuilder->getStride());
    Value * strideBlocks = ConstantInt::get(size_ty, iBuilder->getStride() / iBuilder->getBitBlockWidth());
    
    Function::arg_iterator args = doSegmentFunction->arg_begin();
    Value * self = &*(args++);
    Value * blocksToDo = &*(args);
    
    std::vector<Value *> inbufProducerPtrs;
    std::vector<Value *> endSignalPtrs;
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        Value * param = getStreamSetStructPtr(self, mStreamSetInputs[i].name);
        inbufProducerPtrs.push_back(mStreamSetInputBuffers[i]->getProducerPosPtr(param));
        endSignalPtrs.push_back(mStreamSetInputBuffers[i]->getEndOfInputPtr(param));
    }
    
    std::vector<Value *> producerPos;
    /* Determine the actually available data examining all input stream sets. */
    LoadInst * p = iBuilder->CreateAtomicLoadAcquire(inbufProducerPtrs[0]);
    producerPos.push_back(p);
    Value * availablePos = producerPos[0];
    for (unsigned i = 1; i < inbufProducerPtrs.size(); i++) {
        LoadInst * p = iBuilder->CreateAtomicLoadAcquire(inbufProducerPtrs[i]);
        producerPos.push_back(p);
        /* Set the available position to be the minimum of availablePos and producerPos. */
        availablePos = iBuilder->CreateSelect(iBuilder->CreateICmpULT(availablePos, p), availablePos, p);
    }
    Value * processed = getProcessedItemCount(self, mStreamSetInputs[0].name);
    Value * itemsAvail = iBuilder->CreateSub(availablePos, processed);
//#ifndef NDEBUG
//    iBuilder->CallPrintInt(mKernelName + "_itemsAvail", itemsAvail);
//#endif
    Value * stridesToDo = iBuilder->CreateUDiv(blocksToDo, strideBlocks);
    Value * stridesAvail = iBuilder->CreateUDiv(itemsAvail, stride);
    /* Adjust the number of full blocks to do, based on the available data, if necessary. */
    Value * lessThanFullSegment = iBuilder->CreateICmpULT(stridesAvail, stridesToDo);
    stridesToDo = iBuilder->CreateSelect(lessThanFullSegment, stridesAvail, stridesToDo);
    //iBuilder->CallPrintInt(mKernelName + "_stridesAvail", stridesAvail);
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
    processed = iBuilder->CreateAdd(processed, iBuilder->CreateMul(stridesToDo, stride));
    setProcessedItemCount(self, mStreamSetInputs[0].name, processed);
    iBuilder->CreateCondBr(lessThanFullSegment, checkFinalStride, segmentDone);
    
    iBuilder->SetInsertPoint(checkFinalStride);
    
    /* We had less than a full segment of data; we may have reached the end of input
       on one of the stream sets.  */
    
    Value * alreadyDone = getTerminationSignal(self);
    iBuilder->CreateCondBr(alreadyDone, finalExit, checkEndSignals);
    
    iBuilder->SetInsertPoint(checkEndSignals);
    Value * endOfInput = iBuilder->CreateLoad(endSignalPtrs[0]);
    if (endSignalPtrs.size() > 1) {
        /* If there is more than one input stream set, then we need to confirm that one of
           them has both the endSignal set and the length = to availablePos. */
        endOfInput = iBuilder->CreateAnd(endOfInput, iBuilder->CreateICmpEQ(availablePos, producerPos[0]));
        for (unsigned i = 1; i < endSignalPtrs.size(); i++) {
            Value * e = iBuilder->CreateAnd(iBuilder->CreateLoad(endSignalPtrs[i]), iBuilder->CreateICmpEQ(availablePos, producerPos[i]));
            endOfInput = iBuilder->CreateOr(endOfInput, e);
        }
    }
    iBuilder->CreateCondBr(endOfInput, callFinalBlock, segmentDone);
    
    iBuilder->SetInsertPoint(callFinalBlock);
    
    Value * remainingItems = iBuilder->CreateSub(availablePos, processed);
    createFinalBlockCall(self, remainingItems);
    setProcessedItemCount(self, mStreamSetInputs[0].name, availablePos);
    
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        Value * ssStructPtr = getStreamSetStructPtr(self, mStreamSetOutputs[i].name);
        mStreamSetOutputBuffers[i]->setEndOfInput(ssStructPtr);
    }
    setTerminationSignal(self);
    iBuilder->CreateBr(segmentDone);
    
    iBuilder->SetInsertPoint(segmentDone);
//#ifndef NDEBUG
//    iBuilder->CallPrintInt(mKernelName + "_produced", produced);
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

void KernelBuilder::setBlockNo(Value * self, Value * newFieldVal) const {
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), getScalarIndex(blockNoScalar)});
    iBuilder->CreateStore(newFieldVal, ptr);
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

size_t KernelBuilder::getStreamSetBufferSize(Value * /* self */, const std::string & name) const {
    const unsigned index = getStreamSetIndex(name);
    StreamSetBuffer * buf = nullptr;
    if (index < mStreamSetInputs.size()) {
        buf = mStreamSetInputBuffers[index];
    } else {
        buf = mStreamSetOutputBuffers[index - mStreamSetInputs.size()];
    }
    return buf->getBufferSize();
}

Value * KernelBuilder::getStreamSetStructPtr(Value * self, const std::string & name) const {
    return getScalarField(self, name + structPtrSuffix);
}

Value * KernelBuilder::getStreamSetBlockPtr(Value * self, const std::string &name, Value * blockNo) const {
    Value * const structPtr = getStreamSetStructPtr(self, name);
    const unsigned index = getStreamSetIndex(name);
    StreamSetBuffer * buf = nullptr;
    if (index < mStreamSetInputs.size()) {
        buf = mStreamSetInputBuffers[index];
    } else {
        buf = mStreamSetOutputBuffers[index - mStreamSetInputs.size()];
    }    
    return buf->getStreamSetBlockPointer(structPtr, blockNo);
}

Value * KernelBuilder::getStream(Value * self, const std::string & name, Value * blockNo, Value * index) {
    return iBuilder->CreateGEP(getStreamSetBlockPtr(self, name, blockNo), {iBuilder->getInt32(0), index});
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
    Value * segBlocks = ConstantInt::get(size_ty, segmentBlocks);
   
    BasicBlock * outputCheckBlock = BasicBlock::Create(iBuilder->getContext(), "outputCheck", threadFunc, 0);
    BasicBlock * inputCheckBlock = BasicBlock::Create(iBuilder->getContext(), "inputCheck", threadFunc, 0);
    
    BasicBlock * endSignalCheckBlock = BasicBlock::Create(iBuilder->getContext(), "endSignalCheck", threadFunc, 0);
    BasicBlock * doSegmentBlock = BasicBlock::Create(iBuilder->getContext(), "doSegment", threadFunc, 0);
    BasicBlock * endBlock = BasicBlock::Create(iBuilder->getContext(), "end", threadFunc, 0);
    BasicBlock * doFinalSegBlock = BasicBlock::Create(iBuilder->getContext(), "doFinalSeg", threadFunc, 0);
    BasicBlock * doFinalBlock = BasicBlock::Create(iBuilder->getContext(), "doFinal", threadFunc, 0);

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
  
    createDoSegmentCall(self, segBlocks);

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
    Value * blockSize = ConstantInt::get(size_ty, iBuilder->getBitBlockWidth());
    Value * blocks = iBuilder->CreateUDiv(remainingBytes, blockSize);
    Value * finalBlockRemainingBytes = iBuilder->CreateURem(remainingBytes, blockSize);

    iBuilder->CreateCondBr(iBuilder->CreateICmpEQ(blocks, ConstantInt::get(size_ty, 0)), doFinalBlock, doFinalSegBlock);

    iBuilder->SetInsertPoint(doFinalSegBlock);

    createDoSegmentCall(self, blocks);

    iBuilder->CreateBr(doFinalBlock);

    iBuilder->SetInsertPoint(doFinalBlock);

    createFinalBlockCall(self, finalBlockRemainingBytes);

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
