/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "kernel.h"
#include <llvm/IR/Module.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Value.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/TypeBuilder.h>
#include <llvm/Support/ErrorHandling.h>
#include <toolchain.h>

using namespace llvm;
using namespace kernel;

KernelBuilder::KernelBuilder(IDISA::IDISA_Builder * builder,
                                 std::string kernelName,
                                 std::vector<StreamSetBinding> stream_inputs,
                                 std::vector<StreamSetBinding> stream_outputs,
                                 std::vector<ScalarBinding> scalar_parameters,
                                 std::vector<ScalarBinding> scalar_outputs,
                                 std::vector<ScalarBinding> internal_scalars) :
    KernelInterface(builder, kernelName, stream_inputs, stream_outputs, scalar_parameters, scalar_outputs, internal_scalars) {}

void KernelBuilder::addScalar(Type * t, std::string scalarName) {
    if (LLVM_UNLIKELY(mKernelStateType != nullptr)) {
        llvm::report_fatal_error("Illegal addition of kernel field after kernel state finalized: " + scalarName);
    }
    unsigned index = mKernelFields.size();
    mKernelFields.push_back(t);
    mInternalStateNameMap.emplace(scalarName, index);
}

void KernelBuilder::prepareKernel() {
    unsigned blockSize = iBuilder->getBitBlockWidth();
    if (mStreamSetInputs.size() != mStreamSetInputBuffers.size()) {
        llvm::report_fatal_error("Kernel preparation: Incorrect number of input buffers");
    }
    if (mStreamSetOutputs.size() != mStreamSetOutputBuffers.size()) {
        llvm::report_fatal_error("Kernel preparation: Incorrect number of output buffers");
    }
    addScalar(iBuilder->getSizeTy(), blockNoScalar);
    addScalar(iBuilder->getSizeTy(), logicalSegmentNoScalar);
    addScalar(iBuilder->getSizeTy(), processedItemCount);
    addScalar(iBuilder->getSizeTy(), producedItemCount);
    addScalar(iBuilder->getInt1Ty(), terminationSignal);
    int streamSetNo = 0;
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        if (!(mStreamSetInputBuffers[i]->getBufferStreamSetType() == mStreamSetInputs[i].ssType)) {
             llvm::report_fatal_error("Kernel preparation: Incorrect input buffer type");
        }
        if ((mStreamSetInputBuffers[i]->getBufferSize() > 0) && (mStreamSetInputBuffers[i]->getBufferSize() < codegen::SegmentSize + (blockSize + mLookAheadPositions - 1)/blockSize)) {
             errs() << "buffer size = " << mStreamSetInputBuffers[i]->getBufferSize() << "\n";
             llvm::report_fatal_error("Kernel preparation: Buffer size too small.");
        }
        mScalarInputs.push_back(ScalarBinding{mStreamSetInputBuffers[i]->getStreamSetStructPointerType(), mStreamSetInputs[i].ssName + basePtrSuffix});
        mStreamSetNameMap.emplace(mStreamSetInputs[i].ssName, streamSetNo);
        streamSetNo++;
    }
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        if (!(mStreamSetOutputBuffers[i]->getBufferStreamSetType() == mStreamSetOutputs[i].ssType)) {
             llvm::report_fatal_error("Kernel preparation: Incorrect output buffer type");
        }
        mScalarInputs.push_back(ScalarBinding{mStreamSetOutputBuffers[i]->getStreamSetStructPointerType(), mStreamSetOutputs[i].ssName + basePtrSuffix});
        mStreamSetNameMap.emplace(mStreamSetOutputs[i].ssName, streamSetNo);
        streamSetNo++;
    }
    for (auto binding : mScalarInputs) {
        addScalar(binding.scalarType, binding.scalarName);
    }
    for (auto binding : mScalarOutputs) {
        addScalar(binding.scalarType, binding.scalarName);
    }
    for (auto binding : mInternalScalars) {
        addScalar(binding.scalarType, binding.scalarName);
    }
    mKernelStateType = StructType::create(iBuilder->getContext(), mKernelFields, mKernelName);
}

std::unique_ptr<Module> KernelBuilder::createKernelModule(std::vector<StreamSetBuffer *> input_buffers, std::vector<StreamSetBuffer *> output_buffers) {
    Module * saveModule = iBuilder->getModule();
    IDISA::IDISA_Builder::InsertPoint savePoint = iBuilder->saveIP();
    std::unique_ptr<Module> theModule = make_unique<Module>(mKernelName + "_" + iBuilder->getBitBlockTypeName(), iBuilder->getContext());
    Module * m = theModule.get();
    iBuilder->setModule(m);
    generateKernel(input_buffers, output_buffers);
    iBuilder->setModule(saveModule);
    iBuilder->restoreIP(savePoint);
    return theModule;
}

void KernelBuilder::generateKernel(std::vector<StreamSetBuffer *> input_buffers, std::vector<StreamSetBuffer*> output_buffers) {
    IDISA::IDISA_Builder::InsertPoint savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    mStreamSetInputBuffers = input_buffers;
    mStreamSetOutputBuffers = output_buffers;
    prepareKernel();  // possibly overriden by the KernelBuilder subtype
    KernelInterface::addKernelDeclarations(m);
    generateDoBlockMethod();     // must be implemented by the KernelBuilder subtype
    generateFinalBlockMethod();  // possibly overriden by the KernelBuilder subtype
    generateDoSegmentMethod();

    // Implement the accumulator get functions
    for (auto binding : mScalarOutputs) {
        auto fnName = mKernelName + accumulator_infix + binding.scalarName;
        Function * accumFn = m->getFunction(fnName);
        iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "get_" + binding.scalarName, accumFn, 0));
        Value * self = &*(accumFn->arg_begin());
        Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), getScalarIndex(binding.scalarName)});
        Value * retVal = iBuilder->CreateLoad(ptr);
        iBuilder->CreateRet(retVal);
    }
    // Implement the initializer function
    Function * initFunction = m->getFunction(mKernelName + init_suffix);
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "Init_entry", initFunction, 0));
    
    Function::arg_iterator args = initFunction->arg_begin();
    Value * self = &*(args++);
    iBuilder->CreateStore(Constant::getNullValue(mKernelStateType), self);
    for (auto binding : mScalarInputs) {
        Value * parm = &*(args++);
        Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), getScalarIndex(binding.scalarName)});
        iBuilder->CreateStore(parm, ptr);
    }
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}

//  The default finalBlock method simply dispatches to the doBlock routine.
void KernelBuilder::generateFinalBlockMethod() {
    IDISA::IDISA_Builder::InsertPoint savePoint = iBuilder->saveIP();
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

void KernelBuilder::generateDoBlockLogic(Value * self, Value * blockNo) {
    Function * doBlockFunction = iBuilder->getModule()->getFunction(mKernelName + doBlock_suffix);
    iBuilder->CreateCall(doBlockFunction, {self});
}

//  The default doSegment method dispatches to the doBlock routine for
//  each block of the given number of blocksToDo, and then updates counts.
void KernelBuilder::generateDoSegmentMethod() {
    IDISA::IDISA_Builder::InsertPoint savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Function * doSegmentFunction = m->getFunction(mKernelName + doSegment_suffix);
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", doSegmentFunction, 0));
    BasicBlock * entryBlock = iBuilder->GetInsertBlock();
    BasicBlock * blockLoopCond = BasicBlock::Create(iBuilder->getContext(), "blockLoopCond", doSegmentFunction, 0);
    BasicBlock * blockLoopBody = BasicBlock::Create(iBuilder->getContext(), "blockLoopBody", doSegmentFunction, 0);
    BasicBlock * blocksDone = BasicBlock::Create(iBuilder->getContext(), "blocksDone", doSegmentFunction, 0);
    Type * const size_ty = iBuilder->getSizeTy();
    Value * stride = ConstantInt::get(size_ty, iBuilder->getStride());
    Value * strideBlocks = ConstantInt::get(size_ty, iBuilder->getStride() / iBuilder->getBitBlockWidth());
    
    Function::arg_iterator args = doSegmentFunction->arg_begin();
    Value * self = &*(args++);
    Value * blocksToDo = &*(args);

    Value * segmentNo = getLogicalSegmentNo(self);
    iBuilder->CreateBr(blockLoopCond);

    iBuilder->SetInsertPoint(blockLoopCond);
    PHINode * blocksRemaining = iBuilder->CreatePHI(size_ty, 2, "blocksRemaining");
    blocksRemaining->addIncoming(blocksToDo, entryBlock);
    Value * notDone = iBuilder->CreateICmpUGT(blocksRemaining, ConstantInt::get(size_ty, 0));
    iBuilder->CreateCondBr(notDone, blockLoopBody, blocksDone);

    iBuilder->SetInsertPoint(blockLoopBody);
    Value * blockNo = getScalarField(self, blockNoScalar);   
    generateDoBlockLogic(self, blockNo);
    setBlockNo(self, iBuilder->CreateAdd(blockNo, strideBlocks));
    blocksRemaining->addIncoming(iBuilder->CreateSub(blocksRemaining, strideBlocks), blockLoopBody);
    iBuilder->CreateBr(blockLoopCond);
    
    iBuilder->SetInsertPoint(blocksDone);
    setProcessedItemCount(self, iBuilder->CreateAdd(getProcessedItemCount(self), iBuilder->CreateMul(blocksToDo, stride)));
    // Must be the last action, for synchronization.
    setLogicalSegmentNo(self, iBuilder->CreateAdd(segmentNo, ConstantInt::get(size_ty, 1)));

    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}

Value * KernelBuilder::getScalarIndex(std::string fieldName) {
    const auto f = mInternalStateNameMap.find(fieldName);
    if (LLVM_UNLIKELY(f == mInternalStateNameMap.end())) {
        llvm::report_fatal_error("Kernel does not contain internal state: " + fieldName);
    }
    return iBuilder->getInt32(f->second);
}



Value * KernelBuilder::getScalarField(Value * self, std::string fieldName) {
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), getScalarIndex(fieldName)});
    return iBuilder->CreateLoad(ptr);
}

void KernelBuilder::setScalarField(Value * self, std::string fieldName, Value * newFieldVal) {
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), getScalarIndex(fieldName)});
    iBuilder->CreateStore(newFieldVal, ptr);
}

Value * KernelBuilder::getLogicalSegmentNo(Value * self) { 
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), getScalarIndex(logicalSegmentNoScalar)});
    LoadInst * segNo = iBuilder->CreateAlignedLoad(ptr, sizeof(size_t));
    segNo->setOrdering(AtomicOrdering::Acquire);
    return segNo;
}

Value * KernelBuilder::getProcessedItemCount(Value * self) { 
    return getScalarField(self, processedItemCount);
}

Value * KernelBuilder::getProducedItemCount(Value * self) {
    return getScalarField(self, producedItemCount);
}

//  By default, kernels do not terminate early.  
Value * KernelBuilder::getTerminationSignal(Value * self) {
    return ConstantInt::getNullValue(iBuilder->getInt1Ty());
}


void KernelBuilder::setLogicalSegmentNo(Value * self, Value * newCount) {
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), getScalarIndex(logicalSegmentNoScalar)});
    iBuilder->CreateAlignedStore(newCount, ptr, sizeof(size_t))->setOrdering(AtomicOrdering::Release);
}

void KernelBuilder::setProcessedItemCount(Value * self, Value * newCount) {
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), getScalarIndex(processedItemCount)});
    iBuilder->CreateStore(newCount, ptr);
}

void KernelBuilder::setProducedItemCount(Value * self, Value * newCount) {
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), getScalarIndex(producedItemCount)});
    iBuilder->CreateStore(newCount, ptr);
}

void KernelBuilder::setTerminationSignal(Value * self, Value * newFieldVal) {
    llvm::report_fatal_error("This kernel type does not support setTerminationSignal.");
}


Value * KernelBuilder::getBlockNo(Value * self) {
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), getScalarIndex(blockNoScalar)});
    LoadInst * blockNo = iBuilder->CreateLoad(ptr);
    return blockNo;
}

void KernelBuilder::setBlockNo(Value * self, Value * newFieldVal) {
    Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), getScalarIndex(blockNoScalar)});
    iBuilder->CreateStore(newFieldVal, ptr);
}


Value * KernelBuilder::getParameter(Function * f, std::string paramName) {
    for (Function::arg_iterator argIter = f->arg_begin(), end = f->arg_end(); argIter != end; argIter++) {
        Value * arg = &*argIter;
        if (arg->getName() == paramName) return arg;
    }
    llvm::report_fatal_error("Method does not have parameter: " + paramName);
}

unsigned KernelBuilder::getStreamSetIndex(std::string ssName) {
    const auto f = mStreamSetNameMap.find(ssName);
    if (LLVM_UNLIKELY(f == mStreamSetNameMap.end())) {
        llvm::report_fatal_error("Kernel does not contain stream set: " + ssName);
    }
    return f->second;
}

size_t KernelBuilder::getStreamSetBufferSize(Value * self, std::string ssName) {
    unsigned ssIndex = getStreamSetIndex(ssName);
    if (ssIndex < mStreamSetInputs.size()) {
        return mStreamSetInputBuffers[ssIndex]->getBufferSize();
    }
    else {
        return mStreamSetOutputBuffers[ssIndex - mStreamSetInputs.size()]->getBufferSize();
    }
}

Value * KernelBuilder::getStreamSetBasePtr(Value * self, std::string ssName) {
    return getScalarField(self, ssName + basePtrSuffix);
}

Value * KernelBuilder::getStreamSetBlockPtr(Value * self, std::string ssName, Value * blockNo) {
    Value * basePtr = getStreamSetBasePtr(self, ssName);
    unsigned ssIndex = getStreamSetIndex(ssName);
    if (ssIndex < mStreamSetInputs.size()) {
        return mStreamSetInputBuffers[ssIndex]->getStreamSetBlockPointer(basePtr, blockNo);
    }
    else {
        return mStreamSetOutputBuffers[ssIndex - mStreamSetInputs.size()]->getStreamSetBlockPointer(basePtr, blockNo);
    }
}

Value * KernelBuilder::createInstance(std::vector<Value *> args) {
    Value * kernelInstance = iBuilder->CreateAlloca(mKernelStateType);
    Module * m = iBuilder->getModule();
    std::vector<Value *> init_args = {kernelInstance};
    for (auto a : args) {
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
    return kernelInstance;
}

Function * KernelBuilder::generateThreadFunction(std::string name){
    Module * m = iBuilder->getModule();
    Type * const voidTy = Type::getVoidTy(m->getContext());
    Type * const voidPtrTy = TypeBuilder<void *, false>::get(m->getContext());
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
        Value * basePtr = getStreamSetBasePtr(self, mStreamSetInputs[i].ssName);
        inbufProducerPtrs.push_back(mStreamSetInputBuffers[i]->getProducerPosPtr(basePtr));
        inbufConsumerPtrs.push_back(mStreamSetInputBuffers[i]->getConsumerPosPtr(basePtr));
        endSignalPtrs.push_back(mStreamSetInputBuffers[i]->hasEndOfInputPtr(basePtr));
    }
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        Value * basePtr = getStreamSetBasePtr(self, mStreamSetOutputs[i].ssName);
        outbufProducerPtrs.push_back(mStreamSetOutputBuffers[i]->getProducerPosPtr(basePtr));
        outbufConsumerPtrs.push_back(mStreamSetOutputBuffers[i]->getConsumerPosPtr(basePtr));
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
        LoadInst * producerPos = iBuilder->CreateAlignedLoad(outbufProducerPtrs[i], sizeof(size_t));
        producerPos->setOrdering(AtomicOrdering::Acquire);
        // iBuilder->CallPrintInt(name + ":output producerPos", producerPos);
        LoadInst * consumerPos = iBuilder->CreateAlignedLoad(outbufConsumerPtrs[i], sizeof(size_t));
        consumerPos->setOrdering(AtomicOrdering::Acquire);
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
        LoadInst * producerPos = iBuilder->CreateAlignedLoad(inbufProducerPtrs[i], sizeof(size_t));
        producerPos->setOrdering(AtomicOrdering::Acquire);
        // iBuilder->CallPrintInt(name + ":input producerPos", producerPos);
        LoadInst * consumerPos = iBuilder->CreateAlignedLoad(inbufConsumerPtrs[i], sizeof(size_t));
        consumerPos->setOrdering(AtomicOrdering::Acquire);
        // iBuilder->CallPrintInt(name + ":input consumerPos", consumerPos);
        waitCondTest = iBuilder->CreateAnd(waitCondTest, iBuilder->CreateICmpULE(iBuilder->CreateAdd(consumerPos, requiredSize), producerPos));
    }

    iBuilder->CreateCondBr(waitCondTest, doSegmentBlock, endSignalCheckBlock);
   
    iBuilder->SetInsertPoint(endSignalCheckBlock);
    
    LoadInst * endSignal = iBuilder->CreateAlignedLoad(endSignalPtrs[0], sizeof(size_t));
    // iBuilder->CallPrintInt(name + ":endSignal", endSignal);
    endSignal->setOrdering(AtomicOrdering::Acquire);
    for (unsigned i = 1; i < endSignalPtrs.size(); i++){
        LoadInst * endSignal_next = iBuilder->CreateAlignedLoad(endSignalPtrs[i], sizeof(size_t));
        endSignal_next->setOrdering(AtomicOrdering::Acquire);
        iBuilder->CreateAnd(endSignal, endSignal_next);
    }
        
    iBuilder->CreateCondBr(iBuilder->CreateICmpEQ(endSignal, ConstantInt::get(iBuilder->getInt8Ty(), 1)), endBlock, inputCheckBlock);
    
    iBuilder->SetInsertPoint(doSegmentBlock);
  
    createDoSegmentCall(self, segBlocks);

    for (unsigned i = 0; i < inbufConsumerPtrs.size(); i++) {
        Value * consumerPos = iBuilder->CreateAdd(iBuilder->CreateLoad(inbufConsumerPtrs[i]), segSize);
        iBuilder->CreateAlignedStore(consumerPos, inbufConsumerPtrs[i], sizeof(size_t))->setOrdering(AtomicOrdering::Release);
    }
    
    Value * produced = getProducedItemCount(self);
    for (unsigned i = 0; i < outbufProducerPtrs.size(); i++) {
        iBuilder->CreateAlignedStore(produced, outbufProducerPtrs[i], sizeof(size_t))->setOrdering(AtomicOrdering::Release);
    }
    
    Value * earlyEndSignal = getTerminationSignal(self);
    if (earlyEndSignal != ConstantInt::getNullValue(iBuilder->getInt1Ty())) {
        BasicBlock * earlyEndBlock = BasicBlock::Create(iBuilder->getContext(), "earlyEndSignal", threadFunc, 0);
        iBuilder->CreateCondBr(earlyEndSignal, earlyEndBlock, outputCheckBlock);

        iBuilder->SetInsertPoint(earlyEndBlock);
        for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
            Value * basePtr = getStreamSetBasePtr(self, mStreamSetOutputs[i].ssName);
            mStreamSetOutputBuffers[i]->setEndOfInput(basePtr);
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
        iBuilder->CreateAlignedStore(consumerPos, inbufConsumerPtrs[i], sizeof(size_t))->setOrdering(AtomicOrdering::Release);
    }
    for (unsigned i = 0; i < outbufProducerPtrs.size(); i++) {
        iBuilder->CreateAlignedStore(producerPos, outbufProducerPtrs[i], sizeof(size_t))->setOrdering(AtomicOrdering::Release);
    }

    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        Value * basePtr = getStreamSetBasePtr(self, mStreamSetOutputs[i].ssName);
        mStreamSetOutputBuffers[i]->setEndOfInput(basePtr);
    }

    Value * nullVal = Constant::getNullValue(voidPtrTy);
    Function * pthreadExitFunc = m->getFunction("pthread_exit");
    CallInst * exitThread = iBuilder->CreateCall(pthreadExitFunc, {nullVal}); 
    exitThread->setDoesNotReturn();
    iBuilder->CreateRetVoid();

    return threadFunc;

}
