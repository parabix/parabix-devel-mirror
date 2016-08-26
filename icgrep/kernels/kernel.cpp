/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "kernel.h"
#include <llvm/IR/Module.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Value.h>
#include <llvm/Support/raw_ostream.h>

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
        throw std::runtime_error("Illegal addition of kernel field after kernel state finalized: " + scalarName);
    }
    unsigned index = mKernelFields.size();
    mKernelFields.push_back(t);
    mInternalStateNameMap.emplace(scalarName, index);
}

void KernelBuilder::prepareKernel() {
    if (mStreamSetInputs.size() != mStreamSetInputBuffers.size()) {
        throw std::runtime_error("Kernel preparation: Incorrect number of input buffers");
    }
    if (mStreamSetOutputs.size() != mStreamSetOutputBuffers.size()) {
        throw std::runtime_error("Kernel preparation: Incorrect number of input buffers");
    }
    addScalar(iBuilder->getSizeTy(), blockNoScalar);
    int streamSetNo = 0;
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        if (!(mStreamSetInputBuffers[i]->getBufferStreamSetType() == mStreamSetInputs[i].ssType)) {
             throw std::runtime_error("Kernel preparation: Incorrect input buffer type");
        }
        mScalarInputs.push_back(ScalarBinding{mStreamSetInputBuffers[i]->getStreamBufferPointerType(), mStreamSetInputs[i].ssName + basePtrSuffix});
        mStreamSetNameMap.emplace(mStreamSetInputs[i].ssName, streamSetNo);
        streamSetNo++;
    }
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        if (!(mStreamSetOutputBuffers[i]->getBufferStreamSetType() == mStreamSetOutputs[i].ssType)) {
             throw std::runtime_error("Kernel preparation: Incorrect input buffer type");
        }
        mScalarInputs.push_back(ScalarBinding{mStreamSetOutputBuffers[i]->getStreamBufferPointerType(), mStreamSetOutputs[i].ssName + basePtrSuffix});
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
    mKernelStateType = StructType::create(getGlobalContext(), mKernelFields, mKernelName);
}

std::unique_ptr<Module> KernelBuilder::createKernelModule(std::vector<StreamSetBuffer *> input_buffers, std::vector<StreamSetBuffer *> output_buffers) {
    Module * saveModule = iBuilder->getModule();
    IDISA::IDISA_Builder::InsertPoint savePoint = iBuilder->saveIP();
    std::unique_ptr<Module> theModule = make_unique<Module>(mKernelName + "_" + iBuilder->getBitBlockTypeName(), getGlobalContext());
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

//  The default doSegment method simply dispatches to the doBlock routine.
void KernelBuilder::generateDoSegmentMethod() {
    IDISA::IDISA_Builder::InsertPoint savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Function * doBlockFunction = m->getFunction(mKernelName + doBlock_suffix);
    Function * doSegmentFunction = m->getFunction(mKernelName + doSegment_suffix);
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", doSegmentFunction, 0));
    BasicBlock * entryBlock = iBuilder->GetInsertBlock();
    BasicBlock * blockLoop = BasicBlock::Create(iBuilder->getContext(), "blockLoop", doSegmentFunction, 0);
    BasicBlock * blocksDone = BasicBlock::Create(iBuilder->getContext(), "blocksDone", doSegmentFunction, 0);

    
    Function::arg_iterator args = doSegmentFunction->arg_begin();
    Value * self = &*(args++);
    Value * blocksToDo = &*(args);
    
    iBuilder->CreateBr(blockLoop);
    
    iBuilder->SetInsertPoint(blockLoop);
    PHINode * blocksRemaining = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2, "blocksRemaining");
    blocksRemaining->addIncoming(blocksToDo, entryBlock);
    
    Value * blockNo = getScalarField(self, blockNoScalar);
    
    iBuilder->CreateCall(doBlockFunction, {self});
    setScalarField(self, blockNoScalar, iBuilder->CreateAdd(blockNo, ConstantInt::get(iBuilder->getSizeTy(), iBuilder->getStride() / iBuilder->getBitBlockWidth())));
    blocksToDo = iBuilder->CreateSub(blocksRemaining, ConstantInt::get(iBuilder->getSizeTy(), 1));
    blocksRemaining->addIncoming(blocksToDo, blockLoop);
    Value * notDone = iBuilder->CreateICmpUGT(blocksToDo, ConstantInt::get(iBuilder->getSizeTy(), 0));
    iBuilder->CreateCondBr(notDone, blockLoop, blocksDone);
    
    iBuilder->SetInsertPoint(blocksDone);
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}

Value * KernelBuilder::getScalarIndex(std::string fieldName) {
    const auto f = mInternalStateNameMap.find(fieldName);
    if (LLVM_UNLIKELY(f == mInternalStateNameMap.end())) {
        throw std::runtime_error("Kernel does not contain internal state: " + fieldName);
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


Value * KernelBuilder::getParameter(Function * f, std::string paramName) {
    for (Function::arg_iterator argIter = f->arg_begin(), end = f->arg_end(); argIter != end; argIter++) {
        Value * arg = &*argIter;
        if (arg->getName() == paramName) return arg;
    }
    throw std::runtime_error("Method does not have parameter: " + paramName);
}

unsigned KernelBuilder::getStreamSetIndex(std::string ssName) {
    const auto f = mStreamSetNameMap.find(ssName);
    if (LLVM_UNLIKELY(f == mStreamSetNameMap.end())) {
        throw std::runtime_error("Kernel does not contain stream set: " + ssName);
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
        init_args.push_back(b->getStreamSetBasePtr());
    }
    for (auto b : mStreamSetOutputBuffers) { 
        init_args.push_back(b->getStreamSetBasePtr());
    }
    std::string initFnName = mKernelName + init_suffix;
    Function * initMethod = m->getFunction(initFnName);
    if (!initMethod) {
        throw std::runtime_error("Cannot find " + initFnName);
    }
    iBuilder->CreateCall(initMethod, init_args);
    return kernelInstance;
}




