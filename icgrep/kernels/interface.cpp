/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "interface.h"
#include <llvm/IR/Module.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Value.h>
#include <llvm/Support/raw_ostream.h>

using namespace llvm;
using namespace kernel;

void KernelInterface::addKernelDeclarations(Module * client) {
    Module * saveModule = iBuilder->getModule();
    IDISA::IDISA_Builder::InsertPoint savePoint = iBuilder->saveIP();
    iBuilder->setModule(client);
    if (mKernelStateType == nullptr) {
        throw std::runtime_error("Kernel interface " + mKernelName + " not yet finalized.");
    }
    Type * selfType = PointerType::getUnqual(mKernelStateType);
    // Create the accumulator get function prototypes
    for (auto binding : mScalarOutputs) {
        FunctionType * accumFnType = FunctionType::get(binding.scalarType, {selfType}, false);
        std::string fnName = mKernelName + accumulator_infix + binding.scalarName;
        Function * accumFn = Function::Create(accumFnType, GlobalValue::ExternalLinkage, fnName, client);
        accumFn->setCallingConv(CallingConv::C);
        accumFn->setDoesNotThrow();
        Value * self = &*(accumFn->arg_begin());
        self->setName("self");        
    }
    // Create the initialization function prototype

    std::vector<Type *> initParameters = {selfType};
    for (auto binding : mScalarInputs) {
        initParameters.push_back(binding.scalarType);
    }
    FunctionType * initFunctionType = FunctionType::get(iBuilder->getVoidTy(), initParameters, false);
    std::string initFnName = mKernelName + init_suffix;
    Function * initFn = Function::Create(initFunctionType, GlobalValue::ExternalLinkage, initFnName, client);
    initFn->setCallingConv(CallingConv::C);
    initFn->setDoesNotThrow();
    Function::arg_iterator initArgs = initFn->arg_begin();
    Value * initArg = &*(initArgs++);
    initArg->setName("self");
    for (auto binding : mScalarInputs) {
        initArg = &*(initArgs++);
        initArg->setName(binding.scalarName);
    }

    // Create the doBlock and finalBlock function prototypes
    
    std::vector<Type *> doBlockParameters = {selfType};
    std::vector<Type *> finalBlockParameters = {selfType, iBuilder->getInt64Ty()};
    for (auto inputSet : mStreamSetInputs) {
        Type * inputSetParmType = PointerType::getUnqual(inputSet.ssType.getStreamSetBlockType(iBuilder));
        doBlockParameters.push_back(inputSetParmType);
        finalBlockParameters.push_back(inputSetParmType);
    }
    for (auto outputSet : mStreamSetOutputs) {
        Type * outputSetParmType = PointerType::getUnqual(outputSet.ssType.getStreamSetBlockType(iBuilder));
        doBlockParameters.push_back(outputSetParmType);
        finalBlockParameters.push_back(outputSetParmType);
    }
    FunctionType * doBlockFunctionType = FunctionType::get(mDoBlockReturnType, doBlockParameters, false);
    std::string doBlockName = mKernelName + doBlock_suffix;
    Function * doBlockFn = Function::Create(doBlockFunctionType, GlobalValue::ExternalLinkage, doBlockName, client);
    doBlockFn->setCallingConv(CallingConv::C);
    doBlockFn->setDoesNotThrow();
    for (int i = 1; i <= doBlockParameters.size(); i++) {
        doBlockFn->setDoesNotCapture(i);
    }
    
    FunctionType * finalBlockType = FunctionType::get(mDoBlockReturnType, finalBlockParameters, false);
    std::string finalBlockName = mKernelName + finalBlock_suffix;
    Function * finalBlockFn = Function::Create(finalBlockType, GlobalValue::ExternalLinkage, finalBlockName, client);
    finalBlockFn->setCallingConv(CallingConv::C);
    finalBlockFn->setDoesNotThrow();
    finalBlockFn->setDoesNotCapture(1);
    // Parameter #2 is not a pointer; nocapture is irrelevant
    for (int i = 3; i <= finalBlockParameters.size(); i++) {
        finalBlockFn->setDoesNotCapture(i);
    }
    
    Function::arg_iterator doBlockArgs = doBlockFn->arg_begin();
    Function::arg_iterator finalBlockArgs = finalBlockFn->arg_begin();
    Value * doBlockArg = &*(doBlockArgs++);
    doBlockArg->setName("self");
    Value * finalBlockArg = &*(finalBlockArgs++);
    finalBlockArg->setName("self");
    finalBlockArg = &*(finalBlockArgs++);
    finalBlockArg->setName("remainingBytes");

    for (auto inputSet : mStreamSetInputs) {
        doBlockArg = &*(doBlockArgs++);
        finalBlockArg = &*(finalBlockArgs++);
        doBlockArg->setName(inputSet.ssName);
        finalBlockArg->setName(inputSet.ssName);
    }
    for (auto outputSet : mStreamSetOutputs) {
        doBlockArg = &*(doBlockArgs++);
        finalBlockArg = &*(finalBlockArgs++);
        doBlockArg->setName(outputSet.ssName);
        finalBlockArg->setName(outputSet.ssName);
    }
    iBuilder->setModule(saveModule);
    iBuilder->restoreIP(savePoint);
}


Value * KernelInterface::createInstance(std::vector<Value *> args) {
    Value * kernelInstance = iBuilder->CreateAlloca(mKernelStateType);
    Module * m = iBuilder->getModule();
    std::vector<Value *> init_args = {kernelInstance};
    for (auto a : args) {
        init_args.push_back(a);
    }
    std::string initFnName = mKernelName + init_suffix;
    Function * initMethod = m->getFunction(initFnName);
    if (!initMethod) {
        throw std::runtime_error("Cannot find " + initFnName);
        //Or just zero-initialize???
        //iBuilder->CreateStore(Constant::getNullValue(mKernelStateType), kernelInstance);
        //return kernelInstance;
    }
    iBuilder->CreateCall(initMethod, init_args);
    return kernelInstance;
}

Value * KernelInterface::createDoBlockCall(Value * self, std::vector<Value *> streamSets) {
    Module * m = iBuilder->getModule();
    std::string doBlockName = mKernelName + doBlock_suffix;
    Function * doBlockMethod = m->getFunction(doBlockName);
    if (!doBlockMethod) {
        throw std::runtime_error("Cannot find " + doBlockName);
    }
    std::vector<Value *> args = {self};
    for (auto ss : streamSets) {
        args.push_back(ss);
    }
    return iBuilder->CreateCall(doBlockMethod, args);
}

Value * KernelInterface::createFinalBlockCall(Value * self, Value * remainingBytes, std::vector<Value *> streamSets) {
    Module * m = iBuilder->getModule();
    std::string finalBlockName = mKernelName + finalBlock_suffix;
    Function * finalBlockMethod = m->getFunction(finalBlockName);
    if (!finalBlockMethod) {
        throw std::runtime_error("Cannot find " + finalBlockName);
    }
    std::vector<Value *> args = {self, remainingBytes};
    for (auto ss : streamSets) {
        args.push_back(ss);
    }
    return iBuilder->CreateCall(finalBlockMethod, args);
}

Value * KernelInterface::createGetAccumulatorCall(Value * self, std::string accumName) {
    Module * m = iBuilder->getModule();
    std::string fnName = mKernelName + accumulator_infix + accumName;
    Function * accumMethod = m->getFunction(fnName);
    if (!accumMethod) {
        throw std::runtime_error("Cannot find " + fnName);
    }
    return iBuilder->CreateCall(accumMethod, {self});
}


