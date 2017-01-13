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
using namespace parabix;

void KernelInterface::addKernelDeclarations(Module * client) {
    Module * saveModule = iBuilder->getModule();
    auto savePoint = iBuilder->saveIP();
    iBuilder->setModule(client);
    if (mKernelStateType == nullptr) {
        throw std::runtime_error("Kernel interface " + mKernelName + " not yet finalized.");
    }
    Type * selfType = PointerType::getUnqual(mKernelStateType);

    // Create the accumulator get function prototypes
    for (auto binding : mScalarOutputs) {
        FunctionType * accumFnType = FunctionType::get(binding.type, {selfType}, false);
        std::string fnName = mKernelName + accumulator_infix + binding.name;
        Function * accumFn = Function::Create(accumFnType, GlobalValue::ExternalLinkage, fnName, client);
        accumFn->setCallingConv(CallingConv::C);
        accumFn->setDoesNotThrow();
        Value * self = &*(accumFn->arg_begin());
        self->setName("self");        
    }

    // Create the initialization function prototype
    std::vector<Type *> initParameters = {selfType};
    for (auto binding : mScalarInputs) {
        initParameters.push_back(binding.type);
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
        initArg->setName(binding.name);
    }

    // Create the doBlock and finalBlock function prototypes
    
    std::vector<Type *> doBlockParameters = {selfType};
    std::vector<Type *> finalBlockParameters = {selfType, iBuilder->getSizeTy()};
    FunctionType * doBlockFunctionType = FunctionType::get(iBuilder->getVoidTy(), doBlockParameters, false);
    std::string doBlockName = mKernelName + doBlock_suffix;
    Function * doBlockFn = Function::Create(doBlockFunctionType, GlobalValue::ExternalLinkage, doBlockName, client);
    doBlockFn->setCallingConv(CallingConv::C);
    doBlockFn->setDoesNotThrow();
    doBlockFn->setDoesNotCapture(1);
    
    FunctionType * finalBlockType = FunctionType::get(iBuilder->getVoidTy(), finalBlockParameters, false);
    std::string finalBlockName = mKernelName + finalBlock_suffix;
    Function * finalBlockFn = Function::Create(finalBlockType, GlobalValue::ExternalLinkage, finalBlockName, client);
    finalBlockFn->setCallingConv(CallingConv::C);
    finalBlockFn->setDoesNotThrow();
    finalBlockFn->setDoesNotCapture(1);
    
    Function::arg_iterator doBlockArgs = doBlockFn->arg_begin();
    Function::arg_iterator finalBlockArgs = finalBlockFn->arg_begin();
    Value * doBlockArg = &*(doBlockArgs++);
    doBlockArg->setName("self");
    Value * finalBlockArg = &*(finalBlockArgs++);
    finalBlockArg->setName("self");
    finalBlockArg = &*(finalBlockArgs++);
    finalBlockArg->setName("remainingBytes");

    // Create the doSegment function prototype.
    std::vector<Type *> doSegmentParameters = {selfType, iBuilder->getSizeTy()};
    FunctionType * doSegmentFunctionType = FunctionType::get(iBuilder->getVoidTy(), doSegmentParameters, false);
    std::string doSegmentName = mKernelName + doSegment_suffix;
    Function * doSegmentFn = Function::Create(doSegmentFunctionType, GlobalValue::ExternalLinkage, doSegmentName, client);
    doSegmentFn->setCallingConv(CallingConv::C);
    doSegmentFn->setDoesNotThrow();
    Function::arg_iterator args = doSegmentFn->arg_begin();
    Value * arg = &*(args++);
    arg->setName("self");
    arg = &*(args++);
    arg->setName("blockCnt");
    doSegmentFn->setDoesNotCapture(1); // for self parameter only.
    //
    // Create the finalSegment function prototype.
    std::vector<Type *> finalSegmentParameters = {selfType, iBuilder->getSizeTy()};
    FunctionType * finalSegmentFunctionType = FunctionType::get(iBuilder->getVoidTy(), finalSegmentParameters, false);
    std::string finalSegmentName = mKernelName + finalSegment_suffix;
    Function * finalSegmentFn = Function::Create(finalSegmentFunctionType, GlobalValue::ExternalLinkage, finalSegmentName, client);
    finalSegmentFn->setCallingConv(CallingConv::C);
    finalSegmentFn->setDoesNotThrow();
    Function::arg_iterator finalSegmentArgs = finalSegmentFn->arg_begin();
    Value * finalSegmentArg = &*(finalSegmentArgs++);
    finalSegmentArg->setName("self");
    finalSegmentArg = &*(finalSegmentArgs++);
    finalSegmentArg->setName("blockCnt");
    finalSegmentFn->setDoesNotCapture(1); // for self parameter only.
    iBuilder->setModule(saveModule);
    iBuilder->restoreIP(savePoint);
}

void KernelInterface::setInitialArguments(std::vector<Value *> args) {
    mInitialArguments = args;
}

Value * KernelInterface::createDoBlockCall(Value * self) const {
    Module * m = iBuilder->getModule();
    std::string doBlockName = mKernelName + doBlock_suffix;
    Function * doBlockMethod = m->getFunction(doBlockName);
    if (!doBlockMethod) {
        throw std::runtime_error("Cannot find " + doBlockName);
    }
    std::vector<Value *> args = {self};
    return iBuilder->CreateCall(doBlockMethod, args);
}

Value * KernelInterface::createFinalBlockCall(Value * self, Value * remainingBytes) const {
    Module * m = iBuilder->getModule();
    std::string finalBlockName = mKernelName + finalBlock_suffix;
    Function * finalBlockMethod = m->getFunction(finalBlockName);
    if (!finalBlockMethod) {
        throw std::runtime_error("Cannot find " + finalBlockName);
    }
    std::vector<Value *> args = {self, remainingBytes};
    return iBuilder->CreateCall(finalBlockMethod, args);
}


Value * KernelInterface::createDoSegmentCall(Value * self, Value * blksToDo) const {
    Module * m = iBuilder->getModule();
    std::string fnName = mKernelName + doSegment_suffix;
    Function * method = m->getFunction(fnName);
    if (!method) {
        throw std::runtime_error("Cannot find " + fnName);
    }
    return iBuilder->CreateCall(method, {self, blksToDo});
}

Value * KernelInterface::createFinalSegmentCall(Value * self, Value * blksToDo) const {
    Module * m = iBuilder->getModule();
    std::string fnName = mKernelName + finalSegment_suffix;
    Function * method = m->getFunction(fnName);
    if (!method) {
        throw std::runtime_error("Cannot find " + fnName);
    }
    return iBuilder->CreateCall(method, {self, blksToDo});
}

Value * KernelInterface::createGetAccumulatorCall(Value * self, std::string accumName) const {
    Module * m = iBuilder->getModule();
    std::string fnName = mKernelName + accumulator_infix + accumName;
    Function * accumMethod = m->getFunction(fnName);
    if (!accumMethod) {
        throw std::runtime_error("Cannot find " + fnName);
    }
    return iBuilder->CreateCall(accumMethod, {self});
}


