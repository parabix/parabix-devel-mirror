/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "interface.h"
#include <llvm/IR/Value.h>         // for Value
#include <llvm/IR/CallingConv.h>   // for ::C
#include <llvm/IR/DerivedTypes.h>  // for FunctionType (ptr only), PointerType
#include <llvm/IR/Function.h>      // for Function, Function::arg_iterator
#include <llvm/IR/Module.h>
#include <IR_Gen/idisa_builder.h>
namespace llvm { class Module; }
namespace llvm { class Type; }

using namespace llvm;

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
    std::vector<Type *> doSegmentParameters = {selfType, iBuilder->getInt1Ty()};
    for (auto ss : mStreamSetInputs) {
        doSegmentParameters.push_back(iBuilder->getSizeTy());
    }
    FunctionType * doSegmentFunctionType = FunctionType::get(iBuilder->getVoidTy(), doSegmentParameters, false);
    std::string doSegmentName = mKernelName + doSegment_suffix;
    Function * doSegmentFn = Function::Create(doSegmentFunctionType, GlobalValue::ExternalLinkage, doSegmentName, client);
    doSegmentFn->setCallingConv(CallingConv::C);
    doSegmentFn->setDoesNotThrow();
    Function::arg_iterator args = doSegmentFn->arg_begin();
    Value * arg = &*(args++);
    arg->setName("self");
    arg = &*(args++);
    arg->setName("doFinal");
    for (auto ss : mStreamSetInputs) {
        arg = &*(args++);
        arg->setName(ss.name + "_availableItems");
    }
    doSegmentFn->setDoesNotCapture(1); // for self parameter only.
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


Value * KernelInterface::createDoSegmentCall(std::vector<Value *> args) const {
    Module * m = iBuilder->getModule();
    std::string fnName = mKernelName + doSegment_suffix;
    Function * method = m->getFunction(fnName);
    if (!method) {
        throw std::runtime_error("Cannot find " + fnName);
    }
    return iBuilder->CreateCall(method, args);
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


