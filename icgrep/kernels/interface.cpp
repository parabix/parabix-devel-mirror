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

KernelInterface::KernelInterface(IDISA::IDISA_Builder * builder,
                                 std::string kernelName,
                                 std::vector<StreamSetBinding> stream_inputs,
                                 std::vector<StreamSetBinding> stream_outputs,
                                 std::vector<ScalarBinding> scalar_parameters,
                                 std::vector<ScalarBinding> scalar_outputs,
                                 std::vector<ScalarBinding> internal_scalars) {
    iBuilder = builder;
    mKernelName = kernelName;
    mStreamSetInputs = stream_inputs;
    mStreamSetOutputs = stream_outputs;
    mScalarInputs = scalar_parameters;
    mScalarOutputs = scalar_outputs;
    mInternalScalars = internal_scalars;
    std::vector<Type *> kernelFields;
    for (auto binding : scalar_parameters) {
        unsigned index = kernelFields.size();
        kernelFields.push_back(binding.scalarType);
        mInternalStateNameMap.emplace(binding.scalarName, iBuilder->getInt32(index));
    }
    for (auto binding : scalar_outputs) {
        unsigned index = kernelFields.size();
        kernelFields.push_back(binding.scalarType);
        mInternalStateNameMap.emplace(binding.scalarName, iBuilder->getInt32(index));
    }
    for (auto binding : internal_scalars) {
        unsigned index = kernelFields.size();
        kernelFields.push_back(binding.scalarType);
        mInternalStateNameMap.emplace(binding.scalarName, iBuilder->getInt32(index));
    }
    mKernelStateType = StructType::create(getGlobalContext(), kernelFields, kernelName);
}

void KernelInterface::addKernelDeclarations(Module * client) {
    
    Type * selfType = PointerType::getUnqual(mKernelStateType);
    // Create the accumulator get function prototypes
    for (auto binding : mScalarOutputs) {
        FunctionType * accumFnType = FunctionType::get(binding.scalarType, {selfType}, false);
        Function * accumFn = Function::Create(accumFnType, GlobalValue::ExternalLinkage, mKernelName + "_get_" + binding.scalarName, client);
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
    Function * initFn = Function::Create(initFunctionType, GlobalValue::ExternalLinkage, mKernelName + "_Init", client);
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
    FunctionType * doBlockFunctionType = FunctionType::get(iBuilder->getVoidTy(), doBlockParameters, false);
    Function * doBlockFn = Function::Create(doBlockFunctionType, GlobalValue::ExternalLinkage, mKernelName + "_DoBlock", client);
    doBlockFn->setCallingConv(CallingConv::C);
    doBlockFn->setDoesNotThrow();
    for (int i = 1; i <= doBlockParameters.size(); i++) {
        doBlockFn->setDoesNotCapture(i);
    }
    
    FunctionType * finalBlockFunctionType = FunctionType::get(iBuilder->getVoidTy(), finalBlockParameters, false);
    Function * finalBlockFn = Function::Create(finalBlockFunctionType, GlobalValue::ExternalLinkage, mKernelName + "_FinalBlock", client);
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
}


std::unique_ptr<Module> KernelInterface::createKernelModule() {
    std::unique_ptr<Module> theModule = llvm::make_unique<Module>(mKernelName, getGlobalContext());
    addKernelDeclarations(theModule.get());
    
    // Implement the accumulator get functions
    for (auto binding : mScalarOutputs) {
        auto fnName = mKernelName + "_get_" + binding.scalarName;
        Function * accumFn = theModule->getFunction(fnName);
        iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "get_" + binding.scalarName, accumFn, 0));
        Value * self = &*(accumFn->arg_begin());
        Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), getScalarIndex(binding.scalarName)});
        Value * retVal = iBuilder->CreateLoad(ptr);
        iBuilder->CreateRet(retVal);
    }
    
    // Implement the initializer function
    Function * initFunction = theModule->getFunction(mKernelName + "_Init");
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "Init_entry", initFunction, 0));
    
    Function::arg_iterator args = initFunction->arg_begin();
    Value * self = &*(args++);
    iBuilder->CreateStore(Constant::getNullValue(mKernelStateType), self);
    for (auto binding : mScalarInputs) {
        Value * parm = &*(args++);
        Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), getScalarIndex(binding.scalarName)});
        iBuilder->CreateStore(ptr, parm);
    }
    iBuilder->CreateRetVoid();
    return theModule;
}

llvm::Value * KernelInterface::getScalarIndex(std::string fieldName) {
    const auto f = mInternalStateNameMap.find(fieldName);
    if (LLVM_UNLIKELY(f == mInternalStateNameMap.end())) {
        throw std::runtime_error("Kernel does not contain internal state: " + fieldName);
    }
    return f->second;
}


llvm::Value * KernelInterface::getParameter(Function * f, std::string paramName) {
    for (Function::arg_iterator argIter = f->arg_begin(), end = f->arg_end(); argIter != end; argIter++) {
        Value * arg = &*argIter;
        if (arg->getName() == paramName) return arg;
    }
    throw std::runtime_error("Method does not have parameter: " + paramName);
}

