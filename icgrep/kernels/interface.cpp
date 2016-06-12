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
    llvm::errs() << "0\n";
    iBuilder = builder;
    mKernelName = kernelName;
    mStreamSetInputs = stream_inputs;
    mStreamSetOutputs = stream_outputs;
    mScalarInputs = scalar_parameters;
    mScalarOutputs = scalar_outputs;
    mInternalScalars = internal_scalars;
    llvm::errs() << "1\n";
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
    llvm::errs() << "2\n";

}

void KernelInterface::addKernelDeclarations(Module * client) {
    
    Type * selfType = PointerType::getUnqual(mKernelStateType);
    llvm::errs() << "3\n";
    
    // Create the accumulator get function prototypes
    for (auto binding : mScalarOutputs) {
        FunctionType * accumFnType = FunctionType::get(binding.scalarType, {selfType}, false);
        Function::Create(accumFnType, GlobalValue::ExternalLinkage, mKernelName + "_get_" + binding.scalarName, client);
    }
    llvm::errs() << "4\n";

    // Create the initialization function prototype

    std::vector<Type *> initParameters = {selfType};
    for (auto binding : mScalarInputs) {
        initParameters.push_back(binding.scalarType);
    }
    FunctionType * initFunctionType = FunctionType::get(iBuilder->getVoidTy(), initParameters, false);
    Function::Create(initFunctionType, GlobalValue::ExternalLinkage, mKernelName + "_Init", client);
    
    

    llvm::errs() << "5\n";

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
    Function::Create(doBlockFunctionType, GlobalValue::ExternalLinkage, mKernelName + "_DoBlock", client);
    FunctionType * finalBlockFunctionType = FunctionType::get(iBuilder->getVoidTy(), finalBlockParameters, false);
    Function::Create(finalBlockFunctionType, GlobalValue::ExternalLinkage, mKernelName + "_FinalBlock", client);
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
        const auto f = mInternalStateNameMap.find(binding.scalarName);
        if (LLVM_UNLIKELY(f == mInternalStateNameMap.end())) {
            throw std::runtime_error("Kernel does not contain internal state " + binding.scalarName);
        }
        Value * idx = f->second;
        Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), idx});
        Value * retVal = iBuilder->CreateLoad(ptr);
        iBuilder->CreateRet(retVal);
    }
    
    // Implement the initializer function
    Function * initFunction = theModule->getFunction(mKernelName + "_Init");
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "Init_entry", initFunction, 0));
    
    Function::arg_iterator args = initFunction->arg_begin();
    Value * self = &*(args++);
    self->setName("self");
    for (auto binding : mScalarInputs) {
        Value * parm = &*(args++);
        parm->setName(binding.scalarName);
    }

    iBuilder->CreateStore(Constant::getNullValue(mKernelStateType), self);
    args = initFunction->arg_begin();
    args++;   // skip self argument.
    for (auto binding : mScalarInputs) {
        Value * parm = &*(args++);
        const auto f = mInternalStateNameMap.find(binding.scalarName);
        if (LLVM_UNLIKELY(f == mInternalStateNameMap.end())) {
            throw std::runtime_error("Kernel does not contain internal state " + binding.scalarName);
        }
        Value * idx = f->second;
        Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), idx});
        iBuilder->CreateStore(ptr, parm);
    }
    iBuilder->CreateRetVoid();
    return theModule;
}
