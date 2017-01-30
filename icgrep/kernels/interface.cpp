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

static const auto INIT_SUFFIX = "_Init";

static const auto DO_SEGMENT_SUFFIX = "_DoSegment";

static const auto ACCUMULATOR_INFIX = "_get_";

using namespace llvm;

void KernelInterface::addKernelDeclarations(Module * client) const {
    Module * saveModule = iBuilder->getModule();
    auto savePoint = iBuilder->saveIP();
    iBuilder->setModule(client);
    if (mKernelStateType == nullptr) {
        throw std::runtime_error("Kernel interface " + mKernelName + " not yet finalized.");
    }
    PointerType * selfType = PointerType::getUnqual(mKernelStateType);

    // Create the accumulator get function prototypes
    for (auto binding : mScalarOutputs) {
        FunctionType * accumFnType = FunctionType::get(binding.type, {selfType}, false);
        std::string fnName = mKernelName + ACCUMULATOR_INFIX + binding.name;
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
    std::string initFnName = mKernelName + INIT_SUFFIX;
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

    // Create the doSegment function prototype.
    std::vector<Type *> doSegmentParameters = {selfType, iBuilder->getInt1Ty()};
    for (auto ss : mStreamSetInputs) {
        doSegmentParameters.push_back(iBuilder->getSizeTy());
    }
    FunctionType * doSegmentFunctionType = FunctionType::get(iBuilder->getVoidTy(), doSegmentParameters, false);
    std::string doSegmentName = mKernelName + DO_SEGMENT_SUFFIX;
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

    // Add any additional kernel declarations
    addAdditionalKernelDeclarations(client, selfType);

    iBuilder->setModule(saveModule);
    iBuilder->restoreIP(savePoint);
}

void KernelInterface::addAdditionalKernelDeclarations(llvm::Module * module, llvm::PointerType * selfType) const {

}

void KernelInterface::setInitialArguments(std::vector<Value *> args) {
    mInitialArguments = args;
}

llvm::Function * KernelInterface::getAccumulatorFunction(const std::string & accumName) const {
    const auto name = mKernelName + ACCUMULATOR_INFIX + accumName;
    Function * f = iBuilder->getModule()->getFunction(name);
    if (LLVM_UNLIKELY(f == nullptr)) {
        llvm::report_fatal_error("Cannot find " + name);
    }
    return f;
}

Function * KernelInterface::getInitFunction() const {
    const auto name = mKernelName + INIT_SUFFIX;
    Function * f = iBuilder->getModule()->getFunction(name);
    if (LLVM_UNLIKELY(f == nullptr)) {
        llvm::report_fatal_error("Cannot find " + name);
    }
    return f;
}

Function * KernelInterface::getDoSegmentFunction() const {
    const auto name = mKernelName + DO_SEGMENT_SUFFIX;
    Function * f = iBuilder->getModule()->getFunction(name);
    if (LLVM_UNLIKELY(f == nullptr)) {
        llvm::report_fatal_error("Cannot find " + name);
    }
    return f;
}
