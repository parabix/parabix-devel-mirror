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
#include <kernels/kernel_builder.h>


using namespace llvm;

namespace kernel {

const static auto INIT_SUFFIX = "_Init";
const static auto DO_SEGMENT_SUFFIX = "_DoSegment";
const static auto TERMINATE_SUFFIX = "_Terminate";

void KernelInterface::addKernelDeclarations(const std::unique_ptr<kernel::KernelBuilder> & idb) {

    if (mKernelStateType == nullptr) {
        throw std::runtime_error("Kernel interface " + getName() + " not yet finalized.");
    }

    Module * const module = idb->getModule();
    PointerType * const selfType = mKernelStateType->getPointerTo();
    IntegerType * const sizeTy = idb->getSizeTy();
    PointerType * const consumerTy = StructType::get(idb->getContext(), {sizeTy, sizeTy->getPointerTo()->getPointerTo()})->getPointerTo();
    Type * const voidTy = idb->getVoidTy();

    // Create the initialization function prototype
    std::vector<Type *> initParameters = {selfType};
    for (auto binding : mScalarInputs) {
        initParameters.push_back(binding.getType());
    }
    initParameters.insert(initParameters.end(), mStreamSetOutputs.size(), consumerTy);

    FunctionType * const initType = FunctionType::get(voidTy, initParameters, false);
    Function * const initFunc = Function::Create(initType, GlobalValue::ExternalLinkage, getName() + INIT_SUFFIX, module);
    initFunc->setCallingConv(CallingConv::C);
    initFunc->setDoesNotThrow();
    auto args = initFunc->arg_begin();
    args->setName("self");
    for (const Binding & binding : mScalarInputs) {
        (++args)->setName(binding.getName());
    }
    for (const Binding & binding : mStreamSetOutputs) {
        (++args)->setName(binding.getName() + "ConsumerLocks");
    }

    // Create the doSegment function prototype.
    std::vector<Type *> params = {selfType, idb->getInt1Ty()};

    const auto count = mStreamSetInputs.size();
    params.insert(params.end(), count, sizeTy);

    FunctionType * const doSegmentType = FunctionType::get(voidTy, params, false);
    Function * const doSegment = Function::Create(doSegmentType, GlobalValue::ExternalLinkage, getName() + DO_SEGMENT_SUFFIX, module);
    doSegment->setCallingConv(CallingConv::C);
    doSegment->setDoesNotThrow();
    args = doSegment->arg_begin();
    args->setName("self");
    (++args)->setName("isFinal");
    for (const Binding & input : mStreamSetInputs) {
        (++args)->setName(input.getName() + "AvailableItems");
    }

    // Create the terminate function prototype
    Type * resultType = nullptr;
    if (mScalarOutputs.empty()) {
        resultType = idb->getVoidTy();
    } else {
        const auto n = mScalarOutputs.size();
        Type * outputType[n];
        for (unsigned i = 0; i < n; ++i) {
            outputType[i] = mScalarOutputs[i].getType();
        }
        if (n == 1) {
            resultType = outputType[0];
        } else {
            resultType = StructType::get(idb->getContext(), ArrayRef<Type *>(outputType, n));
        }
    }
    FunctionType * const terminateType = FunctionType::get(resultType, {selfType}, false);
    Function * const terminateFunc = Function::Create(terminateType, GlobalValue::ExternalLinkage, getName() + TERMINATE_SUFFIX, module);
    terminateFunc->setCallingConv(CallingConv::C);
    terminateFunc->setDoesNotThrow();
    args = terminateFunc->arg_begin();
    args->setName("self");

    linkExternalMethods(idb);
}

void  KernelInterface::setInstance(Value * const instance) {
    assert ("kernel instance cannot be null!" && instance);
    assert ("kernel instance must point to a valid kernel state type!" && (instance->getType()->getPointerElementType() == mKernelStateType));
    mKernelInstance = instance;
}

Function * KernelInterface::getInitFunction(Module * const module) const {
    const auto name = getName() + INIT_SUFFIX;
    Function * f = module->getFunction(name);
    if (LLVM_UNLIKELY(f == nullptr)) {
        llvm::report_fatal_error("Cannot find " + name);
    }
    return f;
}

Function * KernelInterface::getDoSegmentFunction(Module * const module) const {
    const auto name = getName() + DO_SEGMENT_SUFFIX;
    Function * f = module->getFunction(name);
    if (LLVM_UNLIKELY(f == nullptr)) {
        llvm::report_fatal_error("Cannot find " + name);
    }
    return f;
}

Function * KernelInterface::getTerminateFunction(Module * const module) const {
    const auto name = getName() + TERMINATE_SUFFIX;
    Function * f = module->getFunction(name);
    if (LLVM_UNLIKELY(f == nullptr)) {
        llvm::report_fatal_error("Cannot find " + name);
    }
    return f;
}

CallInst * KernelInterface::makeDoSegmentCall(kernel::KernelBuilder & idb, const std::vector<llvm::Value *> & args) const {
    Function * const doSegment = getDoSegmentFunction(idb.getModule());
    assert (doSegment->getArgumentList().size() <= args.size());
    return idb.CreateCall(doSegment, args);
}

}
