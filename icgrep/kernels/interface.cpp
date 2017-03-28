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

ProcessingRate FixedRatio(unsigned strmItemsPer, unsigned perPrincipalInputItems, std::string referenceStreamSet) {
    return ProcessingRate(ProcessingRate::ProcessingRateKind::Fixed, strmItemsPer, perPrincipalInputItems, referenceStreamSet);
}

ProcessingRate MaxRatio(unsigned strmItemsPer, unsigned perPrincipalInputItems, std::string referenceStreamSet) {
    return ProcessingRate(ProcessingRate::ProcessingRateKind::Max, strmItemsPer, perPrincipalInputItems, referenceStreamSet);
}

ProcessingRate RoundUpToMultiple(unsigned itemMultiple, std::string referenceStreamSet) {
    return ProcessingRate(ProcessingRate::ProcessingRateKind::RoundUp, itemMultiple, itemMultiple, referenceStreamSet);
}

ProcessingRate Add1(std::string referenceStreamSet) {
    return ProcessingRate(ProcessingRate::ProcessingRateKind::Add1, 0, 0, referenceStreamSet);
}

ProcessingRate UnknownRate() {
    return ProcessingRate(ProcessingRate::ProcessingRateKind::Unknown, 0, 0, "");
}

Value * ProcessingRate::CreateRatioCalculation(IDISA::IDISA_Builder * b, Value * principalInputItems, Value * doFinal) const {
    Type * T = principalInputItems->getType();
    if (mKind == ProcessingRate::ProcessingRateKind::Fixed || mKind == ProcessingRate::ProcessingRateKind::Max) {
        Value * strmItems = (ratio_numerator == 1) ? principalInputItems : b->CreateMul(principalInputItems, ConstantInt::get(T, ratio_numerator)); 
        if (ratio_denominator == 1) return strmItems;
        return b->CreateUDiv(b->CreateAdd(ConstantInt::get(T, ratio_denominator - 1), strmItems), ConstantInt::get(T, ratio_denominator)); 
    }
    if (mKind == ProcessingRate::ProcessingRateKind::RoundUp) {
        Constant * multiple = ConstantInt::get(T, ratio_denominator);
        Constant * multipleLess1 = ConstantInt::get(T, ratio_denominator - 1);
        return b->CreateMul(b->CreateUDiv(b->CreateAdd(principalInputItems, multipleLess1), multiple), multiple);
    }
    if (mKind == ProcessingRate::ProcessingRateKind::Add1) {
        return b->CreateAdd(principalInputItems, b->CreateZExt(doFinal, principalInputItems->getType()));
    }
    return nullptr;
}

Value * ProcessingRate::CreateRatioCalculation(IDISA::IDISA_Builder * b, Value * principalInputItems) const {
    Type * T = principalInputItems->getType();
    if (mKind == ProcessingRate::ProcessingRateKind::Fixed || mKind == ProcessingRate::ProcessingRateKind::Max) {
        Value * strmItems = (ratio_numerator == 1) ? principalInputItems : b->CreateMul(principalInputItems, ConstantInt::get(T, ratio_numerator)); 
        if (ratio_denominator == 1) return strmItems;
        return b->CreateUDiv(b->CreateAdd(ConstantInt::get(T, ratio_denominator - 1), strmItems), ConstantInt::get(T, ratio_denominator)); 
    }
    if (mKind == ProcessingRate::ProcessingRateKind::RoundUp) {
        Constant * multiple = ConstantInt::get(T, ratio_denominator);
        Constant * multipleLess1 = ConstantInt::get(T, ratio_denominator - 1);
        return b->CreateMul(b->CreateUDiv(b->CreateAdd(principalInputItems, multipleLess1), multiple), multiple);
    }
    if (mKind == ProcessingRate::ProcessingRateKind::Add1) {
        return principalInputItems;
    }
    return nullptr;
}

void KernelInterface::addKernelDeclarations(Module * client) {
    Module * saveModule = iBuilder->getModule();
    auto savePoint = iBuilder->saveIP();
    iBuilder->setModule(client);
    if (mKernelStateType == nullptr) {
        throw std::runtime_error("Kernel interface " + getName() + " not yet finalized.");
    }
    PointerType * selfType = PointerType::getUnqual(mKernelStateType);

    // Create the initialization function prototype
    std::vector<Type *> initParameters = {selfType};
    for (auto binding : mScalarInputs) {
        initParameters.push_back(binding.type);
    }
    FunctionType * initType = FunctionType::get(iBuilder->getVoidTy(), initParameters, false);
    Function * init = Function::Create(initType, GlobalValue::ExternalLinkage, getName() + INIT_SUFFIX, client);
    init->setCallingConv(CallingConv::C);
    init->setDoesNotThrow();
    auto args = init->arg_begin();
    args->setName("self");
    for (auto binding : mScalarInputs) {
        (++args)->setName(binding.name);
    }

    // Create the doSegment function prototype.
    std::vector<Type *> doSegmentParameters = {selfType, iBuilder->getInt1Ty()};
    for (unsigned i = 0; i < mStreamSetInputs.size(); ++i) {
        doSegmentParameters.push_back(iBuilder->getSizeTy());
    }
    FunctionType * doSegmentType = FunctionType::get(iBuilder->getVoidTy(), doSegmentParameters, false);
    Function * doSegment = Function::Create(doSegmentType, GlobalValue::ExternalLinkage, getName() + DO_SEGMENT_SUFFIX, client);
    doSegment->setCallingConv(CallingConv::C);
    doSegment->setDoesNotThrow();
    doSegment->setDoesNotCapture(1); // for self parameter only.
    args = doSegment->arg_begin();
    args->setName("self");
    (++args)->setName("doFinal");
    for (auto ss : mStreamSetInputs) {
        (++args)->setName(ss.name + "_availableItems");
    }

    // Create the accumulator get function prototypes
    for (const auto & binding : mScalarOutputs) {
        FunctionType * accumFnType = FunctionType::get(binding.type, {selfType}, false);
        Function * accumFn = Function::Create(accumFnType, GlobalValue::ExternalLinkage, getName() + ACCUMULATOR_INFIX + binding.name, client);
        accumFn->setCallingConv(CallingConv::C);
        accumFn->setDoesNotThrow();
        accumFn->setDoesNotCapture(1);
        auto args = accumFn->arg_begin();
        args->setName("self");
    }

    iBuilder->setModule(saveModule);
    iBuilder->restoreIP(savePoint);
}

void KernelInterface::setInitialArguments(std::vector<Value *> args) {
    mInitialArguments = args;
}

llvm::Function * KernelInterface::getAccumulatorFunction(const std::string & accumName) const {
    const auto name = getName() + ACCUMULATOR_INFIX + accumName;
    Function * f = iBuilder->getModule()->getFunction(name);
    if (LLVM_UNLIKELY(f == nullptr)) {
        llvm::report_fatal_error("Cannot find " + name);
    }
    return f;
}

Function * KernelInterface::getInitFunction() const {
    const auto name = getName() + INIT_SUFFIX;
    Function * f = iBuilder->getModule()->getFunction(name);
    if (LLVM_UNLIKELY(f == nullptr)) {
        llvm::report_fatal_error("Cannot find " + name);
    }
    return f;
}

Function * KernelInterface::getDoSegmentFunction() const {
    const auto name = getName() + DO_SEGMENT_SUFFIX;
    Function * f = iBuilder->getModule()->getFunction(name);
    if (LLVM_UNLIKELY(f == nullptr)) {
        llvm::report_fatal_error("Cannot find " + name);
    }
    return f;
}
