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

static const auto INIT_SUFFIX = "_Init";

static const auto DO_SEGMENT_SUFFIX = "_DoSegment";

static const auto TERMINATE_SUFFIX = "_Terminate";

using namespace llvm;

ProcessingRate FixedRatio(unsigned strmItems, unsigned referenceItems, std::string && referenceStreamSet) {
    return ProcessingRate(ProcessingRate::ProcessingRateKind::FixedRatio, strmItems, referenceItems, std::move(referenceStreamSet));
}

ProcessingRate MaxRatio(unsigned strmItems, unsigned referenceItems, std::string && referenceStreamSet) {
    return ProcessingRate(ProcessingRate::ProcessingRateKind::MaxRatio, strmItems, referenceItems, std::move(referenceStreamSet));
}

ProcessingRate RoundUpToMultiple(unsigned itemMultiple, std::string && referenceStreamSet) {
    return ProcessingRate(ProcessingRate::ProcessingRateKind::RoundUp, itemMultiple, itemMultiple, std::move(referenceStreamSet));
}

ProcessingRate Add1(std::string && referenceStreamSet) {
    return ProcessingRate(ProcessingRate::ProcessingRateKind::Add1, 0, 0, std::move(referenceStreamSet));
}

ProcessingRate UnknownRate() {
    return ProcessingRate(ProcessingRate::ProcessingRateKind::Unknown, 0, 0, "");
}

Value * ProcessingRate::CreateRatioCalculation(IDISA::IDISA_Builder * const b, Value * referenceItems, Value * doFinal) const {
    if (mKind == ProcessingRate::ProcessingRateKind::FixedRatio || mKind == ProcessingRate::ProcessingRateKind::MaxRatio) {
        if (mRatioNumerator == mRatioDenominator) {
            return referenceItems;
        }
        Type * const T = referenceItems->getType();
        Constant * const numerator = ConstantInt::get(T, mRatioNumerator);
        Constant * const denominator = ConstantInt::get(T, mRatioDenominator);
        Constant * const denominatorLess1 = ConstantInt::get(T, mRatioDenominator - 1);
        Value * strmItems = b->CreateMul(referenceItems, numerator);
        return b->CreateUDiv(b->CreateAdd(denominatorLess1, strmItems), denominator);
    }
    if (mKind == ProcessingRate::ProcessingRateKind::RoundUp) {
        Type * const T = referenceItems->getType();
        Constant * const denominator = ConstantInt::get(T, mRatioDenominator);
        Constant * const denominatorLess1 = ConstantInt::get(T, mRatioDenominator - 1);
        return b->CreateMul(b->CreateUDiv(b->CreateAdd(referenceItems, denominatorLess1), denominator), denominator);
    }
    if (mKind == ProcessingRate::ProcessingRateKind::Add1) {
        if (doFinal) {
            Type * const T = referenceItems->getType();
            referenceItems = b->CreateAdd(referenceItems, b->CreateZExt(doFinal, T));
        }
        return referenceItems;
    }
    return nullptr;
}

Value * ProcessingRate::CreateMaxReferenceItemsCalculation(IDISA::IDISA_Builder * const b, Value * outputItems, Value * doFinal) const {
    if (mKind == ProcessingRate::ProcessingRateKind::FixedRatio) {
        if (mRatioNumerator == mRatioDenominator) {
            return outputItems;
        }
        Type * const T = outputItems->getType();
        Constant * const numerator = ConstantInt::get(T, mRatioNumerator);
        Constant * const denominator = ConstantInt::get(T, mRatioDenominator);
        return b->CreateMul(b->CreateUDiv(outputItems, numerator), denominator);
    }
    if (mKind == ProcessingRate::ProcessingRateKind::RoundUp) {
        Type * const T = outputItems->getType();
        Constant * const denominator = ConstantInt::get(T, mRatioDenominator);
        return b->CreateMul(b->CreateUDiv(outputItems, denominator), denominator);
    }
    if (mKind == ProcessingRate::ProcessingRateKind::Add1) {
        Type * const T = outputItems->getType();
        if (doFinal) {
            return b->CreateSub(outputItems, b->CreateZExt(doFinal, T));
        }
        return b->CreateSub(outputItems, ConstantInt::get(T, 1));
    }
    return nullptr;
}

void KernelInterface::addKernelDeclarations() {

    if (mKernelStateType == nullptr) {
        throw std::runtime_error("Kernel interface " + getName() + " not yet finalized.");
    }

    Module * const module = iBuilder->getModule();
    PointerType * const selfType = mKernelStateType->getPointerTo();
    IntegerType * const sizeTy = iBuilder->getSizeTy();
    PointerType * const consumerTy = StructType::get(sizeTy, sizeTy->getPointerTo()->getPointerTo(), nullptr)->getPointerTo();
    Type * const voidTy = iBuilder->getVoidTy();

    // Create the initialization function prototype
    std::vector<Type *> initParameters = {selfType};
    for (auto binding : mScalarInputs) {
        initParameters.push_back(binding.type);
    }
    initParameters.insert(initParameters.end(), mStreamSetOutputs.size(), consumerTy);

    FunctionType * const initType = FunctionType::get(voidTy, initParameters, false);
    Function * const initFunc = Function::Create(initType, GlobalValue::ExternalLinkage, getName() + INIT_SUFFIX, module);
    initFunc->setCallingConv(CallingConv::C);
    initFunc->setDoesNotThrow();
    auto args = initFunc->arg_begin();
    args->setName("self");
    for (auto binding : mScalarInputs) {
        (++args)->setName(binding.name);
    }
    for (auto binding : mStreamSetOutputs) {
        (args++)->setName(binding.name + "ConsumerLocks");
    }

    // Create the doSegment function prototype.
    std::vector<Type *> params = {selfType, iBuilder->getInt1Ty()};
    params.insert(params.end(), mStreamSetInputs.size(), sizeTy);

    FunctionType * const doSegmentType = FunctionType::get(voidTy, params, false);
    Function * const doSegment = Function::Create(doSegmentType, GlobalValue::ExternalLinkage, getName() + DO_SEGMENT_SUFFIX, module);
    doSegment->setCallingConv(CallingConv::C);
    doSegment->setDoesNotThrow();
    doSegment->setDoesNotCapture(1); // for self parameter only.
    args = doSegment->arg_begin();
    args->setName("self");
    (++args)->setName("doFinal");
    for (const Binding & input : mStreamSetInputs) {
        (++args)->setName(input.name + "AvailableItems");
    }

    // Create the terminate function prototype
    Type * resultType = nullptr;
    if (mScalarOutputs.empty()) {
        resultType = iBuilder->getVoidTy();
    } else {
        const auto n = mScalarOutputs.size();
        Type * outputType[n];
        for (unsigned i = 0; i < n; ++i) {
            outputType[i] = mScalarOutputs[i].type;
        }
        if (n == 1) {
            resultType = outputType[0];
        } else {
            resultType = StructType::get(iBuilder->getContext(), ArrayRef<Type *>(outputType, n));
        }
    }
    FunctionType * const terminateType = FunctionType::get(resultType, {selfType}, false);
    Function * const terminateFunc = Function::Create(terminateType, GlobalValue::ExternalLinkage, getName() + TERMINATE_SUFFIX, module);
    terminateFunc->setCallingConv(CallingConv::C);
    terminateFunc->setDoesNotThrow();
    terminateFunc->setDoesNotCapture(1);
    args = terminateFunc->arg_begin();
    args->setName("self");

    linkExternalMethods();
}

Function * KernelInterface::getInitFunction(Module * const module) const {
    const auto name = getName() + INIT_SUFFIX;
    Function * f = module->getFunction(name);
    if (LLVM_UNLIKELY(f == nullptr)) {
        llvm::report_fatal_error("Cannot find " + name);
    }
    return f;
}

Function * KernelInterface::getDoSegmentFunction(llvm::Module * const module) const {
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
