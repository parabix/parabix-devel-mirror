/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "pablo_kernel.h"
#include <pablo/codegenstate.h>
#include <pablo/pablo_compiler.h>
#include <pablo/pe_var.h>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_ones.h>
//#include <llvm/IR/Module.h>
//#include <llvm/IR/Verifier.h>
//#include <IR_Gen/idisa_builder.h>
#include "llvm/Support/Debug.h"

using namespace pablo;
using namespace kernel;
using namespace parabix;
using namespace IDISA;
using namespace llvm;

inline bool isStreamType(const Type * ty) {
    if (ty->isArrayTy()) {
        ty = ty->getArrayElementType();
    }
    if (ty->isVectorTy()) {
        return (ty->getVectorNumElements() == 0);
    }
    return false;
}

Var * PabloKernel::getInputSet(std::string inputSetName) {
    unsigned ssIndex = getStreamSetIndex(inputSetName);
    return mInputs[ssIndex];
}

Var * PabloKernel::getOutputSet(std::string outputSetName) {
    unsigned ssIndex = getStreamSetIndex(outputSetName);
    return mOutputs[ssIndex - mInputs.size()];
}


Var * PabloKernel::getScalarOutput(std::string outputName) {
    const auto f = mScalarOutputNameMap.find(outputName);
    if (LLVM_UNLIKELY(f == mScalarOutputNameMap.end())) {
        llvm::report_fatal_error("Kernel does not contain scalar: " + outputName);
    }
    return f->second;
}



Var * PabloKernel::addInput(const std::string & name, Type * const type) {
    Var * param = new (mAllocator) Var(mSymbolTable->makeString(name, iBuilder), type, mAllocator, Var::ReadOnly);
    param->addUser(this);
    mInputs.push_back(param);
    mVariables.push_back(param);
    if (isStreamType(type)) {
        mStreamSetInputs.emplace_back(type, name);
    } else {
        mScalarInputs.emplace_back(type, name);
    }
    assert (mStreamSetInputs.size() + mScalarInputs.size() == mInputs.size());
    return param;
}

Var * PabloKernel::addOutput(const std::string & name, Type * const type) {
    Var * result = new (mAllocator) Var(mSymbolTable->makeString(name, iBuilder), type, mAllocator, Var::ReadNone);
    result->addUser(this);
    mOutputs.push_back(result);
    mVariables.push_back(result);
    if (isStreamType(type)) {
        mStreamSetOutputs.emplace_back(type, name);
    } else {
        mScalarOutputs.emplace_back(type, name);
        mScalarOutputNameMap.emplace(name, result);
    }
    assert (mStreamSetOutputs.size() + mScalarOutputs.size() == mOutputs.size());
    return result;
}

Var * PabloKernel::makeVariable(String * name, Type * const type) {
    Var * const var = new (mAllocator) Var(name, type, mAllocator);
    mVariables.push_back(var);
    return var;
}

Zeroes * PabloKernel::getNullValue(Type * type) {
    if (type == nullptr) {
        type = getStreamTy();
    }
    for (PabloAST * constant : mConstants) {
        if (isa<Zeroes>(constant) && constant->getType() == type) {
            return cast<Zeroes>(constant);
        }
    }
    Zeroes * value = new (mAllocator) Zeroes(type, mAllocator);
    mConstants.push_back(value);
    return value;
}

Ones * PabloKernel::getAllOnesValue(Type * type) {
    if (type == nullptr) {
        type = getStreamTy();
    }
    for (PabloAST * constant : mConstants) {
        if (isa<Ones>(constant) && constant->getType() == type) {
            return cast<Ones>(constant);
        }
    }
    Ones * value = new (mAllocator) Ones(type, mAllocator);
    mConstants.push_back(value);
    return value;
}

void PabloKernel::prepareKernel() {
    mPabloCompiler->initializeKernelData();
    BlockOrientedKernel::prepareKernel();
}

void PabloKernel::generateDoBlockMethod() {
    mPabloCompiler->compile();
}

void PabloKernel::generateFinalBlockMethod(Value * remainingBytes) {
    // Standard Pablo convention for final block processing: set a bit marking
    // the position just past EOF, as well as a mask marking all positions past EOF.
    setScalarField("EOFbit", iBuilder->bitblock_set_bit(remainingBytes));
    setScalarField("EOFmask", iBuilder->bitblock_mask_from(remainingBytes));
    CreateDoBlockMethodCall();
}

PabloKernel::PabloKernel(IDISA::IDISA_Builder * builder, std::string kernelName)
: BlockOrientedKernel(builder, std::move(kernelName), {}, {}, {}, {}, {Binding{builder->getBitBlockType(), "EOFbit"}, Binding{builder->getBitBlockType(), "EOFmask"}})
, PabloAST(PabloAST::ClassTypeId::Kernel, nullptr, mAllocator)
, mPabloCompiler(new PabloCompiler(this))
, mSymbolTable(new SymbolGenerator(mAllocator))
, mEntryBlock(PabloBlock::Create(this)) {
    setDoBlockUpdatesProducedItemCountsAttribute(false);
}

PabloKernel::PabloKernel(IDISA::IDISA_Builder * builder, std::string kernelName, 
                         std::vector<Binding> && stream_inputs,
                         std::vector<Binding> && stream_outputs,
                         std::vector<Binding> && scalar_outputs)
: BlockOrientedKernel(builder, std::move(kernelName), 
                      std::move(stream_inputs), std::move(stream_outputs), 
                      {}, std::move(scalar_outputs), 
                      {Binding{builder->getBitBlockType(), "EOFbit"}, Binding{builder->getBitBlockType(), "EOFmask"}})
, PabloAST(PabloAST::ClassTypeId::Kernel, nullptr, mAllocator)
, mPabloCompiler(new PabloCompiler(this))
, mSymbolTable(new SymbolGenerator(mAllocator))
, mEntryBlock(PabloBlock::Create(this)) {
    setDoBlockUpdatesProducedItemCountsAttribute(false);
    prepareKernelSignature();
    for (auto ss : mStreamSetInputs) {
        Var * param = new (mAllocator) Var(mSymbolTable->makeString(ss.name, iBuilder), ss.type, mAllocator, Var::ReadOnly);
        param->addUser(this);
        mInputs.push_back(param);
        mVariables.push_back(param);
    }
    for (auto ss : mStreamSetOutputs) {
        Var * result = new (mAllocator) Var(mSymbolTable->makeString(ss.name, iBuilder), ss.type, mAllocator, Var::ReadNone);
        result->addUser(this);
        mOutputs.push_back(result);
        mVariables.push_back(result);
    }
    for (auto ss : mScalarOutputs) {
        Var * result = new (mAllocator) Var(mSymbolTable->makeString(ss.name, iBuilder), ss.type, mAllocator, Var::ReadNone);
        result->addUser(this);
        mOutputs.push_back(result);
        mVariables.push_back(result);
        mScalarOutputNameMap.emplace(ss.name, result);
    }
}

PabloKernel::~PabloKernel() {
    delete mPabloCompiler;
    delete mSymbolTable; 
}

