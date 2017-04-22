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
#include <pablo/pablo_toolchain.h>
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

Var * PabloKernel::getInputStreamVar(const std::string & name) {
    Port port; unsigned index;
    std::tie(port, index) = getStreamPort(name);
    assert (port == Port::Input);
    return mInputs[index];
}

Var * PabloKernel::getOutputStreamVar(const std::string & name) {
    Port port; unsigned index;
    std::tie(port, index) = getStreamPort(name);
    assert (port == Port::Output);
    return mOutputs[index];
}

Var * PabloKernel::getOutputScalarVar(const std::string & name) {
    const auto f = mScalarOutputNameMap.find(name);
    if (LLVM_UNLIKELY(f == mScalarOutputNameMap.end())) {
        report_fatal_error("Kernel does not contain scalar: " + name);
    }
    return f->second;
}

Var * PabloKernel::addInput(const std::string & name, Type * const type) {
    Var * param = new (mAllocator) Var(mSymbolTable->makeString(name, iBuilder), type, mAllocator, Var::KernelInputParameter);
    param->addUser(this);
    mInputs.push_back(param);
    mVariables.push_back(param);
    if (isStreamType(type)) {
        mStreamMap.emplace(name, std::make_pair(Port::Input, mStreamSetInputs.size()));
        mStreamSetInputs.emplace_back(type, name);        
    } else {
        mScalarInputs.emplace_back(type, name);
        param->setScalar();
    }
    assert (mStreamSetInputs.size() + mScalarInputs.size() == mInputs.size());
    return param;
}

Var * PabloKernel::addOutput(const std::string & name, Type * const type) {
    Var * result = new (mAllocator) Var(mSymbolTable->makeString(name, iBuilder), type, mAllocator, Var::KernelOutputParameter);
    result->addUser(this);
    mOutputs.push_back(result);
    mVariables.push_back(result);
    if (isStreamType(type)) {
        mStreamMap.emplace(name, std::make_pair(Port::Output, mStreamSetOutputs.size()));
        mStreamSetOutputs.emplace_back(type, name);
    } else {
        mScalarOutputs.emplace_back(type, name);
        mScalarOutputNameMap.emplace(name, result);
        result->setScalar();
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

PabloKernel::PabloKernel(IDISA::IDISA_Builder * builder, std::string kernelName,
                         std::vector<Binding> stream_inputs,
                         std::vector<Binding> stream_outputs,
                         std::vector<Binding> scalar_parameters,
                         std::vector<Binding> scalar_outputs)
: BlockOrientedKernel(builder, std::move(kernelName), 
                      std::move(stream_inputs), std::move(stream_outputs), 
                      std::move(scalar_parameters), std::move(scalar_outputs),
                      {Binding{builder->getBitBlockType(), "EOFbit"}, Binding{builder->getBitBlockType(), "EOFmask"}})
, PabloAST(PabloAST::ClassTypeId::Kernel, nullptr, mAllocator)
, mPabloCompiler(new PabloCompiler(this))
, mSymbolTable(new SymbolGenerator(mAllocator))
, mEntryBlock(PabloBlock::Create(this)) {
    prepareStreamSetNameMap();
    for (const Binding & ss : mStreamSetInputs) {
        Var * param = new (mAllocator) Var(mSymbolTable->makeString(ss.name, iBuilder), ss.type, mAllocator, Var::KernelInputParameter);
        param->addUser(this);
        mInputs.push_back(param);
        mVariables.push_back(param);
    }
    for (const Binding & ss : mStreamSetOutputs) {
        Var * result = new (mAllocator) Var(mSymbolTable->makeString(ss.name, iBuilder), ss.type, mAllocator, Var::KernelOutputParameter);
        result->addUser(this);
        mOutputs.push_back(result);
        mVariables.push_back(result);
    }
    for (const Binding & ss : mScalarOutputs) {
        Var * result = new (mAllocator) Var(mSymbolTable->makeString(ss.name, iBuilder), ss.type, mAllocator, Var::KernelOutputParameter);
        result->addUser(this);
        mOutputs.push_back(result);
        mVariables.push_back(result);
        mScalarOutputNameMap.emplace(ss.name, result);
        result->setScalar();
    }
    if (DebugOptionIsSet(DumpTrace)) {
        setName(getName() + "_DumpTrace");
    }
}

PabloKernel::~PabloKernel() {
    delete mPabloCompiler;
    delete mSymbolTable; 
}

