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

using namespace pablo;
using namespace kernel;
using namespace parabix;
using namespace IDISA;
using namespace llvm;

Var * PabloKernel::addInput(const std::string & name, Type * const type) {
    Var * param = new (mAllocator) Var(mSymbolTable->makeString(name, iBuilder), type, mAllocator, Var::ReadOnly);
    param->addUser(this);
    mInputs.push_back(param);
    mVariables.push_back(param);
    if (isa<ArrayType>(type) || isa<StreamType>(type)) {
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
    if (isa<ArrayType>(type) || isa<StreamType>(type)) {
        mStreamSetOutputs.emplace_back(type, name);
    } else {
        mScalarOutputs.emplace_back(type, name);
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

PabloKernel::~PabloKernel() {
    delete mPabloCompiler;
    delete mSymbolTable; 
}

