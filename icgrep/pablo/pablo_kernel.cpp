/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <pablo/pablo_kernel.h>
#include <pablo/codegenstate.h>
#include <pablo/pablo_compiler.h>
// #include <llvm/Support/Debug.h>
#include <pablo/pe_var.h>
#include <llvm/IR/Verifier.h>
#include <IR_Gen/idisa_builder.h>

using namespace pablo;
using namespace kernel;
using namespace parabix;
using namespace IDISA;

Var * PabloKernel::addInput(const std::string name, Type * const type) {
    Var * param = new (mAllocator) Var(mSymbolTable->make(name, iBuilder), type, mAllocator, true);
    mInputs.push_back(param);
    if (isa<ArrayType>(type) || isa<StreamType>(type)) {
        mStreamSetInputs.emplace_back(type, name);
    } else {
        mScalarInputs.emplace_back(type, name);
    }
    assert (mStreamSetInputs.size() + mScalarInputs.size() == mInputs.size());
    return param;
}

Var * PabloKernel::addOutput(const std::string name, Type * const type) {
    Var * result = new (mAllocator) Var(mSymbolTable->make(name, iBuilder), type, mAllocator, false);
    mOutputs.push_back(result);
    if (isa<ArrayType>(type) || isa<StreamType>(type)) {
        mStreamSetOutputs.emplace_back(type, name);
    } else {
        mScalarOutputs.emplace_back(type, name);
    }
    assert (mStreamSetOutputs.size() + mScalarOutputs.size() == mOutputs.size());
    return result;
}

Var * PabloKernel::makeVariable(PabloAST * name, Type * const type) {
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
    KernelBuilder::prepareKernel();
}

void PabloKernel::generateDoBlockMethod() {
    auto savePoint = iBuilder->saveIP();
    Module * const m = iBuilder->getModule();
    Function * const f = m->getFunction(mKernelName + doBlock_suffix);
    Value * const self = &*(f->arg_begin());
    mPabloCompiler->compile(self, f);
    Value * produced = getProducedItemCount(self);
    produced = iBuilder->CreateAdd(produced, ConstantInt::get(iBuilder->getSizeTy(), iBuilder->getStride()));
    setProducedItemCount(self, produced);
    iBuilder->CreateRetVoid();
    #ifndef NDEBUG
    llvm::verifyFunction(*f, &errs());
    #endif
    iBuilder->restoreIP(savePoint);
}

void PabloKernel::generateFinalBlockMethod() {
    auto savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Function * doBlockFunction = m->getFunction(mKernelName + doBlock_suffix);
    Function * finalBlockFunction = m->getFunction(mKernelName + finalBlock_suffix);
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "fb_entry", finalBlockFunction, 0));
    // Final Block arguments: self, remaining, then the standard DoBlock args.
    Function::arg_iterator args = finalBlockFunction->arg_begin();
    Value * self = &*(args++);
    Value * remaining = &*(args++);
    std::vector<Value *> doBlockArgs = {self};
    while (args != finalBlockFunction->arg_end()){
        doBlockArgs.push_back(&*args++);
    }
    // Standard Pablo convention for final block processing: set a bit marking
    // the position just past EOF, as well as a mask marking all positions past EOF.
    setScalarField(self, "EOFbit", iBuilder->bitblock_set_bit(remaining));
    setScalarField(self, "EOFmask", iBuilder->bitblock_mask_from(remaining));
    iBuilder->CreateCall(doBlockFunction, doBlockArgs);
    /* Adjust the produced item count */
    Value * produced = getProducedItemCount(self);
    produced = iBuilder->CreateSub(produced, ConstantInt::get(iBuilder->getSizeTy(), iBuilder->getStride()));
    setProducedItemCount(self, iBuilder->CreateAdd(produced, remaining));
    iBuilder->CreateRetVoid();
    #ifndef NDEBUG
    llvm::verifyFunction(*finalBlockFunction, &errs());
    #endif
    iBuilder->restoreIP(savePoint);
}

void PabloKernel::initializeKernelState(Value * self) {
    iBuilder->CreateStore(Constant::getNullValue(mKernelStateType), self);
}

PabloKernel::PabloKernel(IDISA::IDISA_Builder * builder, const std::string & kernelName)
: KernelBuilder(builder, kernelName, {}, {}, {}, {}, {Binding{builder->getBitBlockType(), "EOFbit"}, Binding{builder->getBitBlockType(), "EOFmask"}})
, mPabloCompiler(new PabloCompiler(this))
, mSymbolTable(new SymbolGenerator(mAllocator))
, mEntryBlock(PabloBlock::Create(this))
{

}

PabloKernel::~PabloKernel() {
    delete mPabloCompiler;
    delete mSymbolTable;
}

