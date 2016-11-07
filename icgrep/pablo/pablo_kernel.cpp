/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <pablo/pablo_kernel.h>
#include <pablo/pablo_compiler.h>
#include <llvm/Support/Debug.h>
#include <pablo/pe_var.h>
#include <pablo/type/streamtype.h>
#include <llvm/IR/Verifier.h>

using namespace pablo;
using namespace kernel;
using namespace parabix;

PabloKernel::PabloKernel(IDISA::IDISA_Builder * builder,
                         std::string kernelName,
                         PabloFunction * function)
: KernelBuilder(builder, kernelName, {}, {}, {}, {}, {Binding{builder->getBitBlockType(), "EOFbit"}, Binding{builder->getBitBlockType(), "EOFmask"}})
, mPabloFunction(function)
, mPabloCompiler(new PabloCompiler(this, function))

{
    for (unsigned i = 0; i < function->getNumOfParameters(); ++i) {
        const auto param = function->getParameter(i);
        Type * type = param->getType();
        bool scalar = false;
        if (isa<StreamType>(type)) {
            type = cast<StreamType>(type)->resolveType(builder);
        } else if (type->isSingleValueType()) {
            scalar = true;
        }
        std::string name = param->getName()->to_string();
        if (scalar) {
            mScalarInputs.emplace_back(type, std::move(name));
        } else {
            mStreamSetInputs.emplace_back(type, std::move(name));
        }
    }

    for (unsigned i = 0; i < function->getNumOfResults(); ++i) {
        const auto param = function->getResult(i);
        Type * type = param->getType();
        bool scalar = false;
        if (isa<StreamType>(type)) {
            type = cast<StreamType>(type)->resolveType(builder);
        } else if (type->isSingleValueType()) {
            scalar = true;
        }
        std::string name = param->getName()->to_string();
        if (scalar) {
            mScalarOutputs.emplace_back(type, std::move(name));
        } else {
            mStreamSetOutputs.emplace_back(type, std::move(name));
        }
    }


}

PabloKernel::~PabloKernel() {
    delete mPabloCompiler;
}

void PabloKernel::prepareKernel() {
    Type * carryDataType = mPabloCompiler->initializeKernelData();
    addScalar(carryDataType, "carries");
    KernelBuilder::prepareKernel();
}

void PabloKernel::generateDoBlockMethod() {
    auto savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Function * doBlockFunction = m->getFunction(mKernelName + doBlock_suffix);
    mPabloCompiler->compile(doBlockFunction);
    Function::arg_iterator args = doBlockFunction->arg_begin();
    Value * self = &*(args);
    Value * produced = getProducedItemCount(self);
    produced = iBuilder->CreateAdd(produced, ConstantInt::get(iBuilder->getSizeTy(), iBuilder->getStride()));
    setProducedItemCount(self, produced);
    iBuilder->CreateRetVoid();
    #ifndef NDEBUG
    llvm::verifyFunction(*doBlockFunction, &errs());
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
    llvm::verifyFunction(*doBlockFunction, &errs());
    #endif
    iBuilder->restoreIP(savePoint);
}
