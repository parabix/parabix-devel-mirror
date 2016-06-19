/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <pablo/pablo_kernel.h>
#include <pablo/pablo_compiler.h>
#include <llvm/Support/Debug.h>


using namespace pablo;
using namespace kernel;

PabloKernel::PabloKernel(IDISA::IDISA_Builder * builder,
                         std::string kernelName,
                         PabloFunction * function,
                         std::vector<std::string> accumulators) :
    KernelBuilder(builder, kernelName,
                    {StreamSetBinding{StreamSetType(function->getNumOfParameters(), 1), "inputs"}},
                    {StreamSetBinding{StreamSetType(function->getNumOfResults(), 1), "outputs"}},
                    {},
                    {},
                    {ScalarBinding{builder->getBitBlockType(), "EOFmark"}}),
    mPabloFunction(function) {
    mScalarOutputs = accumBindings(accumulators);
    pablo_compiler = new PabloCompiler(builder, this, function);
}

std::vector<ScalarBinding> PabloKernel::accumBindings(std::vector<std::string> accum_names) {
    std::vector<ScalarBinding> vec;
    Type * accum_t = iBuilder->getInt64Ty();
    for (auto a : accum_names) {
        vec.push_back(ScalarBinding{accum_t, a});
    }
    return vec;
}

void PabloKernel::prepareKernel() {
    errs() << "PabloKernel::prepareKernel\n";
    Type * carryDataType = pablo_compiler->initializeCarryData();
    addScalar(carryDataType, "carries");
    finalizeKernelStateType();
}

void PabloKernel::generateKernel() {
    KernelBuilder::generateKernel();
    Module * m = iBuilder->getModule();
    addFinalBlockMethod(m);
    pablo_compiler->compile(m->getFunction(mKernelName + doBlock_suffix));
}

void PabloKernel::addFinalBlockMethod(Module * m) {
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
    // the position just past EOF.
    Type * bitBlockInt = iBuilder->getIntNTy(iBuilder->getBitBlockWidth());
    Value * EOFmark = iBuilder->CreateShl(ConstantInt::get(bitBlockInt, 1), iBuilder->CreateZExt(remaining, bitBlockInt));
    setScalarField(self, "EOFmark", iBuilder->CreateBitCast(EOFmark, iBuilder->getBitBlockType()));
    iBuilder->CreateCall(doBlockFunction, doBlockArgs);
    iBuilder->CreateRetVoid();
}

