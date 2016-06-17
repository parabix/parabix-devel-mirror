/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernels/streamset.h>
#include <pablo/pablo_kernel.h>
#include <pablo/pablo_compiler.h>
#include <llvm/Support/Debug.h>


using namespace pablo;
using namespace kernel;

PabloKernel::PabloKernel(IDISA::IDISA_Builder * builder,
                         std::string kernelName,
                         PabloFunction * pf,
                         std::vector<std::string> accumulators) :
    KernelInterface(builder, kernelName,
                    {StreamSetBinding{StreamSetType(pf->getNumOfParameters(), 1), "inputs"}},
                    {StreamSetBinding{StreamSetType(pf->getNumOfResults(), 1), "outputs"}},
                    {},
                    {},
                    {ScalarBinding{builder->getBitBlockType(), "EOFmark"}}),
    mPabloFunction(pf) {
    mScalarOutputs = accumBindings(accumulators);
}


std::vector<ScalarBinding> PabloKernel::accumBindings(std::vector<std::string> accum_names) {
    std::vector<ScalarBinding> vec;
    Type * accum_t = iBuilder->getInt64Ty();
    for (auto a : accum_names) {
        vec.push_back(ScalarBinding{accum_t, a});
    }
    return vec;
}

std::unique_ptr<llvm::Module> PabloKernel::createKernelModule() {
    std::unique_ptr<llvm::Module> theModule = KernelInterface::createKernelModule();

    Module * m = theModule.get();
    addFinalBlockMethod(m);
    iBuilder->setModule(m);
    PabloCompiler pablo_compiler(iBuilder);
    pablo_compiler.setKernel(this);
    pablo_compiler.compile(mPabloFunction, m->getFunction(mKernelName + "_DoBlock"));
    return theModule;
}

void PabloKernel::addFinalBlockMethod(Module * m) {
    Function * doBlockFunction = m->getFunction(mKernelName + "_DoBlock");
    Function * finalBlockFunction = m->getFunction(mKernelName + "_FinalBlock");
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

