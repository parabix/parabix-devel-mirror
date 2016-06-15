/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */


#include <pablo/pablo_kernel.h>


using namespace pablo;

PabloKernel::PabloKernel(IDISA::IDISA_Builder * builder,
                    std::string kernelName,
                    PabloFunction * pf,
                    std::vector<string> accumulators);
    KernelInterface(iBuilder, kernelName,
                    {StreamSetBinding{StreamSetType(pf->NumOfParameters(), 1), "inputs"}},
                    {StreamSetBinding{StreamSetType(pf->NumOfResults(), 1), "outputs"}},
                    {},
                    accumBindings(accumulators),
                    {ScalarBinding(iBuilder->getBitBlockType(), "EOFmark"}) {}

std::vector<ScalarBinding> accumBindings(std::vector<std::string> accum_names) {
    std::vector<ScalarBinding> vec;
    accum_t = iBuilder->getInt64Ty();
    for (a in accum_names) {
        vec.push_back(ScalarBinding{accum_t, a});
    }
    return vec;
}

std::unique_ptr<llvm::Module> PabloKernel::createKernelModule() {
    std::unique_ptr<llvm::Module> theModule = KernelInterface::createKernelModule();
    addFinalBlockMethod(theModule);
    Function * doBlockFunction = theModule.get()->getFunction(mKernelName + "_DoBlock");
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", doBlockFunction, 0));
    
    
    
}

void PabloKernel::addFinalBlockMethod(Module * m) {
    Function * doBlockFunction = m->getFunction(mKernelName + "_DoBlock");
    Function * finalBlockFunction = m->getFunction(mKernelName + "_FinalBlock");
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "fb_entry", finalBlockFunction, 0));
    // Final Block arguments: self, remaining, then the standard DoBlock args.
    Function::arg_iterator args = main->arg_begin();
    Value * self = &*(args++);
    Value * remaining = &*(args++);
    std::vector<Value *> doBlockArgs = {self};
    while (args != main->arg_end()){
        doBlockArgs.push_back(&*args++);
    }
    // Standard Pablo convention for final block processing: set a bit marking
    // the position just past EOF.
    Value * EOFmark = iBuilder->CreateShl(ConstantInt::get(iBuilder->getIntNTy(mBlockSize), 1), remaining);
    setScalarField(self, "EOFmark", EOFmark);
    iBuilder->CreateCall(doBlockFunction, doBlockArgs);
    iBuilder->CreateRetVoid();
}

