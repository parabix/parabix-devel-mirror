/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <pablo/pablo_kernel.h>
#include <pablo/pablo_compiler.h>
#include <llvm/Support/Debug.h>


using namespace pablo;
using namespace kernel;
using namespace parabix;

PabloKernel::PabloKernel(IDISA::IDISA_Builder * builder,
                         std::string kernelName,
                         PabloFunction * function,
                         std::vector<std::string> accumulators) :
    KernelBuilder(builder, kernelName,
                    {StreamSetBinding{StreamSetType(function->getNumOfParameters(), 1), "inputs"}},
                    {},
                    {},
                    {},
                    {ScalarBinding{builder->getBitBlockType(), "EOFbit"}, ScalarBinding{builder->getBitBlockType(), "EOFmask"}}),
    mPabloFunction(function) {
    unsigned output_streams = function->getNumOfResults();
    if (output_streams > 0) {
        mStreamSetOutputs.push_back(StreamSetBinding{StreamSetType(output_streams, 1), "outputs"});
    }
    mScalarOutputs = accumBindings(accumulators);
    pablo_compiler = new PabloCompiler(builder, this, function);
}

std::vector<ScalarBinding> PabloKernel::accumBindings(std::vector<std::string> accum_names) {
    std::vector<ScalarBinding> vec;
    Type * accum_t = iBuilder->getSizeTy();
    for (auto a : accum_names) {
        vec.push_back(ScalarBinding{accum_t, a});
        addScalar(accum_t, a);
    }
    return vec;
}

void PabloKernel::prepareKernel() {
    Type * carryDataType = pablo_compiler->initializeCarryData();
    addScalar(carryDataType, "carries");
    KernelBuilder::prepareKernel();
}

void PabloKernel::generateDoBlockMethod() {
    IDISA::IDISA_Builder::InsertPoint savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    pablo_compiler->compile(m->getFunction(mKernelName + doBlock_suffix));
    iBuilder->restoreIP(savePoint);
}

void PabloKernel::generateFinalBlockMethod() {
    IDISA::IDISA_Builder::InsertPoint savePoint = iBuilder->saveIP();
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
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}

