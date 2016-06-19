/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef PIPELINE_H
#define PIPELINE_H

#include <IDISA/idisa_builder.h>
#include <kernels/kernel.h>

namespace llvm {
    class Module;
    class Function;
    class Type;
}

namespace pablo {
    class PabloFunction;
    class PabloBlock;
    class PabloKernel;
}

using namespace llvm;

namespace kernel {

class PipelineBuilder {
public:
    PipelineBuilder(llvm::Module * m, IDISA::IDISA_Builder * b);

    ~PipelineBuilder();

    llvm::Function * ExecuteKernels(pablo::PabloFunction * function, bool isNameExpression, bool CountOnly, bool UTF_16);

private:
    llvm::Module *                      mMod;
    IDISA::IDISA_Builder *              iBuilder;
    llvm::Type *                        mBitBlockType;
    int                                 mBlockSize;
};

}

#endif // PIPELINE_H
