/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef U8U16_PIPELINE_H
#define U8U16_PIPELINE_H

#include <IDISA/idisa_builder.h>
#include "kernel.h"

namespace llvm {
    class Module;
    class Function;
    class Type;
}

namespace pablo {
    class PabloFunction;
    class PabloBlock;
}

using namespace llvm;

namespace kernel {

class PipelineBuilder {
public:
    PipelineBuilder(llvm::Module * m, IDISA::IDISA_Builder * b);

    ~PipelineBuilder();

    void CreateKernels(pablo::PabloFunction * function);
    llvm::Function *  ExecuteKernels();

private:
    llvm::Module *                      mMod;
    IDISA::IDISA_Builder *              iBuilder;
    KernelBuilder *                     mS2PKernel;
    KernelBuilder *                     mU8U16Kernel;
    KernelBuilder *                     mDelKernel;
    KernelBuilder *                     mP2SKernel;
    //KernelBuilder *                     mStdOutKernel;
    llvm::Type*                         mBitBlockType;
    int                                 mBlockSize;
};

}

#endif // PIPELINE_H
