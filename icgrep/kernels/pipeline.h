/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef PIPELINE_H
#define PIPELINE_H

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

	void CreateKernels(pablo::PabloFunction * function, bool isNameExpression);
    llvm::Function * ExecuteKernels();

private:
    llvm::Module *                      mMod;
    IDISA::IDISA_Builder *              iBuilder;
    KernelBuilder *                     mS2PKernel;
    KernelBuilder *                     mICgrepKernel;   
    KernelBuilder *                     mScanMatchKernel;
    llvm::Type *                        mBitBlockType;
    int                                 mBlockSize;
};

}

#endif // PIPELINE_H
