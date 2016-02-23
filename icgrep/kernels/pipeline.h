#ifndef PIPELINE_H
#define PIPELINE_H
/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */


#include <IDISA/idisa_builder.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include "kernel.h"

namespace llvm {
    class Value;
    class Module;
    class ExecutionEngine;
    class VectorType;
    class PointerType;
    class Constant;
    class FunctionType;
    class Function;
    class BasicBlock;
    class Type;
}

namespace pablo { class PabloFunction; }

using namespace llvm;

class PipelineBuilder{
public:
	PipelineBuilder(Module * m, IDISA::IDISA_Builder * b);
	~PipelineBuilder();

	void CreateKernels(pablo::PabloFunction * function, bool isNameExpression);
    void ExecuteKernels();

private:
	Module *                            mMod;
    IDISA::IDISA_Builder *              iBuilder;
    KernelBuilder *                     mS2PKernel;
    KernelBuilder *                     mICgrepKernel;   
    KernelBuilder *                     mScanMatchKernel;
    int                                 mFileBufIdx;
    int                                 mFileSizeIdx;
    int                                 mFileNameIdx;
    Type*                               mBitBlockType;
    int                                 mBlockSize;
};

#endif // PIPELINE_H
