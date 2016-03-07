/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef SYMBOLTABLEPIPELINE_H
#define SYMBOLTABLEPIPELINE_H

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

namespace pablo { class PabloFunction; class PabloBlock; }

using namespace llvm;

class SymbolTableBuilder {
public:
    SymbolTableBuilder(Module * m, IDISA::IDISA_Builder * b);
    void createKernels();
    void ExecuteKernels();
protected:

    pablo::PabloFunction * generateLeadingFunction(const std::vector<unsigned> & endpoints);
    pablo::PabloFunction * generateLookaheadFunction(const pablo::PabloFunction * const leading, const std::vector<unsigned> & endpoints);
    pablo::PabloFunction * generateSortingFunction(const pablo::PabloFunction * const lookahead);

private:
    Module *                            mMod;
    IDISA::IDISA_Builder *              iBuilder;
    KernelBuilder *                     mS2PKernel;
    KernelBuilder *                     mLeadingKernel;
    KernelBuilder *                     mLookaheadKernel;
    KernelBuilder *                     mSortingKernel;
    int                                 mFileBufIdx;
    int                                 mFileSizeIdx;
    int                                 mFileNameIdx;
    Type*                               mBitBlockType;
    int                                 mBlockSize;
};

#endif // SYMBOLTABLEPIPELINE_H
