/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PABLO_COMPILER_H
#define PABLO_COMPILER_H

//Pablo Expressions
#include <string>
#include <vector>
#include <unordered_map>
#include <pablo/carry_manager.h>
#include <pablo/pablo_kernel.h>
#include <llvm/ADT/Twine.h>
#include <llvm/IR/IRBuilder.h>
#include <IDISA/idisa_builder.h>
#include <kernels/kernel.h>
#include <boost/container/flat_set.hpp>

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
}

namespace pablo {

class PabloAST;
class PabloBlock;
class PabloFunction;
class String;
class Var;
class Statement;
class StatementList;
class If;
class While;

class PabloCompiler {
    using IntSet = boost::container::flat_set<unsigned>;
    using MarkerMap = std::unordered_map<const PabloAST *, Value *>;
public:
    PabloCompiler(IDISA::IDISA_Builder * b, PabloKernel * k, PabloFunction * function);
    Type * initializeKernelData();
    void compile(Function * doBlockFunction);

private:

    void Examine(const PabloFunction * const function);
    void Examine(const PabloBlock * const block);

    void compileBlock(const PabloBlock * const block);
    void compileStatement(const Statement * stmt);
    void compileIf(const If * ifStmt);
    void compileWhile(const While * whileStmt);
    Value * compileExpression(const PabloAST * expr);
    void GenerateKernel(PabloFunction * const function);

    MarkerMap                           mMarkerMap;
    IntSet                              mInputStreamOffset;
    Module *                            mMod;
    IDISA::IDISA_Builder *              iBuilder;
    Type* const                         mBitBlockType;

    std::unique_ptr<CarryManager>       mCarryManager;

    PabloFunction *  const             mPabloFunction;
    const PabloBlock *                  mPabloBlock;

    PabloKernel *                       mKernelBuilder;
    Value *                             mSelf;

    unsigned                            mWhileDepth;
    unsigned                            mIfDepth;

    llvm::Function *                    mFunction;

    unsigned                            mMaxWhileDepth;
};

}

#endif // PABLO_COMPILER_H
