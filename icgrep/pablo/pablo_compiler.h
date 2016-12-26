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
#include <pablo/pablo_kernel.h>
#include <llvm/ADT/Twine.h>
#include <llvm/IR/IRBuilder.h>
#include <IR_Gen/idisa_builder.h>
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
class String;
class Var;
class Statement;
class StatementList;
class If;
class While;
class CarryManager;
class Extract;

class PabloCompiler {
    friend class CarryManager;

    using IntSet = boost::container::flat_set<unsigned>;
    using MarkerMap = std::unordered_map<const PabloAST *, Value *>;

public:
    PabloCompiler(PabloKernel * kernel);
    ~PabloCompiler();
    void initializeKernelData();
    void compile(Value * const self, Function * doBlockFunction);

private:

    void Examine();
    void Examine(const PabloBlock * const block);

    void compileBlock(const PabloBlock * const block);
    void compileStatement(const Statement * stmt);
    void compileIf(const If * ifStmt);
    void compileWhile(const While * whileStmt);

    Value * compileExpression(const PabloAST * expr, const bool ensureLoaded = true) const;

private:

    IDISA::IDISA_Builder *  iBuilder;
    CarryManager *          mCarryManager;
    PabloKernel *           mKernel;
    Value *                 mSelf;
    Function *              mFunction;
    MarkerMap               mMarkerMap;
    IntSet                  mInputStreamOffset;

};

}

#endif // PABLO_COMPILER_H
