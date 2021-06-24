/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PABLO_COMPILER_H
#define PABLO_COMPILER_H

#include <kernel/core/block_kernel_compiler.h>
#include <pablo/carry_manager.h>
#include <unordered_map>
#include <vector>
#include <memory>
namespace IDISA { class IDISA_Builder; }
namespace llvm { class BasicBlock; }
namespace llvm { class Function; }
namespace llvm { class Value; }
namespace pablo { class If; }
namespace pablo { class PabloAST; }
namespace pablo { class PabloBlock; }
namespace pablo { class PabloKernel; }
namespace pablo { class Statement; }
namespace pablo { class Var; }
namespace pablo { class While; }

namespace pablo {

class PabloCompiler final : public kernel::BlockKernelCompiler {

    friend class PabloKernel;

    using TranslationMap = std::unordered_map<const PabloAST *, llvm::Value *>;

public:

    using BuilderRef = kernel::Kernel::BuilderRef;

    PabloCompiler(PabloKernel * kernel);

protected:

    void initializeKernelData(BuilderRef b);

    void compile(BuilderRef b);

    void releaseKernelData(BuilderRef b);

    void clearCarryData(BuilderRef b);

private:

    void examineBlock(BuilderRef b, const PabloBlock * const block);

    void compileBlock(BuilderRef b, const PabloBlock * const block);

    void compileStatement(BuilderRef b, const Statement * stmt);

    void compileIf(BuilderRef b, const If * ifStmt);

    void compileWhile(BuilderRef b, const While * whileStmt);

    void addBranchCounter(BuilderRef b);

    const Var * findInputParam(const Statement * const stmt, const Var * const param) const;

    llvm::Value * getPointerToVar(BuilderRef b, const Var * var, llvm::Value * index1, llvm::Value * index2 = nullptr);

    llvm::Value * compileExpression(BuilderRef b, const PabloAST * expr, const bool ensureLoaded = true);

    static void dumpValueToConsole(BuilderRef b, const PabloAST * expr, llvm::Value * value);

private:

    PabloKernel * const                 mKernel;
    std::unique_ptr<CarryManager> const mCarryManager;
    TranslationMap                      mMarker;
    unsigned                            mBranchCount;
    llvm::BasicBlock *                  mEntryBlock;
    std::vector<llvm::BasicBlock *>     mBasicBlock;
};

}

#endif // PABLO_COMPILER_H
