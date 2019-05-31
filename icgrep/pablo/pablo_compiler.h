/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PABLO_COMPILER_H
#define PABLO_COMPILER_H

#include <unordered_map>
#include <vector>
#include <memory>
namespace IDISA { class IDISA_Builder; }
namespace llvm { class BasicBlock; }
namespace llvm { class Function; }
namespace llvm { class Value; }
namespace pablo { class CarryManager; }
namespace pablo { class If; }
namespace pablo { class PabloAST; }
namespace pablo { class PabloBlock; }
namespace pablo { class PabloKernel; }
namespace pablo { class Statement; }
namespace pablo { class Var; }
namespace pablo { class While; }
namespace kernel { class KernelBuilder; }

namespace pablo {

class PabloCompiler {

    friend class PabloKernel;

    using TranslationMap = std::unordered_map<const PabloAST *, llvm::Value *>;

public:

    PabloCompiler(PabloKernel * kernel);

    ~PabloCompiler();

protected:

    void initializeKernelData(const std::unique_ptr<kernel::KernelBuilder> & b);

    void compile(const std::unique_ptr<kernel::KernelBuilder> & b);

    void releaseKernelData(const std::unique_ptr<kernel::KernelBuilder> & b);

    void clearCarryData(const std::unique_ptr<kernel::KernelBuilder> & b);

private:

    void examineBlock(const std::unique_ptr<kernel::KernelBuilder> & b, const PabloBlock * const block);

    void compileBlock(const std::unique_ptr<kernel::KernelBuilder> & b, const PabloBlock * const block);

    void compileStatement(const std::unique_ptr<kernel::KernelBuilder> & b, const Statement * stmt);

    void compileIf(const std::unique_ptr<kernel::KernelBuilder> & b, const If * ifStmt);

    void compileWhile(const std::unique_ptr<kernel::KernelBuilder> & b, const While * whileStmt);

    void addBranchCounter(const std::unique_ptr<kernel::KernelBuilder> & b);

    const Var * findInputParam(const Statement * const stmt, const Var * const param) const;

    llvm::Value * getPointerToVar(const std::unique_ptr<kernel::KernelBuilder> & b, const Var * var, llvm::Value * index1, llvm::Value * index2 = nullptr);

    llvm::Value * compileExpression(const std::unique_ptr<kernel::KernelBuilder> & b, const PabloAST * expr, const bool ensureLoaded = true);

private:

    PabloKernel * const             mKernel;
    std::unique_ptr<CarryManager> const mCarryManager;
    TranslationMap                  mMarker;
    unsigned                        mBranchCount;
    std::vector<llvm::BasicBlock *> mBasicBlock;
};

}

#endif // PABLO_COMPILER_H
