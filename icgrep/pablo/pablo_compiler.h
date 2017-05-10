/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PABLO_COMPILER_H
#define PABLO_COMPILER_H

#include <unordered_map>
#include <memory>
namespace IDISA { class IDISA_Builder; }
namespace llvm { class Function; }
namespace llvm { class Value; }
namespace pablo { class CarryManager; }
namespace pablo { class If; }
namespace pablo { class PabloAST; }
namespace pablo { class PabloBlock; }
namespace pablo { class PabloKernel; }
namespace pablo { class Statement; }
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

    void initializeKernelData(const std::unique_ptr<kernel::KernelBuilder> &  iBuilder);

    void compile(const std::unique_ptr<kernel::KernelBuilder> &  iBuilder);

private:

    void examineBlock(const std::unique_ptr<kernel::KernelBuilder> &  iBuilder, const PabloBlock * const block);

    void compileBlock(const std::unique_ptr<kernel::KernelBuilder> &  iBuilder, const PabloBlock * const block);

    void compileStatement(const std::unique_ptr<kernel::KernelBuilder> &  iBuilder, const Statement * stmt);

    void compileIf(const std::unique_ptr<kernel::KernelBuilder> &  iBuilder, const If * ifStmt);

    void compileWhile(const std::unique_ptr<kernel::KernelBuilder> &  iBuilder, const While * whileStmt);

    llvm::Value * compileExpression(const std::unique_ptr<kernel::KernelBuilder> &  iBuilder, const PabloAST * expr, const bool ensureLoaded = true) const;

private:

    PabloKernel * const             mKernel;
    CarryManager * const            mCarryManager;
    TranslationMap                  mMarker;
    TranslationMap                  mAccumulator;

};

}

#endif // PABLO_COMPILER_H
