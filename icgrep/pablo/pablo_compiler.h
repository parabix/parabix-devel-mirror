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

namespace pablo {

class PabloCompiler {

    friend class PabloKernel;

    using TranslationMap = std::unordered_map<const PabloAST *, llvm::Value *>;

public:

    PabloCompiler(PabloKernel * kernel);

    ~PabloCompiler();

protected:

    void initializeKernelData(IDISA::IDISA_Builder * const builder);

    void compile(IDISA::IDISA_Builder * const builder);

private:

    void examineBlock(IDISA::IDISA_Builder * const builder, const PabloBlock * const block);

    void compileBlock(IDISA::IDISA_Builder * const builder, const PabloBlock * const block);

    void compileStatement(IDISA::IDISA_Builder * const builder, const Statement * stmt);

    void compileIf(IDISA::IDISA_Builder * const builder, const If * ifStmt);

    void compileWhile(IDISA::IDISA_Builder * const builder, const While * whileStmt);

    llvm::Value * compileExpression(IDISA::IDISA_Builder * const builder, const PabloAST * expr, const bool ensureLoaded = true) const;

private:

    PabloKernel * const             mKernel;
    CarryManager * const            mCarryManager;
    TranslationMap                  mMarker;
    TranslationMap                  mAccumulator;

};

}

#endif // PABLO_COMPILER_H
