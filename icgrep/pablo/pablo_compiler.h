/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PABLO_COMPILER_H
#define PABLO_COMPILER_H

#include <unordered_map>
#include <boost/container/flat_set.hpp>
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
    friend class CarryManager;

    using IntSet = boost::container::flat_set<unsigned>;
    using MarkerMap = std::unordered_map<const PabloAST *, llvm::Value *>;

public:
    PabloCompiler(PabloKernel * kernel);
    ~PabloCompiler();
    void initializeKernelData();
    void compile(llvm::Value * const self, llvm::Function * doBlockFunction);

private:

    void Examine();

    void Examine(const PabloBlock * const block);

    void compileBlock(const PabloBlock * const block);

    void compileStatement(const Statement * stmt);

    void compileIf(const If * ifStmt);

    void compileWhile(const While * whileStmt);

    llvm::Value * compileExpression(const PabloAST * expr, const bool ensureLoaded = true) const;

private:

    IDISA::IDISA_Builder *  iBuilder;
    CarryManager *          mCarryManager;
    PabloKernel *           mKernel;
    llvm::Value *           mSelf;
    llvm::Function *        mFunction;
    MarkerMap               mMarkerMap;
    IntSet                  mInputStreamOffset;

};

}

#endif // PABLO_COMPILER_H
