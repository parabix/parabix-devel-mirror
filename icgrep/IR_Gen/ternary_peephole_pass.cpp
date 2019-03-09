/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "ternary_peephole_pass.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/ADT/SetVector.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/IR/InstIterator.h"
#include "llvm/IR/Instruction.h"
#include "llvm/Transforms/Utils/Local.h"
#include <llvm/Support/raw_ostream.h>
using namespace llvm;

#define DEBUG_TYPE "tpph"

STATISTIC(PphEliminated, "Number of insts optimized by ternary peephole");

namespace {
  //===--------------------------------------------------------------------===//
  // Ternary Peephole pass implementation
  //
struct TernaryPeephole : public BasicBlockPass {
    static char ID; // Pass identification, replacement for typeid
    TernaryPeephole() : BasicBlockPass(ID) {
        initializeTernaryPeepholePass(*PassRegistry::getPassRegistry());
    }
    bool runOnBasicBlock(BasicBlock & BB) override {
        bool changed = false;
        for (BasicBlock::iterator DI = ++BB.begin(); DI != BB.end();) {
            // TODO: The pass was not implemented
            errs() << "instr"; (&*DI)->dump();
            Instruction * prev_inst = (&*DI) - 1;
            Instruction * curr_inst = &*DI++;
            if (prev_inst->isBinaryOp() && curr_inst->isBinaryOp()) {
                ++PphEliminated;
            }
        }
        return changed;
    }

    void getAnalysisUsage(AnalysisUsage & AU) const override {
        AU.setPreservesCFG();
    }
};
}

char TernaryPeephole::ID = 0;
INITIALIZE_PASS(TernaryPeephole, "tpph",
                "Peephole Optimization", false, false)

Pass * llvm::createTernaryPeepholePass() {
    return new TernaryPeephole();
}

