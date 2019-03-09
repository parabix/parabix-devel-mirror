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
#include "llvm/Analysis/TargetLibraryInfo.h"
#include "llvm/Transforms/Utils/Local.h"
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
    bool runOnBasicBlock(BasicBlock &BB) override {
      if (skipOptnoneFunction(BB))
        return false;
      auto *TLIP = getAnalysisIfAvailable<TargetLibraryInfoWrapperPass>();
      TargetLibraryInfo *TLI = TLIP ? &TLIP->getTLI() : nullptr;
      bool Changed = false;
      for (BasicBlock::iterator DI = BB.begin(); DI != BB.end(); ) {
        Instruction *Inst = &*DI++;
        if (isInstructionTriviallyDead(Inst, TLI)) {
          Inst->eraseFromParent();
          Changed = true;
          ++PphEliminated;
        }
      }
      return Changed;
    }

    void getAnalysisUsage(AnalysisUsage &AU) const override {
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