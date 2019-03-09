/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef TERNARY_PEEPHOLE_PASS_H
#define TERNARY_PEEPHOLE_PASS_H

#include "llvm/Pass.h"

namespace llvm {
    llvm::Pass * createTernaryPeepholePass();
    void initializeTernaryPeepholePass(llvm::PassRegistry &);
}

#endif