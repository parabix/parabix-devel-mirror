/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "cc_compiler_target.h"
#include <toolchain/toolchain.h>

using namespace std;
using namespace llvm;
using namespace codegen;
using namespace pablo;
using namespace re;

namespace cc {

Parabix_CC_Compiler_Builder::Parabix_CC_Compiler_Builder(pablo::PabloBlock * scope, std::vector<pablo::PabloAST *> basisBitSet)
: CC_Compiler(scope) {
    if (LLVM_LIKELY(codegen::CCCOption.compare("binary") == 0)) {
        ccc = make_unique<Parabix_CC_Compiler>(scope, basisBitSet);
    } else if (LLVM_LIKELY(codegen::CCCOption.compare("ternary") == 0)) {
        ccc = make_unique<Parabix_Ternary_CC_Compiler>(scope, basisBitSet);
    } else {
        llvm::report_fatal_error("Specified character class compiler is not valid");
    }
}

PabloAST * Parabix_CC_Compiler_Builder::compileCC(const std::string & canonicalName, const CC *cc, PabloBlock & block) {
    return ccc->compileCC(canonicalName, cc, block);
}

PabloAST * Parabix_CC_Compiler_Builder::compileCC(const std::string & canonicalName, const CC *cc, PabloBuilder & builder) {
    return ccc->compileCC(canonicalName, cc, builder);
}

}