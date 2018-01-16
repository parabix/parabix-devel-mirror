/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_TOOLCHAIN_H
#define RE_TOOLCHAIN_H
#include <llvm/Support/Compiler.h>
namespace llvm { namespace cl { class OptionCategory; } }
namespace pablo { class PabloKernel; class PabloAST; }
namespace re { class RE; class CC;}

namespace re {

enum RE_PrintFlags {
    ShowREs, ShowAllREs, ShowStrippedREs, ShowSimplifiedREs
};
    
enum RE_AlgorithmFlags {
    DisableLog2BoundedRepetition, DisableIfHierarchy, DisableMatchStar, DisableUnicodeMatchStar, 
    DisableUnicodeLineBreak, UsePregeneratedUnicode
};
    
bool LLVM_READONLY AlgorithmOptionIsSet(RE_AlgorithmFlags flag);
    
extern int IfInsertionGap;

const llvm::cl::OptionCategory * LLVM_READONLY re_toolchain_flags();

RE * resolveModesAndExternalSymbols(RE * r);

RE * excludeUnicodeLineBreak(RE * r);

RE * regular_expression_passes(RE * r);
    
}
#endif
