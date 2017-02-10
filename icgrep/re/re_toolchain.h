/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_TOOLCHAIN_H
#define RE_TOOLCHAIN_H

namespace llvm { namespace cl { class OptionCategory; } }
namespace pablo { class PabloKernel; }
namespace re { class RE; }

namespace re {

enum RE_PrintFlags {
    ShowREs, ShowAllREs, ShowStrippedREs, ShowSimplifiedREs
};
    
enum RE_AlgorithmFlags {
    DisableLog2BoundedRepetition, DisableIfHierarchy, DisableMatchStar, DisableUnicodeMatchStar, 
    DisableUnicodeLineBreak, InvertMatches, UsePregeneratedUnicode
};
    
bool AlgorithmOptionIsSet(RE_AlgorithmFlags flag);
    
extern int IfInsertionGap;

const llvm::cl::OptionCategory * re_toolchain_flags();

RE * regular_expression_passes(RE * re_ast);

void re2pablo_compiler(pablo::PabloKernel * kernel, RE * re_ast, const bool CountOnly = false);
    
}
#endif
