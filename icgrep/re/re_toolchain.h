/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_TOOLCHAIN_H
#define RE_TOOLCHAIN_H

#include <re/re_re.h>
#include <pablo/function.h>
#include <llvm/Support/CommandLine.h>

namespace re {

enum RE_PrintFlags {
    PrintAllREs, PrintParsedREs, PrintStrippedREs, PrintSimplifiedREs
};
    
enum RE_AlgorithmFlags {
    DisableLog2BoundedRepetition, DisableIfHierarchy, DisableMatchStar, DisableUnicodeMatchStar, 
    DisableUnicodeLineBreak, InvertMatches, UsePregeneratedUnicode
};
    
bool AlgorithmOptionIsSet(RE_AlgorithmFlags flag);
    
extern int IfInsertionGap;

const cl::OptionCategory * re_toolchain_flags();

RE * regular_expression_passes(RE * re_ast);

pablo::PabloFunction * re2pablo_compiler(const unsigned encodingBits, RE * re_ast, bool CountOnly = false);
    
}
#endif
