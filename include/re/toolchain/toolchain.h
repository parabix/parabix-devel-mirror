/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#pragma once

#include <map>
#include <set>
#include <string>
#include <llvm/Support/Compiler.h>

namespace llvm { namespace cl { class OptionCategory; } }
namespace pablo { class PabloKernel; class PabloAST; }

namespace re {

enum RE_PrintFlags {
    ShowREs, ShowAllREs
};
    
enum RE_AlgorithmFlags {
    DisableLog2BoundedRepetition, DisableIfHierarchy, DisableMatchStar
};
    
bool LLVM_READONLY PrintOptionIsSet(RE_PrintFlags flag);
bool LLVM_READONLY AlgorithmOptionIsSet(RE_AlgorithmFlags flag);
bool LLVM_READONLY UnicodeLevel2IsSet();

extern int IfInsertionGap;

std::string AnnotateWithREflags(std::string name);

const llvm::cl::OptionCategory * LLVM_READONLY re_toolchain_flags();

}
