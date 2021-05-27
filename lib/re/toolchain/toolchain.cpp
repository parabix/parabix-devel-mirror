/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/toolchain/toolchain.h>

#include <toolchain/toolchain.h>
#include <llvm/Support/CommandLine.h>

using namespace llvm;

namespace re {

static cl::OptionCategory RegexOptions("Regex Toolchain Options",
                                              "These options control the regular expression transformation and compilation.");
const cl::OptionCategory * LLVM_READONLY re_toolchain_flags() {
    return &RegexOptions;
}

static cl::bits<RE_PrintFlags>
    PrintOptions(cl::values(clEnumVal(ShowREs, "Show parsed regular expressions and transformations that change them"),
                            clEnumVal(ShowAllREs, "Print all regular expression passes")
                            CL_ENUM_VAL_SENTINEL), cl::cat(RegexOptions));

static cl::bits<RE_AlgorithmFlags>
    AlgorithmOptions(cl::values(clEnumVal(DisableLog2BoundedRepetition, "disable log2 optimizations for bounded repetition of bytes"),
                              clEnumVal(DisableIfHierarchy, "disable nested if hierarchy for generated Unicode classes (not recommended)"),
                              clEnumVal(DisableMatchStar, "disable MatchStar optimization")
                              CL_ENUM_VAL_SENTINEL), cl::cat(RegexOptions));


static cl::opt<bool> UnicodeLevel2("U2", cl::desc("Enable Unicode Level matching under canonical and compatible (?K) equivalence."), cl::cat(RegexOptions));

bool LLVM_READONLY PrintOptionIsSet(RE_PrintFlags flag) {
    return PrintOptions.isSet(flag);
}

bool LLVM_READONLY AlgorithmOptionIsSet(RE_AlgorithmFlags flag) {
    return AlgorithmOptions.isSet(flag);
}

bool LLVM_READONLY UnicodeLevel2IsSet() {
    return UnicodeLevel2;
}

const int DefaultIfInsertionGap = 3;
int IfInsertionGap;
static cl::opt<int, true>
    IfInsertionGapOption("if-insertion-gap",  cl::location(IfInsertionGap), cl::init(DefaultIfInsertionGap),
                         cl::desc("minimum number of nonempty elements between inserted if short-circuit tests"),
                         cl::cat(RegexOptions));


std::string AnnotateWithREflags(std::string name) {
    if (re::AlgorithmOptionIsSet(re::DisableMatchStar)) {
        name += "-MatchStar";
    }
    if (re::AlgorithmOptionIsSet(re::DisableLog2BoundedRepetition)) {
        name += "-log2rep";
    }
    if (re::AlgorithmOptionIsSet(re::DisableIfHierarchy)) {
        name += "-UCDifHierarchy";
    }
    if (IfInsertionGap != DefaultIfInsertionGap) {
        name += "+ifGap="+std::to_string(IfInsertionGap);
    }
    return name;
}

} // namespace re
