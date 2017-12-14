/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <toolchain/toolchain.h>
#include <re/re_toolchain.h>
#include <cc/cc_compiler.h>            // for CC_Compiler
#include <llvm/Support/CommandLine.h>  // for clEnumVal, clEnumValEnd, Optio...
#include <re/re_compiler.h>            // for RE_Compiler
#include <re/re_nullable.h>            // for RE_Nullable
#include <re/re_star_normal.h>         // for RE_Star_Normal
#include <re/re_simplifier.h>          // for RE_Simplifier
#include <re/re_minimizer.h>
#include <re/re_local.h>
#include <re/printer_re.h>
#include <re/re_analysis.h>
#include <llvm/Support/raw_ostream.h>

using namespace pablo;
using namespace llvm;

namespace re {

static cl::OptionCategory RegexOptions("Regex Toolchain Options",
                                              "These options control the regular expression transformation and compilation.");
const cl::OptionCategory * re_toolchain_flags() {
    return &RegexOptions;
}

static cl::bits<RE_PrintFlags> 
    PrintOptions(cl::values(clEnumVal(ShowREs, "Print parsed or generated regular expressions"),
                            clEnumVal(ShowAllREs, "Print all regular expression passes"),
                            clEnumVal(ShowStrippedREs, "Print REs with nullable prefixes/suffixes removed"),
                            clEnumVal(ShowSimplifiedREs, "Print final simplified REs")
                            CL_ENUM_VAL_SENTINEL), cl::cat(RegexOptions));

static cl::bits<RE_AlgorithmFlags>
    AlgorithmOptions(cl::values(clEnumVal(DisableLog2BoundedRepetition, "disable log2 optimizations for bounded repetition of bytes"),
                              clEnumVal(DisableIfHierarchy, "disable nested if hierarchy for generated Unicode classes (not recommended)"), 
                              clEnumVal(DisableMatchStar, "disable MatchStar optimization"), 
                              clEnumVal(DisableUnicodeMatchStar, "disable Unicode MatchStar optimization"),
                              clEnumVal(DisableUnicodeLineBreak, "disable Unicode line breaks - use LF only")
                              CL_ENUM_VAL_SENTINEL), cl::cat(RegexOptions));

bool AlgorithmOptionIsSet(RE_AlgorithmFlags flag) {
    return AlgorithmOptions.isSet(flag);
}

int IfInsertionGap;
static cl::opt<int, true> 
    IfInsertionGapOption("if-insertion-gap",  cl::location(IfInsertionGap), cl::init(3),
                         cl::desc("minimum number of nonempty elements between inserted if short-circuit tests"), 
                         cl::cat(RegexOptions));



RE * regular_expression_passes(RE * re)  {

    if (PrintOptions.isSet(ShowAllREs) || PrintOptions.isSet(ShowREs)) {
        errs() << "Parser:\n" << Printer_RE::PrintRE(re) << '\n';
    }

    //Optimization passes to simplify the AST.
    re = RE_Nullable::removeNullablePrefix(re);
    if (PrintOptions.isSet(ShowAllREs) || PrintOptions.isSet(ShowStrippedREs)) {
        errs() << "RemoveNullablePrefix:\n" << Printer_RE::PrintRE(re) << '\n';
    }
    re = RE_Nullable::removeNullableSuffix(re);
    if (PrintOptions.isSet(ShowAllREs) || PrintOptions.isSet(ShowStrippedREs)) {
        errs() << "RemoveNullableSuffix:\n" << Printer_RE::PrintRE(re) << '\n';
    }
    re = RE_Nullable::removeNullableAssertion(re);
    if (PrintOptions.isSet(ShowAllREs) || PrintOptions.isSet(ShowStrippedREs)) {
        errs() << "RemoveNullableAssertion:\n" << Printer_RE::PrintRE(re) << '\n';
    }
    //re = RE_Nullable::removeNullableAfterAssertion(re);
    //if (PrintOptions.isSet(ShowAllREs) || PrintOptions.isSet(ShowStrippedREs)) {
    //    errs() << "RemoveNullableAfterAssertion\n" << Printer_RE::PrintRE(re) << '\n';
    //}

    re = RE_Simplifier::simplify(re);

    if (PrintOptions.isSet(ShowAllREs) || PrintOptions.isSet(ShowSimplifiedREs)) {
        //Print to the terminal the AST that was generated by the simplifier.
        errs() << "Simplifier:\n" << Printer_RE::PrintRE(re) << '\n';
    }
    
//    re = RE_Minimizer::minimize(re);

//    if (PrintOptions.isSet(ShowAllREs) || PrintOptions.isSet(ShowSimplifiedREs)) {
//        //Print to the terminal the AST that was generated by the simplifier.
//        errs() << "Minimizer:\n" << Printer_RE::PrintRE(re) << '\n';
//    }

    re = RE_Star_Normal::star_normal(re);

    if (PrintOptions.isSet(ShowAllREs) || PrintOptions.isSet(ShowSimplifiedREs)) {
        //Print to the terminal the AST that was transformed to the star normal form.
        errs() << "Star_Normal_Form:\n" << Printer_RE::PrintRE(re) << '\n';
    }

    return re;
}
    
PabloAST * re2pablo_compiler(PabloKernel * kernel, RE * re_ast) {
    Var * const basis = kernel->getInputStreamVar("basis");
    cc::CC_Compiler cc_compiler(kernel, basis);
    RE_Compiler re_compiler(kernel, cc_compiler);
    re_ast = re_compiler.compileUnicodeNames(re_ast);
    return re_compiler.compile(re_ast);
}

}
