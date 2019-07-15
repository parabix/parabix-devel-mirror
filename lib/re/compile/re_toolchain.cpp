/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

 // TODO: REFACTOR THIS FILE!!!

#include <re/compile/re_toolchain.h>

// #include "../../../tools/icgrep/grep_interface.h"
#include <llvm/Support/CommandLine.h>  // for clEnumVal, clEnumValEnd, Optio...
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/ErrorHandling.h>
#include <re/adt/re_any.h>
#include <re/adt/re_name.h>
#include <re/adt/re_cc.h>
#include <re/adt/re_start.h>
#include <re/adt/re_end.h>
#include <re/adt/re_alt.h>
#include <re/adt/re_seq.h>
#include <re/adt/re_diff.h>
#include <re/adt/re_intersect.h>
#include <re/adt/re_group.h>
#include <re/adt/re_range.h>
#include <re/adt/re_assertion.h>
#include <re/adt/printer_re.h>
#include <re/adt/validation.h>
#include <re/cc/cc_compiler.h>            // for CC_Compiler
#include <re/compile/re_compiler.h>            // for RE_Compiler
#include <re/compile/remove_nullable.h>            // for RE_Nullable
#include <re/compile/re_star_normal.h>         // for RE_Star_Normal
#include <re/compile/re_simplifier.h>          // for RE_Simplifier
#include <re/compile/re_minimizer.h>
#include <re/compile/re_local.h>
#include <re/compile/re_analysis.h>
#include <re/compile/casing.h>
#include <re/compile/exclude_CC.h>
#include <re/compile/re_name_resolve.h>
#include <re/compile/grapheme_clusters.h>
#include <re/compile/re_contextual_simplification.h>
#include <re/compile/resolve_diffs.h>
#include <re/compile/decomposition.h>
#include <re/compile/equivalence.h>
#include <re/toolchain/toolchain.h>
#include <toolchain/toolchain.h>

using namespace pablo;
using namespace llvm;

namespace re {

RE * resolveModesAndExternalSymbols(RE * r, bool globallyCaseInsensitive) {
    if (PrintOptionIsSet(ShowAllREs) || PrintOptionIsSet(ShowREs)) {
        errs() << "Parser:\n" << Printer_RE::PrintRE(r) << '\n';
    }
    r = removeUnneededCaptures(r);
    r = resolveGraphemeMode(r, false /* not in grapheme mode at top level*/);
    r = re::resolveUnicodeNames(r);
    validateNamesDefined(r);
    if (UnicodeLevel2IsSet() && validateAlphabet(&cc::Unicode, r)) {
        r = UCD::toNFD(r);
        r = UCD::addClusterMatches(r);
        r = UCD::addEquivalentCodepoints(r);
    } else {
        r = resolveCaseInsensitiveMode(r, globallyCaseInsensitive);
    }
    r = simplifyAssertions(r);
    //r = lookaheadPromotion(r);
    return r;
}

RE * excludeUnicodeLineBreak(RE * r) {
    r = exclude_CC(r, re::makeCC(re::makeCC(0x0A, 0x0D), re::makeCC(re::makeCC(0x85), re::makeCC(0x2028, 0x2029))));
    if (PrintOptionIsSet(ShowAllREs)) {
        errs() << "excludeUnicodeLineBreak:\n" << Printer_RE::PrintRE(r) << '\n';
    }
    return r;
}

RE * regular_expression_passes(RE * re) {

    //Optimization passes to simplify the AST.
    RE * r = removeNullablePrefix(re);
    r = removeNullableSuffix(r);
    r = RE_Star_Normal().transformRE(r);
    if (codegen::OptLevel > 1) {
        r = minimizeRE(r);
    } else {
        r = simplifyRE(r);
    }
    r = resolveDiffs(r);
    r = resolveAnchors(r, makeAlt());
    if (!DefiniteLengthBackReferencesOnly(r)) {
        llvm::report_fatal_error("Future back reference support: references must be within a fixed distance from a fixed-length capture.");
    }
    return r;
}

}
