/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/re_range.h>
#include <re/re_cc.h>
#include <re/re_name.h>
#include <llvm/Support/Casting.h>
#include <llvm/Support/ErrorHandling.h>

using namespace llvm;

namespace re {

RE * makeRange(RE * lo, RE * hi) {
    if (isa<CC>(lo) && isa<CC>(hi)) {
        if (!((dyn_cast<CC>(lo)->count() == 1) && (dyn_cast<CC>(hi)->count() == 1)))
            llvm::report_fatal_error("invalid range operand");
        auto lo_val = dyn_cast<CC>(lo)->front().first;
        auto hi_val = dyn_cast<CC>(hi)->front().first;
        if (hi_val < lo_val) llvm::report_fatal_error("illegal range");
        return makeCC(lo_val, hi_val);
    }
    else if (isa<Name>(lo) && (cast<Name>(lo)->getDefinition() != nullptr)) {
        return makeRange(cast<Name>(lo)->getDefinition(), hi);
    }
    else if (isa<Name>(hi) && (cast<Name>(hi)->getDefinition() != nullptr)) {
        return makeRange(lo, cast<Name>(hi)->getDefinition());
    }
    else if (lo == hi) { // TODO: general check for equality, not just instance equality
        return lo;
    }
    return new Range(lo, hi);
}
    
}
