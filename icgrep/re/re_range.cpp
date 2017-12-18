/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/re_range.h>
#include <re/re_cc.h>
#include <re/re_name.h>
#include <llvm/Support/Casting.h>

using namespace llvm;

namespace re {

RE * makeRange(RE * lo, RE * hi) {
    if (isa<CC>(lo) && isa<CC>(hi)) {
        assert(dyn_cast<CC>(lo)->count() == 1);
        assert(dyn_cast<CC>(hi)->count() == 1);
        return makeCC(dyn_cast<CC>(lo)->front().first, dyn_cast<CC>(hi)->front().first);
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
