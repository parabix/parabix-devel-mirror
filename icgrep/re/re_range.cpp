/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "re_range.h"
#include "re_cc.h"
#include <llvm/Support/Casting.h>

using namespace llvm;

namespace re {

RE * makeRange(RE * lo, RE * hi) {
    if (isa<CC>(lo) && isa<CC>(hi)) {
        assert(dyn_cast<CC>(lo)->count() == 1);
        assert(dyn_cast<CC>(hi)->count() == 1);
        return makeCC(dyn_cast<CC>(lo)->front().first, dyn_cast<CC>(hi)->front().first);
    }
    else if (lo == hi) { // TODO: general check for equality, not just instance equality
        return lo;
    }
    return new Range(lo, hi);
}
    
}
