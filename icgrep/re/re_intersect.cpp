/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "re_intersect.h"
#include "re_cc.h"
#include <llvm/Support/Casting.h>

namespace re {

RE * makeIntersect(RE * lh, RE * rh) {
    if (isa<CC>(lh) && isa<CC>(rh)) {
        return intersectCC(cast<CC>(lh), cast<CC>(rh));
    }
    return new Intersect(lh, rh);
}
    
}
