/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/adt/adt.h>

using namespace llvm;

namespace re {

RE * makeIntersect(RE * lh, RE * rh) {
    if (lh == rh) return lh;
    if (isEmptySet(lh)) return lh;
    if (isEmptySet(rh)) return rh;
    if (isEmptySeq(lh)) {
        if (isa<Rep>(rh) && (cast<Rep>(rh)->getLB() == 0)) return lh;
        else return makeEmptySet();
    }
    if (isEmptySeq(rh)) {
        if (isa<Rep>(lh) && (cast<Rep>(lh)->getLB() == 0)) return rh;
        else return makeEmptySet();
    }
    if (defined<CC>(lh) && defined<CC>(rh)) {
        CC * lh_cc = defCast<CC>(lh);
        CC * rh_cc = defCast<CC>(rh);
        if (lh_cc->getAlphabet() == rh_cc->getAlphabet())
            return intersectCC(lh_cc, rh_cc);
    }
    return Intersect::Create(lh, rh);
}

}
