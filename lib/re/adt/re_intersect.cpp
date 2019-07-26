/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/adt/re_intersect.h>

#include <re/adt/re_cc.h>
#include <re/adt/re_name.h>
#include <re/adt/re_seq.h>
#include <re/adt/re_empty_set.h>
#include <re/adt/nullable.h>
#include <llvm/Support/Casting.h>

using namespace llvm;

namespace re {

RE * makeIntersect(RE * lh, RE * rh) {
    if (isEmptySet(lh)) return lh;
    if (isEmptySet(rh)) return rh;
    if (isEmptySeq(lh)) {
        if (isNullable(rh)) return lh;
        else return makeEmptySet();
    }
    if (isEmptySeq(rh)) {
        if (isNullable(lh)) return rh;
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
