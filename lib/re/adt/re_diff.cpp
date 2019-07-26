/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/adt/re_diff.h>

#include <re/adt/nullable.h>
#include <re/adt/re_seq.h>
#include <re/adt/re_empty_set.h>
#include <re/adt/validation.h>
#include <llvm/Support/Casting.h>

using namespace llvm;

namespace re {

RE * makeDiff(RE * lh, RE * rh) {
    if (isEmptySeq(lh)) {
        if (isNullable(rh)) return makeEmptySet();
        if (validateAssertionFree(rh)) return lh; // EmptySeq()
    } else if (LLVM_UNLIKELY(isEmptySet(rh))) {
        return lh;
    } else if (LLVM_UNLIKELY(isEmptySet(lh))) {
        return lh;
    }
    return Diff::Create(lh, rh);
}

}
