/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/adt/adt.h>

using namespace llvm;

namespace re {

RE * makeDiff(RE * lh, RE * rh) {
    if (lh == rh) return makeEmptySet();
    if (isEmptySeq(lh)) {
        if (isa<Rep>(rh) && (cast<Rep>(rh)->getLB() == 0)) return makeEmptySet();
        if (isa<CC>(rh) || isa<Seq>(rh)) return lh;
    } else if (LLVM_UNLIKELY(isEmptySet(rh))) {
        return lh;
    } else if (LLVM_UNLIKELY(isEmptySet(lh))) {
        return lh;
    }
    return Diff::Create(lh, rh);
}

}
