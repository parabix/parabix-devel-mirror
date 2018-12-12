/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "re_diff.h"
#include <re/re_cc.h>
#include <re/re_seq.h>
#include <re/re_name.h>
#include <re/re_empty_set.h>
#include <llvm/Support/Casting.h>

using namespace llvm;

namespace re {

#include <re/re_empty_set.h>
    
RE * makeDiff(RE * lh, RE * rh) {
    
    if (LLVM_UNLIKELY(isEmptySeq(lh) && isEmptySeq(rh))) {
        return makeEmptySet();
    } else if (LLVM_UNLIKELY(isEmptySet(rh))) {
        return lh;
    } else if (LLVM_UNLIKELY(isEmptySet(lh))) {
        return makeEmptySet();
    } else {
        return new Diff(lh, rh);
    }
}

}
