/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "re_diff.h"
#include <re/re_cc.h>
#include <re/re_name.h>
#include <llvm/Support/Casting.h>

using namespace llvm;

namespace re {

RE * makeDiff(RE * lh, RE * rh) {
    if (defined<CC>(lh) && defined<CC>(rh)) {
        CC * lh_cc = defCast<CC>(lh);
        CC * rh_cc = defCast<CC>(rh);
        if (lh_cc->getAlphabet() == rh_cc->getAlphabet())
            return subtractCC(lh_cc, rh_cc);
    }
    return new Diff(lh, rh);
}
    
}
