/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/adt/re_diff.h>

#include <re/adt/re_cc.h>
#include <re/adt/re_seq.h>
#include <re/adt/re_name.h>
#include <re/adt/re_empty_set.h>
#include <re/compile/re_nullable.h>
#include <re/compile/re_toolchain.h>
#include <re/compile/validation.h>
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

class DiffResolver : public RE_Transformer {
public:
    DiffResolver() : RE_Transformer("DiffResolver") {}
    RE * transformDiff(Diff * d) override {
        RE * lh = d->getLH();
        RE * rh = d->getRH();
        if (defined<CC>(lh) && defined<CC>(rh)) {
            CC * lh_cc = defCast<CC>(lh);
            CC * rh_cc = defCast<CC>(rh);
            if (lh_cc->getAlphabet() == rh_cc->getAlphabet()) {
                return subtractCC(lh_cc, rh_cc);
            }
        }
        return d;
    }
};

RE * resolveDiffs(RE * r) {
    return DiffResolver().transformRE(r);
}
}
