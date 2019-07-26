/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/compile/resolve_diffs.h>

#include <llvm/Support/Casting.h>
#include <re/adt/re_cc.h>
#include <re/adt/re_diff.h>
#include <re/adt/re_name.h>
#include <re/compile/re_transformer.h>

using namespace llvm;

namespace re {

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
