/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "re_assertion.h"
#include "re_seq.h"
#include "re_alt.h"
#include "re_nullable.h"

using namespace llvm;

namespace re {

RE * expandBoundaryAssertion (RE * re) {
    if (Assertion * a = dyn_cast<Assertion>(re)) {
        if (a->getKind() == Assertion::Kind::Boundary) {
            RE * asserted = a->getAsserted();
            RE * behindP = makeAssertion(asserted, Assertion::Kind::Lookbehind, Assertion::Sense::Positive);
            RE * behindN = makeAssertion(asserted, Assertion::Kind::Lookbehind, Assertion::Sense::Negative);
            RE * aheadP = makeAssertion(asserted, Assertion::Kind::Lookahead, Assertion::Sense::Positive);
            RE * aheadN = makeAssertion(asserted, Assertion::Kind::Lookahead, Assertion::Sense::Negative);
            if (a->getSense() == Assertion::Sense::Positive) {
                return makeAlt({makeSeq({behindP, aheadN}), makeSeq({behindN, aheadP})});
            } else {
                return makeAlt({makeSeq({behindP, aheadP}), makeSeq({behindN, aheadN})});
            }
        }
    }
    return re;
}
}
