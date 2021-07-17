/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/transforms/assertion_transformations.h>

#include <llvm/Support/Casting.h>
#include <re/adt/adt.h>
#include <re/transforms/re_transformer.h>

using namespace llvm;

namespace re {

struct ExpandBoundaryAssertions : public RE_Transformer {
    ExpandBoundaryAssertions() : RE_Transformer("ExpandBoundaryAssertions") {}
    RE * transformAssertion(Assertion * a) override {
        RE * a0 = a->getAsserted();
        RE * a1 = transform(a0);
        if (a->getKind() == Assertion::Kind::Boundary) {
            RE * behindP = makeAssertion(a1, Assertion::Kind::LookBehind, Assertion::Sense::Positive);
            RE * behindN = makeAssertion(a1, Assertion::Kind::LookBehind, Assertion::Sense::Negative);
            RE * aheadP = makeAssertion(a1, Assertion::Kind::LookAhead, Assertion::Sense::Positive);
            RE * aheadN = makeAssertion(a1, Assertion::Kind::LookAhead, Assertion::Sense::Negative);
            if (a->getSense() == Assertion::Sense::Positive) {
                return makeAlt({makeSeq({behindP, aheadN}), makeSeq({behindN, aheadP})});
            } else {
                return makeAlt({makeSeq({behindP, aheadP}), makeSeq({behindN, aheadN})});
            }
        }
        if (a0 == a1) return a;
        return makeAssertion(a1, a->getKind(), a->getSense());
    }
};

RE * expandBoundaryAssertions (RE * r) {
    return ExpandBoundaryAssertions().transformRE(r);
}

struct FinalLookaheadPromotion : public RE_Transformer {
    FinalLookaheadPromotion() : RE_Transformer("FinalLookaheadPromotion") {}
    RE * transformSeq(Seq * s) override {
        if (s->empty()) return s;
        RE * s_last = s->back();
        RE * t = transform(s_last);
        if (s_last == t) return s;
        std::vector<RE *> elems;
        for (unsigned i = 0; i < s->size() - 1; i++) {
            elems.push_back((*s)[i]);
        }
        elems.push_back(t);
        return makeSeq(elems.begin(), elems.end());
    }
    RE * transformDiff(Diff * d) override { return d;}
    RE * transformRep(Rep * r) override { return r;}
    RE * transformAssertion(Assertion * a) override {
        if ((a->getKind() == Assertion::Kind::LookAhead) && (a->getSense() == Assertion::Sense::Positive)) {
            return transform(a->getAsserted());
        }
        return a;
    }
};
    
RE * lookaheadPromotion(RE * r) {
    return FinalLookaheadPromotion().transformRE(r);
}

} // namespace re
