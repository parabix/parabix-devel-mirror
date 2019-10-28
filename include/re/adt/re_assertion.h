/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_ASSERTION_H
#define RE_ASSERTION_H

#include <re/adt/adt.h>
#include <re/adt/re_empty_set.h>
#include <re/analysis/nullable.h>

namespace re {

class Assertion : public RE {
public:
    enum class Kind {LookBehind, LookAhead, Boundary};
    enum class Sense {Positive, Negative};
    
    RE * getAsserted() const {return mAsserted;}
    Assertion::Kind getKind() const {return mKind;}
    Assertion::Sense getSense() const {return mSense;}
    
    static Assertion::Kind reverseKind(Assertion::Kind k);
    static Assertion::Sense negateSense(Assertion::Sense s);
    static Assertion * Create(RE * asserted, Kind k, Sense s) {
        return new Assertion(asserted, k, s);
    }
    RE_SUBTYPE(Assertion)
private:
    Assertion(RE * r, Kind k, Sense s) : RE(ClassTypeId::Assertion), mAsserted(r), mKind(k), mSense(s) {}
    RE * mAsserted;
    Kind mKind;
    Sense mSense;
};

RE * makeAssertion(RE * asserted, Assertion::Kind k, Assertion::Sense s);

RE * makeLookAheadAssertion(RE * r);

RE * makeNegativeLookAheadAssertion(RE * r);

RE * makeLookBehindAssertion(RE * r);

RE * makeNegativeLookBehindAssertion(RE * r);

RE * makeBoundaryAssertion(RE * r);

RE * makeNegativeBoundaryAssertion(RE * r);

// Start-of-text boundary assertion.
RE * makeSOT();
    
// End-of-text boundary assertion.
RE * makeEOT();

} // namespace re

#endif // RE_ASSERTION_H

