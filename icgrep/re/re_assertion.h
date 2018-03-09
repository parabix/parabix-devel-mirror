/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_ASSERTION_H
#define RE_ASSERTION_H

#include <re/re_re.h>
#include <re/re_nullable.h>
#include <re/re_seq.h>
#include <re/re_alt.h>

namespace re {

class Assertion : public RE {
public:
    static inline bool classof(const RE * re) {
        return re->getClassTypeId() == ClassTypeId::Assertion;
    }
    static inline bool classof(const void *) {
        return false;
    }
    enum class Kind {Lookbehind, Lookahead, Boundary};
    enum class Sense {Positive, Negative};

    RE * getAsserted() const {return mAsserted;}
    Assertion::Kind getKind() const {return mKind;}
    Assertion::Sense getSense() const {return mSense;}
    void setAsserted(RE * r) {mAsserted = r;}

    static Assertion::Kind reverseKind(Assertion::Kind k);
    static Assertion::Sense negateSense(Assertion::Sense s);

protected:
    friend RE * makeAssertion(RE * asserted, Kind k, Sense s);
    Assertion(RE * r, Kind k, Sense s) : RE(ClassTypeId::Assertion), mAsserted(r), mKind(k), mSense(s) {}
    virtual ~Assertion() {}

private:
    RE * mAsserted;
    Kind mKind;
    Sense mSense;
};

inline Assertion::Kind Assertion::reverseKind(Assertion::Kind k) {
    if (k == Assertion::Kind::Boundary) return k;
    return k == Assertion::Kind::Lookahead ? Assertion::Kind::Lookbehind : Assertion::Kind::Lookahead;
}

inline Assertion::Sense Assertion::negateSense(Assertion::Sense s) {
    return s == Assertion::Sense::Positive ? Assertion::Sense::Negative : Assertion::Sense::Positive;
}

inline RE * makeAssertion(RE * asserted, Assertion::Kind k, Assertion::Sense s) {
    if (RE_Nullable::isNullable(asserted)) {
        if (k == Assertion::Kind::Boundary) {
            if (s == Assertion::Sense::Positive) return makeAlt();
            else return makeSeq();
        }
        if (s == Assertion::Sense::Positive) return makeSeq();
        else return makeAlt();
    }
    return new Assertion(asserted, k, s);
}

inline RE * makeLookAheadAssertion(RE * r) {
    return makeAssertion(r, Assertion::Kind::Lookahead, Assertion::Sense::Positive);
}

inline RE * makeNegativeLookAheadAssertion(RE * r) {
    return makeAssertion(r, Assertion::Kind::Lookahead, Assertion::Sense::Negative);
}

inline RE * makeLookBehindAssertion(RE * r) {
    return makeAssertion(r, Assertion::Kind::Lookbehind, Assertion::Sense::Positive);
}

inline RE * makeNegativeLookBehindAssertion(RE * r) {
    return makeAssertion(r, Assertion::Kind::Lookbehind, Assertion::Sense::Negative);
}

inline RE * makeBoundaryAssertion(RE * r) {
    return makeAssertion(r, Assertion::Kind::Boundary, Assertion::Sense::Positive);
}

inline RE * makeNegativeBoundaryAssertion(RE * r) {
    return makeAssertion(r, Assertion::Kind::Boundary, Assertion::Sense::Negative);
}

// Start-of-text boundary assertion.
RE * makeSOT();
    
// End-of-text boundary assertion.
RE * makeEOT();
    
RE * expandBoundaryAssertion(RE * r);
}

#endif // RE_ASSERTION_H

