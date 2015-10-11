/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_ASSERTION_H
#define RE_ASSERTION_H

#include <re/re_re.h>

namespace re {

class Assertion : public RE {
public:
    static inline bool classof(const RE * re) {
        return re->getClassTypeId() == ClassTypeId::Assertion;
    }
    static inline bool classof(const void *) {
        return false;
    }
    enum class Kind {Lookbehind, Lookahead};
    enum class Sense {Positive, Negative};

    RE * getAsserted() const {return mAsserted;}
    Assertion::Kind getKind() const {return mKind;}
    Assertion::Sense getSense() const {return mSense;}
    void setAsserted(RE * r) {mAsserted = r;}


protected:
    friend Assertion * makeAssertion(RE * asserted, Kind k, Sense s);
    Assertion(RE * r, Kind k, Sense s) : RE(ClassTypeId::Assertion), mAsserted(r), mKind(k), mSense(s) {}
    virtual ~Assertion() {}

private:
    RE * mAsserted;
    Kind mKind;
    Sense mSense;
};

inline Assertion * makeAssertion(RE * asserted, Assertion::Kind k, Assertion::Sense s) {
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

}

#endif // RE_ASSERTION_H

