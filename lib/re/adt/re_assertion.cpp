/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/adt/re_assertion.h>

#include <re/adt/re_start.h>
#include <re/adt/re_end.h>

using namespace llvm;

namespace re {

Assertion::Kind Assertion::reverseKind(Assertion::Kind k) {
    if (k == Assertion::Kind::Boundary) return k;
    return k == Assertion::Kind::LookAhead ? Assertion::Kind::LookBehind : Assertion::Kind::LookAhead;
}

Assertion::Sense Assertion::negateSense(Assertion::Sense s) {
    return s == Assertion::Sense::Positive ? Assertion::Sense::Negative : Assertion::Sense::Positive;
}

RE * makeAssertion(RE * asserted, Assertion::Kind k, Assertion::Sense s) {
    if (isEmptySet(asserted)) {
        if (s == Assertion::Sense::Negative) return makeSeq();
        else return makeAlt();
    }
    if (isEmptySeq(asserted)) {
        if (k == Assertion::Kind::Boundary) {
            if (s == Assertion::Sense::Positive) return makeAlt();
            else return makeSeq();
        }
        if (s == Assertion::Sense::Positive) return makeSeq();
        else return makeAlt();
    }
    return Assertion::Create(asserted, k, s);
}

RE * makeLookAheadAssertion(RE * r) {
    return makeAssertion(r, Assertion::Kind::LookAhead, Assertion::Sense::Positive);
}

RE * makeNegativeLookAheadAssertion(RE * r) {
    return makeAssertion(r, Assertion::Kind::LookAhead, Assertion::Sense::Negative);
}

RE * makeLookBehindAssertion(RE * r) {
    return makeAssertion(r, Assertion::Kind::LookBehind, Assertion::Sense::Positive);
}

RE * makeNegativeLookBehindAssertion(RE * r) {
    return makeAssertion(r, Assertion::Kind::LookBehind, Assertion::Sense::Negative);
}

RE * makeBoundaryAssertion(RE * r) {
    return makeAssertion(r, Assertion::Kind::Boundary, Assertion::Sense::Positive);
}

RE * makeNegativeBoundaryAssertion(RE * r) {
    return makeAssertion(r, Assertion::Kind::Boundary, Assertion::Sense::Negative);
}
    
RE * makeSOT () {
    //return makeNegativeLookBehindAssertion(makeByte(0x00,0xFF));
    return makeStart();
}

RE * makeEOT () {
    //return makeNegativeLookAheadAssertion(makeByte(0x00,0xFF));
    return makeEnd();
}

}
