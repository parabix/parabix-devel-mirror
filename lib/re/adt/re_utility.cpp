/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/adt/re_utility.h>
#include <re/adt/adt.h>

namespace re {
    
RE * makeComplement(RE * s) {
  return makeDiff(makeAny(), s);
}

RE * makeZerowidthComplement(RE * s) {
    return makeDiff(makeSeq({}), s);
}

RE * makeWordBoundary() {
    auto wordC = makePropertyExpression("word");
    return makeBoundaryAssertion(wordC);
}

RE * makeWordNonBoundary() {
    auto wordC = makePropertyExpression("word");
    return makeNegativeBoundaryAssertion(wordC);
}

RE * makeWordBegin() {
    auto wordC = makePropertyExpression("word");
    return makeNegativeLookBehindAssertion(wordC);
}

RE * makeWordEnd() {
    auto wordC = makePropertyExpression("word");
    return makeNegativeLookAheadAssertion(wordC);
}

RE * makeUnicodeBreak() {
    return makeAlt({makeCC(0x0A, 0x0C), makeCC(0x85), makeCC(0x2028,0x2029), makeSeq({makeCC(0x0D), makeNegativeLookAheadAssertion(makeCC(0x0A))})});
}
    
}
