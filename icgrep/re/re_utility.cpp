/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "re_utility.h"
#include <re/re_any.h>
#include <re/re_name.h>
#include <re/re_alt.h>
#include <re/re_seq.h>
#include <re/re_diff.h>
#include <re/re_assertion.h>

namespace re {
    
RE * makeComplement(RE * s) {
  return makeDiff(makeAny(), s);
}

                           
Name * makeDigitSet() {
    return makeName("nd", Name::Type::UnicodeProperty);
}

Name * makeAlphaNumeric() {
    return makeName("alnum", Name::Type::UnicodeProperty);
}

Name * makeWhitespaceSet() {
    return makeName("whitespace", Name::Type::UnicodeProperty);
}

Name * makeWordSet() {
    return makeName("word", Name::Type::UnicodeProperty);
}

RE * makeWordBoundary() {
    Name * wordC = makeWordSet();
    return makeAlt({makeSeq({makeNegativeLookBehindAssertion(wordC), makeLookAheadAssertion(wordC)}),
        makeSeq({makeLookBehindAssertion(wordC), makeNegativeLookAheadAssertion(wordC)})});
}

RE * makeWordNonBoundary() {
    Name * wordC = makeWordSet();
    return makeAlt({makeSeq({makeNegativeLookBehindAssertion(wordC), makeNegativeLookAheadAssertion(wordC)}),
        makeSeq({makeLookBehindAssertion(wordC), makeLookAheadAssertion(wordC)})});
}

RE * makeWordBegin() {
    Name * wordC = makeWordSet();
    return makeNegativeLookBehindAssertion(wordC);
}

RE * makeWordEnd() {
    Name * wordC = makeWordSet();
    return makeNegativeLookAheadAssertion(wordC);
}

    
}
