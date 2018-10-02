/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "re_utility.h"
#include <re/re_any.h>
#include <re/re_name.h>
#include <re/re_cc.h>
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_alt.h>
#include <re/re_seq.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_group.h>
#include <re/re_range.h>
#include <re/re_assertion.h>
#include <re/printer_re.h>
#include <llvm/Support/Casting.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/ErrorHandling.h>

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

RE * makeUnicodeBreak() {
    return makeAlt({makeCC(0x0A, 0x0C), makeCC(0x85), makeCC(0x2028,0x2029), makeSeq({makeCC(0x0D), makeNegativeLookAheadAssertion(makeCC(0x0A))})});
}
    
}
