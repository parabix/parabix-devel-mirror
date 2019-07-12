/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */


#include <re/cc/alphabet.h>
#include <llvm/Support/ErrorHandling.h>

namespace cc {
    
UnicodeMappableAlphabet::UnicodeMappableAlphabet(const std::string alphabetName,
                                                 unsigned unicodeCommon,
                                                 std::vector <UCD::codepoint_t> aboveCommon)
: Alphabet(std::move(alphabetName), ClassTypeId::UnicodeMappableAlphabet),
mUnicodeCommon(unicodeCommon),
mAboveCommon(std::move(aboveCommon)) {

}

UCD::codepoint_t UnicodeMappableAlphabet::toUnicode(const unsigned n) const {
    UCD::codepoint_t cp = n;
    if (n < mUnicodeCommon) return cp;
    assert(n < mUnicodeCommon + mAboveCommon.size());
    return mAboveCommon[n - mUnicodeCommon];
}
  
unsigned UnicodeMappableAlphabet::fromUnicode(const UCD::codepoint_t codepoint) const {
    unsigned n = codepoint;
    if (n < mUnicodeCommon) return n;
    for (unsigned i = 0; i < mAboveCommon.size(); i++) {
        if (mAboveCommon[i] == codepoint) return mUnicodeCommon + i;
    }
    llvm::report_fatal_error("fromUnicode: codepoint not found in alphabet.");
}

CodeUnitAlphabet::CodeUnitAlphabet(const std::string alphabetName, uint8_t bits) :
Alphabet(std::move(alphabetName), ClassTypeId::CodeUnitAlphabet)
, mCodeUnitBits(bits) {

}

const UnicodeMappableAlphabet Unicode("Unicode", UCD::UNICODE_MAX, {});

const UnicodeMappableAlphabet ASCII("ASCII", 0x7F, {});

const UnicodeMappableAlphabet ISO_Latin1("ISO_Latin1", 0xFF, {});

const CodeUnitAlphabet Byte("Byte", 8);
    
const CodeUnitAlphabet UTF8("UTF8", 8);

const CodeUnitAlphabet UTF16("UTF16", 16);
    
}
