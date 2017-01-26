/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */


#include "alphabet.h"


// Default implementation for simple Unicode subsets.  The codepoint value
// of the nth character is just the given value n, if it is in range.

UCD::codepoint_t Alphabet::toUnicode(unsigned n) {
    UCD::codepoint_t cp = n;
    if (mCharSet.contains(cp)) return cp;
    throw std::runtime_error("toUnicode: n too large.");
}   
  
// Default implementation for simple Unicode subsets.  The ord value
// of a Unicode codepoint is just the given codepoint, if it is in range.

unsigned Alphabet::fromUnicode(UCD::codepoint_t codepoint) {
    if (mCharSet.contains(codepoint)) return codepoint;
    throw std::runtime_error("fromUnicode: codepoint not found in alphabet.");
}


template <class uint_t> ExtendedASCII<uint_t>::ExtendedASCII(std::string alphabetName, const uint_t (& extendedTable)[128]) {
    mAlphabetName = alphabetName;
    mExtendedCharacterTable = extendedTable;
    mCharSet = UCD::UnicodeSet(0, 127);
    for (unsigned i = 0; i < 128; i++) {
        mCharSet.insert(extendedTable[i]);
    }
}   

template <class uint_t> UCD::codepoint_t ExtendedASCII<uint_t>::toUnicode(unsigned n) {
    //  The first 128 characters are just ASCII.
    if (n < 128) return n;
    if (n < 256) return mExtendedCharacterTable[n-128];
    throw std::runtime_error("toUnicode: n too large.");
}   

template <class uint_t> unsigned ExtendedASCII<uint_t>::fromUnicode(UCD::codepoint_t codepoint) {
    if (codepoint < 128) return codepoint;
    for (unsigned i = 0; i < 128; i++) {
        if (mExtendedCharacterTable[i] == codepoint) return i + 128;
    }
    throw std::runtime_error("fromUnicode: codepoint not found in alphabet.");
}

