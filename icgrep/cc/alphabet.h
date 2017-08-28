/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef ALPHABET_H
#define ALPHABET_H

#include <string>
#include <UCD/unicode_set.h>

//
// An Alphabet is the universe of characters used to form strings in 
// a given language, together with a mapping of those characters to 
// numerical character codes.
//

class Alphabet {
public:

    //  Alphabets may simply be a subset of Unicode characters including all
    //  characters up to and including a given maximum Unicode codepoint.
    
    Alphabet(std::string alphabetName, UCD::codepoint_t maxChar) :
        mAlphabetName(alphabetName), mCharSet(UCD::UnicodeSet(0, maxChar)) {}
        
    const std::string & getName() const { return mAlphabetName;}
    
    const UCD::UnicodeSet & getSet() const { return mCharSet;}
    
    //  The Unicode codepoint of the nth character (the character whose alphabet code is n).
    virtual UCD::codepoint_t toUnicode(const unsigned n) const;
    
    //  The ordinal position of the character whose Unicode codepoint value is ucp.
    virtual unsigned fromUnicode(const UCD::codepoint_t ucp) const;

protected:
    std::string mAlphabetName;
    UCD::UnicodeSet mCharSet;
};


Alphabet Unicode("Unicode", UCD::UNICODE_MAX);

Alphabet ASCII("ASCII", 0x7F);

Alphabet ISO_Latin1("ISO_Latin1", 0xFF);


// Extended ASCII alphabets can be defined with a table of 128 entries defining 
// the codepoints for codes in the 0x80 to 0xFF range.
//
// ExtendedASCII<uint16_t> uses compact tables of 16-bit entries, while
// ExtendedASCII<uint32_t> uses tables of 32-bit entries, necessary if any
// codepoint is above 0xFFFF.

template <class uint_t> class ExtendedASCII : public Alphabet {
public:
    ExtendedASCII(std::string alphabetName, const uint_t (& extendedTable)[128]);
    UCD::codepoint_t toUnicode(const unsigned n) const final;
    unsigned fromUnicode(const UCD::codepoint_t ucp) const final;
private:
    const uint_t (& mExtendedCharacterTable)[128];
};


#endif // ALPHABET_H


