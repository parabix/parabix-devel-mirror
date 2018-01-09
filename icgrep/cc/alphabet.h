/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef ALPHABET_H
#define ALPHABET_H

#include <string>
#include <UCD/unicode_set.h>
#include <vector>

namespace cc {
//
// An Alphabet is the universe of characters used to form strings in 
// a given language, together with a mapping of those characters to 
// numerical character codes.
//

class Alphabet {
public:
    const std::string & getName() const { return mAlphabetName;}
    virtual const unsigned getSize() const = 0;
    enum class ClassTypeId : unsigned {UnicodeMappableAlphabet, CodeUnitAlphabet, MultiplexedAlphabet};
    inline ClassTypeId getClassTypeId() const {
        return mClassTypeId;
    }

protected:
    Alphabet(std::string name, ClassTypeId k) : mAlphabetName(name), mClassTypeId(k) {}
private:
    const std::string mAlphabetName;
    const ClassTypeId mClassTypeId;
};

class UnicodeMappableAlphabet : public Alphabet {
public:
    //  Alphabets may be formed by some subset of Unicode characters, together
    //  with a mapping to and from Unicode.  The mapping is defined in terms of unicodeCommon:
    //  the number of character codes (if any) such that all character codes in the range
    //  0..unicodeCommon - 1 map to the same numeric value as the corresponding Unicode
    //  codepoint, together with a vector defining the Unicode codepoints for consecutive
    //  character codes (if any) above unicodeCommon - 1.
    
    UnicodeMappableAlphabet(std::string alphabetName,
                            unsigned unicodeCommon,
                            std::vector <UCD::codepoint_t> aboveCommon);
    
    static inline bool classof(const Alphabet * a) {
        return a->getClassTypeId() == ClassTypeId::UnicodeMappableAlphabet;
    }
    static inline bool classof(const void *) {return false;}
    //  The Unicode codepoint of the nth character (the character whose alphabet code is n).
    UCD::codepoint_t toUnicode(const unsigned n) const;
    
    //  The ordinal position of the character whose Unicode codepoint value is ucp.
    unsigned fromUnicode(const UCD::codepoint_t ucp) const;

    const unsigned getSize() const override {return mUnicodeCommon + mAboveCommon.size();}

protected:
    UCD::codepoint_t mCharSet;
    UCD::codepoint_t mUnicodeCommon;
    std::vector <UCD::codepoint_t> mAboveCommon;
};

class CodeUnitAlphabet : public Alphabet {
public:
    CodeUnitAlphabet(std::string alphabetName, uint8_t codeUnitBits);
    static inline bool classof(const Alphabet * a) {
        return a->getClassTypeId() == ClassTypeId::CodeUnitAlphabet;
    }
    static inline bool classof(const void *) {return false;}
    uint8_t getCodeUnitBitWidth() const { return mCodeUnitBits;}
    const unsigned getSize() const override {return 1<<mCodeUnitBits;}

private:
    uint8_t mCodeUnitBits;
};

//  Some important alphabets are predefined.

const extern UnicodeMappableAlphabet Unicode; // Unicode("Unicode", UCD::UNICODE_MAX, {})

const extern UnicodeMappableAlphabet ASCII;  // ASCII("ASCII", 0x7F, {});

const extern UnicodeMappableAlphabet ISO_Latin1; // ISO_Latin1("ISO_Latin1", 0xFF, {});

const extern CodeUnitAlphabet Byte; // Byte("Byte", 8);
    
const extern CodeUnitAlphabet UTF16; // UTF16("UTF16", 16);
    
}

#endif // ALPHABET_H


