/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <UCD/unicode_set.h>
using codepoint_t = UCD::codepoint_t;

template <int Bits> class UTF {
public:
    static inline unsigned encoded_length(codepoint_t cp);
    static inline codepoint_t max_codepoint_of_length(unsigned lgth);
    static inline bool isLowCodePointAfterNthCodeUnit(codepoint_t cp, unsigned n);
    static inline bool isHighCodePointAfterNthCodeUnit(codepoint_t cp, unsigned n);
    static inline codepoint_t minCodePointWithCommonCodeUnits(codepoint_t cp, unsigned common);
    static inline codepoint_t maxCodePointWithCommonCodeUnits(codepoint_t cp, unsigned common);
    static inline unsigned nthCodeUnit(codepoint_t cp, unsigned n);
};
    
template<> inline unsigned UTF<8>::encoded_length(codepoint_t cp) {
    if (cp <= 0x7F) return 1;
    else if (cp <= 0x7FF) return 2;
    else if (cp <= 0xFFFF) return 3;
    else return 4;
}

template<> inline codepoint_t UTF<8>::max_codepoint_of_length(unsigned length) {
    if (length == 1) return 0x7F;
    else if (length == 2) return 0x7FF;
    else if (length == 3) return 0xFFFF;
    else {
        assert(length == 4);
        return 0x10FFFF;
    }
}

template <> inline unsigned UTF<8>::nthCodeUnit(codepoint_t cp, unsigned n) {
    const auto length = UTF<8>::encoded_length(cp);
    if (n == 1) {
        switch (length) {
            case 1: return static_cast<unsigned>(cp);
            case 2: return static_cast<unsigned>(0xC0 | (cp >> 6));
            case 3: return static_cast<unsigned>(0xE0 | (cp >> 12));
            case 4: return static_cast<unsigned>(0xF0 | (cp >> 18));
        }
    }
    else return static_cast<unsigned>(0x80 | ((cp >> (6 * (length - n))) & 0x3F));
}

template<> inline codepoint_t UTF<8>::minCodePointWithCommonCodeUnits(codepoint_t cp, unsigned common) {
    const auto length = UTF<8>::encoded_length(cp);
    const auto mask = (static_cast<codepoint_t>(1) << (length - common) * 6) - 1;
    const auto lo_cp = cp &~ mask;
    return (lo_cp == 0) ? mask + 1 : lo_cp;
}

template<> inline codepoint_t UTF<8>::maxCodePointWithCommonCodeUnits(codepoint_t cp, unsigned common) {
    const auto length = UTF<8>::encoded_length(cp);
    const auto mask = (static_cast<codepoint_t>(1) << (length - common) * 6) - 1;
    return cp | mask;
}

// Generic for UTF<8> or UTF<16>

template<int Bits> inline bool UTF<Bits>::isLowCodePointAfterNthCodeUnit(codepoint_t cp, unsigned n) {
    if (cp == 0) return true;
    else if (UTF<Bits>::encoded_length(cp - 1) < UTF<Bits>::encoded_length(cp)) return true;
    else return UTF<Bits>::nthCodeUnit(cp - 1, n) != UTF<Bits>::nthCodeUnit(cp, n);
}

template<int Bits> inline bool UTF<Bits>::isHighCodePointAfterNthCodeUnit(codepoint_t cp, unsigned n) {
    if (cp == 0x10FFFF) return true;
    else if (UTF<Bits>::encoded_length(cp + 1) > UTF<Bits>::encoded_length(cp)) return true;
    else return UTF<Bits>::nthCodeUnit(cp + 1, n) != UTF<Bits>::nthCodeUnit(cp, n);
}

// UTF-16

template<> inline unsigned UTF<16>::encoded_length(codepoint_t cp) {
    if (cp <= 0xFFFF) return 1;
    else return 2;
}

template<> inline unsigned UTF<16>::max_codepoint_of_length(unsigned length) {
    if (length == 1) return 0xFFFF;
    else {
        assert(length == 2);
        return 0x10FFFF;
    }
}

template <> inline unsigned UTF<16>::nthCodeUnit(codepoint_t cp, unsigned n) {
    const auto length = UTF<16>::encoded_length(cp);
    if (length == 1) {
        assert(n == 1);
        return static_cast<unsigned>(cp);
    }
    else if (n == 1)
        return static_cast<unsigned>(0xD800 | ((cp - 0x10000) >> 10));
    else
        return static_cast<unsigned>(0xDC00 | ((cp - 0x10000) & 0x3FF));
}

template<> inline codepoint_t UTF<16>::minCodePointWithCommonCodeUnits(codepoint_t cp, unsigned common) {
    const auto len = UTF<16>::encoded_length(cp);
    if (len == common) return cp;
    else {
        assert(len == 2);
        assert(common == 1);
        return cp & 0x1FFC00;
    }
}

template<> inline codepoint_t UTF<16>::maxCodePointWithCommonCodeUnits(codepoint_t cp, unsigned common) {
    const auto len = UTF<16>::encoded_length(cp);
    const auto mask = (static_cast<codepoint_t>(1) << (len - common) * 10) - 1;
    return cp | mask;
}
