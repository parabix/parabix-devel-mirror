/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <unicode/core/unicode_set.h>
#include <assert.h>
#include <algorithm>
#include <stdexcept>
#include <string>

using codepoint_t = UCD::codepoint_t;

template <int Bits> class UTF_Encoder {
public:
    static inline bool isHi_Surrogate(const codepoint_t cp);
    static inline bool isLo_Surrogate(const codepoint_t cp);
    static inline unsigned length(const codepoint_t cp);
    static inline codepoint_t maxCodePoint(const unsigned length);
    static inline codepoint_t encodingByte(const codepoint_t cp, const unsigned n);
    static inline bool isLowCodePointAfterByte(const codepoint_t cp, const unsigned n);
    static inline bool isHighCodePointAfterByte(const codepoint_t cp, const unsigned n);
    static inline codepoint_t minCodePointWithCommonBytes(const codepoint_t cp, const unsigned n);
    static inline codepoint_t maxCodePointWithCommonBytes(const codepoint_t cp, const unsigned n);
    
    static inline bool isPrefix(const codepoint_t cp);
    //static unsigned length(const UCD::codepoint_t cp);
    //static UCD::codepoint_t maxCodePoint(const unsigned length);
    //static UCD::codepoint_t encodingByte(const UCD::codepoint_t cp, const unsigned n);
    //static bool isLowCodePointAfterByte(const UCD::codepoint_t cp, const unsigned n);
    //static bool isHighCodePointAfterByte(const UCD::codepoint_t cp, const unsigned n);
    //static UCD::codepoint_t minCodePointWithCommonBytes(const UCD::codepoint_t cp, const unsigned n);
    //static UCD::codepoint_t maxCodePointWithCommonBytes(const UCD::codepoint_t cp, const unsigned n);
};

template<> inline unsigned UTF_Encoder<8>::length(const codepoint_t cp) {
    if (cp <= 0x7F) {
        return 1;
    }
    else if (cp <= 0x7FF) {
        return 2;
    }
    else if (cp <= 0xFFFF) {
        return 3;
    }
    else {
        return 4;
    }
}

template<> inline bool UTF_Encoder<8>::isPrefix(const codepoint_t cp) {
    return (cp >= 0xC2) && (cp <= 0xF4);
}

template<> inline codepoint_t UTF_Encoder<8>::encodingByte(const codepoint_t cp, const unsigned n) {
    codepoint_t retVal = 0;
    const unsigned len = length(cp);
    if (n == 1) {
        switch (len) {
            case 1: retVal = cp; break;
            case 2: retVal = 0xC0 | (cp >> 6); break;
            case 3: retVal = 0xE0 | (cp >> 12); break;
            case 4: retVal = 0xF0 | (cp >> 18); break;
        }
    }
    else {
        retVal = 0x80 | ((cp >> (6 * (len - n))) & 0x3F);
    }
    return retVal;
}

template<> inline codepoint_t UTF_Encoder<8>::maxCodePoint(const unsigned length) {
    if (length == 1) {
        return 0x7F;
    }
    else if (length == 2) {
        return 0x7FF;
    }
    else if (length == 3) {
        return 0xFFFF;
    }
    else if (length == 4) {
        return 0x10FFFF;
    }
    throw std::runtime_error("Unexpected UTF8 Length: " + std::to_string(length));
}

template<> inline bool UTF_Encoder<8>::isLowCodePointAfterByte(const codepoint_t cp, const unsigned n) {
    const auto l = length(cp);
    for (auto i = n; i != l; ++i) {
        if (encodingByte(cp, i + 1) != 0x80) {
            return false;
        }
    }
    return true;
}

template<> inline bool UTF_Encoder<8>::isHighCodePointAfterByte(const codepoint_t cp, const unsigned n) {
    const auto l = length(cp);
    for (auto i = n; i != l; ++i) {
        if (encodingByte(cp, i + 1) != 0xBF) {
            return false;
        }
    }
    return true;
}

template<> inline codepoint_t UTF_Encoder<8>::minCodePointWithCommonBytes(const UCD::codepoint_t cp, const unsigned n) {
    const auto len = length(cp);
    const auto mask = (static_cast<codepoint_t>(1) << (len - n) * 6) - 1;
    const auto lo_cp = cp &~ mask;
    return (lo_cp == 0) ? mask + 1 : lo_cp;
}

template<> inline codepoint_t UTF_Encoder<8>::maxCodePointWithCommonBytes(const UCD::codepoint_t cp, const unsigned n) {
    const auto len = length(cp);
    const auto mask = (static_cast<codepoint_t>(1) << (len - n) * 6) - 1;
    return cp | mask;
}

// UTF-16

template<> inline unsigned UTF_Encoder<16>::length(const codepoint_t cp) {
    if (cp <= 0xFFFF) return 1;
    else return 2;
}

template<> inline bool UTF_Encoder<16>::isHi_Surrogate(const codepoint_t cp) {
    return (cp >= 0xD800) && (cp <= 0xDBFF);
}

template<> inline bool UTF_Encoder<16>::isLo_Surrogate(const codepoint_t cp) {
    return (cp >= 0xDC00) && (cp <= 0xDFFF);
}

template<> inline codepoint_t UTF_Encoder<16>::encodingByte(const codepoint_t cp, const unsigned n) {
    codepoint_t retVal = 0;
    const unsigned len = length(cp);
    if (len == 1) {
	retVal = cp;
    }
    else {
	codepoint_t code = cp - 0x010000;
	if (n == 1) {
		retVal = (code >> 10) | 0xD800;
	}
	if (n == 2) {
		retVal = (code & 0x3FF) | 0xDC00;
	}
    }
    return retVal;
}

template<> inline codepoint_t UTF_Encoder<16>::maxCodePoint(const unsigned length) {
    if (length == 1) {
	return 0xFFFF;
    }
    else if (length == 2) {
	return 0x10FFFF;
    } 
    throw std::runtime_error("Unexpected UTF16 Length: " + std::to_string(length));
}

template<> inline bool UTF_Encoder<16>::isLowCodePointAfterByte(const codepoint_t cp, const unsigned n) {
    const auto l = length(cp);
    for (auto i = n; i != l; ++i) {
        if (encodingByte(cp, i + 1) != 0xDC00) {
            return false;
        }
    }
    return true;
}

template<> inline bool UTF_Encoder<16>::isHighCodePointAfterByte(const codepoint_t cp, const unsigned n) {
    const auto l = length(cp);
    for (auto i = n; i != l; ++i) {
        if (encodingByte(cp, i + 1) != 0xDFFF) {
            return false;
        }
    }
    return true;
}

template<> inline codepoint_t UTF_Encoder<16>::minCodePointWithCommonBytes(const codepoint_t cp, const unsigned n) {
    const auto len = length(cp);
    const auto mask = (static_cast<codepoint_t>(1) << (len - n) * 10) - 1;
    const auto lo_cp = cp &~ mask;
    return (lo_cp == 0) ? mask + 1 : lo_cp;
}

template<> inline codepoint_t UTF_Encoder<16>::maxCodePointWithCommonBytes(const codepoint_t cp, const unsigned n) {
    const auto len = length(cp);
    const auto mask = (static_cast<codepoint_t>(1) << (len - n) * 10) - 1;
    return cp | mask;
}