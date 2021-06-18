/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_CC_H
#define RE_CC_H

#include <re/adt/re_re.h>
#include <re/alphabet/alphabet.h>
#include <unicode/core/unicode_set.h>

namespace re {

using codepoint_t = UCD::codepoint_t;
using interval_t = UCD::interval_t;

enum class CC_type {UnicodeClass, ByteClass};

class CC : public RE, public UCD::UnicodeSet {
public:

    static inline bool classof(const RE * re) {
        return re->getClassTypeId() == ClassTypeId::CC;
    }
    static inline bool classof(const void *) {
        return false;
    }

    const cc::Alphabet * getAlphabet() const { return mAlphabet;}

    std::string canonicalName() const;

    inline codepoint_t min_codepoint() const {
        return front().first;
    }

    inline codepoint_t max_codepoint() const {
        return back().second;
    }

    virtual ~CC() {}

    const CC* sourceCC;
protected:
    friend CC * makeCC(const cc::Alphabet * alphabet);
    friend CC * makeCC(const codepoint_t codepoint, const cc::Alphabet * alphabet);
    friend CC * makeCC(const codepoint_t lo, const codepoint_t hi, const cc::Alphabet * alphabet);
    friend CC * makeCC(const CC * cc1, const CC * cc2);
    friend CC * makeCC(std::initializer_list<interval_t> list, const cc::Alphabet * alphabet);
    friend CC * makeCC(std::vector<interval_t> && list, const cc::Alphabet * alphabet);
    friend CC * makeCC(UCD::UnicodeSet set, const cc::Alphabet * alphabet);
    friend bool intersects(const CC * a, const CC * b);
    friend CC * subtractCC(const CC * a, const CC * b);
    friend CC * intersectCC(const CC * a, const CC * b);
    friend CC * makeByte(const codepoint_t codepoint);
    friend CC * makeByte(const codepoint_t lo, const codepoint_t hi);

    CC(const cc::Alphabet * alphabet);

    CC(const CC & cc);

    CC(const codepoint_t codepoint, const cc::Alphabet * alphabet);

    explicit CC(const codepoint_t lo_codepoint, const codepoint_t hi_codepoint, const cc::Alphabet * alphabet);

    explicit CC(const CC * cc1, const CC * cc2);

    CC(const UCD::UnicodeSet set, const cc::Alphabet * alphabet);

    CC(std::initializer_list<interval_t>::iterator begin, std::initializer_list<interval_t>::iterator end, const cc::Alphabet * alphabet);

    CC(const std::vector<interval_t>::iterator begin, const std::vector<interval_t>::iterator end, const cc::Alphabet * alphabet);
private:
    const cc::Alphabet * mAlphabet;
    

};

inline static CC::iterator begin(const CC & cc) {
    return cc.begin();
}

inline static CC::iterator end(const CC & cc) {
    return cc.end();
}

inline codepoint_t lo_codepoint(const interval_t & i) {
    return std::get<0>(i);
}
inline codepoint_t lo_codepoint(const CC::iterator i) {
    return lo_codepoint(*i);
}

inline codepoint_t hi_codepoint(const interval_t & i) {
    return std::get<1>(i);
}
inline codepoint_t hi_codepoint(const CC::iterator i) {
    return hi_codepoint(*i);
}

/**
 * @brief RE::makeCC
 *
 * Various factory constructors for the RE CC class
 *
 * @return a CC object
 */

inline CC * makeCC(const cc::Alphabet * alphabet = &cc::Unicode) {
    return new CC(alphabet);
}

inline CC * makeCC(const codepoint_t codepoint, const cc::Alphabet * alphabet = &cc::Unicode) {
    return new CC(codepoint, alphabet);
}

inline CC * makeCC(const codepoint_t lo, const codepoint_t hi, const cc::Alphabet * alphabet = &cc::Unicode) {
    return new CC(lo, hi, alphabet);
}

inline CC * makeCC(const CC * cc1, const CC * cc2) {
    return new CC(cc1, cc2);
}

inline CC * makeCC(std::initializer_list<interval_t> list, const cc::Alphabet * alphabet = &cc::Unicode) {
    return new CC(list.begin(), list.end(), alphabet);
}

inline CC * makeCC(std::vector<interval_t> && list, const cc::Alphabet * alphabet = &cc::Unicode) {
    return new CC(list.begin(), list.end(), alphabet);
}

inline CC * makeCC(UCD::UnicodeSet set, const cc::Alphabet * alphabet = &cc::Unicode) {
    return new CC(set, alphabet);
}

inline CC * subtractCC(const CC * a, const CC * b) {
    //assert (a->getAlphabet() == b->getAlphabet());
    return new CC(*a - *b, a->getAlphabet());
}

inline CC * intersectCC(const CC * a, const CC * b) {
    //assert (a->getAlphabet() == b->getAlphabet());
    return new CC(*a & *b, a->getAlphabet());
}

inline bool intersects(const CC * a, const CC * b) {
    return (*a).intersects(*b);
}

inline bool subset(const CC * a, const CC * b) {
    return (*a).subset(*b);
}

inline CC * makeByte(const codepoint_t codepoint) {
    return new CC(codepoint, &cc::Byte);
}

inline CC * makeByte(const codepoint_t lo, const codepoint_t hi) {
    return new CC(lo, hi, &cc::Byte);
}
    
}

#endif // RE_CC_H
