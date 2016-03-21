/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_CC_H
#define RE_CC_H

#include "re_re.h"

#include <string>
#include <vector>
#include <UCD/unicode_set.h>
#include <util/slab_allocator.h>

namespace re {

using codepoint_t = UCD::UnicodeSet::codepoint_t;
using interval_t = UCD::UnicodeSet::interval_t;

enum CC_type {UnicodeClass, ByteClass};

class CC : public RE, public UCD::UnicodeSet {
public:

    static inline bool classof(const RE * re) {
        return re->getClassTypeId() == ClassTypeId::CC;
    }
    static inline bool classof(const void *) {
        return false;
    }


    std::string canonicalName(const CC_type type) const;

    inline codepoint_t min_codepoint() const {
        return front().first;
    }

    inline codepoint_t max_codepoint() const {
        return back().second;
    }

    virtual ~CC() {}

protected:
    friend CC * makeCC();
    friend CC * makeCC(const codepoint_t codepoint);
    friend CC * makeCC(const codepoint_t lo, const codepoint_t hi);
    friend CC * makeCC(const CC * cc1, const CC * cc2);
    friend CC * makeCC(std::initializer_list<interval_t> list);
    friend CC * makeCC(std::vector<interval_t> && list);
    friend CC * makeCC(UCD::UnicodeSet && set);
    friend CC * subtractCC(const CC * a, const CC * b);
    friend CC * intersectCC(const CC * a, const CC * b);
    friend CC * caseInsensitize(const CC * a, const CC * b);

    inline CC()
    : RE(ClassTypeId::CC) {

    }

    CC(const CC & cc);

    inline CC(const codepoint_t codepoint)
    : RE(ClassTypeId::CC)
    , UCD::UnicodeSet(codepoint) {

    }

    inline CC(const codepoint_t lo_codepoint, const codepoint_t hi_codepoint)
    : RE(ClassTypeId::CC)
    , UCD::UnicodeSet(lo_codepoint, hi_codepoint) {

    }

    CC(const CC * cc1, const CC * cc2);

    inline CC(UCD::UnicodeSet && set)
    : RE(ClassTypeId::CC)
    , UCD::UnicodeSet(std::move(set)) {

    }

    CC(std::initializer_list<interval_t>::iterator begin, std::initializer_list<interval_t>::iterator end)
    : RE(ClassTypeId::CC)
    , UCD::UnicodeSet(begin, end)
    {

    }

    CC(const std::vector<interval_t>::iterator begin, const std::vector<interval_t>::iterator end)
    : RE(ClassTypeId::CC)
    , UCD::UnicodeSet(begin, end)
    {

    }

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

inline CC * makeCC() {
    return new CC();
}

inline CC * makeCC(const codepoint_t codepoint) {
    return new CC(codepoint);
}

inline CC * makeCC(const codepoint_t lo, const codepoint_t hi) {
    return new CC(lo, hi);
}

inline CC * makeCC(const CC * cc1, const CC * cc2) {
    return new CC(cc1, cc2);
}

inline CC * makeCC(std::initializer_list<interval_t> list) {
    return new CC(list.begin(), list.end());
}

inline CC * makeCC(std::vector<interval_t> && list) {
    return new CC(list.begin(), list.end());
}

inline CC * makeCC(UCD::UnicodeSet && set) {
    return new CC(std::move(set));
}

inline CC * subtractCC(const CC * a, const CC * b) {
    return new CC(std::move(*a - *b));
}

inline CC * intersectCC(const CC * a, const CC * b) {
    return new CC(std::move(*a & *b));
}

CC * caseInsensitize(const CC * cc);

}

#endif // RE_CC_H
