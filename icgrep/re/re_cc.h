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
#include <slab_allocator.h>

namespace re {

using codepoint_t = UCD::UnicodeSet::codepoint_t;
using interval_t = UCD::UnicodeSet::interval_t;

enum CC_type {UnicodeClass, ByteClass};

class CC : public RE {
public:

    static inline bool classof(const RE * re) {
        return re->getClassTypeId() == ClassTypeId::CC;
    }
    static inline bool classof(const void *) {
        return false;
    }

    using iterator = UCD::UnicodeSet::iterator;
    using size_type = UCD::UnicodeSet::size_type;

    std::string canonicalName(const CC_type type) const;

    inline codepoint_t min_codepoint() const {
        return mSparseCharSet.front().first;
    }

    inline codepoint_t max_codepoint() const {
        return mSparseCharSet.back().second;
    }

    void insert_range(const codepoint_t lo, const codepoint_t hi) {
        mSparseCharSet.insert_range(lo, hi);
    }

    inline void insert(const codepoint_t codepoint) {
        mSparseCharSet.insert(codepoint);
    }

    inline iterator begin() const {
        return mSparseCharSet.begin();
    }

    inline iterator end() const {
        return mSparseCharSet.end();
    }

    inline interval_t front() const {
        return mSparseCharSet.front();
    }

    inline interval_t back() const {
        return mSparseCharSet.back();
    }

    inline size_type size() const {
        return mSparseCharSet.size();
    }

    inline bool empty() const {
        return mSparseCharSet.empty();
    }

    virtual ~CC() {}

protected:
    friend CC * makeCC();
    friend CC * makeCC(const codepoint_t codepoint);
    friend CC * makeCC(const codepoint_t lo, const codepoint_t hi);
    friend CC * makeCC(const CC * cc1, const CC * cc2);
    friend CC * makeCC(const std::initializer_list<interval_t> list);
    friend CC * makeCC(const std::vector<interval_t> & list);
    friend CC * makeCC(UCD::UnicodeSet && set);
    friend CC * subtractCC(const CC * a, const CC * b);
    friend CC * intersectCC(const CC * a, const CC * b);
    friend CC * caseInsensitize(const CC * a, const CC * b);

    inline CC()
    : RE(ClassTypeId::CC)
    , mSparseCharSet() {

    }
    CC(const CC & cc);
    inline CC(const codepoint_t codepoint)
    : RE(ClassTypeId::CC)
    , mSparseCharSet(codepoint) {

    }
    inline CC(const codepoint_t lo_codepoint, const codepoint_t hi_codepoint)
    : RE(ClassTypeId::CC)
    , mSparseCharSet(lo_codepoint, hi_codepoint) {

    }
    CC(const CC * cc1, const CC * cc2);

    inline CC(UCD::UnicodeSet && set)
    : RE(ClassTypeId::CC)
    , mSparseCharSet(std::move(set)) {

    }

    template <typename itr>
    CC * initialize(itr begin, itr end);
private:    
    UCD::UnicodeSet mSparseCharSet;
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

template<typename itr>
CC * CC::initialize(itr begin, itr end) {
    for (auto i = begin; i != end; ++i) {
        mSparseCharSet.insert_range(i->first, i->second);
    }
    return this;
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

inline CC * makeCC(const std::initializer_list<interval_t> list) {
    return makeCC()->initialize(list.begin(), list.end());
}

inline CC * makeCC(const std::vector<interval_t> & list) {
    return makeCC()->initialize(list.begin(), list.end());
}

inline CC * makeCC(UCD::UnicodeSet && set) {
    return makeCC(std::move(set));
}

CC * subtractCC(const CC * a, const CC * b);
    
CC * intersectCC(const CC * cc1, const CC * cc2);

CC * caseInsensitize(const CC * cc);

}

#endif // RE_CC_H
