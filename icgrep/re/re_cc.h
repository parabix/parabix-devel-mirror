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
#include <slab_allocator.h>

namespace re {

using codepoint_t = unsigned;
using interval_t = std::pair<codepoint_t, codepoint_t>;

enum CC_type {UnicodeClass, ByteClass};

class CC : public RE {
public:

    static inline bool classof(const RE * re) {
        return re->getClassTypeId() == ClassTypeId::CC;
    }
    static inline bool classof(const void *) {
        return false;
    }

    using IntervalAllocator = SlabAllocator<interval_t>;
    using IntervalVector = std::vector<interval_t, IntervalAllocator>;

    using iterator = IntervalVector::iterator;
    using const_iterator = IntervalVector::const_iterator;
    using size_type = IntervalVector::size_type;
    using reference = IntervalVector::reference;
    using const_reference = IntervalVector::const_reference;

    static const codepoint_t UNICODE_MAX = 0x10FFFF;

    std::string canonicalName(const CC_type type) const;

    interval_t & operator [](unsigned i) {
        return mSparseCharSet[i];
    }

    const interval_t & operator [](unsigned i) const {
        return mSparseCharSet[i];
    }

    inline codepoint_t min_codepoint() const {
        return empty() ? 0 : std::get<0>(front());
    }

    inline codepoint_t max_codepoint() const {
        return empty() ? 0 : std::get<1>(back());
    }

    void insert_range(const codepoint_t lo, const codepoint_t hi);

    void remove_range(const codepoint_t lo, const codepoint_t hi);

    inline void insert(const codepoint_t codepoint) {
        insert_range(codepoint, codepoint);
    }

    inline void remove(const codepoint_t codepoint) {
        remove_range(codepoint, codepoint);
    }

    inline iterator begin() {
        return mSparseCharSet.begin();
    }

    inline iterator end() {
        return mSparseCharSet.end();
    }

    inline reference front() {
        return mSparseCharSet.front();
    }

    inline reference back() {
        return mSparseCharSet.back();
    }

    inline const_iterator begin() const {
        return mSparseCharSet.cbegin();
    }

    inline const_iterator end() const {
        return mSparseCharSet.cend();
    }

    inline const_iterator cbegin() const {
        return mSparseCharSet.cbegin();
    }

    inline const_iterator cend() const {
        return mSparseCharSet.cend();
    }

    inline const_reference front() const {
        return mSparseCharSet.front();
    }

    inline const_reference back() const {
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
    friend CC * subtractCC(const CC * cc1, const CC * cc2);

    inline CC()
    : RE(ClassTypeId::CC)
    , mSparseCharSet(mCharSetAllocator) {

    }
    CC(const CC & cc);
    inline CC(const codepoint_t codepoint)
    : RE(ClassTypeId::CC)
    , mSparseCharSet(mCharSetAllocator) {
        insert(codepoint);
    }
    inline CC(const codepoint_t lo_codepoint, const codepoint_t hi_codepoint)
    : RE(ClassTypeId::CC)
    , mSparseCharSet(mCharSetAllocator) {
        insert_range(lo_codepoint, hi_codepoint);
    }
    CC(const CC * cc1, const CC * cc2);

    template <typename itr>
    CC * initialize(itr begin, itr end);
private:    
    IntervalVector mSparseCharSet;
    static IntervalAllocator mCharSetAllocator;
};

inline static CC::iterator begin(CC & cc) {
    return cc.begin();
}

inline static CC::iterator end(CC & cc) {
    return cc.end();
}

inline static CC::const_iterator begin(const CC & cc) {
    return cc.cbegin();
}

inline static CC::const_iterator end(const CC & cc) {
    return cc.cend();
}

inline codepoint_t & lo_codepoint(interval_t & i) {
    return std::get<0>(i);
}
inline codepoint_t lo_codepoint(const interval_t & i) {
    return std::get<0>(i);
}
inline codepoint_t & lo_codepoint(const CC::iterator i) {
    return lo_codepoint(*i);
}
inline codepoint_t lo_codepoint(const CC::const_iterator i) {
    return lo_codepoint(*i);
}

inline codepoint_t & hi_codepoint(interval_t & i) {
    return std::get<1>(i);
}
inline codepoint_t hi_codepoint(const interval_t & i) {
    return std::get<1>(i);
}
inline codepoint_t & hi_codepoint(const CC::iterator i) {
    return hi_codepoint(*i);
}
inline codepoint_t hi_codepoint(const CC::const_iterator i) {
    return hi_codepoint(*i);
}

template<typename itr>
CC * CC::initialize(itr begin, itr end) {
    mSparseCharSet.resize(std::distance(begin, end));
    for (auto i = begin; i != end; ++i) {
        assert (i == begin || lo_codepoint(*i) > max_codepoint());
        mSparseCharSet[std::distance(begin, i)] = *i;
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

CC * subtractCC(const CC * a, const CC * b);
    
CC * intersectCC(const CC * cc1, const CC * cc2);

CC * caseInsensitize(const CC * cc);

}

#endif // RE_CC_H
