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

typedef unsigned codepoint_t;

struct CharSetItem {
    constexpr CharSetItem() : lo_codepoint(0), hi_codepoint(0) {}
    constexpr CharSetItem(const codepoint_t lo, const codepoint_t hi) : lo_codepoint(lo), hi_codepoint(hi) {}
    constexpr codepoint_t operator [](const unsigned i) const {
        return (i == 0) ? lo_codepoint : (i == 1) ? hi_codepoint : throw std::runtime_error("CharSetItem[] can only accept 0 or 1.");
    }
    codepoint_t lo_codepoint;
    codepoint_t hi_codepoint;
};

enum CC_type {UnicodeClass, ByteClass};

class CC : public RE {
public:

    static inline bool classof(const RE * re) {
        return re->getClassTypeId() == ClassTypeId::CC;
    }
    static inline bool classof(const void *) {
        return false;
    }

    using CharSetAllocator = SlabAllocator<CharSetItem>;
    using CharSetVector = std::vector<CharSetItem, CharSetAllocator>;

    typedef CharSetVector::iterator                 iterator;
    typedef CharSetVector::const_iterator           const_iterator;
    typedef CharSetVector::size_type                size_type;
    typedef CharSetVector::reference                reference;
    typedef CharSetVector::const_reference          const_reference;

    static const codepoint_t UNICODE_MAX = 0x10FFFF;

    std::string canonicalName(const CC_type type) const;

    CharSetItem & operator [](unsigned i) {
        return mSparseCharSet[i];
    }

    const CharSetItem & operator [](unsigned i) const {
        return mSparseCharSet[i];
    }

    inline codepoint_t min_codepoint() const {
        return mSparseCharSet.size() == 0 ? 0 : mSparseCharSet.front().lo_codepoint;
    }

    inline codepoint_t max_codepoint() const {
        return mSparseCharSet.size() == 0 ? 0 : mSparseCharSet.back().hi_codepoint;
    }

    void insert_range(const codepoint_t lo_codepoint, const codepoint_t hi_codepoint);

    void remove_range(const codepoint_t lo_codepoint, const codepoint_t hi_codepoint);

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
private:    
    CharSetVector mSparseCharSet;
    static CharSetAllocator mCharSetAllocator;
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

CC * subtractCC(const CC * cc1, const CC * cc2);
    
CC * intersectCC(const CC * cc1, const CC * cc2);

CC * caseInsensitize(const CC * cc);

CC * rangeIntersect(const CC * cc, const codepoint_t lo, const codepoint_t hi);

CC * rangeGaps(const CC * cc, const codepoint_t lo, const codepoint_t hi);

CC * outerRanges(const CC * cc);

CC * innerRanges(const CC * cc);

}

#endif // RE_CC_H
