/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_CC_H
#define RE_CC_H

#include "re_re.h"

#include <iostream>
#include <string>
#include <sstream>
#include <utility>
#include <vector>

typedef int CodePointType;

struct CharSetItem{    
    CharSetItem() : lo_codepoint(0), hi_codepoint(0) {}
    CharSetItem(const CodePointType lo, const CodePointType hi) : lo_codepoint(lo), hi_codepoint(hi) {}
    CodePointType lo_codepoint;
    CodePointType hi_codepoint;
};

typedef std::vector<CharSetItem> CharSetVector;

class CC : public RE {
public:

    typedef CharSetVector::iterator                 iterator;
    typedef CharSetVector::const_iterator           const_iterator;
    typedef CharSetVector::size_type                size_type;
    typedef CharSetVector::reference                reference;
    typedef CharSetVector::const_reference          const_reference;

    static const CodePointType UNICODE_MAX = 0x10FFFF;
    CC();
    CC(const CodePointType codepoint);
    CC(const CodePointType lo_codepoint, const CodePointType hi_codepoint);
    CC(const CC * cc1, const CC * cc2);
    ~CC();
    std::string getName() const;
    void insert_range(const CodePointType lo_codepoint, const CodePointType hi_codepoint);
    void negate();
    void remove_range(const CodePointType lo_codepoint, const CodePointType hi_codepoint);

    inline void insert(const CodePointType codepoint) {
        insert_range(codepoint, codepoint);
    }

    inline void remove(const CodePointType codepoint) {
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

private:    
    void join(const CharSetVector & other);
    CharSetVector mSparseCharSet;
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


#endif // RE_CC_H
