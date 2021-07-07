/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_SEQ_H
#define RE_SEQ_H

#include <string>
#include <vector>
#include <llvm/Support/Casting.h>
#include <re/adt/re_cc.h>
#include <re/adt/re_re.h>
#include <re/adt/re_empty_set.h>
#include <unicode/core/unicode_set.h>
#include <util/slab_allocator.h>

namespace re {

class Seq : public RE, public std::vector<RE*, ProxyAllocator<RE *>> {
public:
    static inline bool classof(const RE * re) {
        return re->getClassTypeId() == ClassTypeId::Seq;
    }
    static inline bool classof(const void *) {
        return false;
    }
protected:
    friend Seq * makeSeq();
    template<typename iterator> friend RE * makeSeq(const iterator, const iterator);
    Seq() : RE(ClassTypeId::Seq), std::vector<RE*, ProxyAllocator<RE *>>(mAllocator) {}
    template<typename iterator>
    Seq(const iterator begin, const iterator end)
    : RE(ClassTypeId::Seq), std::vector<RE*, ProxyAllocator<RE *>>(begin, end, mAllocator) { }
};

inline Seq * makeSeq() {
    return new Seq();
}

template<typename iterator>
inline RE * makeSeq(const iterator begin, const iterator end) {
    if (LLVM_UNLIKELY(begin == end)) {
        return new Seq();
    }
    llvm::SmallVector<RE *, 32> newSeq;
    newSeq.reserve(std::distance(begin, end));
    for (auto i = begin; i != end; ++i) {
        RE * const item = *i;
        if (LLVM_UNLIKELY(isEmptySet(item))) {
            return item;
        } else if (LLVM_UNLIKELY(llvm::isa<Seq>(item))) {
            const Seq & nestedSeq = *llvm::cast<Seq>(item);
            newSeq.reserve(newSeq.capacity() + nestedSeq.size());
            newSeq.insert(newSeq.end(), nestedSeq.begin(), nestedSeq.end());
        } else {
            newSeq.push_back(item);
        }
    }
    if (newSeq.size() == 1) {
        return newSeq.front();
    }
    return new Seq(newSeq.begin(), newSeq.end());
}

inline RE * makeSeq(std::initializer_list<RE *> list) {
    return makeSeq(list.begin(), list.end());
}

inline bool isEmptySeq(RE * s) {
    return llvm::isa<Seq>(s) && llvm::cast<Seq>(s)->empty();
}

inline RE * u32string2re(std::u32string s) {
    llvm::SmallVector<RE *, 32> ccs;
    for (auto c : s) {
        ccs.push_back(makeCC(UCD::UnicodeSet(c)));
    }
    return makeSeq(ccs.begin(), ccs.end());
}

}

#endif // RE_SEQ_H




