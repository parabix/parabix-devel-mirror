/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_SEQ_H
#define RE_SEQ_H

#include <string>
#include <util/slab_allocator.h>
#include <vector>
#include <re/re_cc.h>
#include <re/re_re.h>
#include <UCD/unicode_set.h>
#include <llvm/Support/Casting.h>

namespace re {

class Seq : public RE, public std::vector<RE*, ProxyAllocator<RE *>> {
public:
    static inline bool classof(const RE * re) {
        return re->getClassTypeId() == ClassTypeId::Seq;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Seq() {}
protected:
    friend Seq * makeSeq();
    template<typename iterator> friend RE * makeSeq(const iterator, const iterator);
    Seq() : RE(ClassTypeId::Seq), std::vector<RE*, ProxyAllocator<RE *>>(mAllocator) {}
    Seq(iterator begin, iterator end)
    : RE(ClassTypeId::Seq), std::vector<RE*, ProxyAllocator<RE *>>(begin, end, mAllocator) { }
};

inline Seq * makeSeq() {
    return new Seq();
}

template<typename iterator>
inline RE * makeSeq(const iterator begin, const iterator end) {
    Seq * seq = makeSeq();
    for (auto i = begin; i != end; ++i) {
        RE * const item = *i;
        if (LLVM_UNLIKELY(llvm::isa<Seq>(item))) {
            for (RE * const innerItem : *llvm::cast<Seq>(item)) {
                seq->push_back(innerItem);
            }
        } else {
            seq->push_back(item);
        }
    }
    if (seq->size() == 1) {
        return seq->front();
    }
    return seq;
}

inline RE * makeSeq(RE::InitializerList list) {
    return makeSeq(list.begin(), list.end());
}

inline bool isEmptySeq(RE * s) {
    return llvm::isa<Seq>(s) && llvm::cast<Seq>(s)->empty();
}
    
inline RE * u32string2re(std::u32string s) {
    std::vector<RE *> ccs;
    for (auto c : s) {
        ccs.push_back(makeCC(UCD::UnicodeSet(c)));
    }
    return makeSeq(ccs.begin(), ccs.end());
}
    
}

#endif // RE_SEQ_H




