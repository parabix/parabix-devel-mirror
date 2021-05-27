/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef ALT_H
#define ALT_H

#include <vector>
#include <llvm/Support/Casting.h>
#include <re/adt/re_re.h>
#include <re/adt/re_cc.h>
#include <re/adt/re_seq.h>
#include <re/adt/re_rep.h>
#include <re/adt/printer_re.h>
#include <util/slab_allocator.h>

namespace re {

class Alt : public RE, public std::vector<RE*, ProxyAllocator<RE *>> {
public:
    static inline bool classof(const RE * re) {
        return re->getClassTypeId() == ClassTypeId::Alt;
    }
    static inline bool classof(const void *) {
        return false;
    }
protected:
    friend Alt * makeAlt();
    template<typename iterator> friend RE * makeAlt(iterator, iterator);
    Alt() : RE(ClassTypeId::Alt), std::vector<RE*, ProxyAllocator<RE *>>(mAllocator) {}
    template<typename iterator>
    Alt(const iterator begin, const iterator end)
    : RE(ClassTypeId::Alt), std::vector<RE*, ProxyAllocator<RE *>>(begin, end, mAllocator) { }
};

/**
 * @brief makeAlt
 *
 * Build an Alt, flattening alternative subgroups, and combining character classes and
 * move character classes towards the end of the list to ensure that all combinations are found.
 *
 * @param list
 * @return simplified RE representing the Alt
 */

inline Alt * makeAlt() {
    return new Alt();
}

template<typename iterator>
RE * makeAlt(const iterator begin, const iterator end) {
    if (LLVM_UNLIKELY(begin == end)) {
        return new Alt();
    }

    llvm::SmallVector<CC *, 2> CCs; // CCs with possibly different alphabets
    auto combineCC = [&CCs] (CC * cc) {
        for (CC *& existing : CCs) {
            if (LLVM_LIKELY(existing->getAlphabet() == cc->getAlphabet())) {
                existing = makeCC(existing, cc);
                return;
            }
        }
        CCs.push_back(cc);
    };

    RE * nullableRE = nullptr;
    llvm::SmallVector<RE *, 32> newAlt;
    newAlt.reserve(std::distance(begin, end));
    for (auto i = begin; i != end; ++i) {
        if (llvm::isa<CC>(*i)) {
            combineCC(llvm::cast<CC>(*i));
        } else if (llvm::isa<Alt>(*i)) {
            // We have an Alt to embed within the alt.  We extract the individual
            // elements to include within the new alt.   Note that recursive flattening
            // is not required, if the elements themselves were created with makeAlt.
            const Alt & nestedAlt = *llvm::cast<Alt>(*i);
            newAlt.reserve(newAlt.capacity() + nestedAlt.size());
            for (RE * a : nestedAlt) {
                if (llvm::isa<CC>(a)) {
                    combineCC(llvm::cast<CC>(a));
                } else if (LLVM_UNLIKELY(isEmptySeq(a))) {
                    if (nullableRE == nullptr) {
                        nullableRE = a;
                    }
                } else {
                    newAlt.push_back(a);
                }
            }
        } else if (const Rep * rep = llvm::dyn_cast<Rep>(*i)) {
            if (rep->getLB() == 0) {
                if (nullableRE == nullptr) {
                    // Already have a nullable case.
                    newAlt.push_back(makeRep(rep->getRE(), 1, rep->getUB()));
                } else {
                    // This will be the nullable case.
                    nullableRE = *i;
                }
            } else {
                newAlt.push_back(*i);
            }
        } else if (isEmptySeq(*i)) {
            if (nullableRE == nullptr) {
                nullableRE = *i;
            }
        } else {
            newAlt.push_back(*i);
        }
    }

    newAlt.insert(newAlt.end(), CCs.begin(), CCs.end());

    if (nullableRE) {
        newAlt.push_back(nullableRE);
    }

    if (LLVM_UNLIKELY(newAlt.size() == 1)) {
        return newAlt.front();
    } else {
        return new Alt(newAlt.begin(), newAlt.end());
    }
}

inline RE * makeAlt(std::initializer_list<RE *> list) {
    return makeAlt(list.begin(), list.end());
}

}

#endif // ALT_H

