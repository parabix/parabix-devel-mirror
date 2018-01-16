/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_SEQ_H
#define RE_SEQ_H

#include "re_re.h"
#include <llvm/Support/Casting.h>

namespace re {

class Seq : public Vector {
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
    Seq()
    : Vector(ClassTypeId::Seq) {

    }
    Seq(iterator begin, iterator end)
    : Vector(ClassTypeId::Seq, begin, end) {

    }
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
}

#endif // RE_SEQ_H




