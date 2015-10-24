/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_SEQ_H
#define RE_SEQ_H

#include "re_re.h"
#include <string>
#include <initializer_list>

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
    template<typename iterator> friend RE * makeSeq(iterator, iterator);
    Seq()
    : Vector(ClassTypeId::Seq) {

    }
    Seq(iterator begin, iterator end)
    : Vector(ClassTypeId::Seq, begin, end) {

    }
    template<typename itr> void flatten(itr begin, itr end);
};

inline Seq * makeSeq() {
    return new Seq();
}

template<typename itr>
void Seq::flatten(itr begin, itr end) {
    for (auto i = begin; i != end; ++i) {
        if (LLVM_UNLIKELY(isa<Seq>(*i))) {
            flatten<Seq::iterator>(cast<Seq>(*i)->begin(), cast<Seq>(*i)->end());
        } else {
            push_back(*i);
        }
    }
}

template<typename itr>
inline RE * makeSeq(itr begin, itr end) {
    if (LLVM_UNLIKELY(std::distance(begin, end) == 0)) {
        return makeSeq();
    } else if (std::distance(begin, end) == 1) {
        return *begin;
    } else {
        Seq * seq = makeSeq();
        seq->flatten(begin, end);
        if (seq->size() == 1) {
            return seq->front();
        }
        return seq;
    }
}

inline RE * makeSeq(RE::InitializerList list) {
    return makeSeq(list.begin(), list.end());
}

}

#endif // JOIN_H




