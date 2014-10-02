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
    enum class Type {
        Normal
        , Byte
    };
    std::string getName() const;
    inline Type getType() const {
        return mType;
    }
    inline void setType(const Type type) {
        mType = type;
    }    
    virtual ~Seq() {}
protected:
    friend Seq * makeSeq(const Seq::Type);
    template<typename iterator> friend RE * makeSeq(const Seq::Type, iterator, iterator);
    Seq(const Type type)
    : Vector(ClassTypeId::Seq)
    , mType(type) {

    }
    Seq(const Type type, iterator begin, iterator end)
    : Vector(ClassTypeId::Seq, begin, end)
    , mType(type)
    {

    }
    template<typename itr> void construct(itr begin, itr end);
private:
    Type    mType;
};

inline Seq * makeSeq(const Seq::Type type = Seq::Type::Normal) {
    return new Seq(type);
}

template<typename itr>
void Seq::construct(itr begin, itr end) {
    for (auto i = begin; i != end; ++i) {
        if (Seq * seq = dyn_cast<Seq>(*i)) {
            construct<Seq::iterator>(seq->begin(), seq->end());
            continue;
        }
        push_back(*i);
    }
}

template<typename itr>
inline RE * makeSeq(const Seq::Type type, itr begin, itr end) {
    Seq * seq = makeSeq(type);
    seq->construct(begin, end);
    if (seq->size() == 1) {
        return seq->back();
    }
    return seq;
}

inline RE * makeSeq(RE::InitializerList list) {
    return makeSeq(Seq::Type::Normal, list.begin(), list.end());
}

}

#endif // JOIN_H




