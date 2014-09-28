/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_SEQ_H
#define RE_SEQ_H

#include "re_re.h"
#include <string>

namespace re {

class Seq : public Vector {
public:
    static inline bool classof(const RE * re) {
        return re->getClassTypeId() == ClassTypeId::Seq;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual RE * clone() const {
        return new Seq(*this);
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
    friend Seq * makeSeq(const Seq::Type, Seq::iterator, Seq::iterator);
    Seq(const Type type)
    : Vector(ClassTypeId::Seq)
    , mType(type) {

    }
    Seq(const Seq & seq)
    : Vector(ClassTypeId::Seq, seq.cbegin(), seq.cend(), true)
    , mType(seq.mType) {

    }
    Seq(const Type type, iterator begin, iterator end)
    : Vector(ClassTypeId::Seq, begin, end)
    , mType(type)
    {

    }
private:
    Type    mType;
};

inline Seq * makeSeq(const Seq::Type type = Seq::Type::Normal) {
    return new Seq(type);
}

inline Seq * makeSeq(const Seq::Type type, Seq::iterator begin, Seq::iterator end) {
    return new Seq(type, begin, end);
}

}

#endif // JOIN_H




