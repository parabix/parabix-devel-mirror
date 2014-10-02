/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_H
#define RE_H

#include <vector>
#include <assert.h>
#include <llvm/Support/Casting.h>

using namespace llvm;

namespace re {

class Vector;
class Pair;

class Alt;
class CC;
class Diff;
class End;
class Intersect;
class Name;
class Permute;
class Rep;
class Seq;
class Start;
class SymDiff;
class Union;

class RE {
public:
    enum class ClassTypeId : unsigned {
        Alt
        , CC
        , Diff
        , End
        , Intersect
        , Name
        , Permute
        , Rep
        , Seq
        , Start
        , SymDiff
        , Union
    };
    inline ClassTypeId getClassTypeId() const {
        return mClassTypeId;
    }
    typedef std::initializer_list<RE *> InitializerList;
    virtual RE * clone() const = 0;
    virtual ~RE() = 0;
protected:
    inline RE(const ClassTypeId id)
    : mClassTypeId(id) {

    }
    const ClassTypeId mClassTypeId;
};

//template <typename To, typename From>
//inline static bool isa(const From * object) {
//    return To::classof(object);
//}

//template <typename To, typename From>
//inline static To * dyn_cast(From * object) {
//    if (isa<To, From>(object)) {
//        return reinterpret_cast<To *>(object);
//    }
//    return nullptr;
//}

class Vector : public RE, public std::vector<RE*> {
public:
    virtual ~Vector() {
        for (RE * re : *this) {
            delete re;
        }
    }
protected:
    inline Vector(const ClassTypeId id)
    : RE(id)
    , std::vector<RE*>()
    {

    }
    inline Vector(const ClassTypeId id, const iterator begin, const iterator end)
    : RE(id)
    , std::vector<RE*>(begin, end) {

    }
    inline Vector(const ClassTypeId id, const const_iterator begin, const const_iterator end, const bool deep_copy)
    : RE(id) {
        assert (deep_copy && "Not intended as a shallow copy constructor.");
        this->resize(std::distance(begin, end));
        for (auto i = begin; i != end; ++i) {
            this->assign(std::distance(begin, i), (*i)->clone());
        }
    }
};

//class Pair : public RE {
//protected:
//    inline Pair(const ClassTypeId id)
//    : RE(id)
//    , _lh(nullptr)
//    , _rh(nullptr)
//    {

//    }
//    inline Pair(const ClassTypeId id, const RE * lh, const RE * rh)
//    : RE(id)
//    , _lh(lh)
//    , _rh(rh)
//    {

//    }
//    virtual ~Pair() {
//        delete _lh;
//        delete _rh;
//    }
//protected:
//    const RE * _lh;
//    const RE * _rh;
//};

//static Diff * makeDiff(const RE * lh, const RE * rh);

//static Intersect * makeIntersect(const RE * lh, const RE * rh);

//static Permute * makePermute();
//static Permute * makePermute(Vector::iterator begin, Vector::iterator end);

//static SymDiff * makeSymDiff(const RE * lh, const RE * rh);

//static Union * makeUnion(const RE * lh, const RE * rh);

}

#endif // RE_H


