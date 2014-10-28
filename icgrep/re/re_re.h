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
#include <slab_allocator.h>

using namespace llvm;

namespace re {

class Vector;
class Pair;

class Alt;
class Any;
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
    typedef SlabAllocator<1024> Allocator;
    enum class ClassTypeId : unsigned {
        Alt
        , Any
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
    inline static void release_memory() {
        mAllocator.release_memory();
    }
    typedef std::initializer_list<RE *> InitializerList;
protected:
    inline RE(const ClassTypeId id)
    : mClassTypeId(id) {

    }
    const ClassTypeId mClassTypeId;

    static Allocator mAllocator;
};

class Vector : public RE, public std::vector<RE*> {
public:
    virtual ~Vector() {
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


