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
#include <util/slab_allocator.h>

using namespace llvm;

namespace re {

class Vector;
class Pair;

class Alt;
class Any;
class Assertion;
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
class GraphemeBoundary;

class RE {
public:
    using Allocator = SlabAllocator<u_int8_t>;
    using VectorAllocator = Allocator::rebind<RE *>::other;
    enum class ClassTypeId : unsigned {
        Alt
        , Any
        , Assertion
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
        , GraphemeBoundary
    };
    inline ClassTypeId getClassTypeId() const {
        return mClassTypeId;
    }
    typedef std::initializer_list<RE *> InitializerList;
protected:
    inline RE(const ClassTypeId id)
    : mClassTypeId(id) {

    }
    void* operator new (std::size_t size) noexcept {
        return mAllocator.allocate(size);
    }
    const ClassTypeId mClassTypeId;

    static Allocator mAllocator;
};

class Vector : public RE, public std::vector<RE*, RE::VectorAllocator> {
public:

    virtual ~Vector() {
    }
protected:
    inline Vector(const ClassTypeId id)
    : RE(id)
    , std::vector<RE*, RE::VectorAllocator>(reinterpret_cast<VectorAllocator &>(mAllocator))
    {

    }
    inline Vector(const ClassTypeId id, const iterator begin, const iterator end)
    : RE(id)
    , std::vector<RE*, RE::VectorAllocator>(begin, end, reinterpret_cast<VectorAllocator &>(mAllocator)) {

    }
};

}

#endif // RE_H


