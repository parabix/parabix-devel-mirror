/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_H
#define RE_H

#include <util/slab_allocator.h>
#include <vector>

namespace re {

class RE {
public:
    using Allocator = SlabAllocator<RE *>;
    using VectorAllocator = ProxyAllocator<RE *>;
    enum class ClassTypeId : unsigned {
        Alt
        , Assertion
        , CC
        , Range
        , Diff
        , End
        , Intersect
        , Name
        , Group
        , Rep
        , Seq
        , Start
    };
    inline ClassTypeId getClassTypeId() const {
        return mClassTypeId;
    }
    inline static void Reset() {
        mAllocator.Reset();
    }
    inline static void PrintStats() {
        mAllocator.PrintStats();
    }
    typedef std::initializer_list<RE *> InitializerList;

protected:
    inline RE(const ClassTypeId id)
    : mClassTypeId(id) {

    }
    void* operator new (std::size_t size) noexcept {
        return mAllocator.allocate<uint8_t>(size);
    }
    const ClassTypeId mClassTypeId;
    static Allocator mAllocator;
};

}

#endif // RE_H


