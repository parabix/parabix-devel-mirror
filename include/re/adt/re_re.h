/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_H
#define RE_H

#include <string>
#include <vector>
#include <util/slab_allocator.h>

namespace re {

// REs are defined in a class hierarchy supporting llvm::isa, llvm::dyn_cast.
#define RE_SUBTYPE(kind) \
static inline bool classof(const RE * re) {return re->getClassTypeId() == ClassTypeId::kind;}\
static inline bool classof(const void *) {return false;}

class RE {
public:
    using Allocator = SlabAllocator<RE *>;
    using VectorAllocator = ProxyAllocator<RE *>;
    enum class ClassTypeId : unsigned {
        Alt
        , Any
        , Assertion
        , CC
        , Range
        , Diff
        , End
        , Intersect
        , Name
        , PropertyExpression
        , Capture
        , Reference
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
    using length_t = std::string::size_type;
    inline const char * replicateString(const char * string, const length_t length) {
        if (string && (length > 0)) {
            char * allocated = reinterpret_cast<char*>(mAllocator.allocate(length));
            std::memcpy(allocated, string, length);
            return allocated;
        }
        return nullptr;
    }
};

}

#endif // RE_H


