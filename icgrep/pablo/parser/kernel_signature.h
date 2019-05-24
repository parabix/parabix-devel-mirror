/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#pragma once

#include <ostream>
#include <string>
#include <utility>
#include <vector>
#include <util/slab_allocator.h>

namespace pablo {
namespace parse {

class PabloKernelSignature {
public:

    class Type {
    public:
        using Allocator = SlabAllocator<Type *>;

        enum class ClassTypeId {
            INT, STREAM, STREAM_SET
        };

        explicit Type(ClassTypeId id)
        : mClassTypeId(id)
        {}

        inline void * operator new (size_t size) {
            return mAllocator.allocate<uint8_t>(size);
        }

        ClassTypeId getClassTypeId() const noexcept { return mClassTypeId; }

        virtual std::string asString() const = 0;
    protected:
        static Allocator mAllocator;
        ClassTypeId      mClassTypeId;
    };

    class IntType : public Type {
    public:
        explicit IntType(uint64_t bitWidth)
        : Type(ClassTypeId::INT)
        , mBitWidth(bitWidth)
        {}

        static inline bool classof(Type const * type) { return type->getClassTypeId() == ClassTypeId::INT; }
        static inline bool classof(void const *) { return false; }

        inline uint64_t getBitWidth() const { return mBitWidth; };

        std::string asString() const override;
    private:
        uint64_t mBitWidth;
    };

    class StreamType : public Type {
    public:
        explicit StreamType(IntType * elementType)
        : Type(ClassTypeId::STREAM)
        , mElementType(elementType)
        {}

        static inline bool classof(Type const * type) { return type->getClassTypeId() == ClassTypeId::STREAM; }
        static inline bool classof(void const *) { return false; }

        inline IntType * getElementType() const { return mElementType; }

        std::string asString() const override;
    private:
        IntType * mElementType;
    };

    class StreamSetType : public Type {
    public:
        StreamSetType(IntType * elementType, size_t streamCount)
        : Type(ClassTypeId::STREAM_SET)
        , mElementType(elementType), mStreamCount(streamCount)
        {}

        static inline bool classof(Type const * type) { return type->getClassTypeId() == ClassTypeId::STREAM_SET; }
        static inline bool classof(void const *) { return false; }

        inline IntType * getElementTYpe() const { return mElementType; }
        inline size_t getStreamCount() const { return mStreamCount; }

        std::string asString() const override;
    private:
        IntType *   mElementType;
        size_t      mStreamCount;
    };

    using SignatureBinding = std::pair<std::string, Type *>;
    using SignatureBindings = std::vector<SignatureBinding>;

    PabloKernelSignature(std::string name, SignatureBindings inputBindings, SignatureBindings outputBindings)
    : mName(std::move(name))
    , mInputBindings(std::move(inputBindings))
    , mOutputBindings(std::move(outputBindings))
    {}

    std::string getName() const noexcept {
        return mName;
    }

    SignatureBindings const & getInputBindings() const noexcept {
        return mInputBindings;
    }

    SignatureBindings const & getOutputBindings() const noexcept {
        return mOutputBindings;
    }

private:
    std::string         mName;
    SignatureBindings   mInputBindings;
    SignatureBindings   mOutputBindings;
};

} // namespace pablo::parse
} // namespace pablo

std::ostream & operator << (std::ostream & out, pablo::parse::PabloKernelSignature const & sig);
