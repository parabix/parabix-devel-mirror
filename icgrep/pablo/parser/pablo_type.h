/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#pragma once

#include <string>
#include <vector>
#include <util/slab_allocator.h>

#define PABLO_SUBTYPE(KIND) \
static inline bool classof(PabloType const * t) {return t->getClassTypeId() == ClassTypeId::KIND;} \
static inline bool classof(const void *) {return false;}

namespace pablo {
namespace parse {

class PabloType {
public:
    using Allocator = SlabAllocator<PabloType *>;

    enum class ClassTypeId {
        SCALAR,
        STREAM,
        STREAMSET,
        ALIAS,
        NAMED_STREAMSET
    };

    inline explicit PabloType(ClassTypeId classTypeId)
    : mClassTypeId(classTypeId)
    {}

    virtual ~PabloType() {}

    virtual bool equals(PabloType const * other) const noexcept = 0;
    virtual std::string asString(bool verbose = false) const noexcept = 0;

    inline ClassTypeId getClassTypeId() const {
        return mClassTypeId;
    }

    void * operator new(size_t size) noexcept {
        return mAllocator.allocate<uint8_t>(size);
    }

    static std::string GenerateAnonymousTypeName() {
        return "__anonymous_type_name_" + std::to_string(mNextAnonId++);
    }

protected:
    const ClassTypeId mClassTypeId;
    static size_t     mNextAnonId;
    static Allocator  mAllocator;
};


class ScalarType final : public PabloType {
public:
    PABLO_SUBTYPE(SCALAR);

    explicit ScalarType(size_t bitWidth);
    bool equals(PabloType const * other) const noexcept override;
    std::string asString(bool verbose = false) const noexcept override;

    inline size_t getBitWidth() const noexcept {
        return mBitWidth;
    }
private:
    size_t mBitWidth;
};


class StreamType final : public PabloType {
public:
    PABLO_SUBTYPE(STREAM);

    explicit StreamType(size_t elementBitWidth);
    bool equals(PabloType const * other) const noexcept override;
    std::string asString(bool verbose = false) const noexcept override;

    inline size_t getElementBitWidth() const noexcept {
        return mBitWidth;
    }
private:
    size_t mBitWidth;
};


class StreamSetType final : public PabloType {
public:
    PABLO_SUBTYPE(STREAMSET);

    StreamSetType(size_t elementBitWidth, size_t streamCount);
    bool equals(PabloType const * other) const noexcept override;
    std::string asString(bool verbose = false) const noexcept override;

    inline size_t getElementBitWidth() const noexcept {
        return mBitWidth;
    }
    inline size_t getStreamCount() const noexcept {
        return mStreamCount;
    }
private:
    size_t mBitWidth;
    size_t mStreamCount;
};


class AliasType : public PabloType {
public:
    PABLO_SUBTYPE(ALIAS);

    virtual ~AliasType() {}

    AliasType(std::string typeName, PabloType * aliasType);
    bool equals(PabloType const * other) const noexcept override;
    std::string asString(bool verbose = false) const noexcept override;

    inline std::string const & getTypeName() const noexcept {
        return mTypeName;
    }

    inline PabloType * getAliasedType() const noexcept { 
        return mAliasedType;
    }

    inline void setTypeName(std::string const & name) noexcept {
        mTypeName = name;
    }
protected:
    std::string mTypeName;
    PabloType * mAliasedType;
};


class NamedStreamSetType final : public PabloType {
public:
    PABLO_SUBTYPE(NAMED_STREAMSET);

    NamedStreamSetType(std::string typeName, StreamSetType * aliasType, std::vector<std::string> const & streamNames);
    bool equals(PabloType const * other) const noexcept override;
    std::string asString(bool verbose = false) const noexcept override;

    inline std::string const & getTypeName() const noexcept {
        return mTypeName;
    }

    inline PabloType * getAliasedType() const noexcept { 
        return mAliasedType;
    }

    inline std::vector<std::string> const & getStreamNames() const noexcept {
        return mStreamNames;
    }

    inline void setTypeName(std::string const & name) noexcept {
        mTypeName = name;
    }
private:
    std::string mTypeName;
    PabloType * mAliasedType;
    std::vector<std::string> mStreamNames;
};

} // namespace pablo::parse
} // namespace pablo

#undef PABLO_SUBTYPE
