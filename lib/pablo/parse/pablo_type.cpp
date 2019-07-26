/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/parse/pablo_type.h>

#include <llvm/Support/Casting.h>
#include <llvm/Support/raw_ostream.h>

namespace pablo {
namespace parse {

PabloType::Allocator PabloType::mAllocator;
size_t PabloType::mNextAnonId = 0;

inline llvm::StringRef copyText(llvm::StringRef text, PabloType::Allocator & alloc) {
    ProxyAllocator<char> A(alloc);
    return text.copy(A);
}

inline llvm::ArrayRef<llvm::StringRef> copyArray(std::vector<std::string> array, PabloType::Allocator & alloc) {
    const auto n = array.size();
    llvm::SmallVector<llvm::StringRef, 64> tmp(n);
    for (unsigned i = 0; i != n; ++i) {
        tmp[i] = copyText(array[i], alloc);
    }
    llvm::ArrayRef<llvm::StringRef> out(tmp);
    ProxyAllocator<llvm::StringRef> A(alloc);
    return out.copy(A);
}

bool ScalarType::equals(PabloType const * other) const noexcept {
    if (llvm::isa<ScalarType>(other)) {
        return mBitWidth == llvm::cast<ScalarType>(other)->mBitWidth;
    } else if (llvm::isa<AliasType>(other)) {
        return equals(llvm::cast<AliasType>(other)->getAliasedType());
    } else {
        return false;
    }
}

std::string ScalarType::asString(bool /*verbose*/) const noexcept {
    return "i" + std::to_string(mBitWidth);
}

ScalarType::ScalarType(size_t bitWidth)
: PabloType(ClassTypeId::SCALAR)
, mBitWidth(bitWidth)
{}


bool StreamType::equals(PabloType const * other) const noexcept {
    if (llvm::isa<StreamType>(other)) {
        return mBitWidth == llvm::cast<StreamType>(other)->mBitWidth;
    } else if (llvm::isa<AliasType>(other)) {
        return equals(llvm::cast<AliasType>(other)->getAliasedType());
    } else {
        return false;
    }
}

std::string StreamType::asString(bool /*verbose*/) const noexcept {
    return "<i" + std::to_string(mBitWidth) + ">";
}

StreamType::StreamType(size_t bitWidth)
: PabloType(ClassTypeId::STREAM)
, mBitWidth(bitWidth)
{}


bool StreamSetType::equals(PabloType const * other) const noexcept {
    if (llvm::isa<StreamSetType>(other)) {
        auto ss = llvm::cast<StreamSetType>(other);
        return mBitWidth == ss->mBitWidth && mStreamCount == ss->mStreamCount;
    } else if (llvm::isa<NamedStreamSetType>(other)) {
        return other->equals(this);
    } else if (llvm::isa<AliasType>(other)) {
        return equals(llvm::cast<AliasType>(other)->getAliasedType());
    } else {
        return false;
    }
}

std::string StreamSetType::asString(bool /*verbose*/) const noexcept {
    return "<i" + std::to_string(mBitWidth) + ">[" + std::to_string(mStreamCount) + "]";
}

StreamSetType::StreamSetType(size_t bitWidth, size_t streamCount)
: PabloType(ClassTypeId::STREAMSET)
, mBitWidth(bitWidth)
, mStreamCount(streamCount)
{}


bool AliasType::equals(PabloType const * other) const noexcept {
    return mAliasedType->equals(other);
}

std::string AliasType::asString(bool verbose) const noexcept {
    if (verbose) {
        llvm::SmallVector<char, 256> tmp;
        llvm::raw_svector_ostream out(tmp);
        out << mTypeName << " (aka " << mAliasedType->asString() << ")";
        return out.str().str();
    }
    return mTypeName.str();
}

AliasType::AliasType(llvm::StringRef typeName, PabloType * aliasType)
: PabloType(ClassTypeId::ALIAS)
, mTypeName(copyText(typeName, mAllocator))
, mAliasedType(aliasType)
{}


bool NamedStreamSetType::equals(PabloType const * other) const noexcept {
    if (LLVM_LIKELY(llvm::isa<NamedStreamSetType>(other))) {
        NamedStreamSetType const * o = llvm::cast<NamedStreamSetType>(other);
        bool sameSig = mTypeName == o->mTypeName && mAliasedType->equals(o->mAliasedType);
        if (!sameSig)
            return false;
        // can assume same vector sizes because aliased types are equal and
        // the assertion in the constructor ensures number of stream names
        // is equivalent to the stream count in the aliased type
        for (size_t i = 0; i < mStreamNames.size(); ++i) {
            if (mStreamNames[i] != o->mStreamNames[i])
                return false;
        }
        return true;
    } else if (llvm::isa<StreamSetType>(other)) {
        return mAliasedType->equals(other);
    } else if (llvm::isa<AliasType>(other)) {
        return equals(llvm::cast<AliasType>(other)->getAliasedType());
    } else {
        return false;
    }
}

std::string NamedStreamSetType::asString(bool verbose) const noexcept {
    if (verbose) {
        llvm::SmallVector<char, 256> tmp;
        llvm::raw_svector_ostream out(tmp);
        out << mTypeName << " (aka " << mAliasedType->asString() << ")";
        return out.str().str();
    }
    return mTypeName.str();
}

NamedStreamSetType::NamedStreamSetType(llvm::StringRef typeName, StreamSetType * aliasType, std::vector<std::string> const & streamNames)
: PabloType(ClassTypeId::NAMED_STREAMSET)
, mTypeName(copyText(typeName, mAllocator))
, mAliasedType(aliasType)
, mStreamNames(copyArray(streamNames, mAllocator)) {
    assert (mStreamNames.size() == aliasType->getStreamCount());
}

} // namespace pablo::parse
} // namespace pablo
