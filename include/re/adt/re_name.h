#ifndef RE_NAME_H
#define RE_NAME_H

#include <llvm/Support/Casting.h>
#include <llvm/Support/ErrorHandling.h>
#include <re/adt/re_cc.h>
#include <re/adt/re_re.h>

namespace UCD {
    class UnicodeSet;
}

namespace re {

using length_t = std::string::size_type;
class Name : public RE {
public:
    static inline bool classof(const RE * re) {
        return re->getClassTypeId() == ClassTypeId::Name;
    }
    static inline bool classof(const void *) {
        return false;
    }
    enum class Type {
        Unicode
        , UnicodeProperty
        , ZeroWidth
        , Unknown
    };
    std::string getNamespace() const;
    bool hasNamespace() const;
    std::string getName() const;
    std::string getFullName() const;
    Type getType() const;
    RE * getDefinition() const;
    bool operator<(const Name & other) const;
    bool operator<(const CC & other) const;
    bool operator>(const CC & other) const;
    void setDefinition(RE * definition);
    virtual ~Name() {}
protected:
    friend Name * makeName(const std::string & name, RE * cc);
    friend Name * makeZeroWidth(const std::string & name, RE * zerowidth);
    friend Name * makeName(CC * const cc);
    friend Name * makeName(const std::string &, Type, RE *);
    friend Name * makeName(const std::string &, const std::string &, Type, RE *);
    Name(const char * nameSpace, const length_t namespaceLength, const char * name, const length_t nameLength, Type type, RE * defn)
    : RE(ClassTypeId::Name)
    , mNamespaceLength(namespaceLength)
    , mNamespace(replicateString(nameSpace, namespaceLength))
    , mNameLength(nameLength)
    , mName(replicateString(name, nameLength))
    , mType(type)
    , mDefinition(defn) {

    }

private:
    const length_t      mNamespaceLength;
    const char * const  mNamespace;
    const length_t      mNameLength;
    const char * const  mName;
    Type                mType;
    RE *                mDefinition;
};

inline std::string Name::getNamespace() const {
    return std::string(mNamespace, mNamespaceLength);
}

inline bool Name::hasNamespace() const {
    return (mNamespaceLength != 0);
}

inline std::string Name::getName() const {
    return std::string(mName, mNameLength);
}

inline std::string Name::getFullName() const {
    if (hasNamespace()) return getNamespace() + ":" + getName();
    else return getName();
}

inline Name::Type Name::getType() const {
    return mType;
}

inline RE * Name::getDefinition() const {
    return mDefinition;
}

inline void Name::setDefinition(RE * definition) {
    assert (definition != nullptr);
    assert (definition != this);
    mDefinition = definition;
}

inline bool Name::operator < (const Name & other) const {
    if (LLVM_LIKELY(mDefinition && other.mDefinition && llvm::isa<CC>(mDefinition) && llvm::isa<CC>(other.mDefinition))) {
        return *llvm::cast<CC>(mDefinition) < *llvm::cast<CC>(other.mDefinition);
    } else if (mNamespaceLength < other.mNamespaceLength) {
        return true;
    } else if (mNamespaceLength > other.mNamespaceLength) {
        return false;
    } else if (mNameLength < other.mNameLength) {
        return true;
    } else if (mNameLength > other.mNameLength) {
        return false;
    }
    const auto diff = std::memcmp(mNamespace, other.mNamespace, mNamespaceLength);
    if (diff < 0) {
        return true;
    } else if (diff > 0) {
        return false;
    }
    return (std::memcmp(mName, other.mName, mNameLength) < 0);
}

inline bool Name::operator < (const CC & other) const {
    if (mDefinition && llvm::isa<CC>(mDefinition)) {
        return *llvm::cast<CC>(mDefinition) < other;
    }
    return RE::ClassTypeId::Name < RE::ClassTypeId::CC;
}

inline bool Name::operator > (const CC & other) const {
    if (mDefinition && llvm::isa<CC>(mDefinition)) {
        return other < *llvm::cast<CC>(mDefinition);
    }
    return RE::ClassTypeId::CC < RE::ClassTypeId::Name;
}

inline Name * makeName(const std::string & name, const Name::Type type, RE * defn = nullptr) {
    return new Name(nullptr, 0, name.c_str(), name.length(), type, defn);
}

inline Name * makeName(const std::string & property, const std::string & value, const Name::Type type, RE * defn = nullptr) {
    return new Name(property.c_str(), property.length(), value.c_str(), value.length(), type, defn);
}

inline Name * makeName(const std::string & name, RE * cc) {
    if (llvm::isa<Name>(cc)) {
        return llvm::cast<Name>(cc);
    }
    else if (llvm::isa<CC>(cc)) {
        return new Name(nullptr, 0, name.c_str(), name.length(), Name::Type::Unicode, cc);
    }
    else return new Name(nullptr, 0, name.c_str(), name.length(), Name::Type::Unknown, cc);
}

inline Name * makeName(CC * const cc) {
    const std::string name = cc->canonicalName();
    return new Name(nullptr, 0, name.c_str(), name.length(), Name::Type::Unicode, cc);
}

inline Name * makeZeroWidth(const std::string & name, RE * zerowidth = NULL) {
    return new Name(nullptr, 0, name.c_str(), name.length(), Name::Type::ZeroWidth, zerowidth);
}

template <typename To, typename FromTy> bool defined(FromTy * e) {
    if (llvm::isa<To>(e)) return true;
    if (llvm::isa<re::Name>(e)) {
        re::RE * def = llvm::cast<re::Name>(e)->getDefinition();
        return def && defined<To, FromTy>(def);
    }
    return false;
}

template <typename To, typename FromTy> To * defCast(FromTy * e) {
    if (llvm::isa<To>(e)) return llvm::cast<To>(e);
    if (llvm::isa<re::Name>(e)) {
        re::RE * def = llvm::cast<re::Name>(e)->getDefinition();
        if (def) return defCast<To, FromTy>(def);
    }
    return nullptr;
}

[[ noreturn ]]
inline void UndefinedNameError(const re::Name * n) {
    llvm::report_fatal_error("Error: Undefined name in regular expression: \"" + n->getName() + "\".");
}

class Capture : public RE {
public:
    std::string getName() const {
        return std::string(mName, mNameLength);
    }
    RE * getCapturedRE() const {return mCapturedRE;}
    static Capture * Create(std::string name, RE * captured) {
        return new Capture(name.c_str(), name.length(), captured);
    }
    RE_SUBTYPE(Capture)
private:
    Capture(const char * name, const length_t nameLength, RE * captured): RE(ClassTypeId::Capture)
    , mNameLength(nameLength)
    , mName(replicateString(name, nameLength))
    , mCapturedRE(captured) {}
    const length_t mNameLength;
    const char * const mName;
    RE * mCapturedRE;
};

inline RE * makeCapture(std::string name, RE * captured) {
    return Capture::Create(name, captured);
}

class Reference : public RE {
public:
    std::string getName() const  {
        return std::string(mName, mNameLength);
    }
    RE * getCapture() const {return mCapture;}
    static Reference * Create(std::string name, RE * capture) {
        return new Reference(name.c_str(), name.length(), capture);
    }
    RE_SUBTYPE(Reference)
private:
    Reference(const char * name, const length_t nameLength, RE * capture): RE(ClassTypeId::Reference)
    , mNameLength(nameLength)
    , mName(replicateString(name, nameLength))
    , mCapture(capture) {}
    const length_t mNameLength;
    const char * const mName;
    RE * mCapture;
};

inline RE * makeReference(std::string name, RE * capture){
    return Reference::Create(name, capture);
}

}
#endif // RE_NAME_H
