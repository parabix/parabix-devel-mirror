#ifndef RE_NAME_H
#define RE_NAME_H

#include <re/re_re.h>
#include <re/re_cc.h>
#include <llvm/Support/Casting.h>

namespace UCD {
    class UnicodeSet;
}

namespace re {

class Name : public RE {
public:
    static inline bool classof(const RE * re) {
        return re->getClassTypeId() == ClassTypeId::Name;
    }
    static inline bool classof(const void *) {
        return false;
    }
    using length_t = std::string::size_type;
    enum class Type {
        Unicode
        , UnicodeProperty
        , Capture
        , Reference
        , ZeroWidth
        , Unknown
    };
    std::string getNamespace() const;
    bool hasNamespace() const;
    std::string getName() const;
    Type getType() const;
    RE * getDefinition() const;
    bool operator<(const Name & other) const;
    bool operator<(const CC & other) const;
    bool operator>(const CC & other) const;
    void setDefinition(RE * definition);
    virtual ~Name() {}
protected:
    friend Name * makeName(const std::string & name, RE * cc);
    friend Name * makeCapture(const std::string & name, RE * captured);
    friend Name * makeReference(const std::string & name, RE * captureName);
    friend Name * makeZeroWidth(const std::string & name, RE * zerowidth);
    friend Name * makeName(CC * const cc);
    friend Name * makeName(const std::string &, Type);
    friend Name * makeName(const std::string &, const std::string &, Type);
    friend Name * makeName(const std::string & nm, const Name::Type type, RE * defn); 
    Name(const char * nameSpace, const length_t namespaceLength, const char * name, const length_t nameLength, Type type, RE * defn)
    : RE(ClassTypeId::Name)
    , mNamespaceLength(namespaceLength)
    , mNamespace(replicateString(nameSpace, namespaceLength))
    , mNameLength(nameLength)
    , mName(replicateString(name, nameLength))
    , mType(type)
    , mDefinition(defn) {

    }
    inline const char * replicateString(const char * string, const length_t length) {
        if (string && (length > 0)) {
            char * allocated = reinterpret_cast<char*>(mAllocator.allocate(length));
            std::memcpy(allocated, string, length);
            return allocated;
        }
        return nullptr;
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

inline Name * makeName(const std::string & name, const Name::Type type) {
    return new Name(nullptr, 0, name.c_str(), name.length(), type, nullptr);
}

inline Name * makeName(const std::string & property, const std::string & value, const Name::Type type) {
    return new Name(property.c_str(), property.length(), value.c_str(), value.length(), type, nullptr);
}
    
inline Name * makeName(const std::string & nm, const Name::Type type, RE * defn) {
    return new Name(nullptr, 0, nm.c_str(), nm.length(), type, defn);
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

inline Name * makeCapture(const std::string & name, RE * captured) {
    return new Name(nullptr, 0, name.c_str(), name.length(), Name::Type::Capture, captured);
}
    
inline Name * makeReference(const std::string & name, RE * captureName) {
    return new Name(nullptr, 0, name.c_str(), name.length(), Name::Type::Reference, captureName);
}

inline Name * makeZeroWidth(const std::string & name, RE * zerowidth = NULL) {
    return new Name(nullptr, 0, name.c_str(), name.length(), Name::Type::ZeroWidth, zerowidth);
}
}

#endif // RE_NAME_H
