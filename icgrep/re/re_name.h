#ifndef RE_NAME_H
#define RE_NAME_H

#include <re/re_re.h>
#include <re/re_cc.h>
#include <string>

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
        Byte
        , Unicode
        , UnicodeProperty
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
    friend Name * makeName(CC * const cc);
    friend Name * makeName(const std::string &, const Type);
    friend Name * makeName(const std::string &, const std::string &, const Type);
    Name(const char * nameSpace, const length_t namespaceLength, const char * name, const length_t nameLength, const Type type, RE * defn)
    : RE(ClassTypeId::Name)
    , mNamespaceLength(namespaceLength)
    , mNamespace(replicateString(nameSpace, namespaceLength))
    , mNameLength(nameLength)
    , mName(replicateString(name, nameLength))
    , mType(type)
    , mDefinition(defn)
    {

    }
    inline const char * replicateString(const char * string, const length_t length) {
        if (string) {
            char * allocated = reinterpret_cast<char*>(mAllocator.allocate(length));
            std::memcpy(allocated, string, length);
            string = allocated;
        }
        return string;
    }

private:
    const length_t      mNamespaceLength;
    const char * const  mNamespace;
    const length_t      mNameLength;
    const char * const  mName;
    const Type          mType;
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
    assert (definition != this);
    mDefinition = definition;
}

inline bool Name::operator < (const Name & other) const {
    if (LLVM_LIKELY(mDefinition && other.mDefinition && isa<CC>(mDefinition) && isa<CC>(other.mDefinition))) {
        return *cast<CC>(mDefinition) < *cast<CC>(other.mDefinition);
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
    if (mDefinition && isa<CC>(mDefinition)) {
        return *cast<CC>(mDefinition) < other;
    }
    return false;
}

inline bool Name::operator > (const CC & other) const {
    if (mDefinition && isa<CC>(mDefinition)) {
        return other < *cast<CC>(mDefinition);
    }
    return true;
}

inline Name * makeName(const std::string & name, const Name::Type type) {
    return new Name(nullptr, 0, name.c_str(), name.length(), type, nullptr);
}

inline Name * makeName(const std::string & property, const std::string & value, const Name::Type type) {
    return new Name(property.c_str(), property.length(), value.c_str(), value.length(),  type, nullptr);
}

inline Name * makeName(const std::string & name, RE * cc) {
    if (isa<Name>(cc)) {
        return cast<Name>(cc);
    }
    else if (isa<CC>(cc)) {
        Name::Type ccType = cast<CC>(cc)->max_codepoint() <= 0x7F ? Name::Type::Byte : Name::Type::Unicode;
        return new Name(nullptr, 0, name.c_str(), name.length(), ccType, cc);
    }
    else return new Name(nullptr, 0, name.c_str(), name.length(), Name::Type::Unknown, cc);
}

inline Name * makeName(CC * const cc) {
    const bool ascii = cc->max_codepoint() <= 0x7F;
    const std::string name = cc->canonicalName(ascii ? CC_type::ByteClass : CC_type::UnicodeClass);
    return new Name(nullptr, 0, name.c_str(), name.length(), ascii ? Name::Type::Byte : Name::Type::Unicode, cc);
}

}

#endif // RE_NAME_H
