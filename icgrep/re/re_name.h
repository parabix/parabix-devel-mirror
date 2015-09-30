#ifndef RE_NAME_H
#define RE_NAME_H

#include <re/re_re.h>
#include <re/re_cc.h>
#include <string>
#include <re/printer_re.h>

namespace pablo {
    class PabloAST;
}

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
    pablo::PabloAST * getCompiled() const {
        return mCompiled;
    }
    void setCompiled(pablo::PabloAST * var) {
        mCompiled = var;
    }
    void setDefinition(RE * def);
    virtual ~Name() {}
protected:
    friend Name * makeName(const std::string &, RE *);
    friend Name * makeByteName(const std::string &, RE *);
    friend Name * makeName(const std::string &, const Type);
    friend Name * makeName(const std::string &, const std::string &, const Type);
    Name(const char * nameSpace, const length_t namespaceLength, const char * name, const length_t nameLength, const Type type, RE * defn)
    : RE(ClassTypeId::Name)
    , mNamespaceLength(namespaceLength)
    , mNamespace(replicateString(nameSpace, namespaceLength))
    , mNameLength(nameLength)
    , mName(replicateString(name, nameLength))
    , mType(type)
    , mDefiningRE(defn)
    , mCompiled(nullptr)
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
    RE *                mDefiningRE;
    pablo::PabloAST *   mCompiled;
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
    return mDefiningRE;
}

inline void Name::setDefinition(RE * d) {
    assert (d != this);
    mDefiningRE = d;
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

inline Name * makeByteName(const std::string & name, RE * cc) {
    if (isa<Name>(cc)) {
        return cast<Name>(cc);
    }
    else {
        return new Name(nullptr, 0, name.c_str(), name.length(), Name::Type::Byte, cc);
    }
}

}

#endif // RE_NAME_H
