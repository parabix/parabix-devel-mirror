#ifndef RE_NAME_H
#define RE_NAME_H

#include <re/re_re.h>
#include <re/re_cc.h>
#include <string>
#include <re/printer_re.h>

namespace pablo {
    class PabloAST;
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
    enum class Type {
        Byte
        , Unicode
        , UnicodeProperty
        , Unknown
    };
    const std::string & getNamespace() const;
    const std::string & getName() const;
    void setName(std::string);
    Type getType() const;
    RE *getDefinition() const;
    pablo::PabloAST * getCompiled() const {
        return mCompiled;
    }
    void setCompiled(pablo::PabloAST * var) {
        mCompiled = var;
    }
    void setDefinition(RE * def);
    virtual ~Name() {}
protected:
    friend Name * makeName(const std::string, RE *);    
    friend Name * makeByteName(const std::string, RE *);
    friend Name * makeName(const std::string, const Type);
    friend Name * makeName(const std::string, const std::string, const Type);    
    void* operator new (std::size_t size) noexcept {
        return mAllocator.allocate(size);
    }
    Name(const std::string && nameSpace, const std::string && name, const Type type, RE * defn)
    : RE(ClassTypeId::Name)
    , mNamespace(std::move(nameSpace))
    , mName(std::move(name))
    , mType(type)
    , mDefiningRE(defn)
    , mCompiled(nullptr)
    {

    }

private:
    std::string         mNamespace;
    std::string         mName;
    const Type          mType;
    RE *                mDefiningRE;
    pablo::PabloAST *   mCompiled;
};

inline const std::string & Name::getNamespace() const {
    return mNamespace;
}

    inline const std::string & Name::getName() const {
        return mName;
    }
    
    inline void Name::setName(std::string n) {
        mName = n;
    }
    
inline Name::Type Name::getType() const {
    return mType;
}

inline RE * Name::getDefinition() const {
    return mDefiningRE;
}

inline void Name::setDefinition(RE * d) {
    mDefiningRE = d;
}

inline Name * makeName(const std::string name, const Name::Type type = Name::Type::Unicode) {
    return new Name("", std::move(name), type, nullptr);
}

inline Name * makeName(const std::string property, const std::string value, const Name::Type type = Name::Type::Unicode) {
    return new Name(std::move(property), std::move(value), type, nullptr);
}

inline Name * makeName(const std::string name, RE * cc) {
    if (isa<Name>(cc)) {
        return cast<Name>(cc);
    }
    else if (isa<CC>(cc)) {
        Name::Type ccType = cast<CC>(cc)->max_codepoint() <= 0x7F ? Name::Type::Byte : Name::Type::Unicode;
        return new Name("", std::move(name), ccType, cc);
    }
    else return new Name("", std::move(name), Name::Type::Unknown, cc);
}

inline Name * makeByteName(const std::string name, RE * cc) {
    if (isa<Name>(cc)) {
        return cast<Name>(cc);
    }
    else {
        return new Name("", std::move(name), Name::Type::Byte, cc);
    }
}

}

#endif // RE_NAME_H
