#ifndef RE_NAME_H
#define RE_NAME_H

#include <re/re_re.h>
#include <string>
#include <iostream>
#include <re/printer_re.h>

namespace pablo {
    class Var;
}


namespace re {

class CC;

class Name : public RE {
public:
    static inline bool classof(const RE * re) {
        return re->getClassTypeId() == ClassTypeId::Name;
    }
    static inline bool classof(const void *) {
        return false;
    }
    enum class Type {
        FixedLength
        , Unicode
        , UnicodeCategory
    };
    const std::string & getName() const;
    bool isNegated() const;
    Type getType() const;
    RE *getCC() const;
    pablo::Var * getVar() const {
        return mVar;
    }
    void setVar(pablo::Var * var) {
        mVar = var;
    }
    void setCC(RE *cc);
    virtual ~Name() {}
protected:
    friend Name * makeName(const std::string, RE *);
    friend Name * makeName(const std::string, const bool, const Type);

    Name(const std::string && name, const bool negated, const Type type, RE * cc)
    : RE(ClassTypeId::Name)
    , mName(std::move(name))
    , mNegated(negated)
    , mType(type)
    , mCC(cc)
    , mVar(nullptr)
    {

    }

private:
    const std::string   mName;
    const bool          mNegated;
    const Type          mType;
    RE *                mCC;
    pablo::Var *        mVar;
};

inline const std::string & Name::getName() const {
    return mName;
}

inline bool Name::isNegated() const {
    return mNegated;
}

inline Name::Type Name::getType() const {
    return mType;
}

inline RE * Name::getCC() const {
    return mCC;
}

inline void Name::setCC(RE * cc) {
    mCC = cc;
}

inline Name * makeName(const std::string name, const bool negated = false, const Name::Type type = Name::Type::FixedLength) {
    return new Name(std::move(name), negated, type, nullptr);
}

inline Name * makeName(const std::string name, RE * cc) {
    if (isa<Name>(cc)) {
        return cast<Name>(cc);
    }
    return new Name(std::move(name), false, Name::Type::FixedLength, cc);
}

}

#endif // RE_NAME_H
