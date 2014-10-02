#ifndef RE_NAME_H
#define RE_NAME_H

#include "re_re.h"
#include <string>

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
        FixedLength
        ,Unicode
        ,UnicodeCategory
    };
    void setName(std::string name);
    std::string getName() const;
    void setNegated(const bool is_negated);
    bool isNegated() const;
    void setType(const Type type);
    Type getType() const;
    virtual ~Name() {}
protected:
    friend Name * makeName();
    friend Name * makeName(const Name *);
    friend Name * makeName(std::string, const bool, const Type);
    Name();
    Name(std::string name, const bool negated, const Type type);
private:
    std::string mName;
    bool mNegated;
    Type mType;
};

inline Name::Name()
: RE(ClassTypeId::Name)
, mName()
, mNegated(false)
, mType(Type::FixedLength) {

}

inline Name::Name(std::string name, const bool negated, const Type type)
: RE(ClassTypeId::Name)
, mName(name)
, mNegated(negated)
, mType(type) {

}

inline void Name::setName(std::string name) {
    mName = name;
}

inline std::string Name::getName() const {
    return mName;
}

inline bool Name::isNegated() const {
    return mNegated;
}

inline void Name::setNegated(const bool is_negated) {
    mNegated = is_negated;
}

inline void Name::setType(const Type type) {
    mType = type;
}

inline Name::Type Name::getType() const {
    return mType;
}

inline Name * makeName() {
    return new Name();
}

inline Name * makeName(const Name * name) {
    return new Name(*name);
}

inline Name * makeName(std::string name, const bool negated = false, const Name::Type type = Name::Type::FixedLength) {
    return new Name(name, negated, type);
}

}

#endif // RE_NAME_H
