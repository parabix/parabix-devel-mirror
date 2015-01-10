#ifndef PE_INTEGER_H
#define PE_INTEGER_H

#include <pablo/pabloAST.h>

namespace pablo {

class Integer : public PabloAST {
    using value_t = size_t;
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Integer;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Integer(){

    }
    inline value_t value() const {
        return mValue;
    }
protected:
    Integer(const value_t value) noexcept
    : PabloAST(ClassTypeId::Integer)
    , mValue(value)
    {

    }
private:
    const value_t mValue;
};

}

#endif // PE_INTEGER_H
