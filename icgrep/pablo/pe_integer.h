#ifndef PE_INTEGER_H
#define PE_INTEGER_H

#include <pablo/pabloAST.h>

namespace pablo {

class Integer : public PabloAST {
    friend class SymbolGenerator;
    typedef u_int64_t integer_t;
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Integer;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Integer(){

    }
    inline integer_t value() const {
        return mValue;
    }
protected:
    Integer(const integer_t value) noexcept
    : PabloAST(ClassTypeId::Integer)
    , mValue(value)
    {

    }
private:
    const integer_t mValue;
};

}

#endif // PE_INTEGER_H
