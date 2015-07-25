#ifndef PE_INTEGER_H
#define PE_INTEGER_H

#include <pablo/pabloAST.h>

namespace pablo {

class Integer : public PabloAST {
    friend class SymbolGenerator;
public:
    typedef u_int64_t Type;
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Integer;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Integer(){

    }
    inline Type value() const {
        return mValue;
    }
protected:
    Integer(const Type value) noexcept
    : PabloAST(ClassTypeId::Integer)
    , mValue(value)
    {

    }
private:
    const Type mValue;
};

}

#endif // PE_INTEGER_H
