#ifndef PE_INTEGER_H
#define PE_INTEGER_H

#include <pablo/pabloAST.h>

namespace pablo {

class Integer : public PabloAST {
    friend class SymbolGenerator;
public:


    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Integer;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Integer(){

    }
    inline int64_t value() const {
        return mValue;
    }
protected:
    Integer(const int64_t value, Type * type) noexcept
    : PabloAST(ClassTypeId::Integer, type, nullptr)
    , mValue(value)
    {

    }
private:
    int64_t mValue;
};

}

#endif // PE_INTEGER_H
