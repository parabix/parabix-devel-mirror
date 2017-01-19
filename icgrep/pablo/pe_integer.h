#ifndef PE_INTEGER_H
#define PE_INTEGER_H

#include <pablo/pabloAST.h>

namespace pablo {

class Integer : public PabloAST {
    friend class SymbolGenerator;
public:
    using IntTy = int64_t;

    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Integer;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Integer(){

    }
    inline IntTy value() const {
        return mValue;
    }
protected:
    Integer(const IntTy value, llvm::Type * type, Allocator & allocator) noexcept
    : PabloAST(ClassTypeId::Integer, type, nullptr, allocator)
    , mValue(value)
    {

    }
private:
    IntTy mValue;
};

}

#endif // PE_INTEGER_H
