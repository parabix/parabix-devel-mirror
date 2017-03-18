#ifndef PE_PHI_H
#define PE_PHI_H

#include <pablo/pabloAST.h>
#include <pablo/pe_string.h>

namespace pablo {

class Phi : public Variadic {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Phi;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Phi(){
    }
    PabloAST * getIncomingValue(const unsigned i) const {
        return getOperand(i);
    }
protected:
    Phi(llvm::Type * type, const unsigned NumReservedValues, const String * const name, Allocator & allocator)
    : Variadic(ClassTypeId::Phi, type, NumReservedValues, name, allocator) {

    }
};

}

#endif
