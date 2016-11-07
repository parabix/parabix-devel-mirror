#ifndef PE_NEXT_H
#define PE_NEXT_H

#include <pablo/pabloAST.h>
#include <pablo/ps_assign.h>
#include <pablo/symbol_generator.h>
#include <array>

namespace pablo {

class Next : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Next;
    }
    static inline bool classof(const void *) {
        return false;
    }
    inline PabloAST * getVariable() const {
        return getOperand(1);
    }
    inline PabloAST * getValue() const {
        return getOperand(0);
    }
    virtual ~Next() {}
protected:
    explicit Next(PabloAST * initial, PabloAST * value)
    : Statement(ClassTypeId::Next, initial->getType(), {value, initial}, initial->getName()) {

    }
};

}


#endif // PE_NEXT_H
