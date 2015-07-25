#ifndef PE_SETITHBIT_H
#define PE_SETITHBIT_H

#include <pablo/pabloAST.h>
#include <pablo/pe_integer.h>

namespace pablo {

class SetIthBit : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::SetIthBit;
    }
    static inline bool classof(const void *) {
        return false;
    }
    inline PabloAST * variable() const {
        return getOperand(0);
    }
    inline Integer::Type position() const {
        return cast<Integer>(getOperand(1))->value();
    }
    inline bool value() const {
        return cast<Integer>(getOperand(2))->value() == 1;
    }
    virtual ~SetIthBit() {}
protected:
    explicit SetIthBit(PabloAST * var, PabloAST * position, PabloAST * value, PabloAST * name)
    : Statement(ClassTypeId::SetIthBit, {var, cast<Integer>(position), cast<Integer>(value)}, name) {

    }
};

}


#endif // PE_SETITHBIT_H
