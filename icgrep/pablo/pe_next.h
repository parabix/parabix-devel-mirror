#ifndef PE_NEXT_H
#define PE_NEXT_H

#include <pablo/pabloAST.h>
#include <pablo/ps_assign.h>
#include <pablo/symbol_generator.h>
#include <array>

namespace pablo {

class Assign;

class Next : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Next;
    }
    static inline bool classof(const void *) {
        return false;
    }
    inline const Assign * getInitial() const {
        return cast<const Assign>(getOperand(0));
    }
    inline PabloAST * getExpr() const {
        return getOperand(1);
    }
    virtual ~Next() {}
protected:
    Next(PabloAST * initial, PabloAST * expr, PabloBlock * parent)
    : Statement(ClassTypeId::Next, {cast<Assign>(initial), cast<Assign>(expr)}, cast<Assign>(initial)->getName(), parent) {
        this->addUser(initial);
    }
};

}


#endif // PE_NEXT_H
