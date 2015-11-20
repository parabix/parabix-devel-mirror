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
    inline const Assign * getInitial() const {
        return cast<const Assign>(getOperand(1));
    }
    inline PabloAST * getExpr() const {
        return getOperand(0);
    }
    virtual ~Next() {}
protected:
    explicit Next(PabloAST * initial, PabloAST * expr)
    : Statement(ClassTypeId::Next, {expr, cast<Assign>(initial)}, cast<Assign>(initial)->getName()) {

    }
};

}


#endif // PE_NEXT_H
