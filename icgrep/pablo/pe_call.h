#ifndef PE_CALL_H
#define PE_CALL_H

#include <pablo/pabloAST.h>
#include <pablo/pe_string.h>
#include <iostream>

namespace pablo {

class Call : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Call;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Call() {
    }
    inline const String * getCallee() const {
        return cast<String>(mOperand[0]);
    }
protected:
    Call(PabloAST * callee, PabloBlock * parent)
    : Statement(ClassTypeId::Call, {callee}, cast<String>(callee), parent) {

    }
};
}

#endif // PE_CALL_H


