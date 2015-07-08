#ifndef PE_CALL_H
#define PE_CALL_H

#include <pablo/pabloAST.h>
#include <pablo/pe_string.h>

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
        return cast<String>(getOperand(0));
    }
protected:
    Call(PabloAST * callee)
    : Statement(ClassTypeId::Call, {callee}, cast<String>(callee)) {

    }
};
}

#endif // PE_CALL_H


