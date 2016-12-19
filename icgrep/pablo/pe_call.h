#ifndef PE_CALL_H
#define PE_CALL_H

#include <pablo/pabloAST.h>
#include <pablo/prototype.h>

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
    virtual ~Call() { }

    inline const Prototype * getPrototype() const {
        return cast<Prototype>(getOperand(0));
    }
    inline const String * getCallee() const {
        return getPrototype()->getName();
    }
protected:
    Call(PabloAST * prototype, Allocator & allocator)
    : Statement(ClassTypeId::Call, nullptr, {prototype}, prototype->getName(), allocator) {

    }
};
}

#endif // PE_CALL_H


