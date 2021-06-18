#ifndef PE_REPEAT_H
#define PE_REPEAT_H

#include <pablo/pabloAST.h>

namespace pablo {

class Integer;

class Repeat final : public Statement {
    friend class PabloBlock;
public:
    static bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Repeat;
    }
    static bool classof(const void *) {
        return false;
    }
    virtual ~Repeat() {
    }
    Integer * getFieldWidth() const {
        return llvm::cast<Integer>(getOperand(0));
    }
    PabloAST * getValue() const {
        return getOperand(1);
    }
protected:
    Repeat(Integer * const fieldWidth, PabloAST * const value, llvm::Type * type, const String * name, Allocator & allocator)
    : Statement(ClassTypeId::Repeat, type, { fieldWidth, value }, name, allocator) {

    }
};

}

#endif // PE_REPEAT_H
