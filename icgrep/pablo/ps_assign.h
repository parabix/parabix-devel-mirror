/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_SETMARKER_H
#define PS_SETMARKER_H

#include <pablo/pe_string.h>
#include <pablo/pe_var.h>

namespace pablo {

class Assign final : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Assign;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Assign() {
    }
    inline Var * getVariable() const {
        return llvm::cast<Var>(getOperand(0));
    }
    inline PabloAST * getValue() const {
        return getOperand(1);
    }
    inline void setValue(PabloAST * value) {
        return setOperand(1, value);
    }
protected:
    explicit Assign(Var * variable, PabloAST * expr, Allocator & allocator)
    : Statement(ClassTypeId::Assign, nullptr, {variable, expr}, nullptr, allocator) {

    }
};

}

#endif // PS_SETMARKER_H

