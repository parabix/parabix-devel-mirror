/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_SETMARKER_H
#define PS_SETMARKER_H

#include <pablo/pe_string.h>

namespace pablo {

class Assign : public Statement {
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
    inline PabloAST * getVariable() const {
        return getOperand(0);
    }
    inline PabloAST * getValue() const {
        return getOperand(1);
    }
    inline void setValue(PabloAST * value) {
        return setOperand(1, value);
    }
protected:
    explicit Assign(PabloAST * variable, PabloAST * expr)
    : Statement(ClassTypeId::Assign, nullptr, {variable, expr}, nullptr) {

    }
};

}

#endif // PS_SETMARKER_H

