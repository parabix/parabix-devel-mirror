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
    friend class Next;
    friend class Var;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Assign;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Assign() {
    }
    inline PabloAST * getExpression() const {
        return getOperand(0);
    }
    inline void setExpression(PabloAST * value) {
        return setOperand(0, value);
    }
protected:
    explicit Assign(PabloAST * expr, String * name)
    : Statement(ClassTypeId::Assign, {expr}, name)
    {

    }
};

}

#endif // PS_SETMARKER_H

