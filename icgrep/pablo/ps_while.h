/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_WHILE_H
#define PS_WHILE_H

#include <pablo/pabloAST.h>

namespace pablo {

class While : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::While;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~While() {
    }
    inline PabloAST * getCondition() const {
        return getOperand(0);
    }
    inline PabloBlock & getBody() {
        return mBody;
    }
    inline const PabloBlock & getBody() const {
        return mBody;
    }
protected:
    While(PabloAST * expr, PabloBlock &body, PabloBlock * parent);
private:
    PabloBlock &        mBody;
};

}

#endif // PS_WHILE_H


