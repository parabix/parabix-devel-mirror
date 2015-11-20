/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_WHILE_H
#define PS_WHILE_H

#include <pablo/pabloAST.h>

namespace pablo {

class Next;

class While : public Statement {
    friend class PabloBlock;
public:

    using NextAllocator = VectorAllocator::rebind<Next*>::other;
    using Variants = std::vector<Next *, NextAllocator>;

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
    inline Variants & getVariants() {
        return mVariant;
    }
    inline const Variants & getVariants() const {
        return mVariant;
    }
    inline PabloBlock * getBody() {
        return mBody;
    }
    inline PabloBlock * getBody() const {
        return mBody;
    }
    PabloBlock * setBody(PabloBlock * body);
protected:
    While(PabloAST * expr, const std::initializer_list<Next *> nextVars, PabloBlock * body);
    While(PabloAST * expr, const std::vector<Next *> & nextVars, PabloBlock * body);

private:
    PabloBlock *    mBody;
    Variants        mVariant;
};

}

#endif // PS_WHILE_H


