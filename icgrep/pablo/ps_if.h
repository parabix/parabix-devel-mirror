/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_IF_H
#define PS_IF_H

#include <pablo/pabloAST.h>
#include <vector>

namespace pablo {

class Assign;

class If : public Statement {
    friend class PabloBlock;
    friend class Statement;
    friend class Simplifier;
public:
    using DefinedAllocator = VectorAllocator::rebind<Assign *>::other;
    using DefinedVars = std::vector<Assign *, DefinedAllocator>;

    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::If;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~If() {
        mDefined.clear();
    }
    inline PabloAST * getCondition() const {
        return getOperand(0);
    }
    inline PabloBlock * getBody() {
        return mBody;
    }
    inline  PabloBlock * getBody() const {
        return mBody;
    }
    PabloBlock * setBody(PabloBlock * body);
    inline const DefinedVars & getDefined() const {
        return mDefined;
    }
    inline DefinedVars & getDefined() {
        return mDefined;
    }
    void addDefined(Assign * def);
    DefinedVars::iterator removeDefined(Assign * def);
protected:
    If(PabloAST * expr, const std::initializer_list<Assign *> definedVars, PabloBlock * body);
    If(PabloAST * expr, const std::vector<Assign *> & definedVars, PabloBlock * body);
private:
    PabloBlock *    mBody;
    DefinedVars     mDefined;
};

}

#endif // PS_IF_H


