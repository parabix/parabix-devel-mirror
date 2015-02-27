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
public:
    using DefinedVars = std::vector<PabloAST *, VectorAllocator>;

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
    inline PabloBlock & getBody() {
        return mBody;
    }
    inline const PabloBlock & getBody() const {
        return mBody;
    }
    inline const DefinedVars & getDefined() const {
        return mDefined;
    }
    inline void setInclusiveCarryCount(const unsigned count) {
        mCarryCount = count;
    }
    inline unsigned getInclusiveCarryCount() const {
        return mCarryCount;
    }
    inline void setInclusiveAdvanceCount(const unsigned count) {
        mAdvanceCount = count;
    }
    inline unsigned getInclusiveAdvanceCount() const {
        return mAdvanceCount;
    }
protected:
    If(PabloAST * expr, const std::initializer_list<Assign *> definedVars, PabloBlock & body, PabloBlock * parent);

    If(PabloAST * expr, const std::vector<Assign *> & definedVars, PabloBlock & body, PabloBlock * parent);
private:
    PabloBlock &    mBody;
    DefinedVars     mDefined;
    unsigned        mCarryCount;
    unsigned        mAdvanceCount;
};

}

#endif // PS_IF_H


