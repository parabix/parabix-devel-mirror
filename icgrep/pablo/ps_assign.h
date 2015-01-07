/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_SETMARKER_H
#define PS_SETMARKER_H

#include <pablo/pe_string.h>
#include <pablo/symbol_generator.h>
#include <array>

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
    virtual PabloAST * getOperand(const unsigned index) const {
        assert (index == 0);
        return mExpr;
    }
    virtual unsigned getNumOperands() const {
        return 1;
    }
    virtual void setOperand(const unsigned index, PabloAST * value) {
        assert (index == 0);
        mExpr = value;
    }
    inline PabloAST * getExpr() const {
        return mExpr;
    }
    inline bool isOutputAssignment() const {
        return mOutputIndex >= 0;
    }
    inline int getOutputIndex() const {
        return mOutputIndex;
    }
protected:
    explicit Assign(PabloAST * expr, int outputIndex, String * name, PabloBlock * parent)
    : Statement(ClassTypeId::Assign, name, parent)
    , mExpr(expr)
    , mOutputIndex(outputIndex)
    {
        expr->addUser(this);
    }
private:
    PabloAST *          mExpr;
    const int           mOutputIndex;
};

}

#endif // PS_SETMARKER_H

