/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_SETMARKER_H
#define PS_SETMARKER_H

#include <pablo/pe_string.h>
#include <array>

namespace pablo {

class Assign : public Statement {
    friend class PabloBlock;
    friend class Next;
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
        assert (index < 2);
        return mExprs[index];
    }
    virtual unsigned getNumOperands() const {
        return 2;
    }
    virtual void setOperand(const unsigned index, PabloAST * value) {
        assert (index < 2);
        mExprs[index] = value;
    }
    inline const String * getName() const {
        return cast<String>(mExprs[0]);
    }
    inline PabloAST * getExpr() const {
        return mExprs[1];
    }
    inline bool isOutputAssignment() const {
        return mOutputIndex >= 0;
    }
    inline int getOutputIndex() const {
        return mOutputIndex;
    }
protected:
    Assign(PabloAST * name, PabloAST * expr, const int outputIndex, StatementList * parent)
    : Statement(ClassTypeId::Assign, parent)
    , mExprs({name, expr})
    , mOutputIndex(outputIndex)
    {

    }
private:
    std::array<PabloAST *,2>    mExprs;
    const int                   mOutputIndex;
};

}

#endif // PS_SETMARKER_H

