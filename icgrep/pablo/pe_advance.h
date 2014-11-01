/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_ADVANCE_H
#define PE_ADVANCE_H

#include "pabloAST.h"

namespace pablo {

class Advance : public PabloAST {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Advance;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Advance() {
    }
    virtual PabloAST * getOperand(const unsigned index) const {
        assert (index == 0);
        return mExpr;
    }
    virtual void setOperand(const unsigned index, PabloAST * value) {
        assert (index == 0);
        mExpr = value;
    }
    virtual unsigned getNumOperands() const {
        return 1;
    }
    inline PabloAST * getExpr() const {
        return mExpr;
    }
    inline int getAdvanceAmount() const {
        return mShiftAmount;
    }
protected:
    Advance(PabloAST * expr, int shiftAmount)
    : PabloAST(ClassTypeId::Advance)
    , mExpr(expr)
	, mShiftAmount(shiftAmount) {

    }
private:
    PabloAST * mExpr;
	int const mShiftAmount;
};

}

#endif // PE_ADVANCE_H



