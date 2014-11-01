/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_SEL_H
#define PE_SEL_H

#include <pablo/pabloAST.h>
#include <array>

namespace pablo {

class PabloBlock;

class Sel : public PabloAST {
    friend struct OptimizeSel;
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Sel;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Sel() {
    }
    virtual PabloAST * getOperand(const unsigned index) const {
        assert (index < 3);
        return mExprs[index];
    }
    virtual unsigned getNumOperands() const {
        return 3;
    }
    virtual void setOperand(const unsigned index, PabloAST * value) {
        assert (index < 3);
        mExprs[index] = value;
    }
    inline PabloAST * getCondition() const {
        return mExprs[0];
    }
    inline PabloAST * getTrueExpr() const {
        return mExprs[1];
    }
    inline PabloAST * getFalseExpr() const {
        return mExprs[2];
    }
protected:
    Sel(PabloAST* if_expr, PabloAST* t_expr, PabloAST* f_expr)
    : PabloAST(ClassTypeId::Sel)
    , mExprs({if_expr, t_expr, f_expr})
    {

    }
private:
    std::array<PabloAST*, 3> mExprs;
};

struct OptimizeSel {
    inline OptimizeSel(PabloBlock & cg) : cg(cg) {}
    PabloAST * operator()(PabloAST * if_expr, PabloAST * t_expr, PabloAST * f_expr);
private:
    PabloBlock & cg;
};

}

#endif // PE_SEL_H

