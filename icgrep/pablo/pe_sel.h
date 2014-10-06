/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_SEL_H
#define PE_SEL_H

#include <pablo/pe_pabloe.h>

namespace pablo {

struct CodeGenState;

class Sel : public PabloE {
    friend struct OptimizeSel;
    friend struct CodeGenState;
public:
    static inline bool classof(const PabloE * e) {
        return e->getClassTypeId() == ClassTypeId::Sel;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Sel() {
    }
    inline PabloE * getCondition() const {
        return mIf_expr;
    }
    inline PabloE * getTrueExpr() const {
        return mT_expr;
    }
    inline PabloE * getFalseExpr() const {
        return mF_expr;
    }
protected:
    Sel(PabloE* if_expr, PabloE* t_expr, PabloE* f_expr)
    : PabloE(ClassTypeId::Sel)
    , mIf_expr(if_expr)
    , mT_expr(t_expr)
    , mF_expr(f_expr)
    {

    }
private:
    PabloE * const mIf_expr;
    PabloE * const mT_expr;
    PabloE * const mF_expr;
};

struct OptimizeSel {
    inline OptimizeSel(CodeGenState & cg) : cg(cg) {}
    PabloE * operator()(PabloE * if_expr, PabloE * t_expr, PabloE * f_expr);
private:
    CodeGenState & cg;
};

inline PabloE * makeSel(PabloE * if_expr, PabloE * t_expr, PabloE * f_expr, CodeGenState & cg) {
    OptimizeSel run(cg);
    return run(if_expr, t_expr, f_expr);
}

}

#endif // PE_SEL_H

