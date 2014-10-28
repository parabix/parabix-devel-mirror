/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_SEL_H
#define PE_SEL_H

#include <pablo/pabloAST.h>

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
    inline PabloAST * getCondition() const {
        return mIf_expr;
    }
    inline PabloAST * getTrueExpr() const {
        return mT_expr;
    }
    inline PabloAST * getFalseExpr() const {
        return mF_expr;
    }
protected:
    void* operator new (std::size_t size) noexcept {
        return mAllocator.allocate(size);
    }
    Sel(PabloAST* if_expr, PabloAST* t_expr, PabloAST* f_expr)
    : PabloAST(ClassTypeId::Sel)
    , mIf_expr(if_expr)
    , mT_expr(t_expr)
    , mF_expr(f_expr)
    {

    }
private:
    PabloAST * const mIf_expr;
    PabloAST * const mT_expr;
    PabloAST * const mF_expr;
};

struct OptimizeSel {
    inline OptimizeSel(PabloBlock & cg) : cg(cg) {}
    PabloAST * operator()(PabloAST * if_expr, PabloAST * t_expr, PabloAST * f_expr);
private:
    PabloBlock & cg;
};

}

#endif // PE_SEL_H

