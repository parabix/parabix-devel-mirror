/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/pe_sel.h>
#include <pablo/codegenstate.h>

namespace pablo {

PabloAST * OptimizeSel::operator()(PabloAST * if_expr, PabloAST * t_expr, PabloAST * f_expr) {
    if (All * all_if_expr = dyn_cast<All>(if_expr)) {
        if (all_if_expr->getValue()) {
            return t_expr;
        }
        else {
            return f_expr;
        }
    }
    else if (All * all_t_expr = dyn_cast<All>(t_expr)) {
        if (all_t_expr->getValue()) {
            return cg.createOr(if_expr, f_expr);
        }
        else {
            return cg.createAnd(cg.createNot(if_expr), f_expr);
        }
    }
    else if (All * all_f_expr = dyn_cast<All>(f_expr)) {
        if (all_f_expr->getValue()) {
            return cg.createOr(cg.createNot(if_expr), t_expr);
        }
        else {
            return cg.createAnd(if_expr, t_expr);
        }
    }
    else if (equals(t_expr, f_expr)) {
        return t_expr;
    }
    return new Sel(if_expr, t_expr, f_expr);
}

}
