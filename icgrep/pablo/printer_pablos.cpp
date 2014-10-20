/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "printer_pablos.h"


//Regular Expressions
#include <re/re_re.h>
#include <re/re_cc.h>
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_seq.h>
#include <re/re_name.h>

//Pablo Expressions
#include <pablo/pabloAST.h>
#include <pablo/pe_advance.h>
#include <pablo/pe_and.h>
#include <pablo/pe_call.h>
#include <pablo/pe_matchstar.h>
#include <pablo/pe_not.h>
#include <pablo/pe_or.h>
#include <pablo/pe_scanthru.h>
#include <pablo/pe_sel.h>
#include <pablo/pe_var.h>
#include <pablo/pe_xor.h>
#include <pablo/ps_assign.h>
#include <pablo/ps_if.h>
#include <pablo/ps_while.h>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_ones.h>
#include <pablo/codegenstate.h>

using namespace re;
using namespace pablo;

std::string StatementPrinter::PrintStmts(const PabloBlock & cg_state)
{
    std::string strOut = "[";

    strOut = strOut.substr(0, strOut.length() - 1);
    strOut += "],[";

    strOut += Print_PB_PabloStmts(cg_state.expressions());

    strOut = strOut.substr(0, strOut.length() - 1);
    strOut += "]";

    return strOut;
}

std::string StatementPrinter::Print_PB_PabloStmts(const ExpressionList & stmts) {
    std::string strOut = "";
    for (const auto stmt : stmts) {
        strOut += ShowPabloS(stmt);
    }
    return strOut;
}

std::string StatementPrinter::Print_CC_PabloStmts(const pablo::ExpressionList &stmts) {
    std::string strOut = "Total Statements: " + std::to_string(stmts.size()) + "\n";
    for (const auto stmt : stmts) {
        strOut += ShowPabloS(stmt) + "\n";
    }
    return strOut;
}

std::string StatementPrinter::ShowPabloS(const PabloAST * stmt)
{
    if (const Assign * an = dyn_cast<const Assign>(stmt)) {
        return "Assign('" + an->getName() + "', " + ShowPabloAST(an->getExpr()) + "),";
    }
    else if (const Next * next = dyn_cast<const Next>(stmt)) {
        return "Next(" + next->getName() + ", " + ShowPabloAST(next->getExpr()) + ")";
    }
    else if (const If * ifstmt = dyn_cast<const If>(stmt)) {
        return "If(" + ShowPabloAST(ifstmt->getCondition()) + ", " + Print_PB_PabloStmts(ifstmt->getBody()) + ")";
    }
    else if (const While * whl = dyn_cast<const While>(stmt)) {
        return "While(" + ShowPabloAST(whl->getCondition()) + ", " + Print_PB_PabloStmts(whl->getBody()) + ")";
    }
    return "???";
}

std::string StatementPrinter::ShowPabloAST(const PabloAST *expr) {
    if (isa<const Zeroes>(expr)) {
        return "Zeroes";
    }
    else if (isa<const Ones>(expr)) {
        return "Ones";
    }
    else if (const Call * pablo_call = dyn_cast<const Call>(expr)) {
        return "Call '" + pablo_call->getCallee() + "'";
    }
    else if (const Var * pablo_var = dyn_cast<const Var>(expr)) {
        return "Var '" + pablo_var->getName() + "' ";
    }
    else if (const And * pablo_and = dyn_cast<const And>(expr)) {
        return "And(" + ShowPabloAST(pablo_and->getExpr1()) +", " + ShowPabloAST(pablo_and->getExpr2()) + ")";
    }
    else if (const Or * pablo_or = dyn_cast<const Or>(expr)) {
        return "Or(" + ShowPabloAST(pablo_or->getExpr1()) + ", " + ShowPabloAST(pablo_or->getExpr2()) + ")";
    }
    else if (const Sel * pablo_sel = dyn_cast<const Sel>(expr)) {
        return "((" + ShowPabloAST(pablo_sel->getCondition()) + "And " + ShowPabloAST(pablo_sel->getTrueExpr()) +
                ")|(Not(" + ShowPabloAST(pablo_sel->getCondition()) + ") And " + ShowPabloAST(pablo_sel->getFalseExpr()) + ")";
    }
    else if (const Not * pablo_not = dyn_cast<const Not>(expr)) {
        return "Not (" + ShowPabloAST(pablo_not->getExpr()) + ")";
    }
    else if (const Advance * adv = dyn_cast<const Advance>(expr)) {
        return "Advance(" + ShowPabloAST(adv->getExpr()) + ")";
    }
    else if (const MatchStar * mstar = dyn_cast<const MatchStar>(expr)) {
        return "MatchStar (" + ShowPabloAST(mstar->getMarker()) + ", " + ShowPabloAST(mstar->getCharClass()) + ")";
    }
    else if (const ScanThru * sthru = dyn_cast<const ScanThru>(expr)) {
        return "ScanThru (" + ShowPabloAST(sthru->getScanFrom()) + ", " + ShowPabloAST(sthru->getScanThru()) + ")";
    }
    return "???";
}


