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

std::string PabloPrinter::print(const PabloBlock & block)
{
    std::string strOut = "[";

    strOut = strOut.substr(0, strOut.length() - 1);
    strOut += "],[";

    strOut += print(block.statements());

    strOut = strOut.substr(0, strOut.length() - 1);
    strOut += "]";

    return strOut;
}

std::string PabloPrinter::print(const StatementList & stmts) {
    std::string strOut = "";
    for (const Statement * stmt : stmts) {
        strOut += print(stmt) + "\n";
    }
    return strOut;
}

std::string PabloPrinter::print(const Statement * stmt) {
    if (stmt == nullptr) {
        return "<null>";
    }
    else if (const Assign * an = dyn_cast<const Assign>(stmt)) {
        std::string result = "Assign('" + an->getName()->str() + "'," + print(an->getExpr());
        if (an->isOutputAssignment()) {
            result += ",Output=" + std::to_string(an->getOutputIndex());
        }
        return result + ")";
    }
    else if (const Next * next = dyn_cast<const Next>(stmt)) {
        return "Next(" + next->getName()->str() + "," + print(next->getExpr()) + ")";
    }
    else if (const If * ifstmt = dyn_cast<const If>(stmt)) {
        return "If(" + print(ifstmt->getCondition()) + "," + print(ifstmt->getBody()) + ")";
    }
    else if (const While * whl = dyn_cast<const While>(stmt)) {
        return "While(" + print(whl->getCondition()) + "," + print(whl->getBody()) + ")";
    }
    return "???";
}

std::string PabloPrinter::print(const PabloAST *expr) {
    if (expr == nullptr) {
        return "<null>";
    }
    else if (isa<const Zeroes>(expr)) {
        return "Zeroes";
    }
    else if (isa<const Ones>(expr)) {
        return "Ones";
    }
    else if (const Call * pablo_call = dyn_cast<const Call>(expr)) {
        return "Call '" + pablo_call->getCallee()->str() + "'";
    }
    else if (const Var * pablo_var = dyn_cast<const Var>(expr)) {
        return "Var '" + pablo_var->getName()->str() + "' ";
    }
    else if (const And * pablo_and = dyn_cast<const And>(expr)) {
        return "And(" + print(pablo_and->getExpr1()) +"," + print(pablo_and->getExpr2()) + ")";
    }
    else if (const Or * pablo_or = dyn_cast<const Or>(expr)) {
        return "Or(" + print(pablo_or->getExpr1()) + "," + print(pablo_or->getExpr2()) + ")";
    }
    else if (const Sel * pablo_sel = dyn_cast<const Sel>(expr)) {
        return "(" + print(pablo_sel->getCondition()) + " ? " + print(pablo_sel->getTrueExpr()) + " : " + print(pablo_sel->getFalseExpr()) + ")";
    }
    else if (const Not * pablo_not = dyn_cast<const Not>(expr)) {
        return "Not (" + print(pablo_not->getExpr()) + ")";
    }
    else if (const Advance * adv = dyn_cast<const Advance>(expr)) {
        return "Advance(" + print(adv->getExpr()) + ", " + std::to_string(adv->getAdvanceAmount()) + ")";
    }
    else if (const MatchStar * mstar = dyn_cast<const MatchStar>(expr)) {
        return "MatchStar(" + print(mstar->getMarker()) + ", " + print(mstar->getCharClass()) + ")";
    }
    else if (const ScanThru * sthru = dyn_cast<const ScanThru>(expr)) {
        return "ScanThru(" + print(sthru->getScanFrom()) + ", " + print(sthru->getScanThru()) + ")";
    }
    else if (isa<Statement>(expr)) {
        return print(cast<Statement>(expr));
    }
    return "???";
}


