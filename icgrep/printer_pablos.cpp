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
#include <pablo/pe_pabloe.h>
#include <pablo/ps_pablos.h>
#include <pablo/pe_advance.h>
#include <pablo/pe_all.h>
#include <pablo/pe_and.h>
#include <pablo/pe_call.h>
#include <pablo/pe_charclass.h>
#include <pablo/pe_matchstar.h>
#include <pablo/pe_not.h>
#include <pablo/pe_or.h>
#include <pablo/pe_scanthru.h>
#include <pablo/pe_sel.h>
#include <pablo/pe_var.h>
#include <pablo/pe_xor.h>
#include <pablo/ps_pablos.h>
#include <pablo/ps_assign.h>
#include <pablo/ps_if.h>
#include <pablo/ps_while.h>

using namespace re;
using namespace pablo;

std::string StatementPrinter::PrintStmts(const CodeGenState & cg_state)
{
    std::string strOut = "[";

    strOut = strOut.substr(0, strOut.length() - 1);
    strOut += "],[";

    strOut = Print_PB_PabloStmts(cg_state.stmtsl, strOut);

    strOut = strOut.substr(0, strOut.length() - 1);
    strOut += "],";

    //Print the name of the variable that holds the match result for the overall expression so far.
    strOut += "'" + cg_state.newsym + "'";

    return strOut;
}

std::string StatementPrinter::Print_PB_PabloStmts(const List &stmts, std::string strOut) {
    for (const auto stmt : stmts) {
        strOut += ShowPabloS(stmt);
    }
    return strOut;
}

std::string StatementPrinter::Print_CC_PabloStmts(const List & stmts) {
    std::string strOut = "Total Statements: " + std::to_string(stmts.size()) + "\n";
    for (const auto stmt : stmts) {
        strOut += ShowPabloS(stmt) + "\n";
    }
    return strOut;
}

std::string StatementPrinter::ShowPabloS(const PabloE * stmt)
{
    std::string retVal = "";

    if (const Assign * an = dyn_cast<const Assign>(stmt))
    {
        retVal = "Assign('" + an->getM() + "', " + ShowPabloE(an->getExpr()) + "),";
    }
    else if (const If * ifstmt = dyn_cast<const If>(stmt))
    {
        retVal = "If(" + ShowPabloE(ifstmt->getExpr()) + ", " + Print_PB_PabloStmts(ifstmt->getPSList(), retVal) + ")";
    }
    else if (const While * whl = dyn_cast<const While>(stmt))
    {
        retVal = "While(" + ShowPabloE(whl->getExpr()) + ", " + Print_PB_PabloStmts(whl->getPSList(), retVal) + ")";
    }
    else retVal = "UNKNOWN_STATEMENT_TYPE!!!";
    return retVal;
}

std::string StatementPrinter::ShowPabloE(const PabloE *expr)
{
    std::string retVal = "";

    if (const All * all = dyn_cast<const All>(expr))
    {
        retVal = "All " + std::to_string(all->getValue()) + " ";
    }
    else if (const Call * pablo_call = dyn_cast<const Call>(expr))
    {
        retVal = "Call '" + pablo_call->getCallee() + "'";
    }
    else if (const Var * pablo_var = dyn_cast<const Var>(expr))
    {
        retVal = "Var '" + pablo_var->getVar() + "' ";
    }
    else if (const And * pablo_and = dyn_cast<const And>(expr))
    {
        retVal = "And(" + ShowPabloE(pablo_and->getExpr1()) +", " + ShowPabloE(pablo_and->getExpr2()) + ")";
    }
    else if (const Or * pablo_or = dyn_cast<const Or>(expr))
    {
        retVal = "Or(" + ShowPabloE(pablo_or->getExpr1()) + ", " + ShowPabloE(pablo_or->getExpr2()) + ")";
    }
    else if (const Sel * pablo_sel = dyn_cast<const Sel>(expr))
    {
        retVal = "((" + ShowPabloE(pablo_sel->getIf_expr()) + "And " + ShowPabloE(pablo_sel->getT_expr()) +
                ")|(Not(" + ShowPabloE(pablo_sel->getIf_expr()) + ") And " + ShowPabloE(pablo_sel->getF_expr()) + ")";
    }
    else if (const Not * pablo_not = dyn_cast<const Not>(expr))
    {
        retVal = "Not (" + ShowPabloE(pablo_not->getExpr()) + ")";
    }
    else if (const CharClass * cc = dyn_cast<const CharClass>(expr))
    {
        retVal = "CharClass '" + cc->getCharClass() + "'";
    }
    else if (const Advance * adv = dyn_cast<const Advance>(expr))
    {
        retVal = "Advance(" + ShowPabloE(adv->getExpr()) + ")";
    }
    else if (const MatchStar * mstar = dyn_cast<const MatchStar>(expr))
    {
        retVal = "MatchStar (" + ShowPabloE(mstar->getExpr1()) + ", " + ShowPabloE(mstar->getExpr2()) + ")";
    }
    else if (const ScanThru * sthru = dyn_cast<const ScanThru>(expr))
    {
        retVal = "ScanThru (" + ShowPabloE(sthru->getScanFrom()) + ", " + ShowPabloE(sthru->getScanThru()) + ")";
    }
    else retVal = "UNKNOWN_Pablo_EXPRESSION_TYPE!!!";

    return retVal;
}


