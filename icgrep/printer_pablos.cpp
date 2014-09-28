/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "printer_pablos.h"

//Regular Expressions
#include "re/re_re.h"
#include "re/re_cc.h"
#include "re/re_start.h"
#include "re/re_end.h"
#include "re/re_seq.h"
#include "re/re_name.h"

//Pablo Expressions
#include "pe_advance.h"
#include "pe_all.h"
#include "pe_and.h"
#include "pe_call.h"
#include "pe_charclass.h"
#include "pe_matchstar.h"
#include "pe_not.h"
#include "pe_or.h"
#include "pe_pabloe.h"
#include "pe_scanthru.h"
#include "pe_sel.h"
#include "pe_var.h"
#include "pe_xor.h"

//Pablo Statements
#include "ps_pablos.h"
#include "ps_assign.h"
#include "ps_if.h"
#include "ps_while.h"


std::string StatementPrinter::PrintStmts(CodeGenState cg_state)
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

std::string StatementPrinter::Print_PB_PabloStmts(std::list<PabloS*> stmts, std::string strOut)
{
    std::list<PabloS*>::iterator it;
    //std::cout << "Total Statements: " + std::to_string(stmts.size()) << std::endl;
    for (it = stmts.begin(); it != stmts.end(); ++it)
    {
        strOut += ShowPabloS(*it);
    }

    return strOut;
}

std::string StatementPrinter::Print_CC_PabloStmts(std::list<PabloS*> stmts)
{
    std::string strOut;

    std::cout << "Total Statements: " + std::to_string(stmts.size()) << std::endl;
    std::list<PabloS*>::iterator it;
    for (it = stmts.begin(); it != stmts.end(); ++it)
    {
        strOut += ShowPabloS(*it) + "\n";
    }

    return strOut;
}

std::string StatementPrinter::ShowPabloS(PabloS* stmt)
{
    std::string retVal = "";

    if (Assign* an = dynamic_cast<Assign*>(stmt))
    {
        retVal = "Assign('" + an->getM() + "', " + ShowPabloE(an->getExpr()) + "),";
    }
    else if (If* ifstmt = dynamic_cast<If*>(stmt))
    {
        retVal = "If(" + ShowPabloE(ifstmt->getExpr()) + ", " + Print_PB_PabloStmts(ifstmt->getPSList(), retVal) + ")";
    }
    else if (While* whl = dynamic_cast<While*>(stmt))
    {
        retVal = "While(" + ShowPabloE(whl->getExpr()) + ", " + Print_PB_PabloStmts(whl->getPSList(), retVal) + ")";
    }
    else retVal = "UNKNOWN_STATEMENT_TYPE!!!";
    return retVal;
}

std::string StatementPrinter::ShowPabloE(PabloE* expr)
{
    std::string retVal = "";

    if (All* all = dynamic_cast<All*>(expr))
    {
        retVal = "All " + std::to_string(all->getNum()) + " ";
    }
    else if (Call* pablo_call = dynamic_cast<Call*>(expr))
    {
        retVal = "Call '" + pablo_call->getCallee() + "'";
    }
    else if (Var* pablo_var = dynamic_cast<Var*>(expr))
    {
        retVal = "Var '" + pablo_var->getVar() + "' ";
    }
    else if (And* pablo_and = dynamic_cast<And*>(expr))
    {
        retVal = "And(" + ShowPabloE(pablo_and->getExpr1()) +", " + ShowPabloE(pablo_and->getExpr2()) + ")";
    }
    else if (Or* pablo_or = dynamic_cast<Or*>(expr))
    {
        retVal = "Or(" + ShowPabloE(pablo_or->getExpr1()) + ", " + ShowPabloE(pablo_or->getExpr2()) + ")";
    }
    else if (Sel* pablo_sel = dynamic_cast<Sel*>(expr))
    {
        retVal = "((" + ShowPabloE(pablo_sel->getIf_expr()) + "And " + ShowPabloE(pablo_sel->getT_expr()) +
                ")|(Not(" + ShowPabloE(pablo_sel->getIf_expr()) + ") And " + ShowPabloE(pablo_sel->getF_expr()) + ")";
    }
    else if (Not* pablo_not = dynamic_cast<Not*>(expr))
    {
        retVal = "Not (" + ShowPabloE(pablo_not->getExpr()) + ")";
    }
    else if (CharClass* cc = dynamic_cast<CharClass*>(expr))
    {
        retVal = "CharClass '" + cc->getCharClass() + "'";
    }
    else if (re::Name * name = dynamic_cast<re::Name *>(expr))
    {
        retVal = "Name '" + name->getName() + "'";
    }
    else if (Advance* adv = dynamic_cast<Advance*>(expr))
    {
        retVal = "Advance(" + ShowPabloE(adv->getExpr()) + ")";
    }
    else if (MatchStar* mstar = dynamic_cast<MatchStar*>(expr))
    {
        retVal = "MatchStar (" + ShowPabloE(mstar->getExpr1()) + ", " + ShowPabloE(mstar->getExpr2()) + ")";
    }
    else if (ScanThru* sthru = dynamic_cast<ScanThru*>(expr))
    {
        retVal = "ScanThru (" + ShowPabloE(sthru->getScanFrom()) + ", " + ShowPabloE(sthru->getScanThru()) + ")";
    }
    else retVal = "UNKNOWN_Pablo_EXPRESSION_TYPE!!!";

    return retVal;
}


