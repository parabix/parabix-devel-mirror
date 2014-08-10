/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "pablos_printer.h"

StatementPrinter::StatementPrinter(){}

string StatementPrinter::PrintStmts(CodeGenState cg_state)
{
    string strOut = "[";

    //Print out the symbol environment.    
    //for (std::map<std::string, int>::iterator iter = cg_state.env->getEnv()->begin(); iter != cg_state.env->getEnv()->end(); ++iter)
    //{
    //    strOut += "('" + iter->first + "'," + std::to_string(iter->second) + "),";
    //}
    strOut = strOut.substr(0, strOut.length() - 1);
    strOut += "],[";

    strOut = Print_PB_PabloStmts(cg_state.stmtsl, strOut);

    strOut = strOut.substr(0, strOut.length() - 1);
    strOut += "],";

    //Print the name of the variable that holds the match result for the overall expression so far.
    strOut += "'" + cg_state.newsym + "'";

    return strOut;
}

string StatementPrinter::Print_PB_PabloStmts(std::list<PabloS*> stmts, std::string strOut)
{
    list<PabloS*>::iterator it;
    //std::cout << "Total Statements: " + std::to_string(stmts.size()) << std::endl;
    for (it = stmts.begin(); it != stmts.end(); ++it)
    {
        strOut += ShowPabloS(*it);
    }

    return strOut;
}

string StatementPrinter::Print_CC_PabloStmts(std::list<PabloS*> stmts)
{
    std::string strOut;

    list<PabloS*>::iterator it;
    std::cout << "Total Statements: " + std::to_string(stmts.size()) << std::endl;
    for (it = stmts.begin(); it != stmts.end(); ++it)
    {
        strOut += ShowPabloS(*it) + "\n";
    }

    return strOut;
}

string StatementPrinter::ShowPabloS(PabloS* stmt)
{
    string retVal = "";

    if (Assign* an = dynamic_cast<Assign*>(stmt))
    {
        retVal = "Assign('" + an->getM() + "', " + ShowPabloE(an->getExpr()) + "),";
    }
    if (While* whl = dynamic_cast<While*>(stmt))
    {
        retVal = "While(" + ShowPabloE(whl->getExpr()) + ", " + Print_PB_PabloStmts(whl->getPSList(), retVal) + ")";
    }

    return retVal;
}

string StatementPrinter::ShowPabloE(PabloE* expr)
{
    string retVal = "";

    if (All* all = dynamic_cast<All*>(expr))
    {
        retVal = "All " + std::to_string(all->getNum()) + " ";
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
        retVal = "((" + ShowPabloE(pablo_sel->getIf_expr()) + "&" + ShowPabloE(pablo_sel->getT_expr()) +
                ")|(~(" + ShowPabloE(pablo_sel->getIf_expr()) + ")&" + ShowPabloE(pablo_sel->getF_expr()) + ")";
    }
    else if (Not* pablo_not = dynamic_cast<Not*>(expr))
    {
        retVal = "Not (" + ShowPabloE(pablo_not->getExpr()) + ")";
    }
    else if (CharClass* cc = dynamic_cast<CharClass*>(expr))
    {
        retVal = "CharClass '" + cc->getCharClass() + "'";
    }
    else if (Advance* adv = dynamic_cast<Advance*>(expr))
    {
        retVal = "Advance(" + ShowPabloE(adv->getExpr()) + ")";
    }
    else if (MatchStar* mstar = dynamic_cast<MatchStar*>(expr))
    {
        retVal = "MatchStar (" + ShowPabloE(mstar->getExpr1()) + ", " + ShowPabloE(mstar->getExpr2()) + ")";
    }

    return retVal;
}


