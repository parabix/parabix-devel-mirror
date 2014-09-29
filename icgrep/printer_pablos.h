/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef SHOW_H
#define SHOW_H

#include "ps_pablos.h"
#include <string>
#include <list>

class StatementPrinter
{
public:
    static std::string PrintStmts(CodeGenState cg_state);
    static std::string Print_CC_PabloStmts(std::list<PabloS*> stmts);
    static std::string Print_PB_PabloStmts(std::list<PabloS*> stmts, std::string strOut);
    static std::string ShowPabloE(PabloE* expr);
    static std::string ShowPabloS(PabloS* stmt);
};

#endif // SHOW_H
