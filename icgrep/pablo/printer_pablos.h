/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef SHOW_H
#define SHOW_H

#include <pablo/pabloAST.h>
#include <string>

namespace pablo {
    class PabloBlock;
}

class StatementPrinter {
public:
    static std::string PrintStmts(const pablo::PabloBlock & cg_state);
    static std::string Print_CC_PabloStmts(const pablo::StatementList & stmts);
    static std::string Print_PB_PabloStmts(const pablo::StatementList & stmts);
    static std::string ShowPabloAST(const pablo::PabloAST * expr);
    static std::string ShowPabloS(const pablo::PabloAST *stmt);
};

#endif // SHOW_H
