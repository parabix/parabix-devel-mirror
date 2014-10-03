/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef SHOW_H
#define SHOW_H

#include <string>
#include <list>

namespace pablo {
    class PabloE;
    class CodeGenState;
}

class StatementPrinter {
    typedef std::list<pablo::PabloE *> List;
public:
    static std::string PrintStmts(const pablo::CodeGenState & cg_state);
    static std::string Print_CC_PabloStmts(const List & stmts);
    static std::string Print_PB_PabloStmts(const List & stmts, std::string strOut);
    static std::string ShowPabloE(const pablo::PabloE * expr);
    static std::string ShowPabloS(const pablo::PabloE *stmt);
};

#endif // SHOW_H
