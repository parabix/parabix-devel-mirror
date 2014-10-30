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

class PabloPrinter {
public:
    static std::string print(const pablo::PabloBlock & block);
    static std::string print(const pablo::StatementList & stmts);
    static std::string print(const pablo::PabloAST * expr);
    static std::string print(const pablo::Statement *stmt);
};

#endif // SHOW_H
