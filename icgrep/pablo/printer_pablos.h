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
    static void print(const pablo::PabloBlock & block, std::ostream & strm);
    static void print(const pablo::StatementList & stmts, std::ostream & strm);
    static void print(const pablo::StatementList & stmts, std::string indent, std::ostream & strm);
    static void print(const pablo::PabloAST * expr, std::ostream & strm);
    static void print(const pablo::Statement *stmt, std::string indent, std::ostream & strm);
};

#endif // SHOW_H
