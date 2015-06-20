/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef SHOW_H
#define SHOW_H

#include <pablo/pabloAST.h>
#include <pablo/ps_if.h>
#include <string>
#include <llvm/Support/raw_os_ostream.h>

namespace pablo {
    class PabloBlock;
}

class PabloPrinter {
public:
    using DefinedVars = pablo::If::DefinedVars;
    static void print(const pablo::PabloBlock & block, llvm::raw_ostream & strm);
    static void print(const pablo::StatementList & stmts, llvm::raw_ostream & strm);
    static void print(const pablo::StatementList & stmts, std::string indent, llvm::raw_ostream & strm);
    static void print_vars(const DefinedVars & vars, std::string indent, llvm::raw_ostream & strm);
    static void print(const pablo::PabloAST * expr, llvm::raw_ostream & strm);
    static void print(const pablo::Statement *stmt, std::string indent, llvm::raw_ostream & strm);
};

#endif // SHOW_H
