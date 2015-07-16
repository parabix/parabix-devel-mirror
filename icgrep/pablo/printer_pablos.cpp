/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "printer_pablos.h"
#include <iostream>
#include <ostream>
#include <llvm/Support/raw_os_ostream.h>

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

void PabloPrinter::print(const PabloBlock & block, llvm::raw_ostream & strm)
{
    print(block.statements(), "  ", strm);
}

void PabloPrinter::print(const StatementList & stmts, llvm::raw_ostream & strm) {
    print(stmts, "  ", strm);
}

void PabloPrinter::print(const StatementList & stmts, std::string indent, llvm::raw_ostream & strm) {
    for (const Statement * stmt : stmts) {
        print(stmt, indent, strm);
        strm << "\n";
    }
}

void PabloPrinter::print_vars(const DefinedVars & vars, std::string indent, llvm::raw_ostream & strm) {
    for (const PabloAST * v : vars) {
        strm << indent << dyn_cast<Assign>(v)->getName() << " = 0" << "\n";
    }
}

void PabloPrinter::print(const Statement * stmt, std::string indent, llvm::raw_ostream & strm) {
    strm << indent;
    if (stmt == nullptr) {
        strm << "<null-stmt>";
    }
    else if (const Assign * an = dyn_cast<const Assign>(stmt)) {
//        if (an->isOutputAssignment()) {
//            strm << "output.";
//        }
        strm << an->getName() << " = ";
        print(an->getExpression(), strm);
    }
    else if (const Next * next = dyn_cast<const Next>(stmt)) {        
        strm << next->getName() << "' = ";
        print(next->getExpr(), strm);
    }
    else if (const If * ifstmt = dyn_cast<const If>(stmt)) {
        strm << "if ";
        print(ifstmt->getCondition(), strm);
        strm << ":" << "\n";
        print(ifstmt->getBody(), indent + "  ", strm);
        strm << indent << "else:" << "\n";
        print_vars(ifstmt->getDefined(), indent + "  ", strm);
    }
    else if (const While * whl = dyn_cast<const While>(stmt)) {
        strm << "while ";
        print(whl->getCondition(), strm);
        strm << ":" << "\n";
        print(whl->getBody(), indent + "  ", strm);
    }
    else if (const Call * call = dyn_cast<const Call>(stmt)) {
        strm << " = " << call->getCallee() << "()";
    }
    else if (const And * pablo_and = dyn_cast<const And>(stmt)) {
        print(pablo_and, strm);
        strm << " = (";
        print(pablo_and->getExpr1(), strm);
        strm << " & ";
        print(pablo_and->getExpr2(), strm);
        strm << ")";
    }
    else if (const Or * pablo_or = dyn_cast<const Or>(stmt)) {
        print(pablo_or, strm);
        strm << " = (";
        print(pablo_or->getExpr1(), strm);
        strm << " | ";
        print(pablo_or->getExpr2(), strm);
        strm << ")";
    }
    else if (const Xor * pablo_xor = dyn_cast<const Xor>(stmt)) {
        print(pablo_xor, strm);
        strm << " = (";
        print(pablo_xor->getExpr1(), strm);
        strm << " ^ ";
        print(pablo_xor->getExpr2(), strm);
        strm << ")";
    }
    else if (const Sel * pablo_sel = dyn_cast<const Sel>(stmt)) {
        print(pablo_sel, strm);
        strm << " = (";
        print(pablo_sel->getCondition(), strm);
        strm << " ? ";
        print(pablo_sel->getTrueExpr(), strm);
        strm << " : ";
        print(pablo_sel->getFalseExpr(), strm);
        strm << ")";
    }
    else if (const Not * pablo_not = dyn_cast<const Not>(stmt)) {
        print(pablo_not, strm);
        strm << " = (~";
        print(pablo_not->getExpr(), strm);
        strm << ")";
    }
    else if (const Advance * adv = dyn_cast<const Advance>(stmt)) {
        print(adv, strm);
        strm << " = pablo.Advance(";
        print(adv->getExpr(), strm);
        strm << ", " << std::to_string(adv->getAdvanceAmount()) << ")";
    }
    else if (const MatchStar * mstar = dyn_cast<const MatchStar>(stmt)) {
        print(mstar, strm);
        strm << " = pablo.MatchStar(";
        print(mstar->getMarker(), strm);
        strm << ", ";
        print(mstar->getCharClass(), strm);
        strm << ")";
    }
    else if (const ScanThru * sthru = dyn_cast<const ScanThru>(stmt)) {
        print(sthru, strm);
        strm << " = pablo.ScanThru(";
        print(sthru->getScanFrom(), strm);
        strm << ", ";
        print(sthru->getScanThru(), strm);
        strm << ")";
    }
    else {
        strm << indent << "**UNKNOWN Pablo Statement type **" << "\n";
    }
}

void PabloPrinter::print(const PabloAST * expr, llvm::raw_ostream & strm) {
    if (expr == nullptr) {
        strm << "<null-expr>";
    }
    else if (isa<const Zeroes>(expr)) {
        strm << "0";
    }
    else if (isa<const Ones>(expr)) {
        strm << "1";
    }
    else if (const Var * var = dyn_cast<const Var>(expr)) {
        strm  << var->getName();
    }
    else if (const Next * next = dyn_cast<const Next>(expr)) {
        strm << "Next(" << next->getName() << ")";
    }
    else if (const Statement * stmt = dyn_cast<Statement>(expr)) {
        strm << stmt->getName();
    }
    else {
        strm << "**UNKNOWN Pablo Expression type **\n" << "\n";
    }
}


