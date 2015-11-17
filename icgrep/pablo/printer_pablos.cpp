/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "printer_pablos.h"
#include <pablo/codegenstate.h>
#include <llvm/Support/raw_os_ostream.h>
#include <iostream>

using namespace pablo;

const unsigned BlockIndenting = 2;

void PabloPrinter::print(const pablo::PabloFunction & function, llvm::raw_ostream & out) {
    print(function.getEntryBlock(), out, true);
}

inline void print_vars(const pablo::If::DefinedVars & vars, llvm::raw_ostream & out, const unsigned indent) {
    for (const Assign * def : vars) {
        out.indent(indent);
        out << def->getName() << " = 0\n";
    }
}

void PabloPrinter::print(const Statement * stmt, llvm::raw_ostream & out, const bool expandNested, const unsigned indent) {
    out.indent(indent);
    if (stmt == nullptr) {
        out << "<null-stmt>";
    } else if (const Assign * an = dyn_cast<const Assign>(stmt)) {
        out << an->getName() << " = ";
        print(an->getExpression(), out);
    } else if (const Next * next = dyn_cast<const Next>(stmt)) {
        out << "Next(" << next->getName() << ") = ";
        print(next->getExpr(), out);
    } else if (const If * ifstmt = dyn_cast<const If>(stmt)) {
        out << "if ";
        print(ifstmt->getCondition(), out);
        if (expandNested) {
            out << ":\n";
            print(ifstmt->getBody(), out, true, indent + BlockIndenting);
            out.indent(indent);
            out << "else:\n";
            print_vars(ifstmt->getDefined(), out, indent + BlockIndenting);
        }
    } else if (const While * whileNode = dyn_cast<const While>(stmt)) {
        out << "while ";
        print(whileNode->getCondition(), out);
        if (expandNested) {
            out << ":\n";
            print(whileNode->getBody(), out, true, indent + BlockIndenting);
        }
    } else if (const Call * call = dyn_cast<const Call>(stmt)) {
        out << " = " << call->getCallee() << "()";
    } else if (const And * pablo_and = dyn_cast<const And>(stmt)) {
        out << pablo_and->getName() << " = (";
        for (unsigned i = 0; i != pablo_and->getNumOperands(); ++i) {
            if (i) out << " & ";
            print(pablo_and->getOperand(i), out);
        }
        out << ")";
    } else if (const Or * pablo_or = dyn_cast<const Or>(stmt)) {
        out << pablo_or->getName() << " = (";
        for (unsigned i = 0; i != pablo_or->getNumOperands(); ++i) {
            if (i) out << " | ";
            print(pablo_or->getOperand(i), out);
        }
        out << ")";
    } else if (const Xor * pablo_xor = dyn_cast<const Xor>(stmt)) {
        out << pablo_xor->getName() << " = (";
        for (unsigned i = 0; i != pablo_xor->getNumOperands(); ++i) {
            if (i) out << " ^ ";
            print(pablo_xor->getOperand(i), out);
        }
        out << ")";
    } else if (const Sel * pablo_sel = dyn_cast<const Sel>(stmt)) {
        out << pablo_sel->getName() << " = (";
        print(pablo_sel->getCondition(), out);
        out << " ? ";
        print(pablo_sel->getTrueExpr(), out);
        out << " : ";
        print(pablo_sel->getFalseExpr(), out);
        out << ")";
    } else if (const Not * pablo_not = dyn_cast<const Not>(stmt)) {
        out << pablo_not->getName() << " = (~";
        print(pablo_not->getExpr(), out);
        out << ")";
    } else if (const Advance * adv = dyn_cast<const Advance>(stmt)) {
        out << adv->getName() << " = pablo.Advance(";
        print(adv->getExpr(), out);
        out << ", " << std::to_string(adv->getAdvanceAmount()) << ")";
    } else if (const MatchStar * mstar = dyn_cast<const MatchStar>(stmt)) {
        out << mstar->getName() << " = pablo.MatchStar(";
        print(mstar->getMarker(), out);
        out << ", ";
        print(mstar->getCharClass(), out);
        out << ")";
    } else if (const ScanThru * sthru = dyn_cast<const ScanThru>(stmt)) {
        out << sthru->getName() << " = pablo.ScanThru(";
        print(sthru->getScanFrom(), out);
        out << ", ";
        print(sthru->getScanThru(), out);
        out << ")";
    } else if (const Mod64Advance * adv = dyn_cast<const Mod64Advance>(stmt)) {
        out << adv->getName() << " = pablo.Mod64Advance(";
        print(adv->getExpr(), out);
        out << ", " << std::to_string(adv->getAdvanceAmount()) << ")";
    } else if (const Mod64MatchStar * mstar = dyn_cast<const Mod64MatchStar>(stmt)) {
        out << mstar->getName() << " = pablo.Mod64MatchStar(";
        print(mstar->getMarker(), out);
        out << ", ";
        print(mstar->getCharClass(), out);
        out << ")";
    } else if (const Mod64ScanThru * sthru = dyn_cast<const Mod64ScanThru>(stmt)) {
        out << sthru->getName() << " = pablo.Mod64ScanThru(";
        print(sthru->getScanFrom(), out);
        out << ", ";
        print(sthru->getScanThru(), out);
        out << ")";
    } else if (const Count * count = dyn_cast<const Count>(stmt)) {
        out << count->getName() << " = pablo.Count(";
        print(count->getExpr(), out);
        out << ")";
    } else {
        out << "???";
    }
}

void PabloPrinter::print(const PabloAST * expr, llvm::raw_ostream & out) {
    if (expr == nullptr) {
        out << "<null-expr>";
    } else if (isa<const Zeroes>(expr)) {
        out << "0";
    } else if (isa<const Ones>(expr)) {
        out << "1";
    } else if (const Var * var = dyn_cast<const Var>(expr)) {
        out  << var->getName();
    } else if (const Next * next = dyn_cast<const Next>(expr)) {
        out << "Next(" << next->getName() << ")";
    } else if (const If * ifstmt = dyn_cast<If>(expr)) {
        out << "If ";
        print(ifstmt->getCondition(), out);
    } else if (const While * whl = dyn_cast<While>(expr)) {
        out << "While ";
        print(whl->getCondition(), out);
    } else if (const Statement * stmt = dyn_cast<Statement>(expr)) {
        out << stmt->getName();
    } else if (isa<Integer>(expr)) {
        out << cast<Integer>(expr)->value();
    } else {
        out << "???";
    }
}

void PabloPrinter::print(const PabloBlock * block, llvm::raw_ostream & strm, const bool expandNested, const unsigned indent) {
    for (const Statement * stmt : *block) {
        print(stmt, strm, expandNested, indent);
        strm << "\n";
    }
}


