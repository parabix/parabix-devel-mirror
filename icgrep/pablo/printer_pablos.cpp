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
    } else if (const If * ifNode = dyn_cast<const If>(stmt)) {
        out << "If ";
        print(ifNode->getCondition(), out);
        if (expandNested) {
            out << ":\n";
            print(ifNode->getBody(), out, true, indent + BlockIndenting);
            if (ifNode->getDefined().size() > 0) {
                out.indent(indent);
                out << "Else:\n";
                print_vars(ifNode->getDefined(), out, indent + BlockIndenting);
            }
        }
    } else if (const While * whileNode = dyn_cast<const While>(stmt)) {
        out << "While ";
        print(whileNode->getCondition(), out);
        if (expandNested) {
            out << ":\n";
            print(whileNode->getBody(), out, true, indent + BlockIndenting);
        }
    } else if (const Call * call = dyn_cast<const Call>(stmt)) {
        if (call->getPrototype()->getNumOfResults() > 0) {
            out << " = ";
        }
        out << call->getCallee() << "(";
        for (unsigned i = 0; i != call->getNumOperands(); ++i) {
            print(call->getOperand(i), out);
        }
        out << ")";
    } else if (const And * andNode = dyn_cast<const And>(stmt)) {
        out << andNode->getName() << " = (";
        for (unsigned i = 0; i != andNode->getNumOperands(); ++i) {
            if (i) out << " & ";
            print(andNode->getOperand(i), out);
        }
        out << ")";
    } else if (const Or * orNode = dyn_cast<const Or>(stmt)) {
        out << orNode->getName() << " = (";
        for (unsigned i = 0; i != orNode->getNumOperands(); ++i) {
            if (i) out << " | ";
            print(orNode->getOperand(i), out);
        }
        out << ")";
    } else if (const Xor * xorNode = dyn_cast<const Xor>(stmt)) {
        out << xorNode->getName() << " = (";
        for (unsigned i = 0; i != xorNode->getNumOperands(); ++i) {
            if (i) out << " ^ ";
            print(xorNode->getOperand(i), out);
        }
        out << ")";
    } else if (const Sel * selNode = dyn_cast<const Sel>(stmt)) {
        out << selNode->getName() << " = (";
        print(selNode->getCondition(), out);
        out << " ? ";
        print(selNode->getTrueExpr(), out);
        out << " : ";
        print(selNode->getFalseExpr(), out);
        out << ")";
    } else if (const Not * notNode = dyn_cast<const Not>(stmt)) {
        out << notNode->getName() << " = (~";
        print(notNode->getExpr(), out);
        out << ")";
    } else if (const Advance * adv = dyn_cast<const Advance>(stmt)) {
        out << adv->getName() << " = pablo.Advance(";
        print(adv->getExpr(), out);
        out << ", " << std::to_string(adv->getAmount()) << ")";
    } else if (const Lookahead * adv = dyn_cast<const Lookahead>(stmt)) {
        out << adv->getName() << " = pablo.Lookahead(";
        print(adv->getExpr(), out);
        out << ", " << std::to_string(adv->getAmount()) << ")";
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


