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

using TypeId = PabloAST::ClassTypeId;

const unsigned BlockIndenting = 2;

void PabloPrinter::print(const PabloFunction & function, llvm::raw_ostream & out) {
    print(function.getEntryBlock(), out, true);
}

void PabloPrinter::print(const Statement * stmt, llvm::raw_ostream & out, const bool expandNested, const unsigned indent) {
    out.indent(indent);
    if (stmt == nullptr) {
        out << "<null-stmt>";
    } else if (const Assign * assign = dyn_cast<Assign>(stmt)) {
        print(assign->getVariable(), out);
        out << " = ";
        print(assign->getValue(), out);
    } else if (const If * ifNode = dyn_cast<If>(stmt)) {
        out << "If ";
        print(ifNode->getCondition(), out);
        if (expandNested) {
            out << ":\n";
            print(ifNode->getBody(), out, true, indent + BlockIndenting);
        }
    } else if (const While * whileNode = dyn_cast<While>(stmt)) {
        out << "While ";
        print(whileNode->getCondition(), out);
        if (expandNested) {
            out << ":\n";
            print(whileNode->getBody(), out, true, indent + BlockIndenting);
        }
    } else {
        print(cast<PabloAST>(stmt), out);

        if (const Extract * extract = dyn_cast<Extract>(stmt)) {
            out << " = Extract ";
            print(extract->getArray(), out);
            out << ", ";
            print(extract->getIndex(), out);
        } else if (const And * andNode = dyn_cast<And>(stmt)) {
            out << " = (";
            for (unsigned i = 0; i != andNode->getNumOperands(); ++i) {
                if (i) out << " & ";
                print(andNode->getOperand(i), out);
            }
            out << ")";
        } else if (const Or * orNode = dyn_cast<Or>(stmt)) {
            out << " = (";
            for (unsigned i = 0; i != orNode->getNumOperands(); ++i) {
                if (i) out << " | ";
                print(orNode->getOperand(i), out);
            }
            out << ")";
        } else if (const Xor * xorNode = dyn_cast<Xor>(stmt)) {
            out << " = (";
            for (unsigned i = 0; i != xorNode->getNumOperands(); ++i) {
                if (i) out << " ^ ";
                print(xorNode->getOperand(i), out);
            }
            out << ")";
        } else if (const Sel * selNode = dyn_cast<Sel>(stmt)) {
            out << " = (";
            print(selNode->getCondition(), out);
            out << " ? ";
            print(selNode->getTrueExpr(), out);
            out << " : ";
            print(selNode->getFalseExpr(), out);
            out << ")";
        } else if (const Not * notNode = dyn_cast<Not>(stmt)) {
            out << " = (~";
            print(notNode->getExpr(), out);
            out << ")";
        } else if (const Advance * adv = dyn_cast<Advance>(stmt)) {
            out << " = pablo.Advance(";
            print(adv->getExpr(), out);
            out << ", " << std::to_string(adv->getAmount()) << ")";
        } else if (const Lookahead * adv = dyn_cast<Lookahead>(stmt)) {
            out << " = pablo.Lookahead(";
            print(adv->getExpr(), out);
            out << ", " << std::to_string(adv->getAmount()) << ")";
        } else if (const MatchStar * mstar = dyn_cast<MatchStar>(stmt)) {
            out << " = pablo.MatchStar(";
            print(mstar->getMarker(), out);
            out << ", ";
            print(mstar->getCharClass(), out);
            out << ")";
        } else if (const ScanThru * sthru = dyn_cast<ScanThru>(stmt)) {
            out << " = pablo.ScanThru(";
            print(sthru->getScanFrom(), out);
            out << ", ";
            print(sthru->getScanThru(), out);
            out << ")";
        } else if (const Count * count = dyn_cast<Count>(stmt)) {
            out << " = pablo.Count(";
            print(count->getExpr(), out);
            out << ")";
        } else if (const InFile * e = dyn_cast<InFile>(stmt)) {
            out << " = pablo.InFile(";
            print(e->getExpr(), out);
            out << ")";
        } else if (const AtEOF * e = dyn_cast<AtEOF>(stmt)) {
            out << " = pablo.AtEOF(";
            print(e->getExpr(), out);
            out << ")";
        } else {
            out << "???";
        }
    }
}

void PabloPrinter::print(const PabloAST * expr, llvm::raw_ostream & out) {
    if (expr == nullptr) {
        out << "<null-expr>";
    } else if (isa<Zeroes>(expr)) {
        out << "0";
    } else if (isa<Ones>(expr)) {
        out << "1";
    } else if (const Var * var = dyn_cast<Var>(expr)) {
        out << var->getName();
    } else if (const If * ifstmt = dyn_cast<If>(expr)) {
        out << "If ";
        print(ifstmt->getCondition(), out);
    } else if (const While * whl = dyn_cast<While>(expr)) {
        out << "While ";
        print(whl->getCondition(), out);
    } else if (const Assign * assign = dyn_cast<Assign>(expr)) {
        print(assign->getVariable(), out);
        out << " = ";
        print(assign->getValue(), out);
    } else if (const Statement * stmt = dyn_cast<Statement>(expr)) {
        out << stmt->getName();
    } else if (isa<Integer>(expr)) {
        out << cast<Integer>(expr)->value();
    } else if (isa<Prototype>(expr)) {
        out << cast<Prototype>(expr)->getName();
    } else if (isa<PabloFunction>(expr)) {
        out << cast<PabloFunction>(expr)->getName();
    } else {
        out << "???";
    }
}

void PabloPrinter::print(const PabloBlock * block, llvm::raw_ostream & strm, const bool expandNested, const unsigned indent) {
    for (const Statement * stmt : *block) {
        print(stmt, strm, expandNested, indent);
        if (LLVM_LIKELY(!isa<Branch>(stmt) || !expandNested)) {
            strm << "\n";
        }
    }
}


