/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "printer_pablos.h"
#include <pablo/arithmetic.h>
#include <pablo/boolean.h>
#include <pablo/branch.h>
#include <pablo/codegenstate.h>
#include <pablo/pablo_kernel.h>
#include <pablo/pe_advance.h>
#include <pablo/pe_count.h>
#include <pablo/pe_infile.h>
#include <pablo/pe_integer.h>
#include <pablo/pe_lookahead.h>
#include <pablo/pe_matchstar.h>
#include <pablo/pe_ones.h>
#include <pablo/pe_phi.h>
#include <pablo/pe_scanthru.h>
#include <pablo/pe_string.h>
#include <pablo/pe_var.h>
#include <pablo/pe_zeroes.h>
#include <pablo/ps_assign.h>
#include <llvm/Support/raw_os_ostream.h>

using namespace pablo;
using namespace llvm;
using TypeId = PabloAST::ClassTypeId;

const unsigned BlockIndenting = 2;

void PabloPrinter::print(const PabloKernel * kernel, raw_ostream & out) {
    print(kernel->getEntryBlock(), out, true);
}

void PabloPrinter::print(const Statement * stmt, raw_ostream & out, const bool expandNested, const unsigned indent) {
    out.indent(indent);
    if (stmt == nullptr) {
        out << "<null-stmt>";
    } else if (const Assign * assign = dyn_cast<Assign>(stmt)) {
        print(assign->getVariable(), out);
        out << " = ";
        print(assign->getValue(), out);
    } else if (const Branch * br = dyn_cast<Branch>(stmt)) {
        if (isa<If>(br)) {
            out << "If ";
        } else if (isa<While>(br)) {
            out << "While ";
        }
        print(br->getCondition(), out);
        if (expandNested) {
            out << ":\n";
            print(br->getBody(), out, true, indent + BlockIndenting);
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
            print(adv->getExpression(), out);
            out << ", " << std::to_string(adv->getAmount()) << ")";
        } else if (const IndexedAdvance * adv = dyn_cast<IndexedAdvance>(stmt)) {
            out << " = pablo.IndexedAdvance(";
            print(adv->getExpression(), out);
            out << ", ";
            print(adv->getIndex(), out);
            out << ", " << std::to_string(adv->getAmount()) << ")";
        } else if (const Lookahead * adv = dyn_cast<Lookahead>(stmt)) {
            out << " = pablo.Lookahead(";
            print(adv->getExpression(), out);
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
        } else if (const ScanTo * sto = dyn_cast<ScanTo>(stmt)) {
            out << " = pablo.ScanTo(";
            print(sto->getScanFrom(), out);
            out << ", ";
            print(sto->getScanTo(), out);
            out << ")";
        } else if (const AdvanceThenScanThru * sthru = dyn_cast<AdvanceThenScanThru>(stmt)) {
            out << " = pablo.AdvanceThenScanThru(";
            print(sthru->getScanFrom(), out);
            out << ", ";
            print(sthru->getScanThru(), out);
            out << ")";
        } else if (const AdvanceThenScanTo * sto = dyn_cast<AdvanceThenScanTo>(stmt)) {
            out << " = pablo.AdvanceThenScanTo(";
            print(sto->getScanFrom(), out);
            out << ", ";
            print(sto->getScanTo(), out);
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
    } else if (const Phi * const phi = dyn_cast<Phi>(expr)) {
        out << "phi(";
        for (unsigned i = 0; i != phi->getNumIncomingValues(); ++i) {
            if (i) out << ", ";
            print(phi->getIncomingValue(i), out);
        }
        out << ")";
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
    } else if (const Add * op = dyn_cast<Add>(expr)) {
        print(op->getLH(), out);
        out << " + ";
        print(op->getRH(), out);
    } else if (const Subtract * op = dyn_cast<Subtract>(expr)) {
        print(op->getLH(), out);
        out << " - ";
        print(op->getRH(), out);
    } else if (const LessThan * op = dyn_cast<LessThan>(expr)) {
        print(op->getLH(), out);
        out << " < ";
        print(op->getRH(), out);
    } else if (const LessThanEquals * op = dyn_cast<LessThanEquals>(expr)) {
        print(op->getLH(), out);
        out << " <= ";
        print(op->getRH(), out);
    } else if (const Equals * op = dyn_cast<Equals>(expr)) {
        print(op->getLH(), out);
        out << " == ";
        print(op->getRH(), out);
    } else if (const GreaterThanEquals * op = dyn_cast<GreaterThanEquals>(expr)) {
        print(op->getLH(), out);
        out << " >= ";
        print(op->getRH(), out);
    } else if (const GreaterThan * op = dyn_cast<GreaterThan>(expr)) {
        print(op->getLH(), out);
        out << " > ";
        print(op->getRH(), out);
    } else if (const NotEquals * op = dyn_cast<NotEquals>(expr)) {
        print(op->getLH(), out);
        out << " != ";
        print(op->getRH(), out);
    } else if (const Statement * stmt = dyn_cast<Statement>(expr)) {
        out << stmt->getName();
    } else if (isa<Integer>(expr)) {
        out << cast<Integer>(expr)->value();
    } else {
        out << "???";
    }
}

void PabloPrinter::print(const PabloBlock * block, raw_ostream & strm, const bool expandNested, const unsigned indent) {
    for (const Statement * stmt : *block) {
        print(stmt, strm, expandNested, indent);
        if (LLVM_LIKELY(!isa<Branch>(stmt) || !expandNested)) {
            strm << "\n";
        }
    }
}


