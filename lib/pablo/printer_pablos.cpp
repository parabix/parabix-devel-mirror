/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/printer_pablos.h>

#include <pablo/arithmetic.h>
#include <pablo/boolean.h>
#include <pablo/branch.h>
#include <pablo/codegenstate.h>
#include <pablo/pablo_intrinsic.h>
#include <pablo/pablo_kernel.h>
#include <pablo/pe_advance.h>
#include <pablo/pe_count.h>
#include <pablo/pe_infile.h>
#include <pablo/pe_integer.h>
#include <pablo/pe_lookahead.h>
#include <pablo/pe_matchstar.h>
#include <pablo/pe_ones.h>
#include <pablo/pe_pack.h>
#include <pablo/pe_repeat.h>
#include <pablo/pe_scanthru.h>
#include <pablo/pe_string.h>
#include <pablo/pe_var.h>
#include <pablo/pe_zeroes.h>
#include <pablo/ps_assign.h>
#include <pablo/ps_terminate.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/DerivedTypes.h>  // for get getSequentialElementType
#include <llvm/Support/raw_os_ostream.h>

#define INDENT_WIDTH 4

using namespace llvm;

namespace pablo {

static void PrintStatement(Statement const * stmt, raw_ostream & out, const bool expandNested, unsigned indent) noexcept;
static void PrintExpression(PabloAST const * expr, raw_ostream & out) noexcept;


static void PrintStatement(Statement const * stmt, raw_ostream & out, const bool expandNested, unsigned indent) noexcept {
    out.indent(indent);
    if (stmt == nullptr) {
        out << "<null-stmt>";
    } else if (const Assign * assign = dyn_cast<Assign>(stmt)) {
        PrintExpression(assign->getVariable(), out);
        out << " = ";
        PrintExpression(assign->getValue(), out);
        out << "\n";
    } else if (const Branch * br = dyn_cast<Branch>(stmt)) {
        if (isa<If>(br)) {
            out << "if ";
        } else if (isa<While>(br)) {
            out << "while ";
        }
        PrintExpression(br->getCondition(), out);
        if (expandNested) {
            out << " {\n";
            PabloPrinter::print(br->getBody(), out, expandNested, indent + INDENT_WIDTH);
            out.indent(indent) << "}\n";
        } else {
            out << " {...}\n";
        }
    } else {

        PrintExpression(cast<PabloAST>(stmt), out);
        out << " = ";

        if (const And * andNode = dyn_cast<And>(stmt)) {
            for (unsigned i = 0; i != andNode->getNumOperands(); ++i) {
                if (i) out << " & ";
                PrintExpression(andNode->getOperand(i), out);
            }
        } else if (const Or * orNode = dyn_cast<Or>(stmt)) {
            for (unsigned i = 0; i != orNode->getNumOperands(); ++i) {
                if (i) out << " | ";
                PrintExpression(orNode->getOperand(i), out);
            }
        } else if (const Xor * xorNode = dyn_cast<Xor>(stmt)) {
            for (unsigned i = 0; i != xorNode->getNumOperands(); ++i) {
                if (i) out << " ^ ";
                PrintExpression(xorNode->getOperand(i), out);
            }
        } else if (const Sel * selNode = dyn_cast<Sel>(stmt)) {
            out << "Sel(";
            PrintExpression(selNode->getCondition(), out);
            out << ", ";
            PrintExpression(selNode->getTrueExpr(), out);
            out << ", ";
            PrintExpression(selNode->getFalseExpr(), out);
            out << ")";
        } else if (const Not * notNode = dyn_cast<Not>(stmt)) {
            out << "~";
            PrintExpression(notNode->getExpr(), out);
        } else if (IntrinsicCall const * call = dyn_cast<IntrinsicCall>(stmt)) {
            out << call->getIntrinsicName() << "(";
            for (size_t i = 0; i < call->getNumOperands(); ++i) {
                if (i) out << ", ";
                PrintExpression(call->getOperand(i), out);
            }
            out << ")";
        } else if (const Advance * adv = dyn_cast<Advance>(stmt)) {
            out << "Advance(";
            PrintExpression(adv->getExpression(), out);
            out << ", " << std::to_string(adv->getAmount()) << ")";
        } else if (const IndexedAdvance * adv = dyn_cast<IndexedAdvance>(stmt)) {
            out << "IndexedAdvance(";
            PrintExpression(adv->getExpression(), out);
            out << ", ";
            PrintExpression(adv->getIndex(), out);
            out << ", " << std::to_string(adv->getAmount()) << ")";
        } else if (const Lookahead * adv = dyn_cast<Lookahead>(stmt)) {
            out << "Lookahead(";
            PrintExpression(adv->getExpression(), out);
            out << ", " << std::to_string(adv->getAmount()) << ")";
        } else if (const MatchStar * mstar = dyn_cast<MatchStar>(stmt)) {
            out << "MatchStar(";
            PrintExpression(mstar->getMarker(), out);
            out << ", ";
            PrintExpression(mstar->getCharClass(), out);
            out << ")";
        } else if (const ScanThru * sthru = dyn_cast<ScanThru>(stmt)) {
            out << "ScanThru(";
            PrintExpression(sthru->getScanFrom(), out);
            out << ", ";
            PrintExpression(sthru->getScanThru(), out);
            out << ")";
        } else if (const ScanTo * sto = dyn_cast<ScanTo>(stmt)) {
            out << "ScanTo(";
            PrintExpression(sto->getScanFrom(), out);
            out << ", ";
            PrintExpression(sto->getScanTo(), out);
            out << ")";
        } else if (const AdvanceThenScanThru * sthru = dyn_cast<AdvanceThenScanThru>(stmt)) {
            out << "AdvanceThenScanThru(";
            PrintExpression(sthru->getScanFrom(), out);
            out << ", ";
            PrintExpression(sthru->getScanThru(), out);
            out << ")";
        } else if (const AdvanceThenScanTo * sto = dyn_cast<AdvanceThenScanTo>(stmt)) {
            out << "AdvanceThenScanTo(";
            PrintExpression(sto->getScanFrom(), out);
            out << ", ";
            PrintExpression(sto->getScanTo(), out);
            out << ")";
        } else if (const Ternary * tern = dyn_cast<Ternary>(stmt)) {
            out << "Ternary(";
            out.write_hex(tern->getMask()->value());
            out << ", ";
            PrintExpression(tern->getA(), out);
            out << ", ";
            PrintExpression(tern->getB(), out);
            out << ", ";
            PrintExpression(tern->getC(), out);
            out << ")";
        } else if (const Count * count = dyn_cast<Count>(stmt)) {
            out << "Count(";
            PrintExpression(count->getExpr(), out);
            out << ")";
        } else if (const Repeat * splat = dyn_cast<Repeat>(stmt)) {
            out << "Repeat(";
            PrintExpression(splat->getFieldWidth(), out);
            out << ", ";
            auto fw = splat->getValue()->getType()->getIntegerBitWidth();
            out << "Int" << fw << "(";
            PrintExpression(splat->getValue(), out);
            out << "))";
        } else if (const PackH * p = dyn_cast<PackH>(stmt)) {
            out << " = PackH(";
            PrintExpression(p->getFieldWidth(), out);
            out << ", ";
            PrintExpression(p->getValue(), out);
            out << ")";
        } else if (const PackL * p = dyn_cast<PackL>(stmt)) {
            out << " = PackL(";
            PrintExpression(p->getFieldWidth(), out);
            out << ", ";
            PrintExpression(p->getValue(), out);
            out << ")";
        } else if (const InFile * e = dyn_cast<InFile>(stmt)) {
            out << "InFile(";
            PrintExpression(e->getExpr(), out);
            out << ")";
        } else if (const AtEOF * e = dyn_cast<AtEOF>(stmt)) {
            out << "AtEOF(";
            PrintExpression(e->getExpr(), out);
            out << ")";
        } else if (const TerminateAt * s = dyn_cast<TerminateAt>(stmt)) {
            out << "TerminateAt(";
            PrintExpression(s->getExpr(), out);
            out << ", " << std::to_string(s->getSignalCode()) << ")";
        } else {
            out << "???";
        }

        out << "\n";
    }
}

static void PrintExpression(PabloAST const * expr, raw_ostream & out) noexcept {
    assert (expr);
    if (isa<Zeroes>(expr)) {
        out << "<0>";
    } else if (const If * ifStmt = dyn_cast<If>(expr)) {
        out << "if ";
        PrintExpression(ifStmt->getCondition(), out);
        out << " {...}";
    } else if (const While * whileStmt = dyn_cast<While>(expr)) {
        out << "while ";
        PrintExpression(whileStmt->getCondition(), out);
        out << " {...}";
    } else if (isa<Ones>(expr)) {
        out << "<1>";
    } else if (const Integer * i = dyn_cast<Integer>(expr)) {
        out << i->value();
    } else if (Extract const * extract = dyn_cast<Extract>(expr)) {
        PrintExpression(extract->getArray(), out);
        out << "[";
        PrintExpression(extract->getIndex(), out);
        out << "]";
    } else if (Var const * var = dyn_cast<Var>(expr)) {
        out << var->getName();
    } else if (isa<Integer>(expr)) {
        out << cast<Integer>(expr)->value();
    } else if (const Add * op = dyn_cast<Add>(expr)) {
        PrintExpression(op->getLH(), out);
        out << " + ";
        PrintExpression(op->getRH(), out);
    } else if (const Subtract * op = dyn_cast<Subtract>(expr)) {
        PrintExpression(op->getLH(), out);
        out << " - ";
        PrintExpression(op->getRH(), out);
    } else if (const LessThan * op = dyn_cast<LessThan>(expr)) {
        PrintExpression(op->getLH(), out);
        out << " < ";
        PrintExpression(op->getRH(), out);
    } else if (const LessThanEquals * op = dyn_cast<LessThanEquals>(expr)) {
        PrintExpression(op->getLH(), out);
        out << " <= ";
        PrintExpression(op->getRH(), out);
    } else if (const Equals * op = dyn_cast<Equals>(expr)) {
        PrintExpression(op->getLH(), out);
        out << " == ";
        PrintExpression(op->getRH(), out);
    } else if (const GreaterThanEquals * op = dyn_cast<GreaterThanEquals>(expr)) {
        PrintExpression(op->getLH(), out);
        out << " >= ";
        PrintExpression(op->getRH(), out);
    } else if (const GreaterThan * op = dyn_cast<GreaterThan>(expr)) {
        PrintExpression(op->getLH(), out);
        out << " > ";
        PrintExpression(op->getRH(), out);
    } else if (const NotEquals * op = dyn_cast<NotEquals>(expr)) {
        PrintExpression(op->getLH(), out);
        out << " != ";
        PrintExpression(op->getRH(), out);
    } else if (const Statement * stmt = dyn_cast<Statement>(expr)) {
        out << stmt->getName();
    } else {
        out << "???";
    }
}


static const StringRef printKernelNameAnnotations(StringRef name, raw_ostream & out) {
    auto pos = name.find_first_of("+-");
    if (pos == StringRef::npos) {
        return name;
    }
    out << "#\n" << "# Pablo Debug Annotations:\n";
    auto kernelName = name.substr(0, pos);
    while (pos != StringRef::npos) {
        char a = name[pos];
        auto pos2 = name.find_first_of("+-");
        auto annotation = name.substr(pos + 1, pos2);
        out << "# " << a << annotation << "\n";
        pos = pos2;
    }
    out << "#\n";
    return kernelName;
}


void PabloPrinter::print(PabloKernel const * kernel, raw_ostream & out) noexcept {
    auto kernelName = printKernelNameAnnotations(kernel->getName(), out);
    out << "kernel " << kernelName << " :: ";
    out << "[";
    for (size_t i = 0; i < kernel->getInputStreamSetBindings().size(); ++i) {
        if (i) out << ", ";
        auto const & streamBinding = kernel->getInputStreamSetBinding(i);
        uint32_t numStreams = streamBinding.getNumElements();
        uint32_t fw = streamBinding.getFieldWidth();
        out << "<i" << fw << ">[" << numStreams << "] ";
        out << streamBinding.getName();
    }
    for (auto const & scalarBinding : kernel->getInputScalarBindings()) {
        out << *scalarBinding.getType();
    }

    out << "] -> [";

    for (size_t i = 0; i < kernel->getOutputStreamSetBindings().size(); ++i) {
        if (i) out << ", ";
        auto const & streamBinding = kernel->getOutputStreamSetBinding(i);
        uint32_t numStreams = streamBinding.getNumElements();
        uint32_t fw = streamBinding.getFieldWidth();
        out << "<i" << fw << ">[" << numStreams << "] ";
        out << streamBinding.getName();
    }
    for (auto const & scalarBinding : kernel->getOutputScalarBindings()) {
        out << *scalarBinding.getType() << " " << scalarBinding.getName();
    }

    out << "] {\n";
    print(kernel->getEntryScope(), out, true, INDENT_WIDTH);
    out << "}\n\n";
}


void PabloPrinter::print(PabloAST const * node, raw_ostream & out) noexcept {
    PrintExpression(node, out);
}

void PabloPrinter::print(PabloBlock const * block, raw_ostream & out, const bool expandNested, unsigned indent) noexcept {
    for (auto const & stmt : *block) {
        PrintStatement(stmt, out, expandNested, indent);
    }
}


void PabloPrinter::print(Statement const * stmt, raw_ostream & out, const bool expandNested, unsigned indent) noexcept {
    PrintStatement(stmt, out, expandNested, indent);
}


} // namespace pablo
