/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "codegenstate.h"
#include <pablo/printer_pablos.h>
#include <pablo/boolean.h>
#include <pablo/arithmetic.h>
#include <pablo/branch.h>
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
#include <pablo/pablo_kernel.h>
#include <IR_Gen/idisa_builder.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_os_ostream.h>

using namespace llvm;

namespace pablo {

#ifndef NDEBUG
inline void checkSameType(const Type * const A, const Type * const B) {
    assert ("DIFFERING CONTEXTS" && (&(A->getContext()) == &(B->getContext())));
    assert ("DIFFERING TYPES" && (A == B));
}
inline void checkSameType(const PabloAST * const A, const PabloAST * const B) {
    checkSameType(A->getType(), B->getType());
}
#define CHECK_SAME_TYPE(A, B) checkSameType(A, B)
#else
#define CHECK_SAME_TYPE(A, B)
#endif

/// UNARY CREATE FUNCTIONS
///

Count * PabloBlock::createCount(PabloAST * expr) {
    IntegerType * const type = getParent()->getSizeTy();
    return insertAtInsertionPoint(new (mAllocator) Count(expr, makeName("count"), type, mAllocator));
}

Count * PabloBlock::createCount(PabloAST * const expr, const llvm::StringRef & prefix)  {
    IntegerType * const type = getParent()->getSizeTy();
    return insertAtInsertionPoint(new (mAllocator) Count(expr, makeName(prefix), type, mAllocator));
}

Not * PabloBlock::createNot(PabloAST * expr, String * name) {
    assert (expr);
    if (name == nullptr) {
        name = makeName("not");
    }
    return insertAtInsertionPoint(new (mAllocator) Not(expr, name, mAllocator));
}

Var * PabloBlock::createVar(PabloAST * name, Type * type) {
    if (type == nullptr) {
        type = getParent()->getStreamTy();
    }
    if (LLVM_UNLIKELY(name == nullptr || !isa<String>(name))) {
        throw std::runtime_error("Var objects must have a String name");
    }
    return mParent->makeVariable(cast<String>(name), type);
}

InFile * PabloBlock::createInFile(PabloAST * expr, String * name) {
    assert (expr);
    if (name == nullptr) {
        name = makeName("inFile");
    }
    return insertAtInsertionPoint(new (mAllocator) InFile(expr, name, mAllocator));
}

AtEOF * PabloBlock::createAtEOF(PabloAST * expr, String * name) {
    assert (expr);
    if (name == nullptr) {
        name = makeName("atEOF");
    }
    return insertAtInsertionPoint(new (mAllocator) AtEOF(expr, name, mAllocator));
}

/// BINARY CREATE FUNCTIONS

Advance * PabloBlock::createAdvance(PabloAST * expr, Integer * shiftAmount, String * name) {
    if (name == nullptr) {
        name = makeName("advance");
    }
    return insertAtInsertionPoint(new (mAllocator) Advance(expr, shiftAmount, name, mAllocator));
}

Lookahead * PabloBlock::createLookahead(PabloAST * expr, Integer * shiftAmount, String * name) {
    if (name == nullptr) {
        name = makeName("lookahead");
    }
    return insertAtInsertionPoint(new (mAllocator) Lookahead(expr, shiftAmount, name, mAllocator));
}

Extract * PabloBlock::createExtract(Var * array, Integer * index) {
    assert (array && index);
    Type * type = array->getType();
    if (LLVM_LIKELY(isa<ArrayType>(type))) {
        type = cast<ArrayType>(type)->getArrayElementType();
    } else {
        std::string tmp;
        raw_string_ostream out(tmp);
        out << "cannot extract element from ";
        array->print(out);
        out << ": ";
        type->print(out);
        out << " is not an array type";
        throw std::runtime_error(out.str());
    }
    return new (mAllocator) Extract(array, index, type, mAllocator);
}

And * PabloBlock::createAnd(PabloAST * expr1, PabloAST * expr2, String * name) {
    CHECK_SAME_TYPE(expr1, expr2);
    if (name == nullptr) {
        name = makeName("and");
    }
    return insertAtInsertionPoint(new (mAllocator) And(expr1->getType(), expr1, expr2, name, mAllocator));
}

And * PabloBlock::createAnd(Type * const type, const unsigned reserved, String * name) {
    if (name == nullptr) {
        name = makeName("and");
    }
    return insertAtInsertionPoint(new (mAllocator) And(type, reserved, name, mAllocator));
}

Or * PabloBlock::createOr(PabloAST * expr1, PabloAST * expr2, String * name) {
    CHECK_SAME_TYPE(expr1, expr2);
    if (name == nullptr) {
        name = makeName("or");
    }
    return insertAtInsertionPoint(new (mAllocator) Or(expr1->getType(), expr1, expr2, name, mAllocator));
}

Or * PabloBlock::createOr(Type * const type, const unsigned reserved, String * name) {
    if (name == nullptr) {
        name = makeName("or");
    }
    return insertAtInsertionPoint(new (mAllocator) Or(type, reserved, name, mAllocator));
}

Xor * PabloBlock::createXor(PabloAST * expr1, PabloAST * expr2, String * name) {
    CHECK_SAME_TYPE(expr1, expr2);
    if (name == nullptr) {
        name = makeName("xor");
    }
    return insertAtInsertionPoint(new (mAllocator) Xor(expr1->getType(), expr1, expr2, name, mAllocator));
}

Xor * PabloBlock::createXor(Type * const type, const unsigned reserved, String * name) {
    if (name == nullptr) {
        name = makeName("xor");
    }
    return insertAtInsertionPoint(new (mAllocator) Xor(type, reserved, name, mAllocator));
}

Add * PabloBlock::createAdd(PabloAST * expr1, PabloAST * expr2) {
    CHECK_SAME_TYPE(expr1, expr2);
    return new (mAllocator) Add(expr1->getType(), expr1, expr2, mAllocator);
}

Subtract * PabloBlock::createSubtract(PabloAST * expr1, PabloAST * expr2) {
    CHECK_SAME_TYPE(expr1, expr2);
    return new (mAllocator) Subtract(expr1->getType(), expr1, expr2, mAllocator);
}

LessThan * PabloBlock::createLessThan(PabloAST * expr1, PabloAST * expr2) {
    const Type * const t1 = expr1->getType()->isArrayTy() ? expr1->getType()->getArrayElementType() : expr1->getType();
    const Type * const t2 = expr2->getType()->isArrayTy() ? expr2->getType()->getArrayElementType() : expr2->getType();
    CHECK_SAME_TYPE(t1, t2);
    Type * ty = getParent()->getInt1Ty();
    if (t1->isVectorTy() || t2->isVectorTy()) {
        ty = VectorType::get(ty, 0);
    }
    return new (mAllocator) LessThan(ty, expr1, expr2, mAllocator);
}

Equals * PabloBlock::createEquals(PabloAST * expr1, PabloAST * expr2) {
    const Type * const t1 = expr1->getType()->isArrayTy() ? expr1->getType()->getArrayElementType() : expr1->getType();
    const Type * const t2 = expr2->getType()->isArrayTy() ? expr2->getType()->getArrayElementType() : expr2->getType();
    CHECK_SAME_TYPE(t1, t2);
    Type * ty = getParent()->getInt1Ty();
    if (t1->isVectorTy() || t2->isVectorTy()) {
        ty = VectorType::get(ty, 0);
    }
    return new (mAllocator) Equals(ty, expr1, expr2, mAllocator);
}

Assign * PabloBlock::createAssign(PabloAST * const var, PabloAST * const value) {
    CHECK_SAME_TYPE(var, value);
    Var * test = nullptr;
    if (isa<Extract>(var)) {
        test = cast<Extract>(var)->getArray();
    } else if (isa<Var>(var)) {
        test = cast<Var>(var);
    }
    if (LLVM_UNLIKELY(test == nullptr || test->isReadOnly())) {
        std::string tmp;
        raw_string_ostream out(tmp);
        out << "cannot assign ";
        value->print(out);
        out << " to ";
        var->print(out);
        out << ": ";
        var->print(out);
        if (test == nullptr) {
            out << " must be an Extract expression or Var declaration";
        } else {
            out << " is read only";
        }
        report_fatal_error(out.str());
    }
    return insertAtInsertionPoint(new (mAllocator) Assign(var, value, mAllocator));
}

MatchStar * PabloBlock::createMatchStar(PabloAST * marker, PabloAST * charclass, String * name) {
    CHECK_SAME_TYPE(marker, charclass);
    if (name == nullptr) {
        name = makeName("matchstar");
    }
    return insertAtInsertionPoint(new (mAllocator) MatchStar(marker, charclass, name, mAllocator));
}

ScanThru * PabloBlock::createScanThru(PabloAST * from, PabloAST * thru, String * name) {
    CHECK_SAME_TYPE(from, thru);
    if (name == nullptr) {
        name = makeName("scanthru");
    }
    return insertAtInsertionPoint(new (mAllocator) ScanThru(from, thru, name, mAllocator));
}

ScanTo * PabloBlock::createScanTo(PabloAST * from, PabloAST * to, String * name) {
    CHECK_SAME_TYPE(from, to);
    if (name == nullptr) {
        name = makeName("scanto");
    }
    return insertAtInsertionPoint(new (mAllocator) ScanTo(from, to, name, mAllocator));
}

AdvanceThenScanThru * PabloBlock::createAdvanceThenScanThru(PabloAST * from, PabloAST * thru, String * name) {
    CHECK_SAME_TYPE(from, thru);
    if (name == nullptr) {
        name = makeName("advscanthru");
    }
    return insertAtInsertionPoint(new (mAllocator) AdvanceThenScanThru(from, thru, name, mAllocator));
}

AdvanceThenScanTo * PabloBlock::createAdvanceThenScanTo(PabloAST * from, PabloAST * to, String * name) {
    CHECK_SAME_TYPE(from, to);
    if (name == nullptr) {
        name = makeName("advscanto");
    }
    return insertAtInsertionPoint(new (mAllocator) AdvanceThenScanTo(from, to, name, mAllocator));
}

If * PabloBlock::createIf(PabloAST * condition, PabloBlock * body) {
    assert (condition && body);
    If * const node = insertAtInsertionPoint(new (mAllocator) If(condition, body, mAllocator));
    body->setBranch(node);
    return node;
}

While * PabloBlock::createWhile(PabloAST * condition, PabloBlock * body) {
    assert (condition && body);
    While * const node = insertAtInsertionPoint(new (mAllocator) While(condition, body, mAllocator));
    body->setBranch(node);
    return node;
}

Repeat * PabloBlock::createRepeat(Integer * fieldWidth, PabloAST * value, String * name) {
    assert (fieldWidth && value);
    if (name == nullptr) {
        name = makeName("fill");
    }
    Type * const type = VectorType::get(IntegerType::get(value->getType()->getContext(), fieldWidth->value()), 0);
    return insertAtInsertionPoint(new (mAllocator) Repeat(fieldWidth, value, type, name, mAllocator));
}

PackH * PabloBlock::createPackH(Integer * fieldWidth, PabloAST * value, String * name) {
    assert (fieldWidth && value);
    if (name == nullptr) {
        name = makeName("packh");
    }
    Type * const type = VectorType::get(IntegerType::get(value->getType()->getContext(), fieldWidth->value()), 0);
    return insertAtInsertionPoint(new (mAllocator) PackH(fieldWidth, value, name, type, mAllocator));
}

PackL * PabloBlock::createPackL(Integer * fieldWidth, PabloAST * value, String * name) {
    assert (fieldWidth && value);
    if (name == nullptr) {
        name = makeName("packl");
    }
    Type * const type = VectorType::get(IntegerType::get(value->getType()->getContext(), fieldWidth->value()), 0);
    return insertAtInsertionPoint(new (mAllocator) PackL(fieldWidth, value, name, type, mAllocator));
}

/// TERNARY CREATE FUNCTIONS

Sel * PabloBlock::createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr, String * name) {
    CHECK_SAME_TYPE(trueExpr, falseExpr);
    if (name == nullptr) {
        name = makeName("sel");
    }
    return insertAtInsertionPoint(new (mAllocator) Sel(condition, trueExpr, falseExpr, name, mAllocator));
}

IndexedAdvance * PabloBlock::createIndexedAdvance(PabloAST * expr, PabloAST * indexStream, Integer * shiftAmount, String * name) {
    if (name == nullptr) {
        name = makeName("indexed_advance");
    }
    return insertAtInsertionPoint(new (mAllocator) IndexedAdvance(expr, indexStream, shiftAmount, name, mAllocator));
}
    

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief Create
 ** ------------------------------------------------------------------------------------------------------------- */
PabloBlock * PabloBlock::Create(PabloKernel * const parent) noexcept {
    Allocator & allocator = parent->mAllocator;
    return new (allocator) PabloBlock(parent, allocator);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief insert
 ** ------------------------------------------------------------------------------------------------------------- */
void PabloBlock::insert(Statement * const statement) {
    assert (statement);
    if (LLVM_UNLIKELY(mInsertionPoint == nullptr)) {
        if (mFirst) {
            statement->insertBefore(mFirst);
        } else {
            statement->removeFromParent();
            statement->mParent = this;
            mFirst = mLast = statement;
        }
    } else if (LLVM_LIKELY(statement != mInsertionPoint)) {
        statement->insertAfter(mInsertionPoint);
        mLast = (mLast == mInsertionPoint) ? statement : mLast;
        assert (statement->mPrev == mInsertionPoint);
    }
    mInsertionPoint = statement;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief eraseFromParent
 ** ------------------------------------------------------------------------------------------------------------- */
void PabloBlock::eraseFromParent(const bool recursively) {
    Statement * stmt = front();
    while (stmt) {
        stmt = stmt->eraseFromParent(recursively);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getPredecessor
 ** ------------------------------------------------------------------------------------------------------------- */
PabloBlock * PabloBlock::getPredecessor() const {
    return getBranch() ? getBranch()->getParent() : nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief print
 ** ------------------------------------------------------------------------------------------------------------- */
void PabloBlock::print(raw_ostream & O, const bool expandNested) const {
    PabloPrinter::print(this, O, expandNested);
}

}
