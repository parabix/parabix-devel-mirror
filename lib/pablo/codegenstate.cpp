/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/codegenstate.h>

#include <idisa/idisa_builder.h>
#include <pablo/printer_pablos.h>
#include <pablo/boolean.h>
#include <pablo/arithmetic.h>
#include <pablo/branch.h>
#include <pablo/pablo_intrinsic.h>
#include <pablo/pe_advance.h>
#include <pablo/pe_count.h>
#include <pablo/pe_everynth.h>
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
#include <pablo/pablo_kernel.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_os_ostream.h>

using namespace llvm;

namespace pablo {

#ifndef NDEBUG
inline void __checkSameType(const Type * const A, const Type * const B) {
    assert ("DIFFERING CONTEXTS" && (&(A->getContext()) == &(B->getContext())));
    assert ("DIFFERING TYPES" && (A == B));
}
inline void __checkSameType(const PabloAST * const A, const PabloAST * const B) {
    __checkSameType(A->getType(), B->getType());
}
#define CHECK_SAME_TYPE(A, B) __checkSameType(A, B)
#else
#define CHECK_SAME_TYPE(A, B)
#endif

/// UNARY CREATE FUNCTIONS
///

Count * PabloBlock::createCount(PabloAST * const expr, const String * const name) {
    IntegerType * const type = getParent()->getSizeTy();
    return insertAtInsertionPoint(new (mAllocator) Count(expr, name, type, mAllocator));
}

EveryNth * PabloBlock::createEveryNth(PabloAST * const expr, Integer * n, const String * const name) {
    return insertAtInsertionPoint(new (mAllocator) EveryNth(expr, n, name, mAllocator));
}

Not * PabloBlock::createNot(PabloAST * expr, const String * const name) {
    assert (expr);
    return insertAtInsertionPoint(new (mAllocator) Not(expr, name, mAllocator));
}

Var * PabloBlock::createVar(const String * const name, Type * type) {
    if (type == nullptr) {
        type = getParent()->getStreamTy();
    }
    if (LLVM_UNLIKELY(name == nullptr)) {
        throw std::runtime_error("Var objects must have a String name");
    }
    return mParent->makeVariable(name, type);
}

InFile * PabloBlock::createInFile(PabloAST * expr, const String * const name) {
    assert (expr);
    return insertAtInsertionPoint(new (mAllocator) InFile(expr, name, mAllocator));
}

AtEOF * PabloBlock::createAtEOF(PabloAST * expr, const String * const name) {
    assert (expr);
    return insertAtInsertionPoint(new (mAllocator) AtEOF(expr, name, mAllocator));
}

TerminateAt * PabloBlock::createTerminateAt(PabloAST * strm, Integer *  code, const String * const name) {
    assert (strm); assert(code);
    return insertAtInsertionPoint(new (mAllocator) TerminateAt(strm, code, name, mAllocator));
}

/// BINARY CREATE FUNCTIONS

Advance * PabloBlock::createAdvance(PabloAST * expr, Integer * shiftAmount, const String * const name) {
    return insertAtInsertionPoint(new (mAllocator) Advance(expr, shiftAmount, name, mAllocator));
}

Lookahead * PabloBlock::createLookahead(PabloAST * expr, Integer * shiftAmount, const String * const name) {
    return insertAtInsertionPoint(new (mAllocator) Lookahead(expr, shiftAmount, name, mAllocator));
}

Extract * PabloBlock::createExtract(Var * const array, Integer * const index) {
    assert (array && index);
    return mParent->makeExtract(array, index);
}

And * PabloBlock::createAnd(PabloAST * expr1, PabloAST * expr2, const String * const name) {
    CHECK_SAME_TYPE(expr1, expr2);
    return insertAtInsertionPoint(new (mAllocator) And(expr1->getType(), expr1, expr2, name, mAllocator));
}

Or * PabloBlock::createOr(PabloAST * expr1, PabloAST * expr2, const String * const name) {
    CHECK_SAME_TYPE(expr1, expr2);
    return insertAtInsertionPoint(new (mAllocator) Or(expr1->getType(), expr1, expr2, name, mAllocator));
}

Xor * PabloBlock::createXor(PabloAST * expr1, PabloAST * expr2, const String * const name) {
    CHECK_SAME_TYPE(expr1, expr2);
    return insertAtInsertionPoint(new (mAllocator) Xor(expr1->getType(), expr1, expr2, name, mAllocator));
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

Assign * PabloBlock::createAssign(Var * const var, PabloAST * const value) {
    if (LLVM_UNLIKELY(var->isReadOnly())) {
        std::string tmp;
        raw_string_ostream out(tmp);
        out << "cannot assign ";
        value->print(out);
        out << " to ";
        var->print(out);
        out << ": ";
        var->print(out);
        out << " is read only";
        report_fatal_error(out.str());
    }
    return insertAtInsertionPoint(new (mAllocator) Assign(var, value, mAllocator));
}

MatchStar * PabloBlock::createMatchStar(PabloAST * marker, PabloAST * charclass, const String * const name) {
    CHECK_SAME_TYPE(marker, charclass);
    return insertAtInsertionPoint(new (mAllocator) MatchStar(marker, charclass, name, mAllocator));
}

ScanThru * PabloBlock::createScanThru(PabloAST * from, PabloAST * thru, const String * const name) {
    CHECK_SAME_TYPE(from, thru);
    return insertAtInsertionPoint(new (mAllocator) ScanThru(from, thru, name, mAllocator));
}

ScanTo * PabloBlock::createScanTo(PabloAST * from, PabloAST * to, const String * const name) {
    CHECK_SAME_TYPE(from, to);
    return insertAtInsertionPoint(new (mAllocator) ScanTo(from, to, name, mAllocator));
}

AdvanceThenScanThru * PabloBlock::createAdvanceThenScanThru(PabloAST * from, PabloAST * thru, const String * const name) {
    CHECK_SAME_TYPE(from, thru);
    return insertAtInsertionPoint(new (mAllocator) AdvanceThenScanThru(from, thru, name, mAllocator));
}

AdvanceThenScanTo * PabloBlock::createAdvanceThenScanTo(PabloAST * from, PabloAST * to, const String * const name) {
    CHECK_SAME_TYPE(from, to);
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

Repeat * PabloBlock::createRepeat(Integer * fieldWidth, PabloAST * value, const String * const name) {
    assert (fieldWidth && value);
    Type * const type = VectorType::get(IntegerType::get(value->getType()->getContext(), fieldWidth->value()), 0);
    return insertAtInsertionPoint(new (mAllocator) Repeat(fieldWidth, value, type, name, mAllocator));
}

PackH * PabloBlock::createPackH(Integer * fieldWidth, PabloAST * value, const String * const name) {
    assert (fieldWidth && value);
    Type * const type = VectorType::get(IntegerType::get(value->getType()->getContext(), fieldWidth->value()/2), 0);
    return insertAtInsertionPoint(new (mAllocator) PackH(fieldWidth, value, name, type, mAllocator));
}

PackL * PabloBlock::createPackL(Integer * fieldWidth, PabloAST * value, const String * const name) {
    assert (fieldWidth && value);
    Type * const type = VectorType::get(IntegerType::get(value->getType()->getContext(), fieldWidth->value()/2), 0);
    return insertAtInsertionPoint(new (mAllocator) PackL(fieldWidth, value, name, type, mAllocator));
}

/// TERNARY CREATE FUNCTIONS

Sel * PabloBlock::createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr, const String * const name) {
    CHECK_SAME_TYPE(trueExpr, falseExpr);
    return insertAtInsertionPoint(new (mAllocator) Sel(condition, trueExpr, falseExpr, name, mAllocator));
}

IndexedAdvance * PabloBlock::createIndexedAdvance(PabloAST * expr, PabloAST * indexStream, Integer * shiftAmount, const String * const name) {
    return insertAtInsertionPoint(new (mAllocator) IndexedAdvance(expr, indexStream, shiftAmount, name, mAllocator));
}

/// QUARTERNARY FUNCTIONS

Ternary * PabloBlock::createAnd3(PabloAST * expr1, PabloAST * expr2, PabloAST * expr3, const String * const name) {
    CHECK_SAME_TYPE(expr1, expr2);
    CHECK_SAME_TYPE(expr2, expr3);
    //     (a, b, c) =  (111), (110), (101), (100), (011), (010), (001), (000)
    // and3(a, b, c) =    1      0      0      0      0      0      0      0    = 0x80
    return createTernary(getInteger(0x80), expr1, expr2, expr3, name);
}

Ternary * PabloBlock::createOr3(PabloAST * expr1, PabloAST * expr2, PabloAST * expr3, const String * const name) {
    CHECK_SAME_TYPE(expr1, expr2);
    CHECK_SAME_TYPE(expr2, expr3);
    //    (a, b, c) =  (111), (110), (101), (100), (011), (010), (001), (000)
    // or3(a, b, c) =    1      1      1      1      1      1      1      0    = 0xFE
    return createTernary(getInteger(0xFE), expr1, expr2, expr3, name);
}

Ternary * PabloBlock::createXor3(PabloAST * expr1, PabloAST * expr2, PabloAST * expr3, const String * const name) {
    CHECK_SAME_TYPE(expr1, expr2);
    CHECK_SAME_TYPE(expr2, expr3);
    //     (a, b, c) =  (111), (110), (101), (100), (011), (010), (001), (000)
    // xor3(a, b, c) =    1      0      0      1      0      1      1      0    = 0x96
    return createTernary(getInteger(0x96), expr1, expr2, expr3, name);
}

Ternary * PabloBlock::createMajority3(PabloAST * expr1, PabloAST * expr2, PabloAST * expr3, const String * const name) {
    CHECK_SAME_TYPE(expr1, expr2);
    CHECK_SAME_TYPE(expr2, expr3);
    //          (a, b, c) =  (111), (110), (101), (100), (011), (010), (001), (000)
    // majority3(a, b, c) =    1      1      1      0      1      0      0      0    = 0xE8
    return createTernary(getInteger(0xE8), expr1, expr2, expr3, name);
}

Ternary * PabloBlock::createAndOr(PabloAST * andExpr1, PabloAST * orExpr1, PabloAST * orExpr2, const String * const name) {
    CHECK_SAME_TYPE(andExpr1, orExpr1);
    CHECK_SAME_TYPE(orExpr1, orExpr2);
    //      (a, b, c) =  (111), (110), (101), (100), (011), (010), (001), (000)
    // andOr(a, b, c) =    1      1      1      0      0      0      0      0    = 0xE0
    return createTernary(getInteger(0xE0), andExpr1, orExpr1, orExpr2, name);
}

Ternary * PabloBlock::createAndXor(PabloAST * andExpr1, PabloAST * xorExpr1, PabloAST * xorExpr2, const String * const name) {
    CHECK_SAME_TYPE(andExpr1, xorExpr1);
    CHECK_SAME_TYPE(xorExpr1, xorExpr2);
    //       (a, b, c) =  (111), (110), (101), (100), (011), (010), (001), (000)
    // andXor(a, b, c) =    0      1      1      0      0      0      0      0    = 0x60
    return createTernary(getInteger(0x60), andExpr1, xorExpr1, xorExpr2, name);
}

Ternary * PabloBlock::createOrAnd(PabloAST * orExpr1, PabloAST * andExpr1, PabloAST * andExpr2, const String * const name) {
    CHECK_SAME_TYPE(orExpr1, andExpr1);
    CHECK_SAME_TYPE(andExpr1, andExpr2);
    //      (a, b, c) =  (111), (110), (101), (100), (011), (010), (001), (000)
    // orAnd(a, b, c) =    1      1      1      1      1      0      0      0    = 0xF8
    return createTernary(getInteger(0xF8), orExpr1, andExpr1, andExpr2, name);
}

Ternary * PabloBlock::createOrXor(PabloAST * orExpr1, PabloAST * xorExpr1, PabloAST * xorExpr2, const String * const name) {
    CHECK_SAME_TYPE(orExpr1, xorExpr1);
    CHECK_SAME_TYPE(xorExpr1, xorExpr2);
    //      (a, b, c) =  (111), (110), (101), (100), (011), (010), (001), (000)
    // orXor(a, b, c) =    1      1      1      1      0      1      1      0    = 0xF6
    return createTernary(getInteger(0xF6), orExpr1, xorExpr1, xorExpr2, name);
}

Ternary * PabloBlock::createXorAnd(PabloAST * xorExpr1, PabloAST * andExpr1, PabloAST * andExpr2, const String * const name) {
    CHECK_SAME_TYPE(xorExpr1, andExpr1);
    CHECK_SAME_TYPE(andExpr1, andExpr2);
    //       (a, b, c) =  (111), (110), (101), (100), (011), (010), (001), (000)
    // xorAnd(a, b, c) =    0      1      1      1      1      0      0      0    = 0x78
    return createTernary(getInteger(0x78), xorExpr1, andExpr1, andExpr2, name);
}

Ternary * PabloBlock::createXorOr(PabloAST * xorExpr1, PabloAST * orExpr1, PabloAST * orExpr2, const String * const name) {
    CHECK_SAME_TYPE(xorExpr1, orExpr1);
    CHECK_SAME_TYPE(orExpr1, orExpr2);
    //      (a, b, c) =  (111), (110), (101), (100), (011), (010), (001), (000)
    // xorOr(a, b, c) =    0      0      0      1      1      1      1      0    = 0x1E
    return createTernary(getInteger(0x1E), xorExpr1, orExpr1, orExpr2, name);
}

Ternary * PabloBlock::createTernary(Integer * mask, PabloAST * a, PabloAST * b, PabloAST * c, const String * const name) {
    CHECK_SAME_TYPE(a, b);
    CHECK_SAME_TYPE(b, c);
    return insertAtInsertionPoint(new (mAllocator) Ternary(mask, a, b, c, name, mAllocator));
}



IntrinsicCall * PabloBlock::createIntrinsicCall(Intrinsic intrinsic, llvm::Type * type, llvm::ArrayRef<PabloAST *> argv, const String * name) {
    return insertAtInsertionPoint(new (mAllocator) IntrinsicCall(intrinsic, type, argv, name, mAllocator));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief createScope
 ** ------------------------------------------------------------------------------------------------------------- */
PabloBlock * PabloBlock::createScope() noexcept {
    return new (mAllocator) PabloBlock(mParent, mAllocator);
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
