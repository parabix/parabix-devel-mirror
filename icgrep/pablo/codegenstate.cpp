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
#include <pablo/pe_lookahead.h>
#include <pablo/pe_matchstar.h>
#include <pablo/pe_scanthru.h>
#include <pablo/pe_infile.h>
#include <pablo/pe_count.h>
#include <pablo/pe_integer.h>
#include <pablo/pe_string.h>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_ones.h>
#include <pablo/pe_var.h>
#include <pablo/ps_assign.h>
#include <pablo/pablo_kernel.h>
#include <llvm/Support/raw_os_ostream.h>

#define CHECK_SAME_TYPE(A, B) \
    assert ("DIFFERING CONTEXTS" && (&((A)->getType()->getContext()) == &((B)->getType()->getContext()))); \
    assert ("DIFFERING TYPES" && ((A)->getType() == (B)->getType()))

using namespace llvm;

namespace pablo {

/// UNARY CREATE FUNCTIONS
///

Count * PabloBlock::createCount(PabloAST * expr) {
    Type * type = getParent()->getBuilder()->getSizeTy();
    return insertAtInsertionPoint(new (mAllocator) Count(expr, makeName("count"), type, mAllocator));
}

Count * PabloBlock::createCount(PabloAST * const expr, const llvm::StringRef & prefix)  {
    Type * type = getParent()->getBuilder()->getSizeTy();
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
        type = getParent()->getBuilder()->getStreamTy();
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

Advance * PabloBlock::createAdvance(PabloAST * expr, PabloAST * shiftAmount, String * name) {
    if (name == nullptr) {
        name = makeName("advance");
    }
    return insertAtInsertionPoint(new (mAllocator) Advance(expr, shiftAmount, name, mAllocator));
}

Lookahead * PabloBlock::createLookahead(PabloAST * expr, PabloAST * shiftAmount, String * name) {
    if (name == nullptr) {
        name = makeName("lookahead");
    }
    return insertAtInsertionPoint(new (mAllocator) Lookahead(expr, shiftAmount, name, mAllocator));
}

Extract * PabloBlock::createExtract(PabloAST * array, PabloAST * index, String * name) {
    assert (array && index);
    if (name == nullptr) {
        std::string tmp;
        raw_string_ostream out(tmp);
        array->print(out);
        out << '[';
        index->print(out);
        out << ']';
        name = makeName(out.str());
    }
    Type * type = array->getType();
    if (LLVM_LIKELY(isa<ArrayType>(type))) {
        type = cast<ArrayType>(type)->getArrayElementType();
    } else {
        std::string tmp;
        raw_string_ostream out(tmp);
        out << "cannot extract element from ";
        array->print(out);
        out << " : not a StreamType or ArrayType";
        throw std::runtime_error(out.str());
    }
    return insertAtInsertionPoint(new (mAllocator) Extract(array, index, name, type, mAllocator));
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
    CHECK_SAME_TYPE(expr1, expr2);
    return new (mAllocator) LessThan(getParent()->getBuilder()->getInt1Ty(), expr1, expr2, mAllocator);
}

enum class AssignErrorType {
    TypeMismatch
    , ReadOnlyVar
    , NotAVariable
};

static void reportAssignError(PabloAST * const var, PabloAST * const value, const AssignErrorType type) {
    std::string tmp;
    raw_string_ostream out(tmp);
    out << "Cannot assign ";
    value->print(out);
    out << " to ";
    var->print(out);
    out << ": ";
    switch (type) {
        case AssignErrorType::TypeMismatch:
            out << "type mismatch ";
            value->getType()->print(out);
            out << " vs. ";
            var->getType()->print(out);
            break;
        case AssignErrorType::ReadOnlyVar:
            var->print(out);
            out << " is read only";
            break;
        case AssignErrorType::NotAVariable:
            var->print(out);
            out << " is not a variable";
            break;
    }
    llvm::report_fatal_error(out.str());
}

Assign * PabloBlock::createAssign(PabloAST * const var, PabloAST * const value) {
    CHECK_SAME_TYPE(var, value);

    if (LLVM_UNLIKELY(var->getType() != value->getType())) {
        reportAssignError(var, value, AssignErrorType::TypeMismatch);
    }

    PabloAST * test = var;
    for (;;) {
        if (LLVM_LIKELY(isa<Var>(test))) {
            if (LLVM_UNLIKELY(cast<Var>(test)->isReadOnly())) {
                reportAssignError(var, value, AssignErrorType::ReadOnlyVar);
            }
            break;
        } else if (isa<Extract>(test)) {
            test = cast<Extract>(test)->getArray();
        } else {
            reportAssignError(var, value, AssignErrorType::NotAVariable);
        }
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

If * PabloBlock::createIf(PabloAST * condition, PabloBlock * body) {
    assert (condition);
    If * const node = insertAtInsertionPoint(new (mAllocator) If(condition, body, mAllocator));
    body->setBranch(node);
    return node;
}

While * PabloBlock::createWhile(PabloAST * condition, PabloBlock * body) {
    assert (condition);
    While * const node = insertAtInsertionPoint(new (mAllocator) While(condition, body, mAllocator));
    body->setBranch(node);
    return node;
}

/// TERNARY CREATE FUNCTION

Sel * PabloBlock::createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr, String * name) {
    CHECK_SAME_TYPE(trueExpr, falseExpr);
    if (name == nullptr) {
        name = makeName("sel");
    }
    return insertAtInsertionPoint(new (mAllocator) Sel(condition, trueExpr, falseExpr, name, mAllocator));
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
