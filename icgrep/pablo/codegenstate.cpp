/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/codegenstate.h>

#define CHECK_SAME_TYPE(A, B) \
    assert ("DIFFERING CONTEXTS" && (&((A)->getType()->getContext()) == &((B)->getType()->getContext()))); \
    assert ("DIFFERING TYPES" && ((A)->getType() == (B)->getType()))

using StreamType = IDISA::StreamType;

inline void printType(const Type * type, raw_string_ostream & out) {
    if (auto st = dyn_cast<StreamType>(type)) {
        out << "<" << st->getNumElements() << " x s" << st->getFieldWidth() << ">";
    } else {
        type->print(out);
    }
}

namespace pablo {

/// UNARY CREATE FUNCTIONS
///

Call * PabloBlock::createCall(PabloAST * prototype, const std::vector<PabloAST *> &) {
    assert (prototype);
    return insertAtInsertionPoint(new Call(prototype));
}

Count * PabloBlock::createCount(PabloAST * expr) {
    Type * type = getParent()->getBuilder()->getSizeTy();
    return insertAtInsertionPoint(new Count(expr, makeName("count"), type));
}

Count * PabloBlock::createCount(PabloAST * const expr, const std::string & prefix)  {
    Type * type = getParent()->getBuilder()->getSizeTy();
    return insertAtInsertionPoint(new Count(expr, makeName(prefix), type));
}

Not * PabloBlock::createNot(PabloAST * expr, String * name) {
    assert (expr);
    if (name == nullptr) {
        name = makeName("not");
    }
    return insertAtInsertionPoint(new Not(expr, name));
}

Var * PabloBlock::createVar(PabloAST * name, Type * type) {
    if (type == nullptr) {
        type = getParent()->getStreamSetTy();
    }
    if (LLVM_UNLIKELY(name == nullptr || !isa<String>(name))) {
        throw std::runtime_error("Var objects must have a String name");
    }
    return mParent->makeVariable(name, type);
}

InFile * PabloBlock::createInFile(PabloAST * expr, String * name) {
    assert (expr);
    if (name == nullptr) {
        name = makeName("inFile");
    }
    return insertAtInsertionPoint(new InFile(expr, name));
}

AtEOF * PabloBlock::createAtEOF(PabloAST * expr, String * name) {
    assert (expr);
    if (name == nullptr) {
        name = makeName("atEOF");
    }
    return insertAtInsertionPoint(new AtEOF(expr, name));
}
    
    
/// BINARY CREATE FUNCTIONS

Advance * PabloBlock::createAdvance(PabloAST * expr, PabloAST * shiftAmount, String * name) {
    if (name == nullptr) {
        name = makeName("advance");
    }
    return insertAtInsertionPoint(new Advance(expr, shiftAmount, name));
}

Lookahead * PabloBlock::createLookahead(PabloAST * expr, PabloAST * shiftAmount, String * name) {
    if (name == nullptr) {
        name = makeName("lookahead");
    }
    return insertAtInsertionPoint(new Lookahead(expr, shiftAmount, name));
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
    if (LLVM_LIKELY(isa<StreamType>(type))) {
        type = cast<StreamType>(type)->getStreamElementType();
    } else if (LLVM_LIKELY(isa<ArrayType>(type))) {
        type = cast<ArrayType>(type)->getArrayElementType();
    } else {
        std::string tmp;
        raw_string_ostream out(tmp);
        out << "cannot extract element from ";
        array->print(out);
        out << " : not a StreamType or ArrayType";
        throw std::runtime_error(out.str());
    }
    return insertAtInsertionPoint(new Extract(array, index, name, type));
}

And * PabloBlock::createAnd(PabloAST * expr1, PabloAST * expr2, String * name) {
    CHECK_SAME_TYPE(expr1, expr2);
    if (name == nullptr) {
        name = makeName("and");
    }
    return insertAtInsertionPoint(new And(expr1->getType(), expr1, expr2, name));
}

And * PabloBlock::createAnd(Type * const type, const unsigned reserved, String * name) {
    if (name == nullptr) {
        name = makeName("and");
    }
    return insertAtInsertionPoint(new And(type, reserved, name));
}

Or * PabloBlock::createOr(PabloAST * expr1, PabloAST * expr2, String * name) {
    CHECK_SAME_TYPE(expr1, expr2);
    if (name == nullptr) {
        name = makeName("or");
    }
    return insertAtInsertionPoint(new Or(expr1->getType(), expr1, expr2, name));
}

Or * PabloBlock::createOr(Type * const type, const unsigned reserved, String * name) {
    if (name == nullptr) {
        name = makeName("or");
    }
    return insertAtInsertionPoint(new Or(type, reserved, name));
}

Xor * PabloBlock::createXor(PabloAST * expr1, PabloAST * expr2, String * name) {
    CHECK_SAME_TYPE(expr1, expr2);
    if (name == nullptr) {
        name = makeName("xor");
    }
    return insertAtInsertionPoint(new Xor(expr1->getType(), expr1, expr2, name));
}

Xor * PabloBlock::createXor(Type * const type, const unsigned reserved, String * name) {
    if (name == nullptr) {
        name = makeName("xor");
    }
    return insertAtInsertionPoint(new Xor(type, reserved, name));
}

Add * PabloBlock::createAdd(PabloAST * expr1, PabloAST * expr2) {
    CHECK_SAME_TYPE(expr1, expr2);
    return new Add(expr1->getType(), expr1, expr2);
}

Subtract * PabloBlock::createSubtract(PabloAST * expr1, PabloAST * expr2) {
    CHECK_SAME_TYPE(expr1, expr2);
    return new Subtract(expr1->getType(), expr1, expr2);
}

LessThan * PabloBlock::createLessThan(PabloAST * expr1, PabloAST * expr2) {
    CHECK_SAME_TYPE(expr1, expr2);
    return new LessThan(getParent()->getBuilder()->getInt1Ty(), expr1, expr2);
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
            printType(value->getType(), out);
            out << " vs. ";
            printType(var->getType(), out);
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

    return insertAtInsertionPoint(new Assign(var, value));
}

MatchStar * PabloBlock::createMatchStar(PabloAST * marker, PabloAST * charclass, String * name) {
    CHECK_SAME_TYPE(marker, charclass);
    if (name == nullptr) {
        name = makeName("matchstar");
    }
    return insertAtInsertionPoint(new MatchStar(marker, charclass, name));
}

ScanThru * PabloBlock::createScanThru(PabloAST * from, PabloAST * thru, String * name) {
    CHECK_SAME_TYPE(from, thru);
    if (name == nullptr) {
        name = makeName("scanthru");
    }
    return insertAtInsertionPoint(new ScanThru(from, thru, name));
}

If * PabloBlock::createIf(PabloAST * condition, PabloBlock * body) {
    assert (condition);
    If * const node = insertAtInsertionPoint(new If(condition, body));
    body->setBranch(node);
    return node;
}

While * PabloBlock::createWhile(PabloAST * condition, PabloBlock * body) {
    assert (condition);
    While * const node = insertAtInsertionPoint(new While(condition, body));
    body->setBranch(node);
    return node;
}

/// TERNARY CREATE FUNCTION

Sel * PabloBlock::createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr, String * name) {
    CHECK_SAME_TYPE(trueExpr, falseExpr);
    if (name == nullptr) {
        name = makeName("sel");
    }
    return insertAtInsertionPoint(new Sel(condition, trueExpr, falseExpr, name));
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
    mAllocator.deallocate(reinterpret_cast<Allocator::pointer>(this));
}

}
