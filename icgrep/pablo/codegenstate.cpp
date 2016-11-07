/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/codegenstate.h>
#include <pablo/printer_pablos.h>

namespace pablo {

/// UNARY CREATE FUNCTIONS
///

Call * PabloBlock::createCall(PabloAST * prototype, const std::vector<PabloAST *> &) {
    assert (prototype);
    return insertAtInsertionPoint(new Call(prototype));
}

Count * PabloBlock::createCount(PabloAST * expr) {
    return insertAtInsertionPoint(new Count(expr, makeName("count")));
}

Count * PabloBlock::createCount(PabloAST * const expr, const std::string & prefix)  {
    return insertAtInsertionPoint(new Count(expr, makeName(prefix)));
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
        type = getStreamTy();
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
    if (LLVM_LIKELY(isa<ArrayType>(array->getType()))) {
        if (name == nullptr) {
            std::string tmp;
            raw_string_ostream out(tmp);
            PabloPrinter::print(array, out);
            PabloPrinter::print(index, out);
            name = makeName(out.str());
        }
        return insertAtInsertionPoint(new Extract(array, index, name));
    }
    std::string tmp;
    raw_string_ostream out(tmp);
    out << "cannot extract element from ";
    array->print(out);
    out << ": type is not a valid ArrayType";
    throw std::runtime_error(out.str());
}

And * PabloBlock::createAnd(PabloAST * expr1, PabloAST * expr2, String * name) {
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

Assign * PabloBlock::createAssign(PabloAST * const var, PabloAST * const value) {
    return insertAtInsertionPoint(new Assign(var, value));
}

MatchStar * PabloBlock::createMatchStar(PabloAST * marker, PabloAST * charclass, String * name) {
    if (name == nullptr) {
        name = makeName("matchstar");
    }
    return insertAtInsertionPoint(new MatchStar(marker, charclass, name));
}

ScanThru * PabloBlock::createScanThru(PabloAST * from, PabloAST * thru, String * name) {
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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enumerateScopes
 *
 * Assign sequential scope indexes, returning the next unassigned index
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned PabloBlock::enumerateScopes(unsigned baseScopeIndex) {
    mScopeIndex = baseScopeIndex;
    unsigned nextScopeIndex = baseScopeIndex + 1;
    for (Statement * stmt : *this) {
        if (If * ifStatement = dyn_cast<If>(stmt)) {
            nextScopeIndex = ifStatement->getBody()->enumerateScopes(nextScopeIndex);
        }
        else if (While * whileStatement = dyn_cast<While>(stmt)) {
            nextScopeIndex = whileStatement->getBody()->enumerateScopes(nextScopeIndex);
        }
    }
    return nextScopeIndex;
}    
    
/// CONSTRUCTOR

PabloBlock::PabloBlock(PabloFunction * const parent) noexcept
: PabloAST(PabloAST::ClassTypeId::Block, nullptr, nullptr)
, mParent(parent)
, mBranch(nullptr)
, mScopeIndex(0)
{

}

PabloBlock::~PabloBlock() {

}

}
