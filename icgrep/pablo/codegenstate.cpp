/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/codegenstate.h>
#include <iostream>
#include <pablo/printer_pablos.h>

namespace pablo {

Zeroes PabloBlock::mZeroes;

Ones PabloBlock::mOnes;

inline PabloAST * PabloBlock::renameNonNamedNode(PabloAST * expr, const std::string && prefix) {
    if (Statement * stmt = dyn_cast<Statement>(expr)) {
        if (stmt->getName()->isGenerated()) {
            stmt->setName(makeName(prefix, false));
        }
    }
    return expr;
}

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

/// UNARY CREATE FUNCTIONS

Assign * PabloBlock::createAssign(const std::string && prefix, PabloAST * const expr)  {
    assert ("Assign expression cannot be null!" && expr);
    return insertAtInsertionPoint(new Assign(expr, makeName(prefix, false)));
}

PabloAST * PabloBlock::createAdvance(PabloAST * expr, PabloAST * shiftAmount) {
    if (isa<Zeroes>(expr) || cast<Integer>(shiftAmount)->value() == 0) {
        return expr;
    }
    return insertAtInsertionPoint(new Advance(expr, shiftAmount, makeName("advance")));
}

PabloAST * PabloBlock::createAdvance(PabloAST * expr, PabloAST * shiftAmount, const std::string prefix) {
    if (isa<Zeroes>(expr) || cast<Integer>(shiftAmount)->value() == 0) {
        return expr;
    }
    return insertAtInsertionPoint(new Advance(expr, shiftAmount, makeName(prefix, false)));
}

PabloAST * PabloBlock::createAdvance(PabloAST * expr, const Integer::Type shiftAmount) {
    if (isa<Zeroes>(expr) || shiftAmount == 0) {
        return expr;
    }
    return insertAtInsertionPoint(new Advance(expr, getInteger(shiftAmount), makeName("advance")));
}

PabloAST * PabloBlock::createAdvance(PabloAST * expr, const Integer::Type shiftAmount, const std::string prefix) {
    if (isa<Zeroes>(expr) || shiftAmount == 0) {
        return renameNonNamedNode(expr, std::move(prefix));
    }
    return insertAtInsertionPoint(new Advance(expr, getInteger(shiftAmount), makeName(prefix, false)));
}

PabloAST * PabloBlock::createLookahead(PabloAST * expr, PabloAST * shiftAmount) {
    if (isa<Zeroes>(expr) || cast<Integer>(shiftAmount)->value() == 0) {
        return expr;
    }
    return insertAtInsertionPoint(new Lookahead(expr, shiftAmount, makeName("lookahead")));
}

PabloAST * PabloBlock::createLookahead(PabloAST * expr, PabloAST * shiftAmount, const std::string prefix) {
    if (isa<Zeroes>(expr) || cast<Integer>(shiftAmount)->value() == 0) {
        return expr;
    }
    return insertAtInsertionPoint(new Lookahead(expr, shiftAmount, makeName(prefix, false)));
}

PabloAST * PabloBlock::createLookahead(PabloAST * expr, const Integer::Type shiftAmount) {
    if (isa<Zeroes>(expr) || shiftAmount == 0) {
        return expr;
    }
    return insertAtInsertionPoint(new Lookahead(expr, getInteger(shiftAmount), makeName("lookahead")));
}

PabloAST * PabloBlock::createLookahead(PabloAST * expr, const Integer::Type shiftAmount, const std::string prefix) {
    if (isa<Zeroes>(expr) || shiftAmount == 0) {
        return renameNonNamedNode(expr, std::move(prefix));
    }
    return insertAtInsertionPoint(new Lookahead(expr, getInteger(shiftAmount), makeName(prefix, false)));
}

Call * PabloBlock::createCall(PabloAST * prototype, const std::vector<PabloAST *> &) {
    assert (prototype);
    return insertAtInsertionPoint(new Call(prototype));
}


PabloAST * PabloBlock::createNot(PabloAST * expr) {
    assert (expr);
    if (isa<Ones>(expr)) {
        return createZeroes();
    }
    else if (isa<Zeroes>(expr)){
        return createOnes();
    }
    else if (Not * not1 = dyn_cast<Not>(expr)) {
        return not1->getOperand(0);
    }
    return insertAtInsertionPoint(new Not(expr, makeName("not_")));
}

PabloAST * PabloBlock::createNot(PabloAST * expr, const std::string prefix) {
    assert (expr);
    if (isa<Ones>(expr)) {
        return createZeroes();
    }
    else if (isa<Zeroes>(expr)){
        return createOnes();
    }
    else if (Not * not1 = dyn_cast<Not>(expr)) {        
        return renameNonNamedNode(not1->getOperand(0), std::move(prefix));
    }
    return insertAtInsertionPoint(new Not(expr, makeName(prefix, false)));
}

Var * PabloBlock::createVar(PabloAST * name) {
    assert (name);
    return new Var(name);
}

PabloAST * PabloBlock::createCount(PabloAST * expr) {
    assert (expr);
    return insertAtInsertionPoint(new Count(expr, makeName("count_")));
}

PabloAST * PabloBlock::createCount(PabloAST * expr, const std::string prefix) {
    assert (expr);
    return insertAtInsertionPoint(new Count(expr, makeName(prefix, false)));
}

    
/// BINARY CREATE FUNCTIONS

Next * PabloBlock::createNext(Assign * assign, PabloAST * expr) {
    assert (assign && expr);
    return insertAtInsertionPoint(new Next(assign, expr));
}

PabloAST * PabloBlock::createMatchStar(PabloAST * marker, PabloAST * charclass) {
    assert (marker && charclass);
    if (isa<Zeroes>(marker) || isa<Zeroes>(charclass)) {
        return marker;
    }
    return insertAtInsertionPoint(new MatchStar(marker, charclass, makeName("matchstar")));
}

PabloAST * PabloBlock::createMatchStar(PabloAST * marker, PabloAST * charclass, const std::string prefix) {
    assert (marker && charclass);
    if (isa<Zeroes>(marker) || isa<Zeroes>(charclass)) {
        return renameNonNamedNode(marker, std::move(prefix));
    }
    return insertAtInsertionPoint(new MatchStar(marker, charclass, makeName(prefix, false)));
}

PabloAST * PabloBlock::createScanThru(PabloAST * from, PabloAST * thru) {
    assert (from && thru);
    if (isa<Zeroes>(from) || isa<Zeroes>(thru)) {
        return from;
    }
    return insertAtInsertionPoint(new ScanThru(from, thru, makeName("scanthru")));
}

PabloAST * PabloBlock::createScanThru(PabloAST * from, PabloAST * thru, const std::string prefix) {
    assert (from && thru);
    if (isa<Zeroes>(from) || isa<Zeroes>(thru)) {        
        return renameNonNamedNode(from, std::move(prefix));
    }
    return insertAtInsertionPoint(new ScanThru(from, thru, makeName(prefix, false)));
}

template<typename Type>
static inline Type * isBinary(PabloAST * expr) {
    if (isa<Type>(expr) && cast<Type>(expr)->getNumOperands() == 2) {
        return cast<Type>(expr);
    }
    return nullptr;
}

PabloAST * PabloBlock::createAnd(PabloAST * expr1, PabloAST * expr2) {
    assert (expr1 && expr2);
    if (isa<Zeroes>(expr2) || isa<Ones>(expr1)) {
        return expr2;
    } else if (isa<Zeroes>(expr1) || isa<Ones>(expr2) || equals(expr1, expr2)){
        return expr1;
    } else if (Not * not1 = dyn_cast<Not>(expr1)) {
        if (Not * not2 = dyn_cast<Not>(expr2)) {
            return createNot(createOr(not1->getOperand(0), not2->getOperand(0)));
        } else if (equals(not1->getOperand(0), expr2)) {
            return createZeroes();
        }
    } else if (Not * not2 = dyn_cast<Not>(expr2)) {
        if (equals(expr1, not2->getOperand(0))) {
            return createZeroes();
        }
    } else if (Or * or1 = isBinary<Or>(expr1)) {
        if (equals(or1->getOperand(0), expr2) || equals(or1->getOperand(1), expr2)) {
            return expr2;
        }
    } else if (Or * or2 = isBinary<Or>(expr2)) {
        if (equals(or2->getOperand(0), expr1) || equals(or2->getOperand(1), expr1)) {
            return expr1;
        }
    }
    return insertAtInsertionPoint(new And(expr1, expr2, makeName("and_")));
}

PabloAST * PabloBlock::createAnd(PabloAST * expr1, PabloAST * expr2, const std::string prefix) {
    assert (expr1 && expr2);
    if (isa<Zeroes>(expr2) || isa<Ones>(expr1)) {
        return renameNonNamedNode(expr2, std::move(prefix));
    }
    else if (isa<Zeroes>(expr1) || isa<Ones>(expr2) || equals(expr1, expr2)){
        return renameNonNamedNode(expr1, std::move(prefix));
    } else if (Not * not1 = dyn_cast<Not>(expr1)) {
        if (Not * not2 = dyn_cast<Not>(expr2)) {
            return createNot(createOr(not1->getOperand(0), not2->getOperand(0)), prefix);
        }
        else if (equals(not1->getOperand(0), expr2)) {
            return createZeroes();
        }
    } else if (Not * not2 = dyn_cast<Not>(expr2)) {
        if (equals(expr1, not2->getOperand(0))) {
            return createZeroes();
        }
    } else if (Or * or1 = isBinary<Or>(expr1)) {
        if (equals(or1->getOperand(0), expr2) || equals(or1->getOperand(1), expr2)) {
            return expr2;
        }
    } else if (Or * or2 = isBinary<Or>(expr2)) {
        if (equals(or2->getOperand(0), expr1) || equals(or2->getOperand(1), expr1)) {
            return expr1;
        }
    }
    return insertAtInsertionPoint(new And(expr1, expr2, makeName(prefix, false)));
}

And * PabloBlock::createAnd(const unsigned reserved) {
    return insertAtInsertionPoint(new And(reserved, makeName("and_")));
}

And * PabloBlock::createAnd(const unsigned reserved, const std::string prefix) {
    return insertAtInsertionPoint(new And(reserved, makeName(prefix, false)));
}

PabloAST * PabloBlock::createOr(PabloAST * expr1, PabloAST * expr2) {
    assert (expr1 && expr2);
    if (isa<Zeroes>(expr1) || isa<Ones>(expr2)){
        return expr2;
    }
    if (isa<Zeroes>(expr2) || isa<Ones>(expr1) || equals(expr1, expr2)) {
        return expr1;
    } else if (Not * not1 = dyn_cast<Not>(expr1)) {
        // ¬a∨b = ¬¬(¬a ∨ b) = ¬(a ∧ ¬b)
        return createNot(createAnd(not1->getOperand(0), createNot(expr2)));
    } else if (Not * not2 = dyn_cast<Not>(expr2)) {
        // a∨¬b = ¬¬(¬b ∨ a) = ¬(b ∧ ¬a)
        return createNot(createAnd(not2->getOperand(0), createNot(expr1)));
    } else if (equals(expr1, expr2)) {
        return expr1;
    } else if (And * and1 = isBinary<And>(expr1)) {
        if (And * and2 = isBinary<And>(expr2)) {
            PabloAST * const expr1a = and1->getOperand(0);
            PabloAST * const expr1b = and1->getOperand(1);
            PabloAST * const expr2a = and2->getOperand(0);
            PabloAST * const expr2b = and2->getOperand(1);
            //These optimizations factor out common components that can occur when sets are formed by union
            //(e.g., union of [a-z] and [A-Z].
            if (equals(expr1a, expr2a)) {
                return createAnd(expr1a, createOr(expr1b, expr2b));
            } else if (equals(expr1b, expr2b)) {
                return createAnd(expr1b, createOr(expr1a, expr2a));
            } else if (equals(expr1a, expr2b)) {
                return createAnd(expr1a, createOr(expr1b, expr2a));
            } else if (equals(expr1b, expr2a)) {
                return createAnd(expr1b, createOr(expr1a, expr2b));
            }
        } else if (equals(and1->getOperand(0), expr2) || equals(and1->getOperand(1), expr2)) {
            // (a ∧ b) ∨ a = a
            return expr2;
        }
    } else if (And * and2 = isBinary<And>(expr2)) {
        if (equals(and2->getOperand(0), expr1) || equals(and2->getOperand(1), expr1)) {
            return expr1;
        }
    }
    return insertAtInsertionPoint(new Or(expr1, expr2, makeName("or_")));
}

PabloAST * PabloBlock::createOr(PabloAST * expr1, PabloAST * expr2, const std::string prefix) {
    assert (expr1 && expr2);
    if (isa<Zeroes>(expr1) || isa<Ones>(expr2)){
        return renameNonNamedNode(expr2, std::move(prefix));
    }
    else if (isa<Zeroes>(expr2) || isa<Ones>(expr1) || equals(expr1, expr2)) {
        return renameNonNamedNode(expr1, std::move(prefix));
    } else if (Not * not1 = dyn_cast<Not>(expr1)) {
        // ¬a∨b = ¬¬(¬a∨b) = ¬(a ∧ ¬b)
        return createNot(createAnd(not1->getOperand(0), createNot(expr2)), prefix);
    } else if (Not * not2 = dyn_cast<Not>(expr2)) {
        // a∨¬b = ¬¬(¬b∨a) = ¬(b ∧ ¬a)
        return createNot(createAnd(not2->getOperand(0), createNot(expr1)), prefix);
    } else if (And * and1 = isBinary<And>(expr1)) {
        if (And * and2 = isBinary<And>(expr2)) {
            PabloAST * const expr1a = and1->getOperand(0);
            PabloAST * const expr1b = and1->getOperand(1);
            PabloAST * const expr2a = and2->getOperand(0);
            PabloAST * const expr2b = and2->getOperand(1);
            //These optimizations factor out common components that can occur when sets are formed by union
            //(e.g., union of [a-z] and [A-Z].
            if (equals(expr1a, expr2a)) {
                return createAnd(expr1a, createOr(expr1b, expr2b), prefix);
            } else if (equals(expr1b, expr2b)) {
                return createAnd(expr1b, createOr(expr1a, expr2a), prefix);
            } else if (equals(expr1a, expr2b)) {
                return createAnd(expr1a, createOr(expr1b, expr2a), prefix);
            } else if (equals(expr1b, expr2a)) {
                return createAnd(expr1b, createOr(expr1a, expr2b), prefix);
            }
        } else if (equals(and1->getOperand(0), expr2) || equals(and1->getOperand(1), expr2)) {
            // (a∧b) ∨ a = a
            return expr2;
        }
    } else if (And * and2 = isBinary<And>(expr2)) {
        if (equals(and2->getOperand(0), expr1) || equals(and2->getOperand(1), expr1)) {
            return expr1;
        }
    }
    return insertAtInsertionPoint(new Or(expr1, expr2, makeName(prefix, false)));
}

Or * PabloBlock::createOr(const unsigned reserved) {
    return insertAtInsertionPoint(new Or(reserved, makeName("or_")));
}

Or * PabloBlock::createOr(const unsigned reserved, const std::string prefix) {
    return insertAtInsertionPoint(new Or(reserved, makeName(prefix, false)));
}

PabloAST * PabloBlock::createXor(PabloAST * expr1, PabloAST * expr2) {
    assert (expr1 && expr2);
    if (expr1 == expr2) {
        return PabloBlock::createZeroes();
    }
    if (isa<Ones>(expr1)) {
        return createNot(expr2);
    } else if (isa<Zeroes>(expr1)){
        return expr2;
    } else if (isa<Ones>(expr2)) {
        return createNot(expr1);
    } else if (isa<Zeroes>(expr2)){
        return expr1;
    } else if (Not * not1 = dyn_cast<Not>(expr1)) {
        if (Not * not2 = dyn_cast<Not>(expr2)) {
            return createXor(not1->getOperand(0), not2->getOperand(0));
        }
    }
    return insertAtInsertionPoint(new Xor(expr1, expr2, makeName("xor_")));
}

PabloAST * PabloBlock::createXor(PabloAST * expr1, PabloAST * expr2, const std::string prefix) {
    assert (expr1 && expr2);
    if (expr1 == expr2) {
        return PabloBlock::createZeroes();
    }
    if (isa<Ones>(expr1)) {
        return createNot(expr2, prefix);
    } else if (isa<Zeroes>(expr1)){
        return expr2;
    } else if (isa<Ones>(expr2)) {
        return createNot(expr1, prefix);
    } else if (isa<Zeroes>(expr2)){
        return expr1;
    } else if (Not * not1 = dyn_cast<Not>(expr1)) {
        if (Not * not2 = dyn_cast<Not>(expr2)) {
            return createXor(not1->getOperand(0), not2->getOperand(0), prefix);
        }
    }
    return insertAtInsertionPoint(new Xor(expr1, expr2, makeName(prefix, false)));
}

Xor * PabloBlock::createXor(const unsigned reserved) {
    return insertAtInsertionPoint(new Xor(reserved, makeName("xor_")));
}

Xor * PabloBlock::createXor(const unsigned reserved, const std::string prefix) {
    return insertAtInsertionPoint(new Xor(reserved, makeName(prefix, false)));
}

/// TERNARY CREATE FUNCTION

PabloAST * PabloBlock::createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr) {
    assert (condition && trueExpr && falseExpr);
    if (isa<Ones>(condition)) {
        return trueExpr;
    } else if (isa<Zeroes>(condition)){
        return falseExpr;
    } else if (isa<Ones>(trueExpr)) {
        return createOr(condition, falseExpr);
    } else if (isa<Zeroes>(trueExpr)){
        return createAnd(createNot(condition), falseExpr);
    } else if (isa<Ones>(falseExpr)) {
        return createOr(createNot(condition), trueExpr);
    } else if (isa<Zeroes>(falseExpr)){
        return createAnd(condition, trueExpr);
    } else if (equals(trueExpr, falseExpr)) {
        return trueExpr;
    } else if (isa<Not>(trueExpr) && equals(cast<Not>(trueExpr)->getOperand(0), falseExpr)) {
        return createXor(condition, falseExpr);
    } else if (isa<Not>(falseExpr) && equals(trueExpr, cast<Not>(falseExpr)->getOperand(0))){
        return createXor(condition, trueExpr);
    }
    return insertAtInsertionPoint(new Sel(condition, trueExpr, falseExpr, makeName("sel")));
}

PabloAST * PabloBlock::createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr, const std::string prefix) {
    assert (condition && trueExpr && falseExpr);
    if (isa<Zeroes>(condition)){
        return renameNonNamedNode(falseExpr, std::move(prefix));
    }
    else if (isa<Ones>(condition) || equals(trueExpr, falseExpr)) {
        return renameNonNamedNode(trueExpr, std::move(prefix));
    }
    else if (isa<Ones>(trueExpr)) {
        return createOr(condition, falseExpr, prefix);
    }
    else if (isa<Zeroes>(trueExpr)){
        return createAnd(createNot(condition), falseExpr, prefix);
    }
    else if (isa<Ones>(falseExpr)) {
        return createOr(createNot(condition), trueExpr, prefix);
    }
    else if (isa<Zeroes>(falseExpr)){
        return createAnd(condition, trueExpr, prefix);
    }
    else if (isa<Not>(trueExpr) && equals(cast<Not>(trueExpr)->getOperand(0), falseExpr)) {
        return createXor(condition, falseExpr, prefix);
    }
    else if (isa<Not>(falseExpr) && equals(trueExpr, cast<Not>(falseExpr)->getOperand(0))){
        return createXor(condition, trueExpr, prefix);
    }
    return insertAtInsertionPoint(new Sel(condition, trueExpr, falseExpr, makeName(prefix, false)));
}

If * PabloBlock::createIf(PabloAST * condition, const std::initializer_list<Assign *> definedVars, PabloBlock * body) {
    assert (condition);
    return insertAtInsertionPoint(new If(condition, definedVars, body));
}

If * PabloBlock::createIf(PabloAST * condition, const std::vector<Assign *> & definedVars, PabloBlock * body) {
    assert (condition);
    return insertAtInsertionPoint(new If(condition, definedVars, body));
}

If * PabloBlock::createIf(PabloAST * condition, std::vector<Assign *> && definedVars, PabloBlock * body) {
    assert (condition);
    return insertAtInsertionPoint(new If(condition, definedVars, body));
}

While * PabloBlock::createWhile(PabloAST * condition, const std::initializer_list<Next *> nextVars, PabloBlock * body) {
    assert (condition);
    return insertAtInsertionPoint(new While(condition, nextVars, body));
}

While * PabloBlock::createWhile(PabloAST * condition, const std::vector<Next *> & nextVars, PabloBlock * body) {
    assert (condition);
    return insertAtInsertionPoint(new While(condition, nextVars, body));
}

While * PabloBlock::createWhile(PabloAST * condition, std::vector<Next *> && nextVars, PabloBlock * body) {
    assert (condition);
    return insertAtInsertionPoint(new While(condition, nextVars, body));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief eraseFromParent
 ** ------------------------------------------------------------------------------------------------------------- */
void PabloBlock::eraseFromParent(const bool recursively) {
    Statement * stmt = front();
    // Note: by erasing the scope block, any Assign/Next nodes will be replaced with Zero and removed from
    // the If/While node
    while (stmt) {
        stmt = stmt->eraseFromParent(recursively);
    }
    mAllocator.deallocate(reinterpret_cast<Allocator::pointer>(this));
}


// Assign sequential scope indexes, returning the next unassigned index    

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

PabloBlock::PabloBlock(SymbolGenerator * symbolGenerator) noexcept
: PabloAST(PabloAST::ClassTypeId::Block)
, mSymbolGenerator(symbolGenerator)
, mParent(nullptr)
, mBranch(nullptr)
, mScopeIndex(0)
{

}

PabloBlock::~PabloBlock() {

}

}
