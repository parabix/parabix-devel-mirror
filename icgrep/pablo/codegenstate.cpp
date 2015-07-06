/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/codegenstate.h>

namespace pablo {

inline PabloAST * PabloBlock::renameNonNamedNode(PabloAST * expr, const std::string && prefix) {
    if (Statement * stmt = dyn_cast<Statement>(expr)) {
        if (stmt->getName()->isGenerated()) {
            stmt->setName(makeName(prefix, false));
        }
    }
    return expr;
}

/// UNARY CREATE FUNCTIONS

Assign * PabloBlock::createAssign(const std::string && prefix, PabloAST * expr, const int outputIndex)  {
    return insertAtInsertionPoint(new Assign(expr, outputIndex, makeName(prefix, false), this));
}

PabloAST * PabloBlock::createAdvance(PabloAST * expr, PabloAST * shiftAmount) {
    if (isa<Zeroes>(expr) || cast<Integer>(shiftAmount)->value() == 0) {
        return expr;
    }
    return insertAtInsertionPoint(new Advance(expr, shiftAmount, makeName("advance"), this));
}

PabloAST * PabloBlock::createAdvance(PabloAST * expr, PabloAST * shiftAmount, const std::string prefix) {
    if (isa<Zeroes>(expr) || cast<Integer>(shiftAmount)->value() == 0) {
        return expr;
    }
    return insertAtInsertionPoint(new Advance(expr, shiftAmount, makeName(prefix, false), this));
}

PabloAST * PabloBlock::createAdvance(PabloAST * expr, const Integer::integer_t shiftAmount) {
    if (isa<Zeroes>(expr) || shiftAmount == 0) {
        return expr;
    }
    return insertAtInsertionPoint(new Advance(expr, getInteger(shiftAmount), makeName("advance"), this));
}

PabloAST * PabloBlock::createAdvance(PabloAST * expr, const Integer::integer_t shiftAmount, const std::string prefix) {
    if (isa<Zeroes>(expr) || shiftAmount == 0) {
        return renameNonNamedNode(expr, std::move(prefix));
    }    
    return insertAtInsertionPoint(new Advance(expr, getInteger(shiftAmount), makeName(prefix, false), this));
}

Call * PabloBlock::createCall(PabloAST * name) {
    assert (name);
    return insertAtInsertionPoint(new Call(name, this));
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
        return not1->getExpr();
    }
    return insertAtInsertionPoint(new Not(expr, makeName("not_"), this));
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
        return renameNonNamedNode(not1->getExpr(), std::move(prefix));
    }
    return insertAtInsertionPoint(new Not(expr, makeName(prefix, false), this));
}

Var * PabloBlock::createVar(PabloAST * name) {
    assert (name);
    return new Var(name, this);
}

/// BINARY CREATE FUNCTIONS

Next * PabloBlock::createNext(Assign * assign, PabloAST * expr) {
    assert (assign && expr);
    assert (assign->getName());
    std::string name = assign->getName()->to_string() + "'";
    return new Next(assign, createAssign(std::move(name), expr), this);
}

PabloAST * PabloBlock::createMatchStar(PabloAST * marker, PabloAST * charclass) {
    assert (marker && charclass);
    if (isa<Zeroes>(marker) || isa<Zeroes>(charclass)) {
        return marker;
    }
    return insertAtInsertionPoint(new MatchStar(marker, charclass, makeName("matchstar"), this));
}

PabloAST * PabloBlock::createMatchStar(PabloAST * marker, PabloAST * charclass, const std::string prefix) {
    assert (marker && charclass);
    if (isa<Zeroes>(marker) || isa<Zeroes>(charclass)) {
        return renameNonNamedNode(marker, std::move(prefix));
    }
    return insertAtInsertionPoint(new MatchStar(marker, charclass, makeName(prefix, false), this));
}

PabloAST * PabloBlock::createScanThru(PabloAST * from, PabloAST * thru) {
    assert (from && thru);
    if (isa<Zeroes>(from) || isa<Zeroes>(thru)) {
        return from;
    }
    return insertAtInsertionPoint(new ScanThru(from, thru, makeName("scanthru"), this));
}

PabloAST * PabloBlock::createScanThru(PabloAST * from, PabloAST * thru, const std::string prefix) {
    assert (from && thru);
    if (isa<Zeroes>(from) || isa<Zeroes>(thru)) {        
        return renameNonNamedNode(from, std::move(prefix));
    }
    return insertAtInsertionPoint(new ScanThru(from, thru, makeName(prefix, false), this));
}

PabloAST * PabloBlock::createAnd(PabloAST * expr1, PabloAST * expr2) {
    assert (expr1 && expr2);
    if (isa<Zeroes>(expr2) || isa<Ones>(expr1)) {
        return expr2;
    }
    else if (isa<Zeroes>(expr1) || isa<Ones>(expr2) || equals(expr1, expr2)){
        return expr1;
    }
    else if (Not * not1 = dyn_cast<Not>(expr1)) {
        if (Not * not2 = dyn_cast<Not>(expr2)) {
            return createNot(createOr(not1->getExpr(), not2->getExpr()));
        }
        else if (equals(not1->getExpr(), expr2)) {
            return createZeroes();
        }
    }
    else if (Not * not2 = dyn_cast<Not>(expr2)) {
        if (equals(expr1, not2->getExpr())) {
            return createZeroes();
        }
    }
    if (isa<Not>(expr1)) {
        std::swap(expr1, expr2);
    }
    return insertAtInsertionPoint(new And(expr1, expr2, makeName("and_"), this));
}


PabloAST * PabloBlock::createAnd(PabloAST * expr1, PabloAST * expr2, const std::string prefix) {
    assert (expr1 && expr2);
    if (isa<Zeroes>(expr2) || isa<Ones>(expr1)) {
        return renameNonNamedNode(expr2, std::move(prefix));
    }
    else if (isa<Zeroes>(expr1) || isa<Ones>(expr2) || equals(expr1, expr2)){
        return renameNonNamedNode(expr1, std::move(prefix));
    }
    else if (Not * not1 = dyn_cast<Not>(expr1)) {
        if (Not * not2 = dyn_cast<Not>(expr2)) {
            return createNot(createOr(not1->getExpr(), not2->getExpr()), prefix);
        }
        else if (equals(not1->getExpr(), expr2)) {
            return createZeroes();
        }
    }
    else if (Not * not2 = dyn_cast<Not>(expr2)) {
        if (equals(expr1, not2->getExpr())) {
            return createZeroes();
        }
    }
    if (isa<Not>(expr1)) {
        std::swap(expr1, expr2);
    }
    return insertAtInsertionPoint(new And(expr1, expr2, makeName(prefix, false), this));
}

PabloAST * PabloBlock::createOr(PabloAST * expr1, PabloAST * expr2) {
    assert (expr1 && expr2);
    if (isa<Zeroes>(expr1) || isa<Ones>(expr2)){
        return expr2;
    }
    if (isa<Zeroes>(expr2) || isa<Ones>(expr1) || equals(expr1, expr2)) {
        return expr1;
    }
    else if (Not * not1 = dyn_cast<Not>(expr1)) {
        // ¬a∨b = ¬¬(¬a∨b) = ¬(a ∧ ¬b)
        return createNot(createAnd(not1->getExpr(), createNot(expr2)));
    }
    else if (Not * not2 = dyn_cast<Not>(expr2)) {
        // a∨¬b = ¬¬(¬b∨a) = ¬(b ∧ ¬a)
        return createNot(createAnd(not2->getExpr(), createNot(expr1)));
    }
    else if (equals(expr1, expr2)) {
        return expr1;
    }
    else if (And * and_expr1 = dyn_cast<And>(expr1)) {
        if (And * and_expr2 = dyn_cast<And>(expr2)) {
            PabloAST * const expr1a = and_expr1->getExpr1();
            PabloAST * const expr1b = and_expr1->getExpr2();
            PabloAST * const expr2a = and_expr2->getExpr1();
            PabloAST * const expr2b = and_expr2->getExpr2();
            //These optimizations factor out common components that can occur when sets are formed by union
            //(e.g., union of [a-z] and [A-Z].
            if (equals(expr1a, expr2a)) {
                return createAnd(expr1a, createOr(expr1b, expr2b));
            }
            else if (equals(expr1b, expr2b)) {
                return createAnd(expr1b, createOr(expr1a, expr2a));
            }
            else if (equals(expr1a, expr2b)) {
                return createAnd(expr1a, createOr(expr1b, expr2a));
            }
            else if (equals(expr1b, expr2a)) {
                return createAnd(expr1b, createOr(expr1a, expr2b));
            }
        }
    }
    return insertAtInsertionPoint(new Or(expr1, expr2, makeName("or_"), this));
}

PabloAST * PabloBlock::createOr(PabloAST * expr1, PabloAST * expr2, const std::string prefix) {
    assert (expr1 && expr2);
    if (isa<Zeroes>(expr1) || isa<Ones>(expr2)){
        return renameNonNamedNode(expr2, std::move(prefix));
    }
    if (isa<Zeroes>(expr2) || isa<Ones>(expr1) || equals(expr1, expr2)) {
        return renameNonNamedNode(expr1, std::move(prefix));
    }
    else if (Not * not1 = dyn_cast<Not>(expr1)) {
        // ¬a∨b = ¬¬(¬a∨b) = ¬(a ∧ ¬b)
        return createNot(createAnd(not1->getExpr(), createNot(expr2)), prefix);
    }
    else if (Not * not2 = dyn_cast<Not>(expr2)) {
        // a∨¬b = ¬¬(¬b∨a) = ¬(b ∧ ¬a)
        return createNot(createAnd(not2->getExpr(), createNot(expr1)), prefix);
    }
    else if (And * and_expr1 = dyn_cast<And>(expr1)) {
        if (And * and_expr2 = dyn_cast<And>(expr2)) {
            PabloAST * const expr1a = and_expr1->getExpr1();
            PabloAST * const expr1b = and_expr1->getExpr2();
            PabloAST * const expr2a = and_expr2->getExpr1();
            PabloAST * const expr2b = and_expr2->getExpr2();
            //These optimizations factor out common components that can occur when sets are formed by union
            //(e.g., union of [a-z] and [A-Z].
            if (equals(expr1a, expr2a)) {
                return createAnd(expr1a, createOr(expr1b, expr2b), prefix);
            }
            else if (equals(expr1b, expr2b)) {
                return createAnd(expr1b, createOr(expr1a, expr2a), prefix);
            }
            else if (equals(expr1a, expr2b)) {
                return createAnd(expr1a, createOr(expr1b, expr2a), prefix);
            }
            else if (equals(expr1b, expr2a)) {
                return createAnd(expr1b, createOr(expr1a, expr2b), prefix);
            }
        }
    }
    return insertAtInsertionPoint(new Or(expr1, expr2, makeName(prefix, false), this));
}

PabloAST * PabloBlock::createXor(PabloAST * expr1, PabloAST * expr2) {
    assert (expr1 && expr2);
    if (isa<Ones>(expr1)) {
        return createNot(expr2);
    }
    else if (isa<Zeroes>(expr1)){
        return expr2;
    }
    else if (isa<Ones>(expr2)) {
        return createNot(expr1);
    }
    else if (isa<Zeroes>(expr2)){
        return expr1;
    }
    else if (Not * not1 = dyn_cast<Not>(expr1)) {
        if (Not * not2 = dyn_cast<Not>(expr2)) {
            return createXor(not1->getExpr(), not2->getExpr());
        }
    }
    return insertAtInsertionPoint(new Xor(expr1, expr2, makeName("xor_"), this));
}

PabloAST * PabloBlock::createXor(PabloAST * expr1, PabloAST * expr2, const std::string prefix) {
    assert (expr1 && expr2);
    if (isa<Ones>(expr1)) {
        return createNot(expr2, prefix);
    }
    else if (isa<Zeroes>(expr1)){
        return expr2;
    }
    else if (isa<Ones>(expr2)) {
        return createNot(expr1, prefix);
    }
    else if (isa<Zeroes>(expr2)){
        return expr1;
    }
    else if (Not * not1 = dyn_cast<Not>(expr1)) {
        if (Not * not2 = dyn_cast<Not>(expr2)) {
            return createXor(not1->getExpr(), not2->getExpr(), prefix);
        }
    }
    return insertAtInsertionPoint(new Xor(expr1, expr2, makeName(prefix, false), this));
}

/// TERNARY CREATE FUNCTION

PabloAST * PabloBlock::createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr) {
    assert (condition && trueExpr && falseExpr);

    if (isa<Ones>(condition)) {
        return trueExpr;
    }
    else if (isa<Zeroes>(condition)){
        return falseExpr;
    }
    else if (isa<Ones>(trueExpr)) {
        return createOr(condition, falseExpr);
    }
    else if (isa<Zeroes>(trueExpr)){
        return createAnd(createNot(condition), falseExpr);
    }
    else if (isa<Ones>(falseExpr)) {
        return createOr(createNot(condition), trueExpr);
    }
    else if (isa<Zeroes>(falseExpr)){
        return createAnd(condition, trueExpr);
    }
    else if (equals(trueExpr, falseExpr)) {
        return trueExpr;
    }
    else if (isa<Not>(trueExpr) && equals(cast<Not>(trueExpr)->getExpr(), falseExpr)) {
        return createXor(condition, falseExpr);
    }
    else if (isa<Not>(falseExpr) && equals(trueExpr, cast<Not>(falseExpr)->getExpr())){
        return createXor(condition, falseExpr);
    }
    return insertAtInsertionPoint(new Sel(condition, trueExpr, falseExpr, makeName("sel"), this));
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
    else if (isa<Not>(trueExpr) && equals(cast<Not>(trueExpr)->getExpr(), falseExpr)) {
        return createXor(condition, falseExpr, prefix);
    }
    else if (isa<Not>(falseExpr) && equals(trueExpr, cast<Not>(falseExpr)->getExpr())){
        return createXor(condition, falseExpr, prefix);
    }
    return insertAtInsertionPoint(new Sel(condition, trueExpr, falseExpr, makeName(prefix, false), this));
}

If * PabloBlock::createIf(PabloAST * condition, const std::initializer_list<Assign *> definedVars, PabloBlock & body) {
    assert (condition);
    return insertAtInsertionPoint(new If(condition, definedVars, body, this));
}

If * PabloBlock::createIf(PabloAST * condition, const std::vector<Assign *> & definedVars, PabloBlock & body) {
    assert (condition);
    return insertAtInsertionPoint(new If(condition, definedVars, body, this));
}

If * PabloBlock::createIf(PabloAST * condition, std::vector<Assign *> && definedVars, PabloBlock & body) {
    assert (condition);
    return insertAtInsertionPoint(new If(condition, definedVars, body, this));
}

While * PabloBlock::createWhile(PabloAST * condition, const std::initializer_list<Next *> nextVars, PabloBlock & body) {
    assert (condition);
    return insertAtInsertionPoint(new While(condition, nextVars, body, this));
}

While * PabloBlock::createWhile(PabloAST * condition, const std::vector<Next *> & nextVars, PabloBlock & body) {
    assert (condition);
    return insertAtInsertionPoint(new While(condition, nextVars, body, this));
}

While * PabloBlock::createWhile(PabloAST * condition, std::vector<Next *> && nextVars, PabloBlock & body) {
    assert (condition);
    return insertAtInsertionPoint(new While(condition, nextVars, body, this));
}

/// CONSTRUCTOR

PabloBlock::PabloBlock(SymbolGenerator & symbolGenerator)
: PabloAST(PabloAST::ClassTypeId::Block)
, mZeroes(new Zeroes())
, mOnes(new Ones())
, mSymbolGenerator(symbolGenerator)
, mParent(nullptr)
{

}

PabloBlock::PabloBlock(PabloBlock * predecessor)
: PabloAST(PabloAST::ClassTypeId::Block)
, mZeroes(predecessor->mZeroes) // inherit the original "Zeroes" variable for simplicity
, mOnes(predecessor->mOnes) // inherit the original "Ones" variable for simplicity
, mSymbolGenerator(predecessor->mSymbolGenerator)
, mParent(predecessor) {

}

PabloBlock::~PabloBlock() {

}

}
