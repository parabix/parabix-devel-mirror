/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/pabloAST.h>
#include <pablo/codegenstate.h>
#include <llvm/Support/Compiler.h>
#include <pablo/printer_pablos.h>

#ifndef NDEBUG
#include <queue>
#endif
namespace pablo {

PabloAST::Allocator PabloAST::mAllocator;
PabloAST::VectorAllocator PabloAST::mVectorAllocator;

/*

    Return true if expr1 and expr2 can be proven equivalent according to some rules,
    false otherwise.  Note that false may be returned i some cases when the exprs are
    equivalent.

*/

bool equals(const PabloAST * expr1, const PabloAST * expr2) {
    assert (expr1 && expr2);
    if (expr1->getClassTypeId() == expr2->getClassTypeId()) {
        if ((isa<Zeroes>(expr1)) || (isa<Ones>(expr1))) {
            return true;
        }
        else if (const Var * var1 = dyn_cast<const Var>(expr1)) {
            if (const Var * var2 = cast<const Var>(expr2)) {
                return (var1->getName() == var2->getName());
            }
        }
        else if (const Not* not1 = dyn_cast<const Not>(expr1)) {
            if (const Not* not2 = cast<const Not>(expr2)) {
                return equals(not1->getExpr(), not2->getExpr());
            }
        }
        else if (const And* and1 = dyn_cast<const And>(expr1)) {
            if (const And* and2 = cast<const And>(expr2)) {
                if (equals(and1->getExpr1(), and2->getExpr1())) {
                    return equals(and1->getExpr2(), and2->getExpr2());
                }
                else if (equals(and1->getExpr1(), and2->getExpr2())) {
                    return equals(and1->getExpr2(), and2->getExpr1());
                }
            }
        }
        else if (const Or * or1 = dyn_cast<const Or>(expr1)) {
            if (const Or* or2 = cast<const Or>(expr2)) {
                if (equals(or1->getExpr1(), or2->getExpr1())) {
                    return equals(or1->getExpr2(), or2->getExpr2());
                }
                else if (equals(or1->getExpr1(), or2->getExpr2())) {
                    return equals(or1->getExpr2(), or2->getExpr1());
                }
            }
        }
        else if (const Xor * xor1 = dyn_cast<const Xor>(expr1)) {
            if (const Xor * xor2 = cast<const Xor>(expr2)) {
                if (equals(xor1->getExpr1(), xor2->getExpr1())) {
                    return equals(xor1->getExpr2(), xor2->getExpr2());
                }
                else if (equals(xor1->getExpr1(), xor2->getExpr2())) {
                    return equals(xor1->getExpr2(), xor2->getExpr1());
                }
            }
        }
        else if (const Sel* sel1 = dyn_cast<const Sel>(expr1)) {
            if (const Sel* sel2 = cast<const Sel>(expr2)) {
                if (equals(sel1->getCondition(), sel2->getCondition())) {
                    if (equals(sel1->getTrueExpr(), sel2->getTrueExpr())) {
                        return equals(sel1->getFalseExpr(), sel2->getFalseExpr());
                    }
                }
            }
        }
    }
    return false;
}

void PabloAST::replaceAllUsesWith(PabloAST * expr) {    
    Statement * user[mUsers.size()];
    Users::size_type users = 0;
    for (PabloAST * u : mUsers) {
        if (isa<Statement>(u)) {
            user[users++] = cast<Statement>(u);
        }
    }
    mUsers.clear();
    assert (expr);
    for (auto i = 0; i != users; ++i) {
        user[i]->replaceUsesOfWith(this, expr);
    }
}

void Statement::setOperand(const unsigned index, PabloAST * const value) {
    assert (value);
    assert (index < getNumOperands());
    assert (noRecursiveOperand(value));
    if (LLVM_UNLIKELY(getOperand(index) == value)) {
        return;
    }
    PabloAST * priorValue = getOperand(index);
    // Test just to be sure that we don't have multiple operands pointing to
    // what we're replacing. If not, remove this from the prior value's
    // user list.
    unsigned count = 0;
    for (unsigned i = 0; i != getNumOperands(); ++i) {
        count += (getOperand(i) == priorValue) ? 1 : 0;
    }
    assert (count >= 1);
    if (LLVM_LIKELY(count == 1)) {
        priorValue->removeUser(this);
    }
    mOperand[index] = value;
    value->addUser(this);
}

void Statement::insertBefore(Statement * const statement) {
    if (LLVM_UNLIKELY(statement == this)) {
        return;
    }
    else if (LLVM_UNLIKELY(statement == nullptr)) {
        throw std::runtime_error("Cannot insert before Null statement!");
    }
    else if (LLVM_UNLIKELY(statement->mParent == nullptr)) {
        throw std::runtime_error("Cannot insert before before statement in Null AST!");
    }
    removeFromParent();
    mParent = statement->mParent;
    if (LLVM_UNLIKELY(mParent->mFirst == statement)) {
        mParent->mFirst = this;
    }
    mNext = statement;
    mPrev = statement->mPrev;
    statement->mPrev = this;
    if (LLVM_LIKELY(mPrev != nullptr)) {
        mPrev->mNext = this;
    }
    if (LLVM_UNLIKELY(isa<If>(this) || isa<While>(this))) {
        PabloBlock & body = isa<If>(this) ? cast<If>(this)->getBody() : cast<While>(this)->getBody();
        mParent->addUser(&body);
    }
}

void Statement::insertAfter(Statement * const statement) {
    if (LLVM_UNLIKELY(statement == this)) {
        return;
    }
    else if (LLVM_UNLIKELY(statement == nullptr)) {
        throw std::runtime_error("Cannot insert before Null statement!");
    }
    else if (LLVM_UNLIKELY(statement->mParent == nullptr)) {
        throw std::runtime_error("Cannot insert before before statement in Null AST!");
    }
    removeFromParent();
    mParent = statement->mParent;
    if (LLVM_UNLIKELY(mParent->mLast == statement)) {
        mParent->mLast = this;
    }
    mPrev = statement;
    mNext = statement->mNext;
    statement->mNext = this;
    if (LLVM_LIKELY(mNext != nullptr)) {
        mNext->mPrev = this;
    }
    if (LLVM_UNLIKELY(isa<If>(this) || isa<While>(this))) {
        PabloBlock & body = isa<If>(this) ? cast<If>(this)->getBody() : cast<While>(this)->getBody();
        mParent->addUser(&body);
    }
}

Statement * Statement::removeFromParent() {
    Statement * next = mNext;
    if (LLVM_LIKELY(mParent != nullptr)) {
        if (LLVM_UNLIKELY(mParent->mFirst == this)) {
            mParent->mFirst = mNext;
        }
        if (LLVM_UNLIKELY(mParent->mLast == this)) {
            mParent->mLast = mPrev;
        }
        if (LLVM_UNLIKELY(mParent->mInsertionPoint == this)) {
            mParent->mInsertionPoint = mPrev;
        }
        if (LLVM_LIKELY(mPrev != nullptr)) {
            mPrev->mNext = mNext;
        }
        if (LLVM_LIKELY(mNext != nullptr)) {
            mNext->mPrev = mPrev;
        }
        if (LLVM_UNLIKELY(isa<If>(this) || isa<While>(this))) {
            PabloBlock & body = isa<If>(this) ? cast<If>(this)->getBody() : cast<While>(this)->getBody();
            mParent->removeUser(&body);
        }
    }
    mPrev = nullptr;
    mNext = nullptr;
    mParent = nullptr;
    return next;
}

Statement * Statement::eraseFromParent(const bool recursively) {
    // remove this statement from its operands' users list
    for (auto i = 0; i != mOperands; ++i) {
        mOperand[i]->removeUser(this);
    }
    // If this is an If or While statement, we'll have to remove the statements within the
    // body or we'll lose track of them.
    if (LLVM_UNLIKELY(isa<If>(this) || isa<While>(this))) {
        if (isa<If>(this)) {
            // Eliminate the relationship between the If node and its defined vars ...
            for (PabloAST * var : cast<If>(this)->getDefined()) {
                var->removeUser(this);
                this->removeUser(var);
                var->replaceAllUsesWith(mParent->createZeroes());
            }
        }
        PabloBlock & body = isa<If>(this) ? cast<If>(this)->getBody() : cast<While>(this)->getBody();
        Statement * stmt = body.front();
        while (stmt) {
            stmt = stmt->eraseFromParent(recursively);
        }        
    }

    if (recursively) {
        for (auto i = 0; i != mOperands; ++i) {
            PabloAST * const op = mOperand[i];
            if (op->getNumUses() == 0 && isa<Statement>(op)) {
                cast<Statement>(op)->eraseFromParent(true);
            }
        }
    }
    return removeFromParent();
}

Statement * Statement::replaceWith(PabloAST * const expr, const bool rename, const bool recursively) {
    assert (expr);
    if (LLVM_UNLIKELY(expr == this)) {
        return getNextNode();
    }
    if (LLVM_LIKELY(rename && isa<Statement>(expr))) {
        Statement * const stmt = cast<Statement>(expr);
        if (getName()->isUserDefined() && stmt->getName()->isGenerated()) {
            stmt->setName(getName());
        }
    }
    replaceAllUsesWith(expr);    
    return eraseFromParent(recursively);
}

#ifndef NDEBUG
bool Statement::noRecursiveOperand(const PabloAST * const operand) {
    if (operand && isa<Statement>(operand)) {
        std::queue<const Statement *> Q;
        Q.push(cast<Statement>(operand));
        while (!Q.empty()) {
            const Statement * stmt = Q.front();
            if (stmt == this) {
                return false;
            }
            Q.pop();
            for (auto i = 0; i != stmt->getNumOperands(); ++i) {
                const PabloAST * op = stmt->getOperand(i);
                if (isa<Statement>(op)) {
                    Q.push(cast<Statement>(op));
                }
            }
        }
    }
    return true;
}
#endif

StatementList::~StatementList() {

}

}
