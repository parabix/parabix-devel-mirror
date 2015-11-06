/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/pabloAST.h>
#include <pablo/codegenstate.h>
#include <llvm/Support/Compiler.h>
#include <pablo/printer_pablos.h>
#include <iostream>

namespace pablo {

PabloAST::Allocator PabloAST::mAllocator;

/*

    Return true if expr1 and expr2 can be proven equivalent according to some rules,
    false otherwise.  Note that false may be returned i some cases when the exprs are
    equivalent.

*/

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief equals
 ** ------------------------------------------------------------------------------------------------------------- */
bool equals(const PabloAST * expr1, const PabloAST * expr2) {
    assert (expr1 && expr2);
    if (expr1 == expr2) {
        return true;
    } else if (expr1->getClassTypeId() == expr2->getClassTypeId()) {
        if ((isa<Zeroes>(expr1)) || (isa<Ones>(expr1))) {
            return true;
        } else if (const Var * var1 = dyn_cast<const Var>(expr1)) {
            if (const Var * var2 = cast<const Var>(expr2)) {
                return (var1->getName() == var2->getName());
            }
        } else if (const Not* not1 = dyn_cast<const Not>(expr1)) {
            if (const Not* not2 = cast<const Not>(expr2)) {
                return equals(not1->getExpr(), not2->getExpr());
            }
        } else if (const And* and1 = dyn_cast<const And>(expr1)) {
            if (const And* and2 = cast<const And>(expr2)) {
                if (equals(and1->getExpr1(), and2->getExpr1())) {
                    return equals(and1->getExpr2(), and2->getExpr2());
                } else if (equals(and1->getExpr1(), and2->getExpr2())) {
                    return equals(and1->getExpr2(), and2->getExpr1());
                }
            }
        } else if (const Or * or1 = dyn_cast<const Or>(expr1)) {
            if (const Or* or2 = cast<const Or>(expr2)) {
                if (equals(or1->getExpr1(), or2->getExpr1())) {
                    return equals(or1->getExpr2(), or2->getExpr2());
                } else if (equals(or1->getExpr1(), or2->getExpr2())) {
                    return equals(or1->getExpr2(), or2->getExpr1());
                }
            }
        } else if (const Xor * xor1 = dyn_cast<const Xor>(expr1)) {
            if (const Xor * xor2 = cast<const Xor>(expr2)) {
                if (equals(xor1->getExpr1(), xor2->getExpr1())) {
                    return equals(xor1->getExpr2(), xor2->getExpr2());
                } else if (equals(xor1->getExpr1(), xor2->getExpr2())) {
                    return equals(xor1->getExpr2(), xor2->getExpr1());
                }
            }
        } else if (isa<Integer>(expr1) || isa<String>(expr1) || isa<Call>(expr1)) {
            // If these weren't equivalent by address they won't be equivalent by their operands.
            return false;
        } else { // Non-reassociatable functions (i.e., Sel, Advance, ScanThru, MatchStar, Assign, Next)
            const Statement * stmt1 = cast<Statement>(expr1);
            const Statement * stmt2 = cast<Statement>(expr2);
            assert (stmt1->getNumOperands() == stmt2->getNumOperands());
            for (unsigned i = 0; i != stmt1->getNumOperands(); ++i) {
                if (!equals(stmt1->getOperand(i), stmt2->getOperand(i))) {
                    return false;
                }
            }
            return true;
        }
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief replaceAllUsesWith
 ** ------------------------------------------------------------------------------------------------------------- */
void PabloAST::replaceAllUsesWith(PabloAST * expr) {    
    Statement * user[mUsers.size()];
    Vector::size_type users = 0;
    for (PabloAST * u : mUsers) {
        if (isa<Statement>(u) && u != expr) {
            user[users++] = cast<Statement>(u);
        }
    }
    mUsers.clear();
    assert (expr);
    for (Vector::size_type i = 0; i != users; ++i) {
        user[i]->replaceUsesOfWith(this, expr);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkForReplacementInEscapedValueList
 ** ------------------------------------------------------------------------------------------------------------- */
template <class ValueType, class ValueList>
inline void Statement::checkForReplacementInEscapedValueList(Statement * branch, PabloAST * const from, PabloAST * const to, ValueList & list) {
    if (LLVM_LIKELY(isa<ValueType>(from))) {
        auto f = std::find(list.begin(), list.end(), cast<ValueType>(from));
        if (LLVM_LIKELY(f != list.end())) {
            if (LLVM_LIKELY(isa<ValueType>(to))) {
                if (std::find(list.begin(), list.end(), cast<ValueType>(to)) == list.end()) {
                    *f = cast<ValueType>(to);
                    branch->addUser(to);
                } else {
                    list.erase(f);
                }
                branch->removeUser(from);
                assert (std::find(list.begin(), list.end(), cast<ValueType>(to)) != list.end());
                assert (std::find(branch->user_begin(), branch->user_end(), cast<ValueType>(to)) != branch->user_end());
            } else { // replacement error occured
                std::string tmp;
                raw_string_ostream str(tmp);
                str << "cannot replace escaped value ";
                PabloPrinter::print(from, str);
                str << " with ";
                PabloPrinter::print(to, str);
                str << " in ";
                PabloPrinter::print(branch, str);
                throw std::runtime_error(str.str());
            }
        }                
        assert (std::find(list.begin(), list.end(), cast<ValueType>(from)) == list.end());
        assert (std::find(branch->user_begin(), branch->user_end(), cast<ValueType>(from)) == branch->user_end());
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief replaceUsesOfWith
 ** ------------------------------------------------------------------------------------------------------------- */
void Statement::replaceUsesOfWith(PabloAST * const from, PabloAST * const to) {
    for (unsigned i = 0; i != getNumOperands(); ++i) {
       if (getOperand(i) == from) {
           setOperand(i, to);
       }
    }
    if (LLVM_UNLIKELY(isa<If>(this))) {
        checkForReplacementInEscapedValueList<Assign>(this, from, to, cast<If>(this)->getDefined());
    } else if (LLVM_UNLIKELY(isa<While>(this))) {
        checkForReplacementInEscapedValueList<Next>(this, from, to, cast<While>(this)->getVariants());
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setOperand
 ** ------------------------------------------------------------------------------------------------------------- */
void Statement::setOperand(const unsigned index, PabloAST * const value) {
    assert (value);
    assert (index < getNumOperands());
    PabloAST * const priorValue = getOperand(index);
    if (LLVM_UNLIKELY(priorValue == value)) {
        return;
    }    
    if (LLVM_LIKELY(priorValue != nullptr)) {
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
    }
    mOperand[index] = value;
    value->addUser(this);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief insertBefore
 ** ------------------------------------------------------------------------------------------------------------- */
void Statement::insertBefore(Statement * const statement) {
    if (LLVM_UNLIKELY(statement == this)) {
        return;
    }
    else if (LLVM_UNLIKELY(statement == nullptr)) {
        throw std::runtime_error("cannot insert before null statement!");
    }
    else if (LLVM_UNLIKELY(statement->mParent == nullptr)) {
        throw std::runtime_error("statement is not contained in a pablo block!");
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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief insertAfter
 ** ------------------------------------------------------------------------------------------------------------- */
void Statement::insertAfter(Statement * const statement) {
    if (LLVM_UNLIKELY(statement == this)) {
        return;
    }
    else if (LLVM_UNLIKELY(statement == nullptr)) {
        throw std::runtime_error("cannot insert after null statement!");
    }
    else if (LLVM_UNLIKELY(statement->mParent == nullptr)) {
        throw std::runtime_error("statement is not contained in a pablo block!");
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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief removeFromParent
 ** ------------------------------------------------------------------------------------------------------------- */
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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief eraseFromParent
 ** ------------------------------------------------------------------------------------------------------------- */
Statement * Statement::eraseFromParent(const bool recursively) {
    // remove this statement from its operands' users list
    for (unsigned i = 0; i != mOperands; ++i) {
        mOperand[i]->removeUser(this);
    }
    Statement * redundantBranch = nullptr;
    // If this is an If or While statement, we'll have to remove the statements within the
    // body or we'll lose track of them.
    if (LLVM_UNLIKELY(isa<If>(this) || isa<While>(this))) {
        PabloBlock & body = isa<If>(this) ? cast<If>(this)->getBody() : cast<While>(this)->getBody();
        Statement * stmt = body.front();
        // Note: by erasing the body, any Assign/Next nodes will be replaced with Zero.
        while (stmt) {
            stmt = stmt->eraseFromParent(recursively);
        }
    } else if (LLVM_UNLIKELY(isa<Assign>(this))) {
        for (PabloAST * use : mUsers) {
            if (If * ifNode = dyn_cast<If>(use)) {
                auto & defs = ifNode->getDefined();
                auto f = std::find(defs.begin(), defs.end(), this);
                if (LLVM_LIKELY(f != defs.end())) {
                    this->removeUser(ifNode);
                    ifNode->removeUser(this);
                    defs.erase(f);
                    if (LLVM_UNLIKELY(defs.empty())) {
                        redundantBranch = ifNode;
                    }
                    break;
                }
            }
        }
    } else if (LLVM_UNLIKELY(isa<Next>(this))) {
        for (PabloAST * use : mUsers) {
            if (While * whileNode = dyn_cast<While>(use)) {
                auto & vars = whileNode->getVariants();
                auto f = std::find(vars.begin(), vars.end(), this);
                if (LLVM_LIKELY(f != vars.end())) {
                    this->removeUser(whileNode);
                    whileNode->removeUser(this);
                    vars.erase(f);
                    if (LLVM_UNLIKELY(vars.empty())) {
                        redundantBranch = whileNode;
                    }
                    break;
                }
            }
        }
    }

    replaceAllUsesWith(PabloBlock::createZeroes());

    if (recursively) {
        for (unsigned i = 0; i != mOperands; ++i) {
            PabloAST * const op = mOperand[i];
            if (LLVM_LIKELY(isa<Statement>(op))) {
                bool erase = false;
                if (op->getNumUses() == 0) {
                    erase = true;
                } else if ((isa<Assign>(op) || isa<Next>(op)) && op->getNumUses() == 1) {
                    erase = true;
                }
                if (erase) {
                    cast<Statement>(op)->eraseFromParent(true);
                }
            }
        }
        if (LLVM_UNLIKELY(redundantBranch != nullptr)) {
            redundantBranch->eraseFromParent(true);
        }
    }

    return removeFromParent();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief replaceWith
 ** ------------------------------------------------------------------------------------------------------------- */
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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief contains
 ** ------------------------------------------------------------------------------------------------------------- */
bool StatementList::contains(Statement * const statement) {
    for (Statement * stmt : *this) {
        if (statement == stmt) {
            return true;
        }
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief clear
 ** ------------------------------------------------------------------------------------------------------------- */
void StatementList::clear() {
    Statement * stmt = front();
    while (stmt) {
        Statement * next = stmt->mNext;
        if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
            PabloBlock & body = isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody();
            stmt->mParent->removeUser(&body);
        }
        stmt->mPrev = nullptr;
        stmt->mNext = nullptr;
        stmt->mParent = nullptr;
        stmt = next;
    }
    mInsertionPoint = nullptr;
    mFirst = nullptr;
    mLast = nullptr;
}

StatementList::~StatementList() {

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief escapes
 *
 * Is this statement used outside of its scope?
 ** ------------------------------------------------------------------------------------------------------------- */
bool escapes(const Statement * statement) {
    const PabloBlock * const parent = statement->getParent();
    for (const PabloAST * user : statement->users()) {
        if (LLVM_LIKELY(isa<Statement>(user))) {
            const PabloBlock * used = cast<Statement>(user)->getParent();
            while (used != parent) {
                used = used->getParent();
                if (used == nullptr) {
                    assert (isa<Assign>(statement) || isa<Next>(statement));
                    return true;
                }
            }
        }
    }
    return false;
}

}
