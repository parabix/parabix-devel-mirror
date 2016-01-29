/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/pabloAST.h>
#include <pablo/codegenstate.h>
#include <llvm/Support/Compiler.h>
#include <pablo/printer_pablos.h>
#include <llvm/ADT/SmallVector.h>

namespace pablo {

PabloAST::Allocator PabloAST::mAllocator;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief equals
 *
 *  Return true if expr1 and expr2 can be proven equivalent according to some rules, false otherwise.  Note that
 *  false may be returned i some cases when the exprs are equivalent.
 ** ------------------------------------------------------------------------------------------------------------- */
bool equals(const PabloAST * expr1, const PabloAST * expr2) {
    assert (expr1 && expr2);
    if (expr1 == expr2) {
        return true;
    } else if (expr1->getClassTypeId() == expr2->getClassTypeId()) {
        if ((isa<Zeroes>(expr1)) || (isa<Ones>(expr1))) {
            return true;
        } else if (isa<Var>(expr1)) {
            return (cast<Var>(expr1)->getName() == cast<Var>(expr2)->getName());
        } else if (isa<Not>(expr1)) {
            return equals(cast<Not>(expr1)->getOperand(0), cast<Not>(expr2)->getOperand(0));
        } else if (isa<Variadic>(expr1)) {
            const Variadic * const var1 = cast<Variadic>(expr1);
            const Variadic * const var2 = cast<Variadic>(expr2);
            if (var1->getNumOperands() == var2->getNumOperands()) {
                const unsigned operands = var1->getNumOperands();
                for (unsigned i = 0; i != operands; ++i) {
                    bool missing = true;
                    for (unsigned j = 0; j != operands; ++j) {
                        // odds are both variadics will be sorted; optimize towards testing them in order.
                        unsigned k = i + j;
                        if (LLVM_UNLIKELY(k >= operands)) {
                            k -= operands;
                        }
                        if (equals(var1->getOperand(i), var2->getOperand(k))) {
                            missing = false;
                            break;
                        }
                    }
                    if (missing) {
                        return false;
                    }
                }
                return true;
            }
        } else if (isa<Integer>(expr1) || isa<String>(expr1) || isa<Call>(expr1)) {
            return false; // If these weren't equivalent by address they won't be equivalent by their operands.
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
void PabloAST::replaceAllUsesWith(PabloAST * const expr) {
    assert (expr);
    if (LLVM_UNLIKELY(this == expr)) {
        return;
    }
    Statement * replacement[mUsers.size()];
    Users::size_type replacements = 0;
    PabloAST * last = nullptr;
    for (PabloAST * user : mUsers) {
        if (LLVM_UNLIKELY(user == expr || user == last)) {
            continue;
        } else if (isa<Statement>(user)) {
            replacement[replacements++] = cast<Statement>(user);
            last = user;
        }
    }
    for (Users::size_type i = 0; i != replacements; ++i) {
        replacement[i]->replaceUsesOfWith(this, expr);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkEscapedValueList
 ** ------------------------------------------------------------------------------------------------------------- */
template <class ValueType, class ValueList>
inline void Statement::checkEscapedValueList(Statement * const branch, PabloAST * const from, PabloAST * const to, ValueList & list) {
    if (LLVM_LIKELY(isa<ValueType>(from))) {
        auto f = std::find(list.begin(), list.end(), cast<ValueType>(from));
        if (LLVM_LIKELY(f != list.end())) {
            branch->removeUser(from);
            from->removeUser(branch);
            if (LLVM_LIKELY(isa<ValueType>(to))) {
                if (std::count(list.begin(), list.end(), cast<ValueType>(to)) == 0) {
                    *f = cast<ValueType>(to);
                    branch->addUser(to);
                    to->addUser(branch);
                    return;
                }
            }
            list.erase(f);
        }                              
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief replaceUsesOfWith
 ** ------------------------------------------------------------------------------------------------------------- */
void Statement::replaceUsesOfWith(PabloAST * const from, PabloAST * const to) {
    if (LLVM_UNLIKELY(from == to)) {
        return;
    }
    for (unsigned i = 0; i != getNumOperands(); ++i) {
       if (getOperand(i) == from) {
           setOperand(i, to);
       }
    }
    if (LLVM_UNLIKELY(isa<If>(this))) {
        checkEscapedValueList<Assign>(this, from, to, cast<If>(this)->getDefined());
    } else if (LLVM_UNLIKELY(isa<While>(this))) {
        checkEscapedValueList<Next>(this, from, to, cast<While>(this)->getVariants());
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setOperand
 ** ------------------------------------------------------------------------------------------------------------- */
void Statement::setOperand(const unsigned index, PabloAST * const value) {
    assert ("Operand cannot be null!" && value);
    assert (index < getNumOperands());
    PabloAST * const prior = getOperand(index);
    assert ("Operand cannot be null!" && prior);
    if (LLVM_UNLIKELY(prior == value)) {
        return;
    }    
    prior->removeUser(this);
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
        PabloBlock * body = isa<If>(this) ? cast<If>(this)->getBody() : cast<While>(this)->getBody();
        body->setParent(mParent);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief insertAfter
 ** ------------------------------------------------------------------------------------------------------------- */
void Statement::insertAfter(Statement * const statement) {
    if (LLVM_UNLIKELY(statement == this)) {
        return;
    } else if (LLVM_UNLIKELY(statement == nullptr)) {
        throw std::runtime_error("cannot insert after null statement!");
    } else if (LLVM_UNLIKELY(statement->mParent == nullptr)) {
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
        PabloBlock * body = isa<If>(this) ? cast<If>(this)->getBody() : cast<While>(this)->getBody();
        body->setParent(mParent);
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
            PabloBlock * body = isa<If>(this) ? cast<If>(this)->getBody() : cast<While>(this)->getBody();
            body->setParent(nullptr);
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
    SmallVector<Statement *, 1> redundantBranches;
    // If this is an If or While statement, we'll have to remove the statements within the
    // body or we'll lose track of them.
    if (LLVM_UNLIKELY(isa<If>(this) || isa<While>(this))) {
        PabloBlock * const body = isa<If>(this) ? cast<If>(this)->getBody() : cast<While>(this)->getBody();
        body->eraseFromParent(recursively);
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
                        redundantBranches.push_back(ifNode);
                    }
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
                        redundantBranches.push_back(whileNode);
                    }
                }
            }
        }
    }

    replaceAllUsesWith(PabloBlock::createZeroes());

    if (recursively) {
        for (unsigned i = 0; i != mOperands; ++i) {
            PabloAST * const op = mOperand[i];
            if (LLVM_LIKELY(isa<Statement>(op))) {
                if (op->getNumUses() == 0) {
                    cast<Statement>(op)->eraseFromParent(true);
                }
            }
        }
        if (LLVM_UNLIKELY(redundantBranches.size() != 0)) {
            // By eliminating this redundant branch, we may inadvertantly delete the scope block this statement
            // resides within. Return null if so.
            bool eliminatedScope = false;
            for (Statement * br : redundantBranches) {
                const PabloBlock * const body = isa<If>(br) ? cast<If>(br)->getBody() : cast<While>(br)->getBody();
                if (LLVM_UNLIKELY(body == getParent())) {
                    eliminatedScope = true;
                }
                br->eraseFromParent(true);
            }
            if (eliminatedScope) {
                return nullptr;
            }
        }
    }
    Statement * const next = removeFromParent();
    mAllocator.deallocate(reinterpret_cast<Allocator::pointer>(this));
    return next;
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
 * @brief addOperand
 ** ------------------------------------------------------------------------------------------------------------- */
void Variadic::addOperand(PabloAST * const expr) {
    if (LLVM_UNLIKELY(mOperands == mCapacity)) {
        mCapacity = std::max<unsigned>(mCapacity * 2, 2);
        PabloAST ** expandedOperandSpace = reinterpret_cast<PabloAST**>(mAllocator.allocate(mCapacity * sizeof(PabloAST *)));
        for (unsigned i = 0; i != mOperands; ++i) {
            expandedOperandSpace[i] = mOperand[i];
        }
        mAllocator.deallocate(reinterpret_cast<Allocator::pointer>(mOperand));
        mOperand = expandedOperandSpace;
    }
    mOperand[mOperands++] = expr;
    expr->addUser(this);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief removeOperand
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * Variadic::removeOperand(const unsigned index) {
    assert (index < mOperands);
    PabloAST * const expr = mOperand[index];
    assert (expr);
    --mOperands;
    for (unsigned i = index; i != mOperands; ++i) {
        mOperand[i] = mOperand[i + 1];
    }
    mOperand[mOperands] = nullptr;
    expr->removeUser(this);
    return expr;
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
