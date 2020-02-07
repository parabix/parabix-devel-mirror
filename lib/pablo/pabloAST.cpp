/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/pabloAST.h>
#include <pablo/arithmetic.h>
#include <pablo/codegenstate.h>
#include <pablo/pe_var.h>
#include <pablo/boolean.h>
#include <pablo/pablo_intrinsic.h>
#include <pablo/pe_infile.h>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_ones.h>
#include <pablo/ps_assign.h>
#include <pablo/branch.h>
#include <pablo/printer_pablos.h>
#include <llvm/Support/ErrorHandling.h>
#include <llvm/Support/raw_os_ostream.h>

using namespace boost::container;
using namespace llvm;

namespace pablo {

using TypeId = PabloAST::ClassTypeId;

size_t constexpr __length(const char * const str) {
    return *str ? 1 + __length(str + 1) : 0;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief equals
 *
 *  Return true if expr1 and expr2 can be proven equivalent according to some rules, false otherwise.  Note that
 *  false may be returned i some cases when the exprs are equivalent.
 ** ------------------------------------------------------------------------------------------------------------- */
bool equals(const PabloAST * const expr1, const PabloAST * const expr2) noexcept {
    assert (expr1 && expr2);
    if (LLVM_UNLIKELY(expr1 == expr2)) {
        return true;
    } else if (expr1->getClassTypeId() == expr2->getClassTypeId()) {
        if (LLVM_LIKELY(expr1->getType() == expr2->getType())) {
            if ((isa<Zeroes>(expr1)) || (isa<Ones>(expr1))) {
                return true;
            } else if (isa<Not>(expr1)) {
                return equals(cast<Not>(expr1)->getOperand(0), cast<Not>(expr2)->getOperand(0));
            } else if (isa<InFile>(expr1)) {
                return equals(cast<InFile>(expr1)->getOperand(0), cast<InFile>(expr2)->getOperand(0));
            } else if (isa<AtEOF>(expr1)) {
                return equals(cast<AtEOF>(expr1)->getOperand(0), cast<AtEOF>(expr2)->getOperand(0));
            } else if (isa<And>(expr1) || isa<Or>(expr1) || isa<Xor>(expr1)) {
                PabloAST * op1[2];
                PabloAST * op2[2];
                op1[0] = cast<Statement>(expr1)->getOperand(0);
                op1[1] = cast<Statement>(expr1)->getOperand(1);
                if (op1[1] < op1[0]) {
                    std::swap(op1[0], op1[1]);
                }
                op2[0] = cast<Statement>(expr2)->getOperand(0);
                op2[1] = cast<Statement>(expr2)->getOperand(1);
                if (op2[1] < op2[0]) {
                    std::swap(op2[0], op2[1]);
                }
                return (op1[0] == op2[0]) && (op1[1] == op2[1]);
            } else if (isa<Statement>(expr1)) {
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
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief replaceAllUsesWith
 ** ------------------------------------------------------------------------------------------------------------- */
void PabloAST::replaceAllUsesWith(PabloAST * const expr) noexcept {
    assert (expr);
    if (LLVM_UNLIKELY(this == expr)) {
        return;
    }
    SmallVector<PabloAST *, 16> replacement(mUsers.size());
    Users::size_type replacements = 0;
    PabloAST * last = nullptr;
    for (PabloAST * user : mUsers) {
        if (LLVM_UNLIKELY(user == expr || user == last)) {
            continue;
        } else if (isa<Statement>(user) || isa<Operator>(user)) {
            replacement[replacements++] = user;
            last = user;
        }
    }
    for (Users::size_type i = 0; i != replacements; ++i) {
        replacement[i]->replaceUsesOfWith(this, expr);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addUser
 ** ------------------------------------------------------------------------------------------------------------- */
bool PabloAST::addUser(PabloAST * const user) noexcept {
    assert (user);
    const auto p = std::lower_bound(mUsers.begin(), mUsers.end(), user);
    const bool unique = p == mUsers.end() || *p != user;
    mUsers.insert(p, user);
    return unique;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief removeUser
 ** ------------------------------------------------------------------------------------------------------------- */
bool PabloAST::removeUser(PabloAST * const user) noexcept {
    assert (user);
    const auto p = std::lower_bound(mUsers.begin(), mUsers.end(), user);
    assert (p != mUsers.end() && *p == user);
    const auto n = mUsers.erase(p);
    return n == mUsers.end() || *n != user;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief print
 ** ------------------------------------------------------------------------------------------------------------- */
void PabloAST::print(llvm::raw_ostream & O) const {
    PabloPrinter::print(this, O);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setName
 ** ------------------------------------------------------------------------------------------------------------- */
void NamedPabloAST::setName(const String * const name) {
    if (LLVM_UNLIKELY(name == nullptr)) {
        llvm::report_fatal_error("name cannot be null");
    }
    mName = name;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getName
 ** ------------------------------------------------------------------------------------------------------------- */
const String & Statement::getName() const {
    if (mName == nullptr) {
        if (LLVM_UNLIKELY(mParent == nullptr)) {
            llvm::report_fatal_error("cannot assign a default name to a statement that is not within a PabloBlock");
        }
        const char * prefix = nullptr;
        size_t length = 0;
        #define MAKE_PREFIX(type_id, name) \
            case ClassTypeId:: type_id : prefix = name; length = __length(name); break
        switch (mClassTypeId) {
            // Boolean operations
            MAKE_PREFIX(And, "and");
            MAKE_PREFIX(Or, "or");
            MAKE_PREFIX(Xor, "xor");
            MAKE_PREFIX(Not, "not");
            MAKE_PREFIX(Sel, "sel");
            // Stream operations
            MAKE_PREFIX(Advance, "advance");
            MAKE_PREFIX(IndexedAdvance, "indexed_advance");
            MAKE_PREFIX(ScanThru, "scanthru");
            MAKE_PREFIX(AdvanceThenScanThru, "advscanthru");
            MAKE_PREFIX(ScanTo, "scanto");
            MAKE_PREFIX(AdvanceThenScanTo, "advscanto");
            MAKE_PREFIX(Lookahead, "lookahead");
            MAKE_PREFIX(MatchStar, "matchstar");
            MAKE_PREFIX(InFile, "inFile");
            MAKE_PREFIX(AtEOF, "atEOF");
            MAKE_PREFIX(TerminateAt, "terminateAt");
            MAKE_PREFIX(EveryNth, "everyNth");
            // Statistics operations
            MAKE_PREFIX(Count, "count");
            // Misc. operations
            MAKE_PREFIX(Repeat, "repeat");
            MAKE_PREFIX(PackH, "packh");
            MAKE_PREFIX(PackL, "packl");
            MAKE_PREFIX(Ternary, "ternary");

            case ClassTypeId::IntrinsicCall:
                prefix = cast<IntrinsicCall>(this)->getIntrinsicName().lower().c_str();
                length = __length(prefix);
                break;
            default: llvm_unreachable("invalid statement type");
        }
        #undef MAKE_PREFIX
        const StringRef __prefix(prefix, length);
        mName = mParent->makeName(__prefix);
    }
    return *mName;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief replaceUsesOfWith
 ** ------------------------------------------------------------------------------------------------------------- */
void Statement::replaceUsesOfWith(PabloAST * const from, PabloAST * const to, const bool recursive) {
    if (LLVM_LIKELY(from != to)) {
        for (unsigned i = 0; i != getNumOperands(); ++i) {
           PabloAST * const op = getOperand(i);
           if (op == from) {
               setOperand(i, to);
           } else if (LLVM_UNLIKELY(recursive)) {
               op->replaceUsesOfWith(from, to, true);
           }
        }
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
    if (LLVM_UNLIKELY(prior->getType() != value->getType())) {
        std::string tmp;
        raw_string_ostream out(tmp);
        out << "Type mismatch replacing operand ";
        prior->print(out);
        out << " with ";
        value->print(out);
        out << " in statement ";
        this->print(out);
        llvm::report_fatal_error(out.str());
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
    } else if (LLVM_UNLIKELY(statement == nullptr)) {
        llvm::report_fatal_error("cannot insert before null statement!");
    } else if (LLVM_UNLIKELY(statement->mParent == nullptr)) {
        llvm::report_fatal_error("statement is not contained in a pablo block!");
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
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief insertAfter
 ** ------------------------------------------------------------------------------------------------------------- */
void Statement::insertAfter(Statement * const statement) {
    if (LLVM_UNLIKELY(statement == this)) {
        return;
    } else if (LLVM_UNLIKELY(statement == nullptr)) {
        assert (false);
        llvm::report_fatal_error("cannot insert after null statement!");
    } else if (LLVM_UNLIKELY(statement->mParent == nullptr)) {
        llvm::report_fatal_error("statement is not contained in a pablo block!");
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
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief removeFromParent
 ** ------------------------------------------------------------------------------------------------------------- */
Statement * Statement::removeFromParent() noexcept {
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
    }
    mPrev = nullptr;
    mNext = nullptr;
    mParent = nullptr;
    return next;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief eraseFromParent
 ** ------------------------------------------------------------------------------------------------------------- */
Statement * Statement::eraseFromParent(const bool recursively) noexcept {
    PabloBlock * const scope = getParent();
    if (LLVM_UNLIKELY(scope == nullptr)) {
        return nullptr;
    }
    if (LLVM_UNLIKELY(isa<Branch>(this))) {
        cast<Branch>(this)->getBody()->eraseFromParent(recursively);
    } else if (LLVM_UNLIKELY(!mUsers.empty())) {
        assert (!isa<Assign>(this));
        // We cannot leave an erased statement in the AST.
        // Remove it by replacing all users with 0.
        replaceAllUsesWith(scope->createZeroes(getType()));
    }
    Statement * const next = removeFromParent();
    for (unsigned i = 0; i != mOperands; ++i) {
        PabloAST * const op = mOperand[i]; assert (op);
        op->removeUser(this);
        if (LLVM_UNLIKELY(recursively && op->getNumUses() == 0)) {
            if (LLVM_LIKELY(isa<Statement>(op))) {
                cast<Statement>(op)->eraseFromParent(true);
            }
        }
        mOperand[i] = nullptr;
    }
    return next;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief replaceWith
 ** ------------------------------------------------------------------------------------------------------------- */
Statement * Statement::replaceWith(PabloAST * const expr, const bool rename, const bool recursively) noexcept {
    assert (expr);
    if (LLVM_UNLIKELY(expr == this)) {
        return getNextNode();
    }
    if (LLVM_UNLIKELY(mName && rename)) {
        if (LLVM_LIKELY(isa<Statement>(expr) && cast<Statement>(expr)->mName == nullptr)) {
            cast<Statement>(expr)->setName(mName);
            mName = nullptr;
        }
    }
    replaceAllUsesWith(expr);
    return Statement::eraseFromParent(recursively);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief contains
 ** ------------------------------------------------------------------------------------------------------------- */
bool StatementList::contains(const Statement * const statement) const noexcept {
    for (const Statement * stmt : *this) {
        if (statement == stmt) {
            return true;
        }
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief dominates
 *
 * expr1 dominates (>>) expr2 if every path from the *entry* node to expr2 must go through expr1
 *
 * For example, consider the program (with entry node 1):
 *
 * 1. Assign a, s           1 >> 1, 2, 3, 4, 5, 6
 * 2. While x:              2 >> 2, 3, 4, 5, 6
 * 3.   Assign a, p         3 >> 3
 * 4. While y:              4 >> 4, 5, 6
 * 5.   Assign a, q         5 >> 5
 * 6. Assign a, t           6 >> 6
 ** ------------------------------------------------------------------------------------------------------------- */
bool dominates(const PabloAST * const expr1, const PabloAST * const expr2) noexcept {
    if (LLVM_UNLIKELY(expr1 == nullptr || expr2 == nullptr)) {
        return (expr2 == nullptr);
    } else if (LLVM_LIKELY(isa<Statement>(expr1))) {
        const Statement * stmt1 = cast<Statement>(expr1);
        assert ("expr1 is not in any block!" && stmt1->getParent());
        if (isa<Statement>(expr2)) {
            const Statement * stmt2 = cast<Statement>(expr2);
            assert ("expr2 is not in any block!" && stmt2->getParent());

            while (stmt1->getParent() != stmt2->getParent()) {
                stmt2 = stmt2->getParent()->getBranch();
                if (stmt2 == nullptr) {
                    return false;
                }
            }

            const Statement * temp1 = stmt1, * temp2 = stmt2;
            while (temp1 && temp2) {
                if (temp1 == stmt2) {
                    return true;
                } else if (temp2 == stmt1) {
                    return false;
                }
                temp1 = temp1->getNextNode();
                temp2 = temp2->getNextNode();
            }
            // If temp1 isn't null then temp2 must be null; thus stmt2 must succeed stmt1.
            return (temp1 != nullptr);
        }
        return false;
    }
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief postdominates
 *
 * expr1 post-dominates (<<) expr2 if all paths to the *exit* node starting at expr2 must go through expr1
 *
 * For example, consider the program (with exit node 6):
 *
 * 1. Assign a, s           1 << 1
 * 2. While x:              2 << 1, 2
 * 3.   Assign a, p         3 << 1, 2, 3
 * 4. While y:              4 << 1, 2, 4
 * 5.   Assign a, q         5 << 1, 2, 4, 5
 * 6. Assign a, t           6 << 1, 2, 3, 4, 5, 6
 ** ------------------------------------------------------------------------------------------------------------- */
bool postdominates(const PabloAST * const expr1, const PabloAST * const expr2) noexcept {
    llvm::report_fatal_error("not implemented yet!");
}

}
