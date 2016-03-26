#include "pabloverifier.hpp"
#include <pablo/function.h>
#include <pablo/codegenstate.h>
#include <pablo/printer_pablos.h>
#include <iostream>
#include <boost/container/flat_set.hpp>
#include <queue>


namespace pablo {

template <typename Type>
using SmallSet = boost::container::flat_set<Type>;

using ScopeSet = SmallSet<const PabloBlock *>;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief verifyUseDefInformation
 ** ------------------------------------------------------------------------------------------------------------- */
template<typename VectorType>
bool checkVector(const Statement * value, const VectorType & vector) {
    for (auto escapedValue : vector) {
        if (escapedValue == value) {
            return false;
        }
    }
    return true;
}

void verifyUseDefInformation(const PabloBlock * block, const ScopeSet & validScopes) {
    for (const Statement * stmt : *block) {
        for (const PabloAST * use : stmt->users()) {
            if (LLVM_LIKELY(isa<Statement>(use))) {
                const Statement * const user = cast<Statement>(use);
                // test whether this user is in a block in the program
                if (LLVM_UNLIKELY(validScopes.count(user->getParent()) == 0)) {
                    std::string tmp;
                    raw_string_ostream str(tmp);
                    str << "PabloVerifier: use-def error: ";
                    PabloPrinter::print(user, str);
                    str << " is a user of ";
                    PabloPrinter::print(stmt, str);
                    if (user->getParent() == nullptr) {
                        str << " but is not in any scope.";
                    } else {
                        str << " but is in a deleted scope.";
                    }
                    throw std::runtime_error(str.str());
                }
                bool notFound = true;
                for (unsigned i = 0; i != user->getNumOperands(); ++i) {
                    if (user->getOperand(i) == stmt) {
                        notFound = false;
                        break;
                    }
                }
                if (LLVM_UNLIKELY(notFound)) {
                    if (const If * ifNode = dyn_cast<If>(stmt)) {
                        notFound = checkVector(user, ifNode->getDefined());
                    } else if (const If * ifNode = dyn_cast<If>(user)) {
                        notFound = checkVector(stmt, ifNode->getDefined());
                    } else if (const While * whileNode = dyn_cast<While>(stmt)) {
                        notFound = checkVector(user, whileNode->getVariants());
                    } else if (const While * whileNode = dyn_cast<While>(user)) {
                        notFound = checkVector(stmt, whileNode->getVariants());
                    } else if (isa<Next>(stmt) && isa<Assign>(use)) {
                        notFound = (use != cast<Next>(stmt)->getInitial());
                    }
                    if (LLVM_UNLIKELY(notFound)) {
                        std::string tmp;
                        raw_string_ostream str(tmp);
                        str << "PabloVerifier: use-def error: ";
                        PabloPrinter::print(stmt, str);
                        str << " is not a definition of ";
                        PabloPrinter::print(use, str);
                        throw std::runtime_error(str.str());
                    }
                }
            }
        }
        for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
            const PabloAST * const def = stmt->getOperand(i);
            bool notFound = true;
            for (const PabloAST * use : def->users()) {
                if (use == stmt) {
                    notFound = false;
                    break;
                }
            }
            if (LLVM_UNLIKELY(notFound)) {
                std::string tmp;
                raw_string_ostream str(tmp);
                str << "PabloVerifier: def-use error: ";
                PabloPrinter::print(stmt, str);
                str << " is not recorded in ";
                PabloPrinter::print(def, str);
                str << "'s user list";
                throw std::runtime_error(str.str());
            }
        }
        if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
            verifyUseDefInformation(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody(), validScopes);
        }
    }
}

void gatherValidScopes(const PabloBlock * block, ScopeSet & validScopes) {
    validScopes.insert(block);
    for (const Statement * stmt : *block) {
        if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
            gatherValidScopes(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody(), validScopes);
        }
    }
}

void verifyUseDefInformation(const PabloFunction & function) {
    ScopeSet validScopes;
    gatherValidScopes(function.getEntryBlock(), validScopes);
    verifyUseDefInformation(function.getEntryBlock(), validScopes);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief unreachable
 ** ------------------------------------------------------------------------------------------------------------- */
bool unreachable(const Statement * stmt, const PabloBlock * const block) {
    PabloBlock * parent = stmt->getParent();
    while (parent)  {
        if (parent == block) {
            return false;
        }
        parent = parent->getParent();
    }
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief throwUncontainedEscapedValueError
 ** ------------------------------------------------------------------------------------------------------------- */
static void throwUncontainedEscapedValueError(const Statement * const stmt, const PabloAST * const value) {
    std::string tmp;
    raw_string_ostream str(tmp);
    str << "PabloVerifier: structure error: escaped value \"";
    PabloPrinter::print(value, str);
    str << "\" is not contained within the body of ";
    PabloPrinter::print(stmt, str);
    throw std::runtime_error(str.str());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief throwEscapedValueUsageError
 ** ------------------------------------------------------------------------------------------------------------- */
static void throwEscapedValueUsageError(const Statement * const stmt, const PabloAST * const value, const PabloAST * const def, const PabloAST * const user, const unsigned count) {
    std::string tmp;
    raw_string_ostream str(tmp);
    str << "PabloVerifier: structure error: ";
    PabloPrinter::print(value, str);
    str << " is an escaped value of ";
    PabloPrinter::print(stmt, str);
    str << " but ";
    PabloPrinter::print(user, str);
    if (count == 0) {
        str << " is not considered a user of ";
    } else if (count > 0) {
        str << " was recorded too many times (" << count << ") in the user list of ";
    }
    PabloPrinter::print(def, str);
    throw std::runtime_error(str.str());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief throwReportedScopeError
 ** ------------------------------------------------------------------------------------------------------------- */
static void throwReportedScopeError(const Statement * const stmt) {
    std::string tmp;
    raw_string_ostream str(tmp);
    str << "PabloVerifier: structure error: ";
    PabloPrinter::print(stmt, str);
    str << " is not contained in its reported scope block";
    throw std::runtime_error(str.str());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief throwMisreportedBranchError
 ** ------------------------------------------------------------------------------------------------------------- */
static void throwMisreportedBranchError(const Statement * const stmt, const Statement * const branch) {
    std::string tmp;
    raw_string_ostream str(tmp);
    str << "PabloVerifier: structure error: ";
    PabloPrinter::print(stmt, str);
    str << " branches into a scope block that reports ";
    PabloPrinter::print(stmt, str);
    str << " as its branching statement.";
    throw std::runtime_error(str.str());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief throwReflexiveIfConditionError
 ** ------------------------------------------------------------------------------------------------------------- */
static void throwReflexiveIfConditionError(const PabloAST * const ifNode) {
    std::string tmp;
    raw_string_ostream str(tmp);
    str << "PabloVerifier: structure error: the condition of ";
    PabloPrinter::print(ifNode, str);
    str << " cannot be defined by the If node itself.";
    throw std::runtime_error(str.str());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief verifyProgramStructure
 ** ------------------------------------------------------------------------------------------------------------- */
void verifyProgramStructure(const PabloBlock * block, unsigned & nestingDepth) {
    const Statement * prev = nullptr;
    for (const Statement * stmt : *block) {
        if (LLVM_UNLIKELY(stmt->getPrevNode() != prev)) {
            std::string tmp;
            raw_string_ostream str(tmp);
            str << "PabloVerifier: structure error: ";
            PabloPrinter::print(stmt, str);
            str << " succeeds ";
            PabloPrinter::print(prev, str);
            str << " but expects to preceed ";
            PabloPrinter::print(stmt->getPrevNode(), str);
            throw std::runtime_error(str.str());
        }
        prev = stmt;
        if (LLVM_UNLIKELY(stmt->getParent() != block)) {
            std::string tmp;
            raw_string_ostream str(tmp);
            str << "PabloVerifier: structure error: ";
            PabloPrinter::print(stmt, str);
            str << " is not contained in its reported scope block";
            throw std::runtime_error(str.str());
        }
        if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
            const PabloBlock * nested = isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody();
            if (LLVM_UNLIKELY(nested->getBranch() != stmt)) {
                throwMisreportedBranchError(stmt, nested->getBranch());
            } else if (LLVM_UNLIKELY(nested->getParent() != block)) {
                throwReportedScopeError(stmt);
            }
            if (isa<If>(stmt)) {
                for (const Assign * def : cast<If>(stmt)->getDefined()) {
                    if (LLVM_UNLIKELY(def == cast<If>(stmt)->getCondition())) {
                        throwReflexiveIfConditionError(stmt);
                    } else if (LLVM_UNLIKELY(unreachable(def, nested))) {
                        throwUncontainedEscapedValueError(stmt, def);
                    }
                    unsigned count = std::count(stmt->user_begin(), stmt->user_end(), def);
                    if (LLVM_UNLIKELY(count != 1)) {
                        throwEscapedValueUsageError(stmt, def, stmt, def, count);
                    }
                    count = std::count(def->user_begin(), def->user_end(), stmt);
                    if (LLVM_UNLIKELY(count != 1)) {
                        throwEscapedValueUsageError(stmt, def, def, stmt, count);
                    }
                }
            } else {
                for (const Next * var : cast<While>(stmt)->getVariants()) {
                    if (LLVM_UNLIKELY(unreachable(var, nested))) {
                        throwUncontainedEscapedValueError(stmt, var);
                    }
                    unsigned count = std::count(stmt->user_begin(), stmt->user_end(), var);
                    if (LLVM_UNLIKELY(count != 1)) {
                        throwEscapedValueUsageError(stmt, var, stmt, var, count);
                    }
                    count = std::count(var->user_begin(), var->user_end(), stmt);
                    if (LLVM_UNLIKELY(count != ((cast<While>(stmt)->getCondition() == var) ? 2 : 1))) {
                        throwEscapedValueUsageError(stmt, var, var, stmt, count);
                    }
                }
            }
            ++nestingDepth;
            verifyProgramStructure(nested, nestingDepth);
            --nestingDepth;
        }
    }    
}

inline void verifyProgramStructure(const PabloFunction & function) {
    unsigned nestingDepth = 0;
    verifyProgramStructure(function.getEntryBlock(), nestingDepth);
    if (LLVM_UNLIKELY(nestingDepth != 0)) {
        // This error isn't actually possible to occur with the current AST structure but that could change
        // in the future. Leaving this test in for a reminder to check for it.
        throw std::runtime_error("PabloVerifier: unbalanced If or While nesting depth.");
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isTopologicallyOrdered
 ** ------------------------------------------------------------------------------------------------------------- */
struct OrderingVerifier {
    OrderingVerifier() : mParent(nullptr) {}
    OrderingVerifier(const OrderingVerifier & parent) : mParent(&parent) {}
    bool count(const PabloAST * expr) const {
        if (mSet.count(expr)) {
            return true;
        } else if (mParent) {
            return mParent->count(expr);
        }
        return false;
    }
    void insert(const PabloAST * expr) {
        mSet.insert(expr);
    }
private:
    const OrderingVerifier * const mParent;
    SmallSet<const PabloAST *> mSet;
};

bool recursivelyDefined(const Statement * const stmt) {
    std::queue<const Statement *> Q;
    SmallSet<const PabloAST *> V;
    V.insert(stmt);
    for (const Statement * ancestor = stmt;;) {
        for (unsigned i = 0; i != ancestor->getNumOperands(); ++i) {
            const PabloAST * op = ancestor->getOperand(i);
            if (isa<Statement>(op) && V.count(op) == 0) {
                if (op == stmt) {
                    return true;
                }
                Q.push(cast<Statement>(op));
                V.insert(op);
            }
        }
        if (LLVM_UNLIKELY(Q.empty())) {
            break;
        }
        ancestor = Q.front();
        Q.pop();
    }
    return false;
}

void isTopologicallyOrdered(const PabloBlock * block, const OrderingVerifier & parent) {
    OrderingVerifier ov(parent);
    for (const Statement * stmt : *block) {
        if (LLVM_UNLIKELY(isa<While>(stmt))) {
            isTopologicallyOrdered(cast<While>(stmt)->getBody(), ov);
            for (const Next * var : cast<While>(stmt)->getVariants()) {
                ov.insert(var);
            }
        }
        for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
            const PabloAST * const op = stmt->getOperand(i);
            if (LLVM_UNLIKELY((isa<Statement>(op) || isa<Var>(op)) && ov.count(op) == 0)) {
                std::string tmp;
                raw_string_ostream str(tmp);
                str << "PabloVerifier: ordering volation: ";
                if (LLVM_UNLIKELY(recursivelyDefined(stmt))) {
                    PabloPrinter::print(stmt, str);
                    str << " is defined by a recursive function!";
                    throw std::runtime_error(str.str());
                }
                // TODO: make this actually test whether the operand is ever defined,
                // or if it was defined in a scope that cannot be reached?

                str << "function is not topologically ordered! ";
                PabloAST * op = stmt->getOperand(i);
                PabloPrinter::print(op, str);
                if (LLVM_UNLIKELY(isa<Statement>(op) && unreachable(stmt, cast<Statement>(op)->getParent()))) {
                    str << " was defined in a scope that is unreachable by ";
                } else {
                    str << " was used before definition by ";
                }
                PabloPrinter::print(stmt, str);
                throw std::runtime_error(str.str());
            }
        }
        ov.insert(stmt);
        if (LLVM_UNLIKELY(isa<If>(stmt))) {
            isTopologicallyOrdered(cast<If>(stmt)->getBody(), ov);
            for (const Assign * def : cast<If>(stmt)->getDefined()) {
                ov.insert(def);
            }
        }
    }
}

void isTopologicallyOrdered(const PabloFunction & function) {
    OrderingVerifier ov;
    for (unsigned i = 0; i != function.getNumOfParameters(); ++i) {
        ov.insert(function.getParameter(i));
    }
    isTopologicallyOrdered(function.getEntryBlock(), ov);
}

void PabloVerifier::verify(const PabloFunction & function, const std::string location) {
    try {
        verifyProgramStructure(function);
        verifyUseDefInformation(function);
        isTopologicallyOrdered(function);
    } catch(std::runtime_error err) {
        raw_os_ostream out(std::cerr);
        PabloPrinter::print(function, out);
        out.flush();
        if (location.empty()) {
            throw err;
        } else {
            throw std::runtime_error(std::string(err.what()) + " @ " + location);
        }
    }
}

}
