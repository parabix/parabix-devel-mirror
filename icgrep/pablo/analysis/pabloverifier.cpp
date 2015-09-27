#include "pabloverifier.hpp"
#include <pablo/function.h>
#include <pablo/codegenstate.h>
#include <pablo/printer_pablos.h>
#include <iostream>
#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>

using namespace boost::container;

namespace pablo {

using ScopeSet = flat_set<const PabloBlock *>;

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

void verifyUseDefInformation(const PabloBlock & block, const ScopeSet & validScopes) {
    for (const Statement * stmt : block) {

        for (const PabloAST * use : stmt->users()) {
            if (LLVM_LIKELY(isa<Statement>(use))) {
                const Statement * const user = cast<Statement>(use);
                // test whether this user is in a block in the program
                if (LLVM_UNLIKELY(validScopes.count(user->getParent()) == 0)) {
                    std::string tmp;
                    raw_string_ostream str(tmp);
                    PabloPrinter::print(user, "PabloVerifier: use-def error: ", str);
                    PabloPrinter::print(stmt, " is a user of ", str);
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
                PabloPrinter::print(stmt, "PabloVerifier: def-use error: ", str);
                str << " is not a user of ";
                PabloPrinter::print(def, str);
                throw std::runtime_error(str.str());
            }
        }
        if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
            verifyUseDefInformation(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody(), validScopes);
        }
    }
}

void gatherValidScopes(const PabloBlock & block, ScopeSet & validScopes) {
    validScopes.insert(&block);
    for (const Statement * stmt : block) {
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
 * @brief verifyProgramStructure
 ** ------------------------------------------------------------------------------------------------------------- */
void verifyProgramStructure(const PabloBlock & block) {
    const Statement * prev = nullptr;
    for (const Statement * stmt : block) {
        if (LLVM_UNLIKELY(stmt->getPrevNode() != prev)) {
            std::string tmp;
            raw_string_ostream str(tmp);
            PabloPrinter::print(stmt, "PabloVerifier: structure error: ", str);
            str << " succeeds ";
            PabloPrinter::print(prev, str);
            str << " but expects to preceed  ";
            PabloPrinter::print(stmt->getPrevNode(), str);
            throw std::runtime_error(str.str());
        }
        prev = stmt;
        if (LLVM_UNLIKELY(stmt->getParent() != &block)) {
            std::string tmp;
            raw_string_ostream str(tmp);
            PabloPrinter::print(stmt, "PabloVerifier: structure error: ", str);
            str << " is not contained in its reported scope block";
            throw std::runtime_error(str.str());
        }
        if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
            const PabloBlock & nested = isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody();
            if (LLVM_UNLIKELY(nested.getParent() != &block)) {
                std::string tmp;
                raw_string_ostream str(tmp);
                str << "PabloVerifier: structure error: body of ";
                PabloPrinter::print(stmt, str);
                str << " is not nested within the expected scope block";
                throw std::runtime_error(str.str());
            }
            const Statement * misreportedEscapingValue = nullptr;
            if (isa<If>(stmt)) {
                for (const Assign * def : cast<If>(stmt)->getDefined()) {
                    if (def->getParent() != &block) {
                        misreportedEscapingValue = def;
                        break;
                    }
                }
            } else {
                for (const Next * var : cast<While>(stmt)->getVariants()) {
                    if (var->getParent() != &block) {
                        misreportedEscapingValue = var;
                        break;
                    }
                }
            }
            if (misreportedEscapingValue) {
                std::string tmp;
                raw_string_ostream str(tmp);
                str << "PabloVerifier: structure error: ";
                PabloPrinter::print(misreportedEscapingValue, str);
                str << " is not contained within the body of ";
                PabloPrinter::print(stmt, str);
                throw std::runtime_error(str.str());
            }
            verifyProgramStructure(nested);
        }
    }
}

inline void verifyProgramStructure(const PabloFunction & function) {
    verifyProgramStructure(function.getEntryBlock());
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
    boost::container::flat_set<const PabloAST *> mSet;
};

void isTopologicallyOrdered(const PabloBlock & block, const OrderingVerifier & parent) {
    OrderingVerifier ov(parent);
    for (const Statement * stmt : block) {
        if (LLVM_UNLIKELY(isa<While>(stmt))) {
            isTopologicallyOrdered(cast<While>(stmt)->getBody(), ov);
            for (const Next * var : cast<While>(stmt)->getVariants()) {
                ov.insert(var);
            }
        }
        for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
            const PabloAST * const op = stmt->getOperand(i);
            if (LLVM_UNLIKELY((isa<Statement>(op) || isa<Var>(op)) && ov.count(op) == 0)) {
                // TODO: make this actually test whether the operand is ever defined,
                // or if it was defined in a scope that cannot be reached?
                std::string tmp;
                raw_string_ostream str(tmp);
                str << "PabloVerifier: function is not topologically ordered! ";
                PabloPrinter::print(stmt->getOperand(i), str);
                PabloPrinter::print(stmt, " was used before definition by ", str);
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
        PabloPrinter::print(function.getEntryBlock().statements(), out);
        out.flush();
        if (location.empty()) {
            throw err;
        } else {
            throw std::runtime_error(std::string(err.what()) + " @ " + location);
        }
    }
}

}
