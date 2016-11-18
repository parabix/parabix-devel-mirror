#include "pabloverifier.hpp"
#include <pablo/pablo_kernel.h>
#include <pablo/codegenstate.h>
#include <pablo/printer_pablos.h>
#include <iostream>
#include <boost/container/flat_set.hpp>
#include <queue>


namespace pablo {

using TypeId = PabloAST::ClassTypeId;

template <typename Type>
using SmallSet = boost::container::flat_set<Type>;

using ScopeSet = SmallSet<const PabloBlock *>;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief verifyUseDefInformation
 ** ------------------------------------------------------------------------------------------------------------- */
void testUsers(const PabloAST * expr, const ScopeSet & validScopes) { 
    size_t uses = 0;
    SmallSet<const PabloAST *> verified;
    for (const PabloAST * use : expr->users()) {
        if (LLVM_UNLIKELY(verified.count(use) != 0)) {
            continue;
        }
        if (const Statement * const user = dyn_cast<Statement>(use)) {
            // test whether this user is in a block in the program
            if (LLVM_UNLIKELY(user->getParent() == nullptr || validScopes.count(user->getParent()) == 0)) {
                std::string tmp;
                raw_string_ostream str(tmp);
                str << "use-def error: ";
                PabloPrinter::print(user, str);
                str << " is a user of ";
                PabloPrinter::print(expr, str);
                str << " but ";
                PabloPrinter::print(use, str);
                if (user->getParent() == nullptr) {
                    str << " is not defined in any scope.";
                } else {
                    str << " is in an unreachable scope.";
                }
                throw std::runtime_error(str.str());
            }
            // expr may be used more than once by the same user.
            bool notFound = true;
            for (unsigned i = 0; i != user->getNumOperands(); ++i) {
                if (user->getOperand(i) == expr) {
                    notFound = false;
                    ++uses;
                }
            }
            if (isa<Branch>(user)) {
                for (const PabloAST * var : cast<Branch>(user)->getEscaped()) {
                    if (var == expr) {
                        notFound = false;
                        ++uses;
                    }
                }
            }
            if (LLVM_UNLIKELY(notFound)) {
                std::string tmp;
                raw_string_ostream str(tmp);
                str << "use-def error: ";
                PabloPrinter::print(expr, str);
                str << " is not a definition of ";
                PabloPrinter::print(use, str);
                throw std::runtime_error(str.str());
            }
        } else if (isa<Var>(use)) {
            if (LLVM_UNLIKELY(isa<Branch>(expr))) {
                ++uses;
            } else {
                std::string tmp;
                raw_string_ostream str(tmp);
                str << "use-def error: var ";
                PabloPrinter::print(use, str);
                str << " is a user of ";
                PabloPrinter::print(expr, str);
                str << " but can only be a user of a Branch or Function.";
                throw std::runtime_error(str.str());
            }
        }
        verified.insert(use);
    }
    if (LLVM_UNLIKELY(uses != expr->getNumUses())) {
        std::string tmp;
        raw_string_ostream str(tmp);
        str << "use-def error: ";
        PabloPrinter::print(expr, str);
        str << " is reported having " << expr->getNumUses() << " user(s)"
            << " but was observed having " << uses << " user(s)";
        throw std::runtime_error(str.str());
    }
}

void testDefs(const Statement * stmt) {
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
}

void verifyUseDefInformation(const PabloBlock * block, const ScopeSet & validScopes) {
    for (const Statement * stmt : *block) {
        testUsers(stmt, validScopes);
        testDefs(stmt);
        if (LLVM_UNLIKELY(isa<Branch>(stmt))) {
            verifyUseDefInformation(cast<Branch>(stmt)->getBody(), validScopes);
        }
    }
}

void gatherValidScopes(const PabloBlock * block, ScopeSet & validScopes) {
    validScopes.insert(block);
    for (const Statement * stmt : *block) {
        if (LLVM_UNLIKELY(isa<Branch>(stmt))) {
            gatherValidScopes(cast<Branch>(stmt)->getBody(), validScopes);
        }
    }
}

void verifyUseDefInformation(const PabloKernel * kernel) {
    ScopeSet validScopes;
    gatherValidScopes(kernel->getEntryBlock(), validScopes);
    for (unsigned i = 0; i < kernel->getNumOfInputs(); ++i) {
        testUsers(kernel->getInput(i), validScopes);
    }
    for (unsigned i = 0; i < kernel->getNumOfOutputs(); ++i) {
        testUsers(kernel->getOutput(i), validScopes);
    }
    verifyUseDefInformation(kernel->getEntryBlock(), validScopes);
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
        parent = parent->getPredecessor();
    }
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief throwReportedScopeError
 ** ------------------------------------------------------------------------------------------------------------- */
static void throwReportedScopeError(const Statement * const stmt) {
    std::string tmp;
    raw_string_ostream str(tmp);
    str << "structure error: ";
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
    str << "structure error: ";
    PabloPrinter::print(stmt, str);
    str << " branches into a scope block that reports ";
    PabloPrinter::print(branch, str);
    str << " as its branching statement.";
    throw std::runtime_error(str.str());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief illegalOperandType
 ** ------------------------------------------------------------------------------------------------------------- */
static inline bool illegalOperandType(const PabloAST * const op) {
    switch (op->getClassTypeId()) {
        case TypeId::Block:
        case TypeId::Function:
        case TypeId::Prototype:
        case TypeId::Assign:
        case TypeId::Call:
        case TypeId::SetIthBit:
        case TypeId::If:
        case TypeId::While:
            return true;
        default:
            return false;
    }
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
            PabloPrinter::print(stmt, str);
            str << " succeeds ";
            PabloPrinter::print(prev, str);
            str << " but ";
            PabloPrinter::print(cast<PabloAST>(stmt), str);
            str << " expects to succeed ";
            PabloPrinter::print(stmt->getPrevNode(), str);
            throw std::runtime_error(str.str());
        }
        prev = stmt;
        if (LLVM_UNLIKELY(stmt->getParent() != block)) {
            std::string tmp;
            raw_string_ostream str(tmp);
            PabloPrinter::print(stmt, str);
            str << " is not contained in its reported scope block";
            throw std::runtime_error(str.str());
        }

        for (unsigned i = 0; i < stmt->getNumOperands(); ++i) {
            PabloAST * op = stmt->getOperand(i);
            if (LLVM_UNLIKELY(illegalOperandType(op))) {
                std::string tmp;
                raw_string_ostream str(tmp);
                PabloPrinter::print(op, str);
                str << " cannot be an operand of ";
                PabloPrinter::print(stmt, str);
                throw std::runtime_error(str.str());
            }
        }

        if (LLVM_UNLIKELY(isa<Assign>(stmt))) {

            PabloAST * const variable = cast<Assign>(stmt)->getVariable();
            if (LLVM_UNLIKELY(!isa<Var>(variable) && !isa<Extract>(variable))) {
                std::string tmp;
                raw_string_ostream out(tmp);
                out << "invalid assignment: ";
                PabloPrinter::print(stmt, out);
                out << "  --- ";
                PabloPrinter::print(variable, out);
                out << " must be a Var or Extract";
                throw std::runtime_error(out.str());
            }

            PabloAST * const value = cast<Assign>(stmt)->getValue();
            if (LLVM_UNLIKELY(variable->getType() != value->getType())) {
                std::string tmp;
                raw_string_ostream out(tmp);
                out << "invalid assignment: ";
                PabloPrinter::print(stmt, out);
                out << "  --- type of ";
                PabloPrinter::print(variable, out);
                out << " differs from ";
                PabloPrinter::print(value, out);
                throw std::runtime_error(out.str());
            }

        } else if (LLVM_UNLIKELY(isa<Branch>(stmt))) {
            const PabloBlock * nested = cast<Branch>(stmt)->getBody();
            if (LLVM_UNLIKELY(nested->getBranch() != stmt)) {
                throwMisreportedBranchError(stmt, nested->getBranch());
            } else if (LLVM_UNLIKELY(nested->getPredecessor() != block)) {
                throwReportedScopeError(stmt);
            }
            ++nestingDepth;
            verifyProgramStructure(nested, nestingDepth);
            --nestingDepth;
        }
    }    
}

inline void verifyProgramStructure(const PabloKernel * kernel) {
    unsigned nestingDepth = 0;
    verifyProgramStructure(kernel->getEntryBlock(), nestingDepth);
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

void isTopologicallyOrdered(const PabloBlock * block, const OrderingVerifier & parent) {
    OrderingVerifier ov(parent);
    for (const Statement * stmt : *block) {
        if (LLVM_UNLIKELY(isa<While>(stmt))) {
            isTopologicallyOrdered(cast<While>(stmt)->getBody(), ov);
            for (const Var * var : cast<While>(stmt)->getEscaped()) {
                ov.insert(var);
            }
        } else if (LLVM_UNLIKELY(isa<Assign>(stmt))) {
            ov.insert(cast<Assign>(stmt)->getVariable());
        }
        for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
            const PabloAST * const op = stmt->getOperand(i);
            if (LLVM_UNLIKELY((isa<Statement>(op) || isa<Var>(op)) && ov.count(op) == 0)) {
                std::string tmp;
                raw_string_ostream out(tmp);
                if (isa<Var>(op)) {
                    PabloPrinter::print(op, out);
                    out << " is used by ";
                    PabloPrinter::print(stmt, out);
                    out << " before being assigned a value.";
                } else {
                    PabloPrinter::print(op, out);
                    if (LLVM_UNLIKELY(isa<Statement>(op) && unreachable(stmt, cast<Statement>(op)->getParent()))) {
                        out << " was defined in a scope that is unreachable by ";
                    } else {
                        out << " was used before definition by ";
                    }
                    PabloPrinter::print(stmt, out);
                }
                throw std::runtime_error(out.str());
            }
        }
        ov.insert(stmt);
        if (LLVM_UNLIKELY(isa<If>(stmt))) {
            isTopologicallyOrdered(cast<If>(stmt)->getBody(), ov);
            for (const Var * def : cast<If>(stmt)->getEscaped()) {
                ov.insert(def);
            }
        }
    }
}

void isTopologicallyOrdered(const PabloKernel * kernel) {
    OrderingVerifier ov;
    for (unsigned i = 0; i != kernel->getNumOfInputs(); ++i) {
        ov.insert(kernel->getInput(i));
    }
    for (unsigned i = 0; i != kernel->getNumOfOutputs(); ++i) {
        ov.insert(kernel->getOutput(i));
    }
    isTopologicallyOrdered(kernel->getEntryBlock(), ov);
}

void PabloVerifier::verify(const PabloKernel * kernel, const std::string & location) {
    try {
        verifyProgramStructure(kernel);
        verifyUseDefInformation(kernel);
        isTopologicallyOrdered(kernel);
    } catch(std::runtime_error & err) {
        raw_os_ostream out(std::cerr);
        PabloPrinter::print(kernel, out);
        out.flush();
        if (location.empty()) {
            llvm::report_fatal_error(err.what());
        } else {
            llvm::report_fatal_error(std::string(err.what()) + " @ " + location);
        }
    }
}

}
