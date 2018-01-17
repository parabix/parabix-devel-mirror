#include "pabloverifier.hpp"
#include <pablo/branch.h>
#include <pablo/pe_var.h>
#include <pablo/ps_assign.h>
#include <pablo/arithmetic.h>
#include <pablo/codegenstate.h>
#include <pablo/pablo_kernel.h>
#include <pablo/printer_pablos.h>
#include <llvm/Support/ErrorHandling.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/ADT/SmallSet.h>
#include <llvm/ADT/DenseMap.h>
#include <llvm/ADT/SmallBitVector.h>
#include <llvm/IR/Type.h>

using namespace llvm;

namespace pablo {

using TypeId = PabloAST::ClassTypeId;

using ScopeSet = SmallSet<const PabloBlock *, 32>;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief verifyUseDefInformation
 ** ------------------------------------------------------------------------------------------------------------- */
void testUsers(const PabloAST * expr, const ScopeSet & validScopes) { 
    size_t uses = 0;
    SmallSet<const PabloAST *, 16> verified;
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
        } else if (isa<Var>(expr)) {
            if (LLVM_UNLIKELY(isa<Branch>(use) || isa<Operator>(use) || isa<PabloKernel>(use))) {
                ++uses;
            } else {
                std::string tmp;
                raw_string_ostream str(tmp);
                str << "use-def error: var ";
                PabloPrinter::print(use, str);
                str << " is a user of ";
                PabloPrinter::print(expr, str);
                str << " but can only be a user of a Branch, Operator or Kernel.";
                throw std::runtime_error(str.str());
            }
        } else if (const Operator * const user = dyn_cast<Operator>(use)) {
            if (user->getLH() == expr) {
                ++uses;
            }
            if (user->getRH() == expr) {
                ++uses;
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
    gatherValidScopes(kernel->getEntryScope(), validScopes);
    for (unsigned i = 0; i < kernel->getNumOfInputs(); ++i) {
        testUsers(kernel->getInput(i), validScopes);
    }
    for (unsigned i = 0; i < kernel->getNumOfOutputs(); ++i) {
        testUsers(kernel->getOutput(i), validScopes);
    }
    verifyUseDefInformation(kernel->getEntryScope(), validScopes);
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
        case TypeId::Assign:
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
    verifyProgramStructure(kernel->getEntryScope(), nestingDepth);
    if (LLVM_UNLIKELY(nestingDepth != 0)) {
        // This error isn't actually possible to occur with the current AST structure but that could change
        // in the future. Leaving this test in for a reminder to check for it.
        throw std::runtime_error("PabloVerifier: unbalanced If or While nesting depth.");
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief verifyAllPathsDominate
 ** ------------------------------------------------------------------------------------------------------------- */
void verifyAllPathsDominate(const PabloBlock * block) {
    for (const Statement * stmt : *block) {
        for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
            const PabloAST * const op = stmt->getOperand(i);
            if (LLVM_UNLIKELY(!dominates(op, stmt))) {
                std::string tmp;
                raw_string_ostream out(tmp);
                PabloPrinter::print(cast<Statement>(op), out);
                out << " does not dominate ";
                PabloPrinter::print(stmt, out);
                throw std::runtime_error(out.str());
            }
        }
        if (LLVM_UNLIKELY(isa<Branch>(stmt))) {
            verifyAllPathsDominate(cast<Branch>(stmt)->getBody());
        }
    }
}

void verifyAllPathsDominate(const PabloKernel * kernel) {
    verifyAllPathsDominate(kernel->getEntryScope());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief verifyVariableAssignments
 ** ------------------------------------------------------------------------------------------------------------- */
struct AssignmentSet {
    AssignmentSet() : mParent(nullptr), mSet() {}
    AssignmentSet(const AssignmentSet & parent) : mParent(&parent) {}
    bool contains(const Var * expr) const {
        if (mSet.count(expr)) {
            return true;
        } else if (mParent) {
            return mParent->contains(expr);
        }
        return false;
    }

    void insert_full(const Var * expr) {
        const auto n = getNumOfElements(expr);
        auto f = mAssignment.find(expr);
        if (LLVM_LIKELY(f == mAssignment.end())) {
            mAssignment.insert(std::move(std::make_pair(expr, SmallBitVector(n, true))));
        } else {
            f->second.resize(n, true);
        }
    }

    void insert(const Var * expr, const unsigned i) {
        mSet.insert(expr);
    }
protected:

    static unsigned getNumOfElements(const Var * expr) {
        const Type * const ty = expr->getType();
        if (ty->isArrayTy()) {
            return ty->getArrayNumElements();
        }
        return 1;
    }

private:
    const AssignmentSet * const mParent;
    DenseMap<const Var *, SmallBitVector> mAssignment;

    SmallSet<const Var *, 16> mSet;
};

//void verifyVariableUsages(const PabloBlock * block, const AssignmentSet & parent) {
//    AssignmentSet A(parent);
//    for (const Statement * stmt : *block) {
//        if (isa<Assign>(stmt)) {
//            PabloAST * var = cast<Assign>(stmt)->getVariable();
//            if (isa<Extract>(var)) {
//                var = cast<Extract>(var)->getArray();
//            }
//            A.insert(cast<Var>(var));
//        } else if (isa<Extract>(stmt)) {
//            Var * const var = cast<Var>(cast<Extract>(var)->getArray());
//            if (A.contains(var)) {
//                continue;
//            }
//        } else {
//            for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
//                const PabloAST * const op = stmt->getOperand(i);
//                if (isa<Var>(op)) {

//                }
//            }
//        }



//        for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
//            const PabloAST * const op = stmt->getOperand(i);
//            if (LLVM_UNLIKELY(!dominates(op, stmt))) {
//                std::string tmp;
//                raw_string_ostream out(tmp);
//                PabloPrinter::print(cast<Statement>(op), out);
//                out << " does not dominate ";
//                PabloPrinter::print(stmt, out);
//                throw std::runtime_error(out.str());
//            }
//        }
//        if (LLVM_UNLIKELY(isa<Branch>(stmt))) {
//            verifyAllPathsDominate(cast<Branch>(stmt)->getBody());
//        }
//    }
//}

//void verifyVariableUsages(const PabloKernel * kernel) {
//    AssignmentSet A;
//    for (unsigned i = 0; i != kernel->getNumOfInputs(); ++i) {
//        A.insert(kernel->getInput(i));
//    }
//    for (unsigned i = 0; i != kernel->getNumOfOutputs(); ++i) {
//        A.insert(kernel->getOutput(i));
//    }
//    verifyVariableUsages(kernel->getEntryScope(), A);
//}



void PabloVerifier::verify(const PabloKernel * kernel, const std::string & location) {
    try {
        verifyProgramStructure(kernel);
        verifyUseDefInformation(kernel);
        verifyAllPathsDominate(kernel);
    } catch(std::runtime_error & err) {
        PabloPrinter::print(kernel, errs());
        errs().flush();
        if (location.empty()) {
            llvm::report_fatal_error(err.what());
        } else {
            llvm::report_fatal_error(std::string(err.what()) + " @ " + location);
        }
    }
}

}
