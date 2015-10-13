#include <pablo/optimizers/pablo_simplifier.hpp>
#include <pablo/codegenstate.h>
#include <pablo/expression_map.hpp>
#include <pablo/function.h>
#include <pablo/printer_pablos.h>
#include <pablo/analysis/pabloverifier.hpp>
#ifdef USE_BOOST
#include <boost/container/flat_set.hpp>
#else
#include <unordered_set>
#endif
#include <iostream>

namespace pablo {

#ifdef USE_BOOST
template <typename Type>
using SmallSet = boost::container::flat_set<Type>;
#else
template <typename Type>
using SmallSet = std::unordered_set<Type>;
#endif

bool Simplifier::optimize(PabloFunction & function) {
    eliminateRedundantCode(function.getEntryBlock());
    deadCodeElimination(function.getEntryBlock());
    eliminateRedundantEquations(function.getEntryBlock());
    #ifndef NDEBUG
    PabloVerifier::verify(function, "post-simplification");
    #endif
    return true;
}

inline static PabloAST * canTriviallyFold(Statement * stmt, PabloBlock & block) {
    for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
        if (LLVM_UNLIKELY(isa<Zeroes>(stmt->getOperand(i)))) {
            switch (stmt->getClassTypeId()) {
                case PabloAST::ClassTypeId::And:
                case PabloAST::ClassTypeId::Advance:
                    return block.createZeroes();
                case PabloAST::ClassTypeId::Xor:
                case PabloAST::ClassTypeId::Or:
                    return stmt->getOperand(1 - i);
                case PabloAST::ClassTypeId::Not:
                    return block.createOnes();
                case PabloAST::ClassTypeId::Sel:
                    block.setInsertPoint(stmt->getPrevNode());
                    switch (i) {
                        case 0: return stmt->getOperand(2);
                        case 1: return block.createAnd(block.createNot(stmt->getOperand(0)), stmt->getOperand(2));
                        case 2: return block.createAnd(stmt->getOperand(0), stmt->getOperand(1));
                    }
                case PabloAST::ClassTypeId::ScanThru:
                case PabloAST::ClassTypeId::MatchStar:
                    return stmt->getOperand(0);
                default: break;
            }
        } else if (LLVM_UNLIKELY(isa<Ones>(stmt->getOperand(i)))) {
            block.setInsertPoint(stmt->getPrevNode());
            switch (stmt->getClassTypeId()) {
                case PabloAST::ClassTypeId::And:
                    return stmt->getOperand(1 - i);
                case PabloAST::ClassTypeId::Or:
                    return block.createOnes();
                case PabloAST::ClassTypeId::Xor:
                    return block.createNot(stmt->getOperand(1 - i));
                case PabloAST::ClassTypeId::Not:
                    return block.createZeroes();
                case PabloAST::ClassTypeId::Sel:
                    block.setInsertPoint(stmt->getPrevNode());
                    switch (i) {
                        case 0: return stmt->getOperand(1);
                        case 1: return block.createOr(stmt->getOperand(0), stmt->getOperand(2));
                        case 2: return block.createOr(block.createNot(stmt->getOperand(0)), stmt->getOperand(1));
                    }
                default: break;
            }
        }
    }
    return nullptr;
}

inline bool Simplifier::isSuperfluous(const Assign * const assign) {
    for (const PabloAST * inst : assign->users()) {
        if (isa<Next>(inst) || isa<PabloFunction>(inst)) {
            return false;
        } else if (isa<If>(inst)) {
            const If * ifNode = cast<If>(inst);
            if (ifNode->getCondition() == assign) {
                bool notFound = true;
                for (Assign * defVar : ifNode->getDefined()) {
                    if (defVar == assign) {
                        notFound = false;
                        break;
                    }
                }
                if (notFound) {
                    continue;
                }
            }
            return false;
        }
    }
    return true;
}

inline bool demoteDefinedVar(const If * ifNode, const Assign * def) {
    // If this value isn't used outside of this scope then there is no reason to allow it to escape it.
    if (!escapes(def)) {
        return true;
    }
    // Similarly if the defined variable is equivalent to the condition, an equivalent value is already available.
    if (ifNode->getCondition() == def->getExpression()) {
        return true;
    }
    // Finally, if the assignment is a constant, it's already known.
    if (isa<Zeroes>(def->getExpression()) || isa<Ones>(def->getExpression()))  {
        return true;
    }
    return false;
}

inline void replaceReachableUsersOfWith(Statement * stmt, PabloAST * expr) {
    const PabloBlock * const root = stmt->getParent();
    SmallSet<const PabloBlock *> forbidden;
    for (PabloAST * use : stmt->users()) {
        if (LLVM_UNLIKELY(isa<Next>(use))) {
            const PabloBlock * parent = cast<Next>(use)->getParent();
            if (parent != root) {
                forbidden.insert(parent);
            }
        }
    }
    for (PabloAST * use : stmt->users()) {
        if (Statement * user = dyn_cast<Statement>(use)) {
            const PabloBlock * parent = user->getParent();
            while (parent && forbidden.count(parent) == 0) {
                if (LLVM_UNLIKELY(parent == root)) {
                    user->replaceUsesOfWith(stmt, expr);
                    break;
                }
                parent = parent->getParent();
            }
        }
    }
}

template <class ValueList>
inline void removeIdenticalEscapedValues(ValueList & list) {
    for (auto i = list.begin(); i != list.end(); ++i) {
        for (auto j = i + 1; j != list.end(); ) {
            if (LLVM_UNLIKELY(equals(*i, *j))) {
                Statement * redundantValue = *j;
                j = list.erase(j);
                redundantValue->replaceWith(*i, false, true);
                continue;
            }
            ++j;
        }
    }
}

void Simplifier::eliminateRedundantCode(PabloBlock & block, ExpressionTable * predecessor) {
    ExpressionTable encountered(predecessor);
    Statement * stmt = block.front();
    while (stmt) {
        if (Assign * assign = dyn_cast<Assign>(stmt)) {
            // If we have an Assign whose users do not contain an If or Next node, we can replace its users with
            // the Assign's expression directly.
            if (isSuperfluous(assign)) {
                if (assign->getNumUses() == 0) {
                    stmt = assign->eraseFromParent();
                } else {
                    stmt = assign->replaceWith(assign->getExpression(), true, true);
                }
                continue;
            }
            // Force the uses of an Assign node that can reach the original expression to use the expression instead.
            replaceReachableUsersOfWith(assign, assign->getExpression());
        } else if (Next * next = dyn_cast<Next>(stmt)) {
            replaceReachableUsersOfWith(next, next->getExpr());
        } else if (If * ifNode = dyn_cast<If>(stmt)) {
            // Check to see if the Cond is Zero and delete the loop.
            if (LLVM_UNLIKELY(isa<Zeroes>(ifNode->getCondition()))) {
                stmt = stmt->eraseFromParent(true);
                continue;
            }

            bool evaluate = true;
            If::DefinedVars & defs = ifNode->getDefined();
            while (evaluate) {
                evaluate = false;
                // Process the If body
                eliminateRedundantCode(cast<If>(stmt)->getBody(), &encountered);
                // Now test whether all of the defined variables are necessary
                for (auto itr = defs.begin(); itr != defs.end(); ) {
                    Assign * def = *itr;
                    if (LLVM_UNLIKELY(demoteDefinedVar(ifNode, def))) {
                        itr = defs.erase(itr);
                        def->replaceWith(def->getExpression(), false, true);
                        evaluate = true;
                        continue;
                    }
                    ++itr;
                }
            }

            // If we ended up removing all of the defined variables, delete the If node.
            if (LLVM_UNLIKELY(defs.empty())) {
                stmt = stmt->eraseFromParent(true);
                continue;
            }
            // Otherwise check if we any Assign reports the same value as another. If so, replace all uses of the
            // second with the first. This will simplify future analysis.
            removeIdenticalEscapedValues(ifNode->getDefined());

        } else if (While * whileNode = dyn_cast<While>(stmt)) {
            const PabloAST * initial = whileNode->getCondition();
            if (LLVM_LIKELY(isa<Next>(initial))) {
                initial = cast<Next>(initial)->getInitial();
            }
            if (LLVM_UNLIKELY(isa<Zeroes>(initial))) {
                stmt = stmt->eraseFromParent(true);
                continue;
            }
            eliminateRedundantCode(whileNode->getBody(), &encountered);
            removeIdenticalEscapedValues(whileNode->getVariants());
        } else if (PabloAST * expr = canTriviallyFold(stmt, block)) {
            stmt = stmt->replaceWith(expr, true, true);
            continue;
        } else {
            // When we're creating the Pablo program, it's possible to have multiple instances of an "identical"
            // statement. By recording which statements have already been seen, we can detect the redundant statements
            // as any having the same type and operands. If so, we can replace its users with the prior statement.
            // and erase this statement from the AST
            const auto f = encountered.findOrAdd(stmt);
            if (!f.second) {
                stmt = stmt->replaceWith(f.first, true, true);
                continue;
            }
        }
        stmt = stmt->getNextNode();
    }
}


void Simplifier::deadCodeElimination(PabloBlock & block) {
    Statement * stmt = block.front();
    while (stmt) {
        if (isa<If>(stmt)) {
            deadCodeElimination(cast<If>(stmt)->getBody());
        } else if (isa<While>(stmt)) {
            deadCodeElimination(cast<While>(stmt)->getBody());
        } else if (stmt->getNumUses() == 0){
            stmt = stmt->eraseFromParent(true);
            continue;
        }
        stmt = stmt->getNextNode();
    }
}

void Simplifier::eliminateRedundantEquations(PabloBlock & block) {
    Statement * stmt = block.front();
    while (stmt) {
        if (isa<If>(stmt)) {
            eliminateRedundantEquations(cast<If>(stmt)->getBody());
        }
        else if (isa<While>(stmt)) {
            eliminateRedundantEquations(cast<While>(stmt)->getBody());
        } else if (isa<Advance>(stmt)) {
            Advance * adv = cast<Advance>(stmt);
            if (LLVM_UNLIKELY(isa<Advance>(adv->getOperand(0)))) {
                // Replace an Advance(Advance(x, n), m) with an Advance(x,n + m)
                Advance * op = cast<Advance>(stmt->getOperand(0));
                if (LLVM_UNLIKELY(op->getNumUses() == 1)) {
                    adv->setOperand(0, op->getOperand(0));
                    adv->setOperand(1, block.getInteger(adv->getAdvanceAmount() + op->getAdvanceAmount()));
                    op->eraseFromParent(false);
                }
            }
        } else if (LLVM_UNLIKELY(isa<ScanThru>(stmt))) {
            ScanThru * scanThru = cast<ScanThru>(stmt);
            if (LLVM_UNLIKELY(isa<Advance>(scanThru->getOperand(0)))) {
                // Replace a ScanThru(Advance(x,n),y) with an ScanThru(Advance(x, n - 1), Advance(x, n - 1) | y), where Advance(x, 0) = x
                Advance * op = cast<Advance>(stmt->getOperand(0));
                if (LLVM_UNLIKELY(op->getNumUses() == 1)) {
                    block.setInsertPoint(scanThru->getPrevNode());
                    PabloAST * expr = block.createAdvance(op->getOperand(0), op->getAdvanceAmount() - 1);
                    scanThru->setOperand(0, expr);
                    scanThru->setOperand(1, block.createOr(scanThru->getOperand(1), expr));
                    op->eraseFromParent(false);
                }
            }
        }
        stmt = stmt->getNextNode();
    }
    block.setInsertPoint(block.back());
}

}
