#include <pablo/optimizers/pablo_simplifier.hpp>
#include <pablo/codegenstate.h>
#include <pablo/expression_map.hpp>
#include <pablo/function.h>
#include <pablo/printer_pablos.h>



namespace pablo {

bool Simplifier::optimize(PabloFunction & function) {
    eliminateRedundantCode(function.getEntryBlock());
    deadCodeElimination(function.getEntryBlock());
    eliminateRedundantComplexStatements(function.getEntryBlock());
    return true;
}

inline static bool canTriviallyFold(const Statement * stmt) {
    for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
        switch (stmt->getOperand(i)->getClassTypeId()) {
            case PabloAST::ClassTypeId::Zeroes:
            case PabloAST::ClassTypeId::Ones:
                return true;
            default: break;
        }
    }
    return false;
}

#ifndef NDEBUG
static bool verifyStatementIsInSameBlock(const Statement * const stmt, const PabloBlock & block) {
    Statement * curr = block.front();
    while (curr) {
        if (curr == stmt) {
            return true;
        }
        curr = curr->getNextNode();
    }
    return false;
}
#endif

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

void Simplifier::eliminateRedundantCode(PabloBlock & block, ExpressionTable * predecessor) {
    ExpressionTable encountered(predecessor);

    Statement * stmt = block.front();

    while (stmt) {

        assert (stmt);
        assert (verifyStatementIsInSameBlock(stmt, block));

        if (Assign * assign = dyn_cast<Assign>(stmt)) {
            // If we have an Assign whose users do not contain an If or Next node, we can replace its users with
            // the Assign's expression directly.
            if (isSuperfluous(assign)) {
                if (assign->getNumUses() == 0) {
                    stmt = assign->eraseFromParent();
                }
                else {
                    stmt = assign->replaceWith(assign->getExpression(), true, true);
                }
                continue;
            }
        }
        else if (If * ifNode = dyn_cast<If>(stmt)) {
            // Check to see if the Cond is Zero and delete the loop.
            if (LLVM_UNLIKELY(isa<Zeroes>(ifNode->getCondition()))) {
                for (Assign * defVar : ifNode->getDefined()) {
                    defVar->replaceWith(block.createZeroes(), false, true);
                }
                stmt = stmt->eraseFromParent(true);
                continue;
            }
            // Process the If body
            eliminateRedundantCode(cast<If>(stmt)->getBody(), &encountered);            
            // If the defined variable is actually equivalent to the condition, promote the assignment out of If
            // scope and remove it from the list of defined variables. Otherwise, replace any defined variable that
            // is assigned Zero with a Zero object and remove it from the defined variable list.
            If::DefinedVars & defs = ifNode->getDefined();
            for (auto itr = defs.begin(); itr != defs.end(); ) {
                Assign * def = *itr;
                if (LLVM_UNLIKELY((ifNode->getCondition() == def->getExpression()) || isa<Zeroes>(def->getExpression()))) {
                    itr = defs.erase(itr);
                    def->replaceWith(def->getExpression(), false, true);
                    continue;
                }
                ++itr;
            }
            // If we ended up removing all of the defined variables, delete the If node.
            if (LLVM_UNLIKELY(defs.empty())) {
                stmt = stmt->eraseFromParent(true);
                continue;
            }
            // Otherwise check if we any Assign reports the same value as another. If so, replace all uses of the
            // second with the first. This will simplify future analysis.
            for (auto i = defs.begin(); i != defs.end(); ++i) {
                PabloAST * expr = (*i)->getExpression();
                for (auto j = i + 1; j != defs.end(); ) {
                    Assign * def = (*j);
                    if (LLVM_UNLIKELY(def->getExpression() == expr)) {
                        j = defs.erase(j);
                        def->replaceWith(*i, false, true);
                        continue;
                    }
                    ++j;
                }
            }
        }
        else if (isa<While>(stmt)) {
            eliminateRedundantCode(cast<While>(stmt)->getBody(), &encountered);
        }        
        else if (stmt->getNumUses() == 0) {
            // If this statement has no users, we can discard it. If a future "identical" statement occurs that does
            // have users, we can use that statement instead.
            stmt = stmt->eraseFromParent();
            continue;
        }
        else if (canTriviallyFold(stmt)) { // non-Assign node

            // Do a trivial folding test to see if we're using all 0s or 1s as an operand.
            PabloAST * expr = nullptr;
            block.setInsertPoint(stmt->getPrevNode());
            switch (stmt->getClassTypeId()) {
                case PabloAST::ClassTypeId::Advance:
                    expr = block.createAdvance(stmt->getOperand(0), stmt->getOperand(1));
                    break;
                case PabloAST::ClassTypeId::And:
                    expr = block.createAnd(stmt->getOperand(0), stmt->getOperand(1));
                    break;
                case PabloAST::ClassTypeId::Or:
                    expr = block.createOr(stmt->getOperand(0), stmt->getOperand(1));
                    break;
                case PabloAST::ClassTypeId::Not:
                    expr = block.createNot(stmt->getOperand(0));
                    break;
                case PabloAST::ClassTypeId::Xor:
                    expr = block.createXor(stmt->getOperand(0), stmt->getOperand(1));
                    break;
                case PabloAST::ClassTypeId::Sel:
                    expr = block.createSel(stmt->getOperand(0), stmt->getOperand(1), stmt->getOperand(2));
                    break;
                case PabloAST::ClassTypeId::ScanThru:
                    expr = block.createScanThru(stmt->getOperand(0), stmt->getOperand(1));
                    break;
                case PabloAST::ClassTypeId::MatchStar:
                    expr = block.createMatchStar(stmt->getOperand(0), stmt->getOperand(1));
                    break;
                case PabloAST::ClassTypeId::Next:
                    expr = stmt;
                    break;
                default: {
                    std::string tmp;
                    llvm::raw_string_ostream msg(tmp);
                    PabloPrinter::print(stmt, "Unhandled trivial folding optimization! ", msg);
                    throw std::runtime_error(msg.str());
                }
            }
            stmt = stmt->replaceWith(expr);
            // the next statement could be an Assign; just restart the loop.
            continue;
        }
        else {
            // When we're creating the Pablo program, it's possible to have multiple instances of an "identical"
            // statement. By recording which statements have already been seen, we can detect the redundant statements
            // as any having the same type and operands. If so, we can replace its users with the prior statement.
            // and erase this statement from the AST
            const auto f = encountered.findOrAdd(stmt);
            if (!std::get<1>(f)) {
                stmt = stmt->replaceWith(std::get<0>(f));
                continue;
            }
        }
        assert (stmt);
        stmt = stmt->getNextNode();
    }
    block.setInsertPoint(block.back());
}


void Simplifier::deadCodeElimination(PabloBlock & block) {
    Statement * stmt = block.front();
    while (stmt) {
        if (isa<If>(stmt)) {
            deadCodeElimination(cast<If>(stmt)->getBody());
        }
        else if (isa<While>(stmt)) {
            deadCodeElimination(cast<While>(stmt)->getBody());
        }
        else if (stmt->getNumUses() == 0){
            stmt = stmt->eraseFromParent(true);
            continue;
        }
        stmt = stmt->getNextNode();
    }
}

void Simplifier::eliminateRedundantComplexStatements(PabloBlock & block) {
    Statement * stmt = block.front();
    while (stmt) {
        if (isa<If>(stmt)) {
            eliminateRedundantComplexStatements(cast<If>(stmt)->getBody());
        }
        else if (isa<While>(stmt)) {
            eliminateRedundantComplexStatements(cast<While>(stmt)->getBody());
        }
        else if (stmt->getNumOperands() == 2){
            if (isa<Advance>(stmt)) {
                // If we're advancing an Advance and the internal Advance does not have any other user,
                // we can merge both Advance instructions.
                if (LLVM_UNLIKELY(isa<Advance>(stmt->getOperand(0)))) {
                    Advance * op = cast<Advance>(stmt->getOperand(0));
                    if (LLVM_UNLIKELY(op->getNumUses() == 1)) {
                        Advance * adv = cast<Advance>(stmt);
                        adv->setOperand(0, op->getOperand(0));
                        adv->setOperand(1, block.getInteger(adv->getAdvanceAmount() + op->getAdvanceAmount()));
                        assert(op->getNumUses() == 0);
                        op->eraseFromParent();
                    }
                }
            } else if (LLVM_UNLIKELY(isa<Advance>(stmt->getOperand(1)) && isa<Advance>(stmt->getOperand(0)))) {
                // If an AND, OR or XOR instruction has two Advance instructions as inputs and neither Advance
                // has another user and both shift their input by the same amount, we can perform the AND, OR
                // or XOR on the inputs to the Advances and remove one of the Advance statements.

                Advance * const a0 = cast<Advance>(stmt->getOperand(0));
                Advance * const a1 = cast<Advance>(stmt->getOperand(1));
                switch (stmt->getClassTypeId()) {
                    case PabloAST::ClassTypeId::And:
                    case PabloAST::ClassTypeId::Or:
                    case PabloAST::ClassTypeId::Xor:
                        if (LLVM_UNLIKELY(a0->getNumUses() == 1 && a1->getNumUses() == 1 && a0->getOperand(1) == a1->getOperand(1))) {
                            block.setInsertPoint(stmt);
                            stmt->setOperand(0, a0->getOperand(0));
                            stmt->setOperand(1, a1->getOperand(0));
                            a0->insertAfter(stmt);
                            a0->setOperand(0, stmt);
                            stmt->replaceAllUsesWith(a0);
                            assert(a1->getNumUses() == 0);
                            a1->eraseFromParent();
                            stmt = a0;
                    }
                    default: break;
                }
            } else if (LLVM_UNLIKELY(isa<MatchStar>(stmt->getOperand(1)) && isa<MatchStar>(stmt->getOperand(0))) && isa<Or>(stmt)) {


            } /*else if (LLVM_UNLIKELY(isa<Or>(stmt) && isa<And>(stmt->getOperand(0)) && isa<And>(stmt->getOperand(1)))) {

                // If we have an OR(AND(A,B),AND(NOT(A),C)) statement and neither of the inner operands are used elsewhere, we can
                // promote the Or to a Sel statement.

                And * const a0 = cast<And>(stmt->getOperand(0));
                And * const a1 = cast<And>(stmt->getOperand(1));

                if (LLVM_UNLIKELY(a0->getNumUses() == 1 && a1->getNumUses() == 1)) {

                    bool neg[4] = { false, false, false, false };

                    for (unsigned i = 0; i != 2; ++i) {
                        if (isa<Not>(a0->getOperand(i))) {
                            PabloAST * i0 = cast<Not>(a0->getOperand(i))->getOperand(0);
                            for (unsigned j = 0; j != 2; ++j) {
                                if (a0->getOperand(j) == i0) {
                                    neg[i + j * 2] = true;
                                }
                            }
                        }
                    }









                }

            }*/
        }
        stmt = stmt->getNextNode();
    }
    block.setInsertPoint(block.back());
}


Simplifier::Simplifier()
{

}


}
