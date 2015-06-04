#include <pablo/optimizers/pablo_simplifier.hpp>
#include <pablo/codegenstate.h>
#include <pablo/expression_map.hpp>

#include <pablo/printer_pablos.h>



namespace pablo {

bool Simplifier::optimize(PabloBlock & block) {
    eliminateRedundantCode(block);
    deadCodeElimination(block);
    eliminateRedundantComplexStatements(block);
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

void Simplifier::eliminateRedundantCode(PabloBlock & block, ExpressionTable * predecessor) {
    ExpressionTable encountered(predecessor);

    Statement * stmt = block.front();

    while (stmt) {

        assert (stmt);
        assert (verifyStatementIsInSameBlock(stmt, block));

        if (Assign * assign = dyn_cast<Assign>(stmt)) {
            // If we have an Assign whose users do not contain an If or Next node, we can replace its users with
            // the Assign's expression directly.
            if (assign->superfluous()) {
                if (stmt->getNumUses() == 0) {
                    stmt = assign->eraseFromParent();
                }
                else {
                    stmt = assign->replaceWith(assign->getExpr());
                }
                continue;
            }
        }
        else if (If * ifNode = dyn_cast<If>(stmt)) {
            // Check to see if the Cond is Zero and delete the loop.
            if (LLVM_UNLIKELY(isa<Zeroes>(ifNode->getCondition()))) {
                for (PabloAST * defVar : ifNode->getDefined()) {
                    cast<Assign>(defVar)->replaceWith(block.createZeroes(), false, true);
                }
                stmt = stmt->eraseFromParent(true);
                continue;
            }
            // Process the If body
            eliminateRedundantCode(cast<If>(stmt)->getBody(), &encountered);
            // Scan through and replace any defined variable that is assigned Zero with a Zero object
            // and remove it from the defined variable list.
            If::DefinedVars & defVars = ifNode->getDefined();
            for (auto i = defVars.begin(); i != defVars.end(); ) {
                Assign * defVar = cast<Assign>(*i);
                if (LLVM_UNLIKELY(isa<Zeroes>(defVar->getExpr()))) {
                    i = defVars.erase(i);
                    defVar->replaceWith(block.createZeroes(), false, true);
                    continue;
                }
                ++i;
            }
            // If we ended up Zero-ing out all of the defined variables, delete the If node.
            if (LLVM_UNLIKELY(defVars.empty())) {
                stmt = stmt->eraseFromParent(true);
                continue;
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
                default:
                    throw std::runtime_error("Unhandled trivial folding optimization!");
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
            assert ("Any Assign passed to this function ought to have users or be an output var!" && (!isa<Assign>(stmt) || cast<Assign>(stmt)->isOutputAssignment()));
            if (!isa<Assign>(stmt)) {
                stmt = stmt->eraseFromParent(true);
                continue;
            }
        }
        stmt = stmt->getNextNode();
    }
    block.setInsertPoint(block.back());
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
            }
            // If an AND, OR or XOR instruction has two Advance instructions as inputs and neither Advance
            // has another user and both shift their input by the same amount, we can perform the AND, OR
            // or XOR on the inputs to the Advances and remove one of the Advance statements.
            else if (LLVM_UNLIKELY(isa<Advance>(stmt->getOperand(1)) && isa<Advance>(stmt->getOperand(0)))) {
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
            }
            else if (LLVM_UNLIKELY(isa<MatchStar>(stmt->getOperand(1)) && isa<MatchStar>(stmt->getOperand(0))) && isa<Or>(stmt)) {


            }
        }
        stmt = stmt->getNextNode();
    }
    block.setInsertPoint(block.back());
}


Simplifier::Simplifier()
{

}


}
