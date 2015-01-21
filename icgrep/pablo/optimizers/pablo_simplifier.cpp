#include <pablo/optimizers/pablo_simplifier.hpp>
#include <pablo/codegenstate.h>
#include <pablo/expression_map.hpp>

#include <pablo/printer_pablos.h>



namespace pablo {

bool Simplifier::optimize(PabloBlock & block) {
    eliminateRedundantCode(block);
    deadCodeElimination(block);

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


void Simplifier::eliminateRedundantCode(PabloBlock & block, ExpressionTable * predecessor) {
    ExpressionTable encountered(predecessor);

    Statement * stmt = block.front();
    while (stmt) {
        if (isa<Assign>(stmt)) {
            Assign * assign = cast<Assign>(stmt);
            // If we have an Assign whose users do not contain an If or Next node, we can replace its users with
            // the Assign's expression directly. However, since Assigns were likely named for clarity, propagate
            // it's name upwards into the expression's statement.
            if (assign->isConstant()) {
                PabloAST * expr = assign->getExpr();
                if (isa<Statement>(expr)) {
                    cast<Statement>(expr)->setName(assign->getName());
                }
                assign->replaceAllUsesWith(expr);
                // Note: we cannot delete recursively yet because our "encountered" table may would not be
                // updated properly. The dead code elimination pass will perform any final clean-up.
                stmt = assign->eraseFromParent();
                continue;
            }
        }
        else { // non-Assign node

            // Do a trivial folding test to see if we're using all 0s or 1s as an operand.
            if (canTriviallyFold(stmt)) {
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
                assert (expr);
                stmt = stmt->replaceWith(expr);                
                // the next statement could be an Assign; just restart the loop.
                continue;
            }
            else if (isa<If>(stmt)) {
                eliminateRedundantCode(cast<If>(stmt)->getBody(), &encountered);
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
            else {
                // When we're creating the Pablo program, it's possible to have multiple instances of an "identical"
                // statement. By recording which statements have already been seen, we can detect the redundant statements
                // as any having the same type and operands. If so, we can replace its users with the prior statement.
                // and erase this statement from the AST
                const auto f = encountered.insert(stmt);
                if (!std::get<1>(f)) {
                    stmt->replaceAllUsesWith(std::get<0>(f));
                    stmt = stmt->eraseFromParent();
                    continue;
                }
            }
        }
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
}




Simplifier::Simplifier()
{

}


}
