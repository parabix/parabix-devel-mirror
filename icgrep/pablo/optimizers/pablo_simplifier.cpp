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
                stmt = (stmt->getNumUses() == 0) ? assign->eraseFromParent() : assign->replaceWith(assign->getExpr());
                continue;
            }
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
            const auto f = encountered.insert(stmt);
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
}




Simplifier::Simplifier()
{

}


}
