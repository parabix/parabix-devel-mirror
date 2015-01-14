#include <pablo/optimizers/pablo_simplifier.hpp>
#include <pablo/codegenstate.h>
#include <pablo/expression_map.hpp>

//#include <pablo/printer_pablos.h>



namespace pablo {

bool Simplifier::optimize(PabloBlock & block) {

    eliminateRedundantCode(nullptr, block);



    return true;
}

void Simplifier::eliminateRedundantCode(ExpressionTable * predecessor, PabloBlock & block) {
    ExpressionTable encountered(predecessor);

    Statement * stmt = block.front();
    while (stmt) {

        if (isa<Assign>(stmt)) {
            Assign * assign = cast<Assign>(stmt);
            // If we have an Assign whose users do not contain an If or Next node, we can replace its users with
            // the Assign's expression directly. However, since Assigns were likely named for clarity, propagate
            // it's name upwards into the expression.
            if (assign->isConstant()) {
                PabloAST * expr = assign->getExpr();
                if (isa<Statement>(expr)) {
                    cast<Statement>(expr)->setName(assign->getName());
                }
                assign->replaceAllUsesWith(expr);
                stmt = assign->removeFromParent();
                continue;
            }
        }
        else { // non-Assign node

            if (isa<If>(stmt)) {
                eliminateRedundantCode(&encountered, cast<If>(stmt)->getBody());
            }
            else if (isa<While>(stmt)) {
                eliminateRedundantCode(&encountered, cast<While>(stmt)->getBody());
            }
            else if (stmt->getNumUses() == 0) {
                stmt = stmt->removeFromParent();
                continue;
            }
            // When we're creating the Pablo program, it's possible that statements may occur more than once.
            // By recording which statements have already been seen, we can detect which statements to remove.
            const auto f = encountered.insert(stmt);
            if (!std::get<1>(f)) {
                stmt->replaceAllUsesWith(std::get<0>(f));
                stmt = stmt->removeFromParent();
                continue;
            }
        }
        stmt = stmt->getNextNode();
    }
}


void Simplifier::foldConstantAssigns(PabloBlock &) {


}




Simplifier::Simplifier()
{

}


}
