#include "pablo_codesinking.hpp"
#include <pablo/printer_pablos.h>
#include <iostream>

namespace pablo {

bool CodeSinking::optimize(PabloBlock & block)
{
    // determine the block dominator "tree"
    sink(block);
    return true;
}

inline static bool isSafeToMove(Statement * stmt) {
    if (isa<Assign>(stmt) || isa<Next>(stmt)) {
        return false;
    }
    return true;
}

//void CodeSinking::buildDominatorTree(PabloBlock & block) {
//    Statement * stmt = block.front();
//    while (stmt) {
//        if (isa<If>(stmt)) {
//            buildDominatorTree(cast<If>(stmt)->getBody());
//        }
//        else if (isa<While>(stmt)) {
//            buildDominatorTree(cast<While>(stmt)->getBody());
//        }

//        stmt = stmt->getNextNode();
//    }
//}

void CodeSinking::sink(PabloBlock & block) {
    Statement * stmt = block.back();
    while (stmt) {
        Statement * next = stmt->getPrevNode();
        if (isa<If>(stmt)) {
            If * ifNode = cast<If>(stmt);
            sink(ifNode->getBody());
        }
        else if (isa<While>(stmt)) {
            sink(cast<While>(stmt)->getBody());
        }
        else if (isSafeToMove(stmt)) {
            const PabloBlock * usedInBlock = nullptr;
            bool usedInOnlyOneBlock = true;
            for (const PabloAST * u : stmt->users()) {
                const Statement * use = dyn_cast<Statement>(u);
                if (use) {
                    if (usedInBlock == nullptr || usedInBlock == use->getParent()) {
                        usedInBlock = use->getParent();
                        continue;
                    }
                    usedInOnlyOneBlock = false;
                    break;
                }
            }
            assert (usedInBlock);
            if (usedInOnlyOneBlock && usedInBlock != &block) {
//                PabloPrinter::print(stmt, " moving ", std::cerr);
//                PabloPrinter::print(usedInBlock->front(), " before ", std::cerr);
//                std::cerr << std::endl;
                stmt->insertBefore(usedInBlock->front());
            }
        }
        stmt = next;
    }
}



CodeSinking::CodeSinking()
{
}

}
