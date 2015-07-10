#include "pablo_codesinking.hpp"
#include <pablo/function.h>

namespace pablo {

bool CodeSinking::optimize(PabloFunction & function)
{
    CodeSinking lcf;
    lcf.sink(function.getEntryBlock());
    return true;
}

inline static bool isSafeToMove(Statement * stmt) {
    if (isa<Assign>(stmt) || isa<Next>(stmt)) {
        return false;
    }
    return true;
}

void CodeSinking::sink(PabloBlock & block) {

    Statement * stmt = block.back();
    while (stmt) {
        Statement * next = stmt->getPrevNode();
        if (isa<If>(stmt)) {
            If * ifNode = cast<If>(stmt);
            assert (ifNode->getParent() == &block);
            sink(ifNode->getBody());
        }
        else if (isa<While>(stmt)) {
            While * whileNode = cast<While>(stmt);
            assert (whileNode->getParent() == &block);
            sink(whileNode->getBody());
        }
        else if (isSafeToMove(stmt)) {

            BlockSetVector ancestors;
            bool canSinkInstruction = false;
            for (const PabloAST * u : stmt->users()) {
                if (const Statement * use = dyn_cast<Statement>(u)) {
                    if (mProcessed.count(use->getParent())) {
                        canSinkInstruction = true;
                        ancestors.insert(use->getParent());
                        continue;
                    }
                    canSinkInstruction = false;
                    break;
                }
            }
            if (canSinkInstruction) {

                assert (ancestors.size() >= 1);

                if (ancestors.size() > 1) {
                    BlockSetVector P1, P2;
                    while (ancestors.size() > 1) {

                        PabloBlock * u = ancestors.back();
                        while (u != &block) {
                            P1.push_back(u);
                            u = u->getParent();
                        }
                        ancestors.pop_back();

                        PabloBlock * v = ancestors.back();
                        while (v != &block) {
                            P2.push_back(v);
                            v = v->getParent();
                        }
                        ancestors.pop_back();

                        // P1 and P2 now contain the paths from the 'sink' to the 'source'. Scan backwards though them
                        // to find the first differing element. The last non-differing block is our LCA. If none
                        // exists for any two paths, this instruction cannot be sunk (or would just be sunk to the
                        // same position it's at now.)

                        PabloBlock * lca = nullptr;
                        auto p1 = P1.rbegin(), p2 = P2.rbegin();

                        for (; p1 != P1.rend() && p2 != P2.rend(); ++p1, ++p2) {
                            if (*p1 != *p2) {
                                break;
                            }

                            lca = *p1;
                        }

                        if (lca == nullptr) {
                            canSinkInstruction = false;
                            break;
                        }

                        P1.clear();
                        P2.clear();
                        ancestors.insert(lca);
                    }
                }

                if (canSinkInstruction) {
                    assert (ancestors.size() == 1);
                    PabloBlock * const lca = *ancestors.begin();
                    stmt->insertBefore(lca->front());
                }
            }
        }
        stmt = next;
    }
    mProcessed.insert(&block);
}

}
