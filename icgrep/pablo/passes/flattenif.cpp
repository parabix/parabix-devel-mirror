#include <pablo/passes/flattenif.hpp>
#include <pablo/pablo_kernel.h>
#include <pablo/codegenstate.h>
#include <pablo/analysis/pabloverifier.hpp>

namespace pablo {
    
void FlattenIf::flattenIf(PabloBlock * const block) {
    Statement * stmt = block->front();
    while (stmt) {
        if (LLVM_UNLIKELY(isa<If>(stmt))) {
            Statement * prior = stmt;
            Statement * const initial = cast<If>(stmt)->getBody()->front();
            Statement * body_stmt = initial;
            while (body_stmt) {
                assert (body_stmt->getPrevNode() == nullptr);
                Statement * const next = body_stmt->getNextNode();
                body_stmt->insertAfter(prior);
                assert (body_stmt->getPrevNode() == prior);
                prior = body_stmt;
                body_stmt = next;
            }
            stmt = stmt->eraseFromParent(true);
            assert (stmt == initial);
        } else {
            stmt = stmt->getNextNode();
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief transform
 ** ------------------------------------------------------------------------------------------------------------- */
void FlattenIf::transform(PabloKernel * function) {
    flattenIf(function->getEntryBlock());
    #ifndef NDEBUG
    PabloVerifier::verify(function, "flatten-if");
    #endif
}

}
