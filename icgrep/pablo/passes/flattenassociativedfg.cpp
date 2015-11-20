#include "flattenassociativedfg.h"

#include <pablo/codegenstate.h>
#include <pablo/optimizers/pablo_simplifier.hpp>
#include <pablo/analysis/pabloverifier.hpp>

namespace pablo {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief flatten
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool FlattenAssociativeDFG::flatten(Variadic * const var, PabloBlock * const block) {
    bool modified = false;
    for (;;) {
        bool unmodified = true;
        for (unsigned i = 0; i < var->getNumOperands(); ) {
            PabloAST * const op = var->getOperand(i);
            if ((op->getNumUses() == 1) && (op->getClassTypeId() == var->getClassTypeId()) && (cast<Variadic>(op)->getParent() == block)) {
                var->removeOperand(i);
                for (unsigned j = 0; j != cast<Variadic>(op)->getNumOperands(); ++j) {
                    var->addOperand(cast<Variadic>(op)->getOperand(j));
                }
                assert (op->getNumUses() == 0);
                cast<Variadic>(op)->eraseFromParent(true);
                unmodified = false;
                modified = true;
                continue;
            }
            ++i;
        }
        if (unmodified) {
            break;
        }
    }
    return modified;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief flatten
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool FlattenAssociativeDFG::flatten(PabloBlock * const block) {
    bool modified = false;
    Statement * stmt = block->front();
    while (stmt) {
        if (isa<And>(stmt) || isa<Or>(stmt) || isa<Xor>(stmt)) {
            Variadic * var = cast<Variadic>(stmt);
            if (flatten(var, block)) {
                modified = true;
                if (PabloAST * replacement = Simplifier::foldReassociativeFunction(var, block)) {
                    stmt = stmt->replaceWith(replacement);
                    continue;
                }
            }
        }
        stmt = stmt->getNextNode();
    }
    return modified;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief factorize
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool FlattenAssociativeDFG::factorize(PabloBlock * const block) {
    bool modified = false;
    Statement * stmt = block->front();
    while (stmt) {
        if (isa<And>(stmt) || isa<Or>(stmt) || isa<Xor>(stmt)) {
            // Does this function share two operands with another function of the same type?
            // If so, pull them out of both functions.
            Variadic * var = cast<Variadic>(stmt);
            for (unsigned i = 1; i < var->getNumOperands(); ++i) {
                for (unsigned j = 0; j != i; ++j) {

                }
            }
        }
        stmt = stmt->getNextNode();
    }
    return modified;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief traverse
 ** ------------------------------------------------------------------------------------------------------------- */
void FlattenAssociativeDFG::traverse(PabloBlock * const block) {
    for (Statement * stmt : *block) {
        if (isa<If>(stmt) || isa<While>(stmt)) {
            traverse(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody());
        }
    }
    flatten(block);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief process
 ** ------------------------------------------------------------------------------------------------------------- */
void FlattenAssociativeDFG::process(PabloFunction & function) {
    FlattenAssociativeDFG::traverse(function.getEntryBlock());
//    #ifndef NDEBUG
    PabloVerifier::verify(function, "post-flatten-associative-dfg");
//    #endif
    Simplifier::optimize(function);
}

}
