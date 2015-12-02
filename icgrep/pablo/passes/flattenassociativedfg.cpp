#include "flattenassociativedfg.h"

#include <pablo/codegenstate.h>
#include <pablo/optimizers/pablo_simplifier.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <pablo/analysis/pabloverifier.hpp>

using namespace boost;
using namespace boost::container;

namespace pablo {

using TypeId = PabloAST::ClassTypeId;
using Graph = adjacency_list<hash_setS, vecS, bidirectionalS, PabloAST *>;
using Map = flat_map<PabloAST *, Graph::vertex_descriptor>;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief coalesce
 ** ------------------------------------------------------------------------------------------------------------- */
inline void FlattenAssociativeDFG::coalesce(Variadic * const var) {
    const TypeId typeId = var->getClassTypeId();
    for (unsigned i = 0; i < var->getNumOperands(); ) {
        PabloAST * const op = var->getOperand(i);
        if (op->getClassTypeId() == typeId) {
            Variadic * removedVar = cast<Variadic>(var->removeOperand(i));
            for (unsigned j = 0; j != cast<Variadic>(op)->getNumOperands(); ++j) {
                var->addOperand(cast<Variadic>(op)->getOperand(j));
            }
            if (removedVar->getNumOperands() == 1) {
                removedVar->replaceWith(removedVar->getOperand(0));
            } else if (removedVar->getNumUsers() == 0) {
                removedVar->eraseFromParent(true);
            }
            continue;
        }
        ++i;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief coalesce
 ** ------------------------------------------------------------------------------------------------------------- */
void FlattenAssociativeDFG::coalesce(PabloBlock * const block, const bool traverse) {
    Statement * stmt = block->front();
    while (stmt) {
        Statement * next = stmt->getNextNode();
        if (traverse && (isa<If>(stmt) || isa<While>(stmt))) {
            coalesce(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody(), true);
        } else if (isa<And>(stmt) || isa<Or>(stmt) || isa<Xor>(stmt)) {
            coalesce(cast<Variadic>(stmt));
        } else if (isa<Not>(stmt)) {
            deMorgansExpansion(cast<Not>(stmt), block);
        }
        stmt = next;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief deMorgansExpansion
 *
 * Apply the De Morgans' law to any negated And or Or statement with the intent of further coalescing its operands
 * thereby allowing the Simplifier to check for tautologies and contradictions.
 ** ------------------------------------------------------------------------------------------------------------- */
inline void FlattenAssociativeDFG::deMorgansExpansion(Not * const var, PabloBlock * const block) {
    PabloAST * const negatedVar = var->getOperand(0);
    if (isa<And>(negatedVar) || isa<Or>(negatedVar)) {
        Variadic * src = cast<Variadic>(negatedVar);
        const unsigned operands = src->getNumOperands();
        Variadic * replacement = nullptr;
        block->setInsertPoint(var->getPrevNode());
        if (isa<And>(negatedVar)) {
            replacement = block->createOr(operands);
        } else {
            replacement = block->createAnd(operands);
        }
        block->setInsertPoint(replacement->getPrevNode());
        for (unsigned i = 0; i != operands; ++i) {
            replacement->addOperand(block->createNot(src->getOperand(i)));
        }
        coalesce(replacement);
        var->replaceWith(replacement, true, true);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief deMorgansReduction
 ** ------------------------------------------------------------------------------------------------------------- */
inline void FlattenAssociativeDFG::deMorgansReduction(Variadic * const var, PabloBlock * const block) {
    unsigned negations = 0;
    for (unsigned i = 0; i < var->getNumOperands(); ++i) {
        if (isa<Not>(var->getOperand(i))) {
            ++negations;
        }
    }
    if (negations > 1) {
        PabloAST * negated[negations];
        for (unsigned i = var->getNumOperands(), j = negations; i && j; ) {
            if (isa<Not>(var->getOperand(--i))) {
                negated[--j] = cast<Not>(var->removeOperand(i))->getOperand(0);
            }
        }
        block->setInsertPoint(var->getPrevNode());
        Variadic * extractedVar = nullptr;
        if (isa<And>(var)) {
            extractedVar = block->createOr(negations);
        } else { // if (isa<Or>(var)) {
            extractedVar = block->createAnd(negations);
        }
        for (unsigned i = 0; i != negations; ++i) {
            extractedVar->addOperand(negated[i]);
        }
        var->addOperand(block->createNot(extractedVar));
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief extractNegationsOutwards
 ** ------------------------------------------------------------------------------------------------------------- */
inline void FlattenAssociativeDFG::deMorgansReduction(PabloBlock * const block) {
    for (Statement * stmt : *block) {
        if (isa<If>(stmt) || isa<While>(stmt)) {
            deMorgansReduction(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody());
        } else if (isa<And>(stmt) || isa<Or>(stmt)) {
            deMorgansReduction(cast<Variadic>(stmt), block);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief transform
 ** ------------------------------------------------------------------------------------------------------------- */
void FlattenAssociativeDFG::transform(PabloFunction & function) {

    FlattenAssociativeDFG::coalesce(function.getEntryBlock(), true);
    #ifndef NDEBUG
    PabloVerifier::verify(function, "post-coalescence");
    #endif

    Simplifier::optimize(function);

    FlattenAssociativeDFG::deMorgansReduction(function.getEntryBlock());
    #ifndef NDEBUG
    PabloVerifier::verify(function, "post-demorgans-reduction");
    #endif

    Simplifier::optimize(function);
}

}
