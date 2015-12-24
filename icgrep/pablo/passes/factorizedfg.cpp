#include "factorizedfg.h"

#include <pablo/codegenstate.h>
#include <pablo/analysis/pabloverifier.hpp>
#include <pablo/optimizers/pablo_simplifier.hpp>
#include <pablo/passes/flattenassociativedfg.h>
#include <boost/container/flat_set.hpp>
#include <set>

using namespace boost;
using namespace boost::container;


namespace pablo {

using VertexSet = std::vector<PabloAST *>;
using Biclique = std::pair<VertexSet, VertexSet>; // [{Operands}, {Users}]
using BicliqueSet = std::vector<Biclique>;
using TypeId = PabloAST::ClassTypeId;

///** ------------------------------------------------------------------------------------------------------------- *
// * @brief lower
// ** ------------------------------------------------------------------------------------------------------------- */
//inline PabloAST * FactorizeDFG::lower(Variadic * const var, PabloBlock * block, OrderingMap & order) {
//    PabloAST * result = nullptr;
//    assert (var->getNumOperands() > 2);
////    // Sort the operands by their order in the current AST
////    std::sort(var->begin(), var->end(),
////        [&order](const PabloAST * const a, const PabloAST * const b)
////    {
////        return order.of(a) > order.of(b);
////    });
////    unsigned operands = var->getNumOperands();
////    // Swap the final two operands so that we can just append the temporary calculations onto the end
////    // and trust that the PENULTIMATE operand is the "latest" in the AST.
////    PabloAST * const item = var->getOperand(operands - 1);
////    var->setOperand(operands - 1, var->getOperand(operands - 2));
////    var->setOperand(operands - 2, item);
//    // Finally rewrite all of the variadic statements as binary operations
//    block->setInsertPoint(var->getPrevNode());
//    while (var->getNumOperands() > 1) {
//        PabloAST * const op2 = var->removeOperand(1);
//        PabloAST * const op1 = var->removeOperand(0);
//        assert (op1 != op2);
//        if (isa<And>(var)) {
//            result = block->createAnd(op1, op2);
//        } else if (isa<Or>(var)) {
//            result = block->createOr(op1, op2);
//        } else { // if (isa<Xor>(var)) {
//            result = block->createXor(op1, op2);
//        }
//        var->addOperand(result);
//    }
//    assert (result);
//    return result;
//}

///** ------------------------------------------------------------------------------------------------------------- *
// * @brief lower
// ** ------------------------------------------------------------------------------------------------------------- */
//void FactorizeDFG::lower(PabloBlock * const block, OrderingMap & parent) {
//    OrderingMap order(parent);
//    Statement * stmt = block->front();
//    while (stmt) {
//        if (isa<If>(stmt) || isa<While>(stmt)) {
//            lower(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody(), order);
//            if (isa<If>(stmt)) {
//                for (Assign * def : cast<If>(stmt)->getDefined()) {
//                    order.enumerate(def);
//                }
//            } else { // if (isa<While>(stmt)) {
//                for (Next * var : cast<While>(stmt)->getVariants()) {
//                    order.enumerate(var);
//                }
//            }
//        } else {
//            if ((isa<And>(stmt) || isa<Or>(stmt) || isa<Xor>(stmt)) && (stmt->getNumOperands() > 2)) {
//                // We should maintain an ordering list and sort each Variadic according to it prior to writing them out.
//                PabloAST * result = lower(cast<Variadic>(stmt), block, order);
//                order.enumerate(result);
//                stmt = stmt->replaceWith(result, true);
//                continue;
//            }
//            order.enumerate(stmt);
//        }
//        stmt = stmt->getNextNode();
//    }
//}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief finalize
 ** ------------------------------------------------------------------------------------------------------------- */
inline Statement * FactorizeDFG::lower(Variadic * const var, PabloBlock * block) {
    PabloAST * result = nullptr;
    assert (var->getNumOperands() > 2);
    block->setInsertPoint(var->getPrevNode());
    while (var->getNumOperands() > 1) {
        PabloAST * const op2 = var->removeOperand(1);
        PabloAST * const op1 = var->removeOperand(0);
        assert (op1 != op2);
        if (isa<And>(var)) {
            result = block->createAnd(op1, op2);
        } else if (isa<Or>(var)) {
            result = block->createOr(op1, op2);
        } else { // if (isa<Xor>(var)) {
            result = block->createXor(op1, op2);
        }
        var->addOperand(result);
    }
    return var->replaceWith(result, true);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief finalize
 ** ------------------------------------------------------------------------------------------------------------- */
void FactorizeDFG::lower(PabloBlock * const block) {
    Statement * stmt = block->front();
    while (stmt) {
        if (isa<If>(stmt) || isa<While>(stmt)) {
            lower(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody());
        } else if ((isa<And>(stmt) || isa<Or>(stmt) || isa<Xor>(stmt)) && (stmt->getNumOperands() > 2)) {
            // We should maintain an ordering list and sort each Variadic according to it prior to writing them out.
            stmt = lower(cast<Variadic>(stmt), block);
            continue;
        }
        stmt = stmt->getNextNode();
    }
}

///** ------------------------------------------------------------------------------------------------------------- *
// * @brief lower
// ** ------------------------------------------------------------------------------------------------------------- */
//inline void FactorizeDFG::lower(PabloFunction & function) {
//    OrderingMap ordering;
//    for (unsigned i = 0; i != function.getNumOfParameters(); ++i) {
//        ordering.enumerate(function.getParameter(i));
//    }
//    lower(function.getEntryBlock(), ordering);
//}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enumerateBicliques
 *
 * Adaptation of the MICA algorithm as described in "Consensus algorithms for the generation of all maximal
 * bicliques" by Alexe et. al. (2003).
 ** ------------------------------------------------------------------------------------------------------------- */
static BicliqueSet enumerateBicliques(Variadic * const var) {
    using IntersectionSets = std::set<VertexSet>;

    IntersectionSets B1;

    const TypeId typeId = var->getClassTypeId();
    for (unsigned i = 0; i != var->getNumOperands(); ++i) {
        PabloAST * const op = var->getOperand(i);
        VertexSet B;
        B.reserve(op->getNumUses());
        for (PabloAST * user : op->users()) {
            if (user->getClassTypeId() == typeId) {
                B.push_back(user);
            }
        }
        std::sort(B.begin(), B.end());
        B1.insert(std::move(B));
    }

    IntersectionSets B(B1);

    IntersectionSets Bi;

    VertexSet clique;
    for (auto i = B1.begin(); i != B1.end(); ++i) {
        for (auto j = i; ++j != B1.end(); ) {
            std::set_intersection(i->begin(), i->end(), j->begin(), j->end(), std::back_inserter(clique));
            if (clique.size() > 0) {
                if (B.count(clique) == 0) {
                    Bi.insert(clique);
                }
                clique.clear();
            }
        }
    }

    while (!Bi.empty()) {
        B.insert(Bi.begin(), Bi.end());
        IntersectionSets Bk;
        for (auto i = B1.begin(); i != B1.end(); ++i) {
            for (auto j = Bi.begin(); j != Bi.end(); ++j) {
                std::set_intersection(i->begin(), i->end(), j->begin(), j->end(), std::back_inserter(clique));
                if (clique.size() > 0) {
                    if (B.count(clique) == 0) {
                        Bk.insert(clique);
                    }
                    clique.clear();
                }
            }
        }
        Bi.swap(Bk);
    }

    BicliqueSet bicliques;
    unsigned userSize = 2;
    for (auto Bi = B.begin(); Bi != B.end(); ++Bi) {
        if (userSize <= Bi->size()) {
            VertexSet Ai(var->begin(), var->end());
            for (const PabloAST * user : *Bi) {
                VertexSet Aj(cast<Variadic>(user)->begin(), cast<Variadic>(user)->end());
                std::sort(Aj.begin(), Aj.end());
                VertexSet Ak;
                Ak.reserve(std::min<unsigned>(Ai.size(), Aj.size()));
                std::set_intersection(Ai.begin(), Ai.end(), Aj.begin(), Aj.end(), std::back_inserter(Ak));
                Ai.swap(Ak);
            }
            if (Ai.size() > 1) {
                if (userSize < Bi->size()) {
                    bicliques.clear();
                    userSize = Bi->size();
                }
                bicliques.emplace_back(std::move(Ai), std::move(*Bi));
            }
        }
    }
    return bicliques;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief pick
 ** ------------------------------------------------------------------------------------------------------------- */
inline static Biclique pick(BicliqueSet && bicliques) {
    unsigned best_w = 0;
    auto best_c = bicliques.begin();
    for (auto c = bicliques.begin(); c != bicliques.end(); ++c) {
        const unsigned w = std::get<0>(*c).size();
        if (best_w < w) {
            best_c = c;
            best_w = w;
        }
    }
    return std::move(*best_c);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief chooseInsertionPoint
 ** ------------------------------------------------------------------------------------------------------------- */
inline PabloBlock * FactorizeDFG::chooseInsertionScope(const ASTVector & users) {

    ScopeSet scopes;
    scopes.reserve(users.size());

    flat_set<PabloBlock *> visited;
    visited.reserve(users.size());

    for (PabloAST * user : users) {
        PabloBlock * const scope = cast<Statement>(user)->getParent();
        assert (scope);
        if (visited.insert(scope).second) {
            scopes.push_back(scope);
        }
    }

    while (scopes.size() > 1) {
        // Find the LCA of both scopes then add the LCA back to the list of scopes.
        PabloBlock * scope1 = scopes.back(); scopes.pop_back();
        PabloBlock * scope2 = scopes.back(); scopes.pop_back();
        if (scope1 != scope2) {
            unsigned depth1 = mScopeDepth.find(scope1)->second;
            unsigned depth2 = mScopeDepth.find(scope2)->second;
            // If one of these scopes is nested deeper than the other, scan upwards through
            // the scope tree until both scopes are at the same depth.
            while (depth1 > depth2) {
                scope1 = scope1->getParent();
                --depth1;
            }
            while (depth1 < depth2) {
                scope2 = scope2->getParent();
                --depth2;
            }
            // Then iteratively step backwards until we find a matching set of scopes; this
            // must be the LCA of our original scopes.
            while (scope1 != scope2) {
                scope1 = scope1->getParent();
                scope2 = scope2->getParent();
            }
            assert (scope1 && scope2);
        }
        scopes.push_back(scope1);
    }
    assert (scopes.size() == 1);
    return scopes[0];
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief findInsertionPoint
 ** ------------------------------------------------------------------------------------------------------------- */
void FactorizeDFG::findInsertionPoint(const ASTVector & operands, PabloBlock * const scope) {
    const flat_set<const PabloAST *> ops(operands.begin(), operands.end());
    Statement * stmt = scope->back();
    scope->setInsertPoint(nullptr);
    while (stmt) {
        if (isa<If>(stmt)) {
            for (Assign * def : cast<If>(stmt)->getDefined()) {
                if (ops.count(def)) {
                    scope->setInsertPoint(stmt);
                    return;
                }
            }
        } else if (isa<While>(stmt)) {
            for (Next * var : cast<While>(stmt)->getVariants()) {
                if (ops.count(var)) {
                    scope->setInsertPoint(stmt);
                    return;
                }
            }
        } else if (ops.count(stmt)) {
            scope->setInsertPoint(stmt);
            break;
        }
        stmt = stmt->getPrevNode();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief cse
 ** ------------------------------------------------------------------------------------------------------------- */
inline void FactorizeDFG::cse(Variadic * var) {
    while (var->getNumOperands() > 2) {
        BicliqueSet bicliques = enumerateBicliques(var);
        if (bicliques.empty()) {
            break;
        }
        ASTVector operands;
        ASTVector users;
        std::tie(operands, users) = pick(std::move(bicliques));
        PabloBlock * const block = chooseInsertionScope(users);
        findInsertionPoint(operands, block);
        Variadic * factored = nullptr;
        if (isa<And>(var)) {
            factored = block->createAnd(operands.begin(), operands.end());
        } else if (isa<Or>(var)) {
            factored = block->createOr(operands.begin(), operands.end());
        } else { // if (isa<Xor>(var)) {
            factored = block->createXor(operands.begin(), operands.end());
        }
        for (PabloAST * user : users) {
            for (PabloAST * op : operands) {
                cast<Variadic>(user)->deleteOperand(op);
            }
            cast<Variadic>(user)->addOperand(factored);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief cse
 *
 * Perform common subexpression elimination on the Variadic statements
 ** ------------------------------------------------------------------------------------------------------------- */
void FactorizeDFG::cse(PabloBlock * const block) {
    Statement * stmt = block->front();
    while (stmt) {
        if (isa<If>(stmt) || isa<While>(stmt)) {            
            cse(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody());
        } else if (isa<And>(stmt) || isa<Or>(stmt) || isa<Xor>(stmt)) {
            cse(cast<Variadic>(stmt));
        }
        stmt = stmt->getNextNode();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enumerateScopeDepth
 ** ------------------------------------------------------------------------------------------------------------- */
void FactorizeDFG::enumerateScopeDepth(const PabloBlock * const block, const unsigned depth) {
    const Statement * stmt = block->front();
    while (stmt) {
        if (isa<If>(stmt) || isa<While>(stmt)) {
            PabloBlock * body = isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody();
            mScopeDepth.emplace(body, depth);
            enumerateScopeDepth(body, depth + 1);
        }
        stmt = stmt->getNextNode();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enumerateScopeDepth
 ** ------------------------------------------------------------------------------------------------------------- */
inline void FactorizeDFG::enumerateScopeDepth(const PabloFunction & function) {
    mScopeDepth.emplace(function.getEntryBlock(), 0);
    enumerateScopeDepth(function.getEntryBlock(), 1);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief transform
 ** ------------------------------------------------------------------------------------------------------------- */
void FactorizeDFG::transform(PabloFunction & function) {
    FactorizeDFG ldfg;
    ldfg.enumerateScopeDepth(function);
    ldfg.cse(function.getEntryBlock());
    #ifndef NDEBUG
    PabloVerifier::verify(function, "post-cse");
    #endif
    Simplifier::optimize(function);
    ldfg.lower(function.getEntryBlock());
    #ifndef NDEBUG
    PabloVerifier::verify(function, "post-lowering");
    #endif    
}

}
