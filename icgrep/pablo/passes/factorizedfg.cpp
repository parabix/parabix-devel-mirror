#include "factorizedfg.h"

#include <pablo/codegenstate.h>
#include <pablo/analysis/pabloverifier.hpp>
#include <pablo/optimizers/pablo_simplifier.hpp>
#include <pablo/passes/flattenassociativedfg.h>
#include <boost/container/flat_set.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <queue>
#include <llvm/Support/CommandLine.h>

#include <pablo/printer_pablos.h>
#include <iostream>

using namespace boost;
using namespace boost::container;

//static cl::opt<unsigned> RematerializationThreshold("factoring-remat", cl::desc("Number of registers available for factoring rematerialization"), cl::init(16));

namespace pablo {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enumerateFactoringsOf
 *
 * Adaptation of the MICA algorithm as described in "Consensus algorithms for the generation of all maximal
 * bicliques" by Alexe et. al. (2003).
 ** ------------------------------------------------------------------------------------------------------------- */
FactorizeDFG::BicliqueSet FactorizeDFG::enumerateFactoringSets(ObjectSet params, PabloBlock * const entryScope, const TypeId typeId) {
    using IntersectionSets = flat_set<ObjectSet>;

    ObjectSet tmp;
    tmp.reserve(params.size());

    std::sort(params.begin(), params.end());


    IntersectionSets B1;
    for (PabloAST * op : params) {
        tmp.reserve(op->getNumUses());
        for (PabloAST * user : op->users()) {
            if (user->getClassTypeId() == typeId) {
                if (isa<Var>(op) || cast<Statement>(user)->getParent() == entryScope) {
                    tmp.push_back(user);
                }
            }
        }
        if (LLVM_LIKELY(tmp.size() > 1)) {
            std::sort(tmp.begin(), tmp.end());
            B1.emplace(tmp.begin(), tmp.end());
        }
        tmp.clear();
    }

    IntersectionSets B(B1);

    IntersectionSets Bi;

    for (auto i = B1.begin(); i != B1.end(); ++i) {
        for (auto j = i; ++j != B1.end(); ) {
            std::set_intersection(i->begin(), i->end(), j->begin(), j->end(), std::back_inserter(tmp));
            assert (std::is_sorted(tmp.begin(), tmp.end()));
            if (tmp.size() > 1 && B.count(tmp) == 0) {
                Bi.emplace(tmp.begin(), tmp.end());
            }
            tmp.clear();
        }
    }

    while (!Bi.empty()) {
        B.insert(Bi.begin(), Bi.end());
        IntersectionSets Bk;
        for (auto i = B1.begin(); i != B1.end(); ++i) {
            for (auto j = Bi.begin(); j != Bi.end(); ++j) {
                std::set_intersection(i->begin(), i->end(), j->begin(), j->end(), std::back_inserter(tmp));
                assert (std::is_sorted(tmp.begin(), tmp.end()));
                if (tmp.size() > 1 && B.count(tmp) == 0) {
                    Bk.emplace(tmp.begin(), tmp.end());
                }
                tmp.clear();
            }
        }
        Bi.swap(Bk);
    }

    BicliqueSet factoringSets(0);
    factoringSets.reserve(B.size());
    for (auto Bi = B.begin(); Bi != B.end(); ++Bi) {
        ObjectSet Ai(params);
        assert (Bi->size() > 1);
        for (const PabloAST * user : *Bi) {
            ObjectSet Aj(cast<Variadic>(user)->begin(), cast<Variadic>(user)->end());
            std::sort(Aj.begin(), Aj.end());
            ObjectSet Ak;
            Ak.reserve(std::min<unsigned>(Ai.size(), Aj.size()));
            std::set_intersection(Ai.begin(), Ai.end(), Aj.begin(), Aj.end(), std::back_inserter(Ak));
            Ai.swap(Ak);
        }
        if (Ai.size() > 1) {
            factoringSets.emplace_back(std::move(Ai), std::move(*Bi));
        }
    }

    return factoringSets;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enumerateFactoringsOf
 *
 * Adaptation of the MICA algorithm as described in "Consensus algorithms for the generation of all maximal
 * bicliques" by Alexe et. al. (2003).
 ** ------------------------------------------------------------------------------------------------------------- */
inline FactorizeDFG::BicliqueSet FactorizeDFG::enumerateFactoringSets(Variadic * const var) {
    using IntersectionSets = flat_set<ObjectSet>;

    ObjectSet tmp;
    tmp.reserve(var->getNumOperands());

    IntersectionSets B1;
    for (unsigned i = 0; i != var->getNumOperands(); ++i) {
        PabloAST * const op = var->getOperand(i);        
        tmp.reserve(op->getNumUses());
        for (PabloAST * user : op->users()) {
            if (user->getClassTypeId() == var->getClassTypeId()) {                
                for (PabloBlock * scope = cast<Variadic>(user)->getParent(); scope; scope = scope->getParent()) {
                    if (LLVM_UNLIKELY(scope == var->getParent())) {
                        tmp.push_back(user);
                        break;
                    }
                }
            }
        }
        if (LLVM_LIKELY(tmp.size() > 1)) {
            std::sort(tmp.begin(), tmp.end());
            B1.emplace(tmp.begin(), tmp.end());
        }
        tmp.clear();
    }

    IntersectionSets B(B1);

    IntersectionSets Bi;

    for (auto i = B1.begin(); i != B1.end(); ++i) {
        for (auto j = i; ++j != B1.end(); ) {
            std::set_intersection(i->begin(), i->end(), j->begin(), j->end(), std::back_inserter(tmp));
            assert (std::is_sorted(tmp.begin(), tmp.end()));
            if (tmp.size() > 1 && B.count(tmp) == 0) {
                Bi.emplace(tmp.begin(), tmp.end());
            }
            tmp.clear();
        }
    }

    while (!Bi.empty()) {
        B.insert(Bi.begin(), Bi.end());
        IntersectionSets Bk;
        for (auto i = B1.begin(); i != B1.end(); ++i) {
            for (auto j = Bi.begin(); j != Bi.end(); ++j) {
                std::set_intersection(i->begin(), i->end(), j->begin(), j->end(), std::back_inserter(tmp));
                assert (std::is_sorted(tmp.begin(), tmp.end()));
                if (tmp.size() > 1 && B.count(tmp) == 0) {
                    Bk.emplace(tmp.begin(), tmp.end());
                }
                tmp.clear();
            }
        }
        Bi.swap(Bk);
    }

    BicliqueSet factoringSets(0);
    factoringSets.reserve(B.size());
    for (auto Bi = B.begin(); Bi != B.end(); ++Bi) {        
        ObjectSet Ai(var->begin(), var->end());
        assert (Bi->size() > 1);
        for (const PabloAST * user : *Bi) {
            ObjectSet Aj(cast<Variadic>(user)->begin(), cast<Variadic>(user)->end());
            std::sort(Aj.begin(), Aj.end());
            ObjectSet Ak;
            Ak.reserve(std::min<unsigned>(Ai.size(), Aj.size()));
            std::set_intersection(Ai.begin(), Ai.end(), Aj.begin(), Aj.end(), std::back_inserter(Ak));
            Ai.swap(Ak);
        }
        if (Ai.size() > 1) {
            factoringSets.emplace_back(std::move(Ai), std::move(*Bi));
        }
    }

    return factoringSets;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief intersects
 ** ------------------------------------------------------------------------------------------------------------- */
template <class Type>
inline bool intersects(const Type & A, const Type & B) {
    auto first1 = A.begin(), last1 = A.end();
    auto first2 = B.begin(), last2 = B.end();
    assert (std::is_sorted(first1, last1));
    assert (std::is_sorted(first2, last2));
    while (first1 != last1 && first2 != last2) {
        if (*first1 < *first2) {
            ++first1;
        } else if (*first2 < *first1) {
            ++first2;
        } else {
            return true;
        }
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief independentCliqueSets
 ** ------------------------------------------------------------------------------------------------------------- */
FactorizeDFG::BicliqueSet FactorizeDFG::independentFactoringSets(BicliqueSet && factorings, const unsigned side) {
    using IndependentSetGraph = adjacency_matrix<undirectedS, unsigned>;
    using Vertex = IndependentSetGraph::vertex_descriptor;

    const auto l = factorings.size();
    IndependentSetGraph I(l);

    // Initialize our weights and determine the constraints
    for (auto i = factorings.begin(); i != factorings.end(); ++i) {
        I[std::distance(factorings.begin(), i)] = std::pow(side == 0 ? std::get<0>(*i).size() : std::get<1>(*i).size(), 2);
        for (auto j = i; ++j != factorings.end(); ) {
            if (intersects(i->first, j->first) && intersects(i->second, j->second)) {
                add_edge(std::distance(factorings.begin(), i), std::distance(factorings.begin(), j), I);
            }
        }
    }

    // Use the greedy algorithm to choose our independent set
    std::vector<Vertex> selected(0);
    selected.reserve(l);

    for (;;) {
        unsigned w = 0;
        Vertex u = 0;
        for (unsigned i = 0; i != l; ++i) {
            if (I[i] > w) {
                w = I[i];
                u = i;
            }
        }
        if (w < 2) break;
        selected.push_back(u);
        I[u] = 0;
        for (auto v : make_iterator_range(adjacent_vertices(u, I))) {
            I[v] = 0;
        }
    }

    // Sort the selected list and then remove the unselected cliques
    std::sort(selected.begin(), selected.end(), std::greater<Vertex>());
    auto end = factorings.end();
    for (const unsigned offset : selected) {
        end = factorings.erase(factorings.begin() + offset + 1, end) - 1;
    }
    factorings.erase(factorings.begin(), end);

    return std::move(factorings);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeCheckSet
 ** ------------------------------------------------------------------------------------------------------------- */
FactorizeDFG::CheckSet FactorizeDFG::makeCheckSet(PabloBlock * const scope, const ObjectSet & values) const {
    CheckSet checks;
    assert (scope);
    const unsigned baseScopeDepth = scopeDepthOf(scope);
    for (PabloAST * value : values) {
        if (isa<Statement>(value)) {
            unsigned scopeDepth = scopeDepthOf(cast<Statement>(value)->getParent());
            // Is this from a preceeding scope? Or a nested scope?
            if (scopeDepth < baseScopeDepth) {
                continue;
            } else if (scopeDepth > baseScopeDepth) {
                // If we're in a nested scope, we want to know what branch statement enters it and
                // add that to our checks instead of the operand itself.
                PabloBlock * nested = cast<Statement>(value)->getParent();
                for (;;) {
                    assert (nested);
                    value = nested->getBranch();
                    nested = nested->getParent();
                    if (nested == scope) {
                        break;
                    }
                }
            }
            checks.insert(value);
        }
    }
    return checks;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief firstIn
 ** ------------------------------------------------------------------------------------------------------------- */
inline Statement * FactorizeDFG::firstIn(PabloBlock * const scope, Statement * const initial, const ObjectSet & users) const {
    const auto checks = makeCheckSet(scope, users);
    Statement * stmt = initial;
    while (stmt) {
        if (LLVM_UNLIKELY(checks.count(stmt) != 0)) {
            return stmt;
        }
        stmt = stmt->getNextNode();
    }
    return scope->back();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief lastIn
 ** ------------------------------------------------------------------------------------------------------------- */
inline Statement * FactorizeDFG::lastIn(PabloBlock * const scope, Statement * const initial, const ObjectSet & operands) const {
    const auto checks = makeCheckSet(scope, operands);
    Statement * stmt = initial;
    while (stmt) {
        if (LLVM_UNLIKELY(checks.count(stmt) != 0)) {
            return stmt;
        }
        stmt = stmt->getPrevNode();
    }
    return scope->front();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief findInsertionScope
 ** ------------------------------------------------------------------------------------------------------------- */
inline PabloBlock * FactorizeDFG::findInsertionScope(const ObjectSet & users) const {
    std::vector<PabloBlock *> scopes;
    scopes.reserve(users.size());
    for (PabloAST * user : users) {
        PabloBlock * const scope = cast<Statement>(user)->getParent(); assert (scope);
        if (std::find(scopes.begin(), scopes.end(), scope) == scopes.end()) {
            scopes.push_back(scope);
        }
    }
    while (scopes.size() > 1) {
        // Find the LCA of both scopes then add the LCA back to the list of scopes.
        PabloBlock * scope1 = scopes.back(); scopes.pop_back();
        PabloBlock * scope2 = scopes.back(); scopes.pop_back();
        assert (scope1 != scope2);
        unsigned depth1 = scopeDepthOf(scope1);
        unsigned depth2 = scopeDepthOf(scope2);
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
        assert (depth1 == depth2);
        // Then iteratively step backwards until we find a matching set of scopes; this
        // must be the LCA of our original scopes.
        while (scope1 != scope2) {
            scope1 = scope1->getParent();
            scope2 = scope2->getParent();
        }
        assert (scope1 && scope2);
        if (std::find(scopes.begin(), scopes.end(), scope1) == scopes.end()) {
            scopes.push_back(scope1);
        }
    }
    assert (scopes.size() == 1);
    return scopes.front();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief factorize
 ** ------------------------------------------------------------------------------------------------------------- */
inline Variadic * FactorizeDFG::factorize(const TypeId typeId, PabloBlock * const scope, ObjectSet & operands, ObjectSet & users) const {
    Statement * const lastOperand = lastIn(scope, scope->back(), operands);
    Statement * const firstUsage = firstIn(scope, lastOperand, users);
    scope->setInsertPoint(firstUsage ? firstUsage->getPrevNode() : lastOperand);
    Variadic * factoring = nullptr;
    if (typeId == TypeId::Or) {
        factoring = scope->createOr(operands.begin(), operands.end());
    } else if (typeId == TypeId::And) {
        factoring = scope->createAnd(operands.begin(), operands.end());
    } else { // if (typeId == TypeId::Xor) {
        factoring = scope->createXor(operands.begin(), operands.end());
    }
    for (PabloAST * user : users) {
        assert (user->getClassTypeId() == typeId);
        for (PabloAST * op : operands) {
            cast<Variadic>(user)->deleteOperand(op);
        }
        cast<Variadic>(user)->addOperand(factoring);
    }
    return factoring;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief processFactoringSets
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool FactorizeDFG::processFactoringSets(const TypeId typeId, PabloBlock * const scope, BicliqueSet && factoringSets) const {
    const auto S = independentFactoringSets(std::move(factoringSets), 0);
    for (auto i : S) {
        factorize(typeId, scope, std::get<0>(i), std::get<1>(i));
    }
    return !S.empty();
}

///** ------------------------------------------------------------------------------------------------------------- *
// * @brief processFactoringSets
// ** ------------------------------------------------------------------------------------------------------------- */
//inline bool FactorizeDFG::processFactoringSets(const TypeId typeId, BicliqueSet && factoringSets, ObjectSet & factorings) const {
//    const auto S = independentFactoringSets(std::move(factoringSets), 0);
//    for (auto i : S) {
//        factorings.push_back(factorize(typeId, findInsertionScope(std::get<1>(i)), std::get<0>(i), std::get<1>(i)));
//    }
//    return !S.empty();
//}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief factor
 ** ------------------------------------------------------------------------------------------------------------- */
bool FactorizeDFG::factor(PabloBlock * const block) {
    Statement * stmt = block->front();
    bool factored = false;
    while (stmt) {
        if (isa<If>(stmt) || isa<While>(stmt)) {
            if (factor(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody())) {
                factored = true;
            }
        } else if (isa<Variadic>(stmt)) {
            if (processFactoringSets(stmt->getClassTypeId(), block, enumerateFactoringSets(cast<Variadic>(stmt)))) {
                factored = true;
            }
        }
        stmt = stmt->getNextNode();
    }
    return factored;
}

///** ------------------------------------------------------------------------------------------------------------- *
// * @brief factor
// ** ------------------------------------------------------------------------------------------------------------- */
//void FactorizeDFG::factor(PabloFunction & function, const TypeId typeId) {
//    ObjectSet vars;
//    for (unsigned i = 0; i != function.getNumOfParameters(); ++i) {
//        vars.push_back(function.getParameter(i));
//    }
//    while (processFactoringSets(typeId, enumerateFactoringSets(vars, function.getEntryBlock(), typeId), vars));
//}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief factor
 ** ------------------------------------------------------------------------------------------------------------- */
inline void FactorizeDFG::factor(PabloFunction & function) {
    while (factor(function.getEntryBlock()));
}

///** ------------------------------------------------------------------------------------------------------------- *
// * @brief rematerialize
// ** ------------------------------------------------------------------------------------------------------------- */
//void FactorizeDFG::rematerialize(PabloBlock * const block, LiveSet & priorSet) {

//    LiveSet liveSet(priorSet);

//    Statement * stmt = block->front();
//    block->setInsertPoint(nullptr);
//    while (stmt) {
//        for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
//            PabloAST * const op = stmt->getOperand(i);
//            auto f = std::find(liveSet.begin(), liveSet.end(), op);
//            if (f != liveSet.end()) {
//                liveSet.erase(f);
//                liveSet.push_back(op);
//            } else if (isa<Variadic>(op)) {
//                Variadic * const var = cast<Variadic>(op);
//                const double minimum = 4.0 + (3.0 / (double)var->getNumUses());
//                if ((double)(var->getNumOperands()) < minimum) {
//                    if (std::find(liveSet.begin(), liveSet.end(), var) == liveSet.end()) {
//                        // If we don't have this value in the live set, test whether it is cheaper to recompute it
//                        // rather than spill and reload it; if so, rematerialize the value and replace its reachable users.
//                        bool rematerialize = true;
//                        for (unsigned j = 0; j != var->getNumOperands(); ++j) {
//                            if (std::find(liveSet.begin(), liveSet.end(), var->getOperand(j)) == liveSet.end()) {
//                                rematerialize = false;
//                                break;
//                            }
//                        }
//                        if (rematerialize) {
//                            Variadic * replacement = nullptr;
//                            if (isa<And>(var)) {
//                                replacement = block->createAnd(var->begin(), var->end());
//                            } else if (isa<Or>(var)) {
//                                replacement = block->createOr(var->begin(), var->end());
//                            } else if (isa<Xor>(var)) {
//                                replacement = block->createXor(var->begin(), var->end());
//                            }
//                            raw_os_ostream out(std::cerr);
//                            out << "Remateralizing ";
//                            PabloPrinter::print(var, out);
//                            out << " as ";
//                            PabloPrinter::print(replacement, out);
//                            out << '\n';
//                            out.flush();
//                            for (PabloAST * user : var->users()) {
//                                if (LLVM_LIKELY(isa<Statement>(user))) {
//                                    PabloBlock * parent = cast<Statement>(user)->getParent();
//                                    while (parent) {
//                                        if (parent == block) {
//                                            stmt->replaceUsesOfWith(var, replacement);
//                                            break;
//                                        }
//                                        parent = parent->getParent();
//                                    }
//                                }
//                            }
//                            if (liveSet.size() > RematerializationThreshold) {
//                                liveSet.pop_front();
//                            }
//                            liveSet.push_back(replacement);
//                        }
//                    }
//                }
//            }
//        }
//        if (liveSet.size() > RematerializationThreshold) {
//            liveSet.pop_front();
//        }
//        liveSet.push_back(stmt);

//        if (isa<If>(stmt) || isa<While>(stmt)) {
//            rematerialize(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody(), liveSet);
//        }
//        block->setInsertPoint(stmt);
//        stmt = stmt->getNextNode();
//    }

//    // Let the prior set be an intersection between it and the current live set.
//    for (auto i = priorSet.begin(); i != priorSet.end(); ) {
//        if (LLVM_LIKELY(std::find(liveSet.begin(), liveSet.end(), *i) == liveSet.end())) {
//            i = priorSet.erase(i);
//        } else {
//            ++i;
//        }
//    }


//}

///** ------------------------------------------------------------------------------------------------------------- *
// * @brief rematerialize
// ** ------------------------------------------------------------------------------------------------------------- */
//inline void FactorizeDFG::rematerialize(PabloFunction & function) {
//    LiveSet live;
//    rematerialize(function.getEntryBlock(), live);
//}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief lower
 ** ------------------------------------------------------------------------------------------------------------- */
inline PabloAST * FactorizeDFG::lower(Variadic * const var, PabloBlock * block) const {

    using Graph = adjacency_list<hash_setS, vecS, bidirectionalS, PabloAST *, unsigned>;
    using Vertex = Graph::vertex_descriptor;
    using Map = flat_map<const PabloAST *, Vertex>;
    using SchedulingData = std::pair<unsigned, PabloAST *>;
    using SchedulingPriorityQueue = std::priority_queue<SchedulingData, std::vector<SchedulingData>, std::greater<SchedulingData>>;

    const unsigned NOT_STEP = 1;
    const unsigned BOOLEAN_STEP = 10;
    const unsigned OTHER_STEP = 30;
    const unsigned DESIRED_GAP = 30;

    assert (var->getParent() == block);
    assert (var->getNumOperands() > 2);

    const unsigned operands = var->getNumOperands();

    Graph G(operands + 1);
    Map M;

    G[operands] = var;
    M.emplace(var, operands);

    for (Vertex i = 0; i < operands; ++i) {
        PabloAST * const op = var->getOperand(i);
        G[i] = op;
        M.emplace(op, i);
        assert ("AST structural error!" && (op->getNumUses() > 0));
        for (PabloAST * user : op->users()) {
            if (LLVM_LIKELY(isa<Statement>(user))) {
                Statement * usage = cast<Statement>(user);
                PabloBlock * scope = usage->getParent();
                while (scope) {
                    if (scope == block) {
                        auto f = M.find(usage);
                        Vertex v = 0;
                        if (f == M.end()) {
                            v = add_vertex(usage, G);
                            M.emplace(usage, v);
                        } else {
                            v = f->second;
                        }
                        G[add_edge(i, v, G).first] = 0;
                        break;
                    }
                    usage = scope->getBranch();
                    assert (scope != scope->getParent());
                    scope = scope->getParent();
                }
            }
        }
    }

    unsigned time = 0;
    circular_buffer<std::pair<unsigned, Vertex>> defs(operands);
    for (Statement * stmt : *block) {
        switch (stmt->getClassTypeId()) {
            case TypeId::And:
            case TypeId::Or:
            case TypeId::Xor:
                assert (stmt->getNumOperands() == 2 || stmt == var);
                time += BOOLEAN_STEP;
                break;
            case TypeId::Not:
                time += NOT_STEP;
                break;
            default:
                time += OTHER_STEP;
        }
        auto f = M.find(stmt);
        if (LLVM_UNLIKELY(f != M.end())) {
            const auto u = f->second;
            if (LLVM_UNLIKELY(u == operands)) {
                assert (stmt == var);
                break;
            }
            for (auto e : make_iterator_range(in_edges(u, G)))   {
                G[e] = time;
            }
            if (u < operands) {
                defs.push_back(std::make_pair(time + DESIRED_GAP, u));
            }
        }
        assert (stmt != var);
        // Annotate G to indicate when we expect a statement will be available
        while (defs.size() > 0) {
            unsigned avail = 0;
            Vertex u = 0;
            std::tie(avail, u) = defs.front();
            if (avail > time) {
                break;
            }
            defs.pop_front();
            auto f = M.find(stmt);
            Vertex v = 0;
            if (f == M.end()) {
                v = add_vertex(stmt, G);
                M.emplace(stmt, v);
            } else {
                v = f->second;
            }
            G[add_edge(u, v, G).first] = time;
        }
    }

    for (Vertex u = 0; u < operands; ++u) {
        unsigned quantum = 0;
        Vertex v = operands;
        for (auto e : make_iterator_range(out_edges(u, G))) {
            if (quantum < G[e]) {
                quantum = G[e];
                v = target(e, G);
            }
        }
        clear_out_edges(u, G);
        G[add_edge(u, v, G).first] = quantum;
    }

    for (auto e : make_iterator_range(in_edges(operands, G)))   {
        G[e] = time;
    }

    assert (num_edges(G) == var->getNumOperands());

    SchedulingPriorityQueue Q;
    while (num_edges(G) > 0) {

        Graph::edge_descriptor f;
        unsigned t = std::numeric_limits<unsigned>::max();
        for (auto e : make_iterator_range(edges(G))) {
            if (in_degree(source(e, G), G) == 0) {
                if (t > G[e]) {
                    t = G[e];
                    f = e;
                }
            }
        }

        assert ("No edge selected!" && (t < std::numeric_limits<unsigned>::max()));

        const auto u = source(f, G);
        assert (u < operands);
        PabloAST * const op1 = var->getOperand(u);
        const auto v = target(f, G);
        assert (isa<Statement>(G[v]));
        remove_edge(f, G);

        // Since this might have been a target of a prior pairing, read the original operand value instead of
        // G when checking which value is indicated by u.
        block->setInsertPoint(cast<Statement>(G[v]));
        if (LLVM_LIKELY(Q.size() > 0)) {
            unsigned min = 0; PabloAST * op2 = nullptr;
            std::tie(min, op2) = Q.top();
            if (min < t) {
                Q.pop();
                PabloAST * result = nullptr;
                if (isa<And>(var)) {
                    result = block->createAnd(op1, op2);
                } else if (isa<Or>(var)) {
                    result = block->createOr(op1, op2);
                } else { // if (isa<Xor>(var)) {
                    result = block->createXor(op1, op2);
                }
                if (LLVM_LIKELY(isa<Statement>(result))) {
                    G[v] = result; // update the insertion point node value
                    t += DESIRED_GAP;
                } else {
                    G[v] = cast<Statement>(op2); // update the insertion point node value
                    t = time;
                }
                Q.emplace(t, result);
                continue;
            }
        }
        Q.emplace(t, op1);
    }

    // If we've done our best to schedule the statements and have operands remaining in our queue, generate a
    // tree of the remaining operands.
    while (Q.size() > 1) {
        unsigned q1 = 0; PabloAST * op1 = nullptr;
        std::tie(q1, op1) = Q.top();
        Q.pop();

        unsigned q2 = 0; PabloAST * op2 = nullptr;
        std::tie(q2, op2) = Q.top();
        Q.pop();

        assert (q2 >= q1);

        PabloAST * result = nullptr;
        if (isa<And>(var)) {
            result = block->createAnd(op1, op2);
        } else if (isa<Or>(var)) {
            result = block->createOr(op1, op2);
        } else { // if (isa<Xor>(var)) {
            result = block->createXor(op1, op2);
        }
        Q.emplace(q2 + DESIRED_GAP, result);
    }

    assert (Q.size() == 1);
    return std::get<1>(Q.top());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief lower
 ** ------------------------------------------------------------------------------------------------------------- */
void FactorizeDFG::lower(PabloBlock * const block) const {
    Statement * stmt = block->front();
    while (stmt) {
        if (isa<If>(stmt) || isa<While>(stmt)) {
            lower(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody());
        } else if (LLVM_UNLIKELY(stmt->getNumOperands() > 2 && isa<Variadic>(stmt))) {
            PabloAST * const replacement = lower(cast<Variadic>(stmt), block);
            stmt = stmt->replaceWith(replacement);
            if (LLVM_LIKELY(isa<Variadic>(replacement))) {
                elevate(cast<Variadic>(replacement), block);
            }
            continue;
        }
        stmt = stmt->getNextNode();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief elevate
 ** ------------------------------------------------------------------------------------------------------------- */
inline void FactorizeDFG::lower(PabloFunction & function) const {
    lower(function.getEntryBlock());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief noUsesOfAfter
 ** ------------------------------------------------------------------------------------------------------------- */
static bool noUsesOfAfter(const PabloAST * value, const Statement * const stmt) {
    if (value->getNumUses() == 1) {
        return true;
    }
    for (const Statement * after = stmt->getNextNode(); after; after = after->getNextNode()) {
        for (unsigned i = 0; i != after->getNumOperands(); ++i) {
            if (LLVM_UNLIKELY(after->getOperand(i) == value)) {
                return false;
            }
        }
    }
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief elevate
 ** ------------------------------------------------------------------------------------------------------------- */
inline void FactorizeDFG::elevate(Variadic * const var, PabloBlock * block) const {
    assert (var->getParent() == block);
    const unsigned operands = var->getNumOperands();
    PabloAST * operand[operands];
    unsigned count = 0;
    for (unsigned i = 0; i != operands; ++i) {
        PabloAST * const op = var->getOperand(i);
        if (noUsesOfAfter(op, var)) {
            operand[count++] = op;
        }
    }
    if (count) {
        PabloAST * def[operands];
        for (unsigned i = 0; i != operands; ++i) {
            PabloAST * op = var->getOperand(i);
            if (LLVM_LIKELY(isa<Statement>(op))) {
                PabloBlock * scope = cast<Statement>(op)->getParent();
                while (scope) {
                    if (scope == block) {
                        break;
                    }
                    op = scope->getBranch();
                    scope = scope->getParent();
                }
            }
            def[i] = op;
        }
        std::sort(operand, operand + count);
        std::sort(def, def + operands);
        for (Statement * ip = var->getPrevNode(); ip; ip = ip->getPrevNode()) {
            if (std::binary_search(def, def + operands, ip)) {
                var->insertAfter(ip);
                return;
            }
            for (unsigned i = 0; i != ip->getNumOperands(); ++i) {
                if (std::binary_search(operand, operand + count, ip->getOperand(i))) {
                    var->insertAfter(ip);
                    return;
                }
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief elevate
 ** ------------------------------------------------------------------------------------------------------------- */
void FactorizeDFG::elevate(PabloBlock * const block) const {
    Statement * stmt = block->front();
    while (stmt) {
        Statement * next = stmt->getNextNode();
        if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
            elevate(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody());
        } else if (LLVM_LIKELY(isa<Variadic>(stmt))) {
            elevate(cast<Variadic>(stmt), block);
        }
        stmt = next;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief elevate
 ** ------------------------------------------------------------------------------------------------------------- */
inline void FactorizeDFG::elevate(PabloFunction & function) const {
    elevate(function.getEntryBlock());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initialize
 ** ------------------------------------------------------------------------------------------------------------- */
void FactorizeDFG::initialize(PabloBlock * const block, const unsigned depth) {
    Statement * stmt = block->front();
    while (stmt) {
        if (isa<If>(stmt) || isa<While>(stmt)) {
            PabloBlock * body = isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody();
            mScopeDepth.emplace(body, depth);
            initialize(body, depth + 1);
        }
        stmt = stmt->getNextNode();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enumerateScopeDepth
 ** ------------------------------------------------------------------------------------------------------------- */
inline void FactorizeDFG::initialize(PabloFunction & function) {
    mScopeDepth.emplace(nullptr, 0);
    mScopeDepth.emplace(function.getEntryBlock(), 1);
    initialize(function.getEntryBlock(), 2);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief scopeDepthOf
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned FactorizeDFG::scopeDepthOf(const PabloAST * const expr) const {
    return LLVM_LIKELY(isa<Statement>(expr)) ? scopeDepthOf(cast<Statement>(expr)->getParent()) : 0;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief scopeDepthOf
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned FactorizeDFG::scopeDepthOf(const PabloBlock * const block) const {
    assert (block);
    const auto f = mScopeDepth.find(block);
    assert (f != mScopeDepth.end());
    return f->second;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief transform
 ** ------------------------------------------------------------------------------------------------------------- */
void FactorizeDFG::transform(PabloFunction & function) {
    FactorizeDFG ldfg;
    ldfg.initialize(function);

    ldfg.factor(function);
    #ifndef NDEBUG
    PabloVerifier::verify(function, "post-factoring");
    #endif
    Simplifier::optimize(function);

    ldfg.elevate(function);
    #ifndef NDEBUG
    PabloVerifier::verify(function, "post-elevation");
    #endif

    ldfg.lower(function);
    #ifndef NDEBUG
    PabloVerifier::verify(function, "post-lowering");
    #endif
}

}
