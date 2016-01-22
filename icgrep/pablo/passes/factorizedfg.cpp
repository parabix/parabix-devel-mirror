#include "factorizedfg.h"

#include <pablo/codegenstate.h>
#include <pablo/analysis/pabloverifier.hpp>
#include <pablo/optimizers/pablo_simplifier.hpp>
#include <pablo/passes/flattenassociativedfg.h>
#include <boost/container/flat_set.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/graph/topological_sort.hpp>
#include <queue>
#include <tuple>

#include <set>
#include <type_traits>

#include <pablo/printer_pablos.h>
#include <iostream>

using namespace boost;
using namespace boost::container;

namespace pablo {

using TypeId = PabloAST::ClassTypeId;

// TODO: move the "tryToPartiallyExtractVariadic" from the coalescedfg class into here to run prior to lowering?

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief lower
 *
 * The goal of this function is to try and lower a variadic operation into binary form while attempting to mitigate
 * register pressure under the assumption that LLVM is not going to significantly change the IR (which was observed
 * when assessing the ASM output of LLVM 3.6.1 using O3.)
 ** ------------------------------------------------------------------------------------------------------------- */
inline Statement * FactorizeDFG::lower(Variadic * const var, PabloBlock * block) const {

    using Graph = adjacency_list<hash_setS, vecS, bidirectionalS, PabloAST *, int>;
    using Vertex = Graph::vertex_descriptor;
    using Map = flat_map<const PabloAST *, Vertex>;
    using SchedulingData = std::pair<unsigned, PabloAST *>;
    using SchedulingQueue = std::priority_queue<SchedulingData, std::vector<SchedulingData>, std::greater<SchedulingData>>;

    const unsigned NOT_STEP = 1;
    const unsigned BOOLEAN_STEP = 10;
    const unsigned OTHER_STEP = 30;
    const unsigned DESIRED_GAP = 30;

    assert (var->getParent() == block);
    assert (var->getNumOperands() > 2);

    // Begin by computing a graph G depicting which operands are used by which statements. We know that
    // all of them are used by the Variadic but they might be used elsewhere in the current block.


    // If this operand was defined in this block, we want to indicate that in G as well so that we
    // don't mistakingly pair them together.

    const unsigned operands = var->getNumOperands();

    Graph G(operands + 1);
    Map M;

    G[operands] = var;
    M.emplace(var, operands);

    for (Vertex u = 0; u != operands; ++u) {
        PabloAST * const op = var->getOperand(u);
        G[u] = op;
        M.emplace(op, u);
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
                        G[add_edge(u, v, G).first] = 0;
                        break;
                    }
                    usage = scope->getBranch();
                    scope = scope->getParent();
                }
            }
        }
    }

    assert (M.count(var) == 1);

    unsigned estQuantum = 0;
    circular_buffer<std::pair<unsigned, Vertex>> defs(operands);
    for (Statement * stmt : *block) {
        switch (stmt->getClassTypeId()) {
            case TypeId::And:
            case TypeId::Or:
            case TypeId::Xor:
                estQuantum += BOOLEAN_STEP;
                break;
            case TypeId::Not:
                estQuantum += NOT_STEP;
                break;
            default:
                estQuantum += OTHER_STEP;
        }
        auto f = M.find(stmt);
        if (LLVM_UNLIKELY(f != M.end())) {
            const auto u = f->second;
            if (LLVM_UNLIKELY(u == operands)) {
                assert (stmt == var);
                break;
            }
            for (auto e : make_iterator_range(in_edges(u, G)))   {
                G[e] = estQuantum;
            }
            if (u < operands) {
                defs.push_back(std::make_pair(estQuantum + DESIRED_GAP, u));
            }
        }
        // Annotate G to indicate when we expect a statement will be available
        while (defs.size() > 0) {
            unsigned availQuantum = 0;
            Vertex u = 0;
            std::tie(availQuantum, u) = defs.front();
            if (availQuantum > estQuantum) {
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
            G[add_edge(u, v, G).first] = estQuantum;
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
        G[e] = estQuantum;
    }

    assert (num_edges(G) == var->getNumOperands());

    SchedulingQueue Q;
    while (num_edges(G) > 0) {

        Graph::edge_descriptor f;
        unsigned quantum = std::numeric_limits<unsigned>::max();
        for (auto e : make_iterator_range(edges(G))) {
            if (in_degree(source(e, G), G) == 0) {
                if (quantum > G[e]) {
                    quantum = G[e];
                    f = e;
                }
            }
        }

        assert ("No edge selected!" && (quantum < std::numeric_limits<unsigned>::max()));

        const auto u = source(f, G);
        assert (u < operands);
        const auto v = target(f, G);
        assert (isa<Statement>(G[v]));
        // Since this might have been a target of a prior pairing, read the original operand value instead of
        // G when checking which value is indicated by u.
        PabloAST * const op1 = var->getOperand(u);
        block->setInsertPoint(cast<Statement>(G[v]));
        remove_edge(f, G);

        if (LLVM_LIKELY(Q.size() > 0)) {
            unsigned minQuantum = 0; PabloAST * op2 = nullptr;
            std::tie(minQuantum, op2) = Q.top();
            if (minQuantum < quantum) {
                Q.pop();
                PabloAST * result = nullptr;
                if (isa<And>(var)) {
                    result = block->createAnd(op1, op2);
                } else if (isa<Or>(var)) {
                    result = block->createOr(op1, op2);
                } else { // if (isa<Xor>(var)) {
                    result = block->createXor(op1, op2);
                }
                Q.emplace(quantum + DESIRED_GAP, result);
                G[v] = result; // update the insertion point node value
                continue;
            }
        }
        Q.emplace(quantum, op1);
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

        PabloAST * result = nullptr;
        if (isa<And>(var)) {
            result = block->createAnd(op1, op2);
        } else if (isa<Or>(var)) {
            result = block->createOr(op1, op2);
        } else { // if (isa<Xor>(var)) {
            result = block->createXor(op1, op2);
        }
        Q.emplace(std::max(q1, q2) + DESIRED_GAP, result);
    }

    assert (Q.size() == 1);
    return var->replaceWith(std::get<1>(Q.top()));
}

#if 0
using EnumerationMap = flat_map<const PabloAST *, unsigned>;

inline void enumerate(PabloBlock * const block, EnumerationMap & M, std::vector<Statement *> & S) {
    for (Statement * stmt : *block) {
        M.emplace(stmt, static_cast<int>(S.size()));
        S.push_back(stmt);
    }
}

inline static unsigned indexOf(const PabloAST * const expr, const EnumerationMap & M) {
    assert (expr);
    const auto f = M.find(expr);
    assert (f != M.end());
    return f->second;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief lower
 *
 * The goal of this function is to try and lower a variadic operation into binary form while attempting to mitigate
 * register pressure under the assumption that LLVM is not going to significantly change the IR (which was observed
 * when assessing the ASM output of LLVM 3.6.1 using O3.)
 ** ------------------------------------------------------------------------------------------------------------- */
inline Statement * FactorizeDFG::lower(Variadic * const var, PabloBlock * block) const {

    assert (var->getNumOperands() > 2);

    EnumerationMap M;
    std::vector<Statement *> S;

    enumerate(block, M, S);

    const int varIdx = indexOf(var, M);

    circular_buffer<std::tuple<unsigned, unsigned, PabloAST *>> lowered(var->getNumOperands());
    for (unsigned i = 0; i != var->getNumOperands(); ++i) {
        PabloAST * const op = var->getOperand(i);

        unsigned defIdx = 0;
        if (LLVM_LIKELY(isa<Statement>(op))) {
            Statement * producer = cast<Statement>(op);
            PabloBlock * scope = producer->getParent();
            while (scope && (scope != block)) {
                producer = scope->getBranch();
                scope = scope->getParent();
            }
            defIdx = producer ? indexOf(producer, M) : 0;
        }

        unsigned useIdx = defIdx;
        for (PabloAST * user : op->users()) {
            // if this user is in the current scope or a nested scope of this block
            if ((cast<Statement>(user)->getParent() == block) && (user != var)) {
                useIdx = std::max(useIdx, indexOf(user, M));
            }
        }
        if (useIdx > varIdx) {
            useIdx = varIdx;
        }
        assert (!lowered.full());
        lowered.push_back(std::make_tuple(useIdx, defIdx, op));
    }

    std::sort(lowered.begin(), lowered.end());

    PabloAST * result = nullptr;
    while (lowered.size() > 1) {

        PabloAST * op1, * op2;
        unsigned def1, def2;
        unsigned use1, use2;

        std::tie(use1, def1, op1) = lowered.front(); lowered.pop_front();
        std::tie(use2, def2, op2) = lowered.front(); lowered.pop_front();

//        if (((def1 < def2) ? ((def1 + 3) > def2) : ((def2 + 3) > def1)) && (lowered.size() > 0)) {
//            unsigned def3 = def1;
//            unsigned use3 = use1;
//            PabloAST * op3 = op1;
//            std::tie(use1, def1, op1) = lowered.front(); lowered.pop_front();
//            lowered.push_front(std::make_tuple(use3, def3, op3));
//        }

        // Is the last use of op1 prior to the definition of op2?
        if (use1 < def2) {
            use1 = def2;
        }

        block->setInsertPoint(S[use1]);

        if (isa<And>(var)) {
            result = block->createAnd(op1, op2);
        } else if (isa<Or>(var)) {
            result = block->createOr(op1, op2);
        } else { // if (isa<Xor>(var)) {
            result = block->createXor(op1, op2);
        }

        S[use1] = cast<Statement>(result);

        assert (!lowered.full());
        lowered.push_front(std::make_tuple(use1, use1, result));
    }
    assert (result);

    return var->replaceWith(result, true);
}
#endif

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief lower
 ** ------------------------------------------------------------------------------------------------------------- */
void FactorizeDFG::lower(PabloBlock * const block) const {
    Statement * stmt = block->front();
    while (stmt) {
        if (isa<If>(stmt) || isa<While>(stmt)) {
            lower(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody());
        } else if ((stmt->getNumOperands() > 2) && (isa<And>(stmt) || isa<Or>(stmt) || isa<Xor>(stmt))) {
            stmt = lower(cast<Variadic>(stmt), block);
            continue;
        }
        stmt = stmt->getNextNode();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief lower
 ** ------------------------------------------------------------------------------------------------------------- */
inline void FactorizeDFG::lower(PabloFunction & function) const {
    lower(function.getEntryBlock());
}

#if 0

using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, PabloAST *>;
using Vertex = Graph::vertex_descriptor;
using Map = flat_map<const PabloAST *, Vertex>;

using VertexSet = std::vector<PabloAST *>;
using Biclique = std::pair<VertexSet, VertexSet>; // [{Operands}, {Users}]
using BicliqueSet = std::set<Biclique>;
using TypeId = PabloAST::ClassTypeId;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enumerateBicliques
 *
 * Adaptation of the MICA algorithm as described in "Consensus algorithms for the generation of all maximal
 * bicliques" by Alexe et. al. (2003).
 ** ------------------------------------------------------------------------------------------------------------- */
inline void FactorizeDFG::enumerateBicliques(Variadic * const var, BicliqueSet & bicliques) {
    using IntersectionSets = flat_set<VertexSet>;

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

    for (auto Bi = B.begin(); Bi != B.end(); ++Bi) {
        VertexSet Ai(var->begin(), var->end());
        for (const PabloAST * user : *Bi) {
            VertexSet Aj(cast<Variadic>(user)->begin(), cast<Variadic>(user)->end());
            std::sort(Aj.begin(), Aj.end());
            VertexSet Ak;
            Ak.reserve(std::min<unsigned>(Ai.size(), Aj.size()));
            std::set_intersection(Ai.begin(), Ai.end(), Aj.begin(), Aj.end(), std::back_inserter(Ak));
            Ai.swap(Ak);
        }
        if (Ai.size() > 1 && Bi->size() > 1) {
            bicliques.emplace(std::move(Ai), std::move(*Bi));
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief intersects
 ** ------------------------------------------------------------------------------------------------------------- */
template <class Type>
inline bool intersects(const Type & A, const Type & B) {
    auto first1 = A.begin(), last1 = A.end();
    auto first2 = B.begin(), last2 = B.end();
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
inline void FactorizeDFG::independentCliqueSets(BicliqueSet & bicliques) {
    using IndependentSetGraph = adjacency_list<hash_setS, vecS, undirectedS, unsigned>;

    const auto l = bicliques.size();
    IndependentSetGraph I(l);

    // Initialize our weights and determine the constraints
    for (auto i = bicliques.begin(); i != bicliques.end(); ++i) {
        I[std::distance(bicliques.begin(), i)] = std::pow(std::get<0>(*i).size(), 2);
        for (auto j = i; ++j != bicliques.end(); ) {
            if (intersects(i->second, j->second) && intersects(i->first, j->first)) {
                add_edge(std::distance(bicliques.begin(), i), std::distance(bicliques.begin(), j), I);
            }
        }
    }

    // Use the greedy algorithm to choose our independent set
    std::vector<Vertex> selected;
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
    auto end = bicliques.end();
    for (const unsigned offset : selected) {
        end = bicliques.erase(bicliques.begin() + offset + 1, end) - 1;
    }
    bicliques.erase(bicliques.begin(), end);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief findInsertionPoint
 *
 * Look for the first user in this scope; if none can be found, look for the last operand.
 ** ------------------------------------------------------------------------------------------------------------- */
inline PabloBlock * FactorizeDFG::findInsertionPoint(NodeSet & operands, NodeSet & users) const {

    ScopeSet scopes;
    scopes.reserve(users.size());

    for (PabloAST * user : users) {
        PabloBlock * const scope = cast<Statement>(user)->getParent(); assert (scope);
        if (std::find(scopes.begin(), scopes.end(), scope) == scopes.end()) {
            scopes.push_back(scope);
        }
    }

    if (LLVM_UNLIKELY(scopes.size() == 1)) {
        PabloBlock * const scope = scopes.front();
        Statement * stmt = scope->front();
        std::sort(users.begin(), users.end());
        while (stmt) {
            if (LLVM_UNLIKELY(std::binary_search(users.begin(), users.end(), stmt))) {
                scope->setInsertPoint(stmt->getPrevNode());
                return scope;
            }
            stmt = stmt->getNextNode();
        }
        llvm_unreachable("Failed to locate user in single scope block!");
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

    PabloBlock * const scope = scopes.front();
    Statement * stmt = scope->back();
    std::sort(operands.begin(), operands.end());
    while (stmt) {
        if (isa<If>(stmt)) {
            for (Assign * def : cast<If>(stmt)->getDefined()) {
                if (LLVM_UNLIKELY(std::binary_search(operands.begin(), operands.end(), def))) {
                    scope->setInsertPoint(stmt);
                    return scope;
                }
            }
        } else if (isa<While>(stmt)) {
            for (Next * var : cast<While>(stmt)->getVariants()) {
                if (LLVM_UNLIKELY(std::binary_search(operands.begin(), operands.end(), var))) {
                    scope->setInsertPoint(stmt);
                    return scope;
                }
            }
        } else if (LLVM_UNLIKELY(std::binary_search(operands.begin(), operands.end(), stmt))) {
            scope->setInsertPoint(stmt);
            return scope;
        }
        stmt = stmt->getPrevNode();
    }
    scope->setInsertPoint(nullptr);
    return scope;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief processBicliques
 ** ------------------------------------------------------------------------------------------------------------- */
void FactorizeDFG::processBicliques(BicliqueSet & bicliques) const {
    independentCliqueSets(bicliques);
    for (Biclique & B : bicliques) {
        VertexSet & operands = B.first;
        VertexSet & users = B.second;
        PabloBlock * const block = findInsertionPoint(operands, users);
        Variadic * factored = nullptr;
        if (isa<And>(users.front())) {
            factored = block->createAnd(operands.begin(), operands.end());
        } else if (isa<Or>(users.front())) {
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
    bicliques.clear();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief factor
 *
 * Perform common subexpression elimination on the Variadic statements
 ** ------------------------------------------------------------------------------------------------------------- */
void FactorizeDFG::factor(PabloBlock * const block, BicliqueSet & bicliques) const {
    Statement * stmt = block->front();
    while (stmt) {
        if (isa<If>(stmt) || isa<While>(stmt)) {
            processBicliques(bicliques);
            factor(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody(), bicliques);
        } else if (stmt->getNumOperands() > 2 && (isa<And>(stmt) || isa<Or>(stmt) || isa<Xor>(stmt))) {
            enumerateBicliques(cast<Variadic>(stmt), bicliques);
        }
        stmt = stmt->getNextNode();
    }
    processBicliques(bicliques);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief factor
 ** ------------------------------------------------------------------------------------------------------------- */
inline void FactorizeDFG::factor(PabloFunction & function) const {
    BicliqueSet bicliques;
    factor(function.getEntryBlock(), bicliques);
}

#endif

#if 1

using VertexSet = std::vector<PabloAST *>;
using Biclique = std::pair<VertexSet, VertexSet>; // [{Operands}, {Users}]
using BicliqueSet = std::vector<Biclique>;
using TypeId = PabloAST::ClassTypeId;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief findAllFactoringsOf
 *
 * Adaptation of the MICA algorithm as described in "Consensus algorithms for the generation of all maximal
 * bicliques" by Alexe et. al. (2003).
 ** ------------------------------------------------------------------------------------------------------------- */
inline BicliqueSet FactorizeDFG::findAllFactoringsOf(Variadic * const var) {
    using IntersectionSets = flat_set<VertexSet>;

    IntersectionSets B1;
    const TypeId typeId = var->getClassTypeId();
    for (unsigned i = 0; i != var->getNumOperands(); ++i) {
        PabloAST * const op = var->getOperand(i);
        VertexSet B;
        B.reserve(op->getNumUses());
        for (PabloAST * user : op->users()) {
            if (user->getClassTypeId() == typeId) {
                // Only consider a user who is in the same or a nested scope?
                PabloBlock * scope = cast<Variadic>(user)->getParent();
                while (scope) {
                    if (scope == var->getParent()) {
                        B.push_back(user);
                        break;
                    }
                    scope = scope->getParent();
                }

//                B.push_back(user);
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
    for (auto Bi = B.begin(); Bi != B.end(); ++Bi) {
        VertexSet Ai(var->begin(), var->end());
        for (const PabloAST * user : *Bi) {
            VertexSet Aj(cast<Variadic>(user)->begin(), cast<Variadic>(user)->end());
            std::sort(Aj.begin(), Aj.end());
            VertexSet Ak;
            Ak.reserve(std::min<unsigned>(Ai.size(), Aj.size()));
            std::set_intersection(Ai.begin(), Ai.end(), Aj.begin(), Aj.end(), std::back_inserter(Ak));
            Ai.swap(Ak);
        }
        if (Ai.size() > 1 && Bi->size() > 1) {
            bicliques.emplace_back(std::move(Ai), std::move(*Bi));
        }
    }
    return bicliques;
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
inline void FactorizeDFG::independentCliqueSets(BicliqueSet & bicliques) {
    using IndependentSetGraph = adjacency_list<hash_setS, vecS, undirectedS, unsigned>;
    using Vertex = IndependentSetGraph::vertex_descriptor;

    const auto l = bicliques.size();
    IndependentSetGraph I(l);

    // Initialize our weights and determine the constraints
    for (auto i = bicliques.begin(); i != bicliques.end(); ++i) {
        I[std::distance(bicliques.begin(), i)] = std::pow(std::get<0>(*i).size(), 2);
        for (auto j = i; ++j != bicliques.end(); ) {
            if (intersects(i->second, j->second) && intersects(i->first, j->first)) {
                add_edge(std::distance(bicliques.begin(), i), std::distance(bicliques.begin(), j), I);
            }
        }
    }

    // Use the greedy algorithm to choose our independent set
    std::vector<Vertex> selected;
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
    auto end = bicliques.end();
    for (const unsigned offset : selected) {
        end = bicliques.erase(bicliques.begin() + offset + 1, end) - 1;
    }
    bicliques.erase(bicliques.begin(), end);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief findInsertionScope
 ** ------------------------------------------------------------------------------------------------------------- */
inline PabloBlock * FactorizeDFG::findInsertionScope(const NodeSet & users) const {
    ScopeSet scopes;
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
        if (scope1 != scope2) {
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
            // Then iteratively step backwards until we find a matching set of scopes; this
            // must be the LCA of our original scopes.
            while (scope1 != scope2) {
                scope1 = scope1->getParent();
                scope2 = scope2->getParent();
            }
            assert (scope1 && scope2);
        }
        if (LLVM_LIKELY(std::find(scopes.begin(), scopes.end(), scope1) == scopes.end())) {
            scopes.push_back(scope1);
        }
    }
    assert (scopes.size() == 1);

    return scopes.front();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeCheckSet
 ** ------------------------------------------------------------------------------------------------------------- */
FactorizeDFG::CheckSet FactorizeDFG::makeCheckSet(PabloBlock * const scope, const NodeSet & values) const {
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
inline Statement * FactorizeDFG::firstIn(PabloBlock * const scope, Statement * stmt, const NodeSet & users) const {
    const auto checks = makeCheckSet(scope, users);
    while (stmt) {
        if (LLVM_UNLIKELY(checks.count(stmt) != 0)) {
            break;
        }
        stmt = stmt->getNextNode();
    }
    return stmt;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief lastIn
 ** ------------------------------------------------------------------------------------------------------------- */
inline Statement * FactorizeDFG::lastIn(PabloBlock * const scope, Statement * stmt, const NodeSet & operands) const {
    const auto checks = makeCheckSet(scope, operands);
    while (stmt) {
        if (LLVM_UNLIKELY(checks.count(stmt) != 0)) {
            break;
        }
        stmt = stmt->getPrevNode();
    }
    return stmt;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief findInsertionPoint
 *
 * Look for the first user in this scope; if none can be found, look for the last operand.
 ** ------------------------------------------------------------------------------------------------------------- */
inline PabloBlock * FactorizeDFG::findInsertionPoint(const NodeSet & operands, const NodeSet & users) const {
    PabloBlock * const scope = findInsertionScope(users);
    Statement * const lastOperand = lastIn(scope, scope->back(), operands);
    Statement * const firstUsage = firstIn(scope, lastOperand, users);
    scope->setInsertPoint(firstUsage ? firstUsage->getPrevNode() : lastOperand);
    return scope;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief factor
 ** ------------------------------------------------------------------------------------------------------------- */
inline void FactorizeDFG::factor(Variadic * const var, Variadics & factors) const {
    if (var->getNumOperands() > 2) {
        BicliqueSet S = findAllFactoringsOf(var);
        if (S.empty()) return;
        independentCliqueSets(S);
        assert (S.size() > 0);
        for (Biclique & B : S) {
            VertexSet & operands = B.first;
            VertexSet & users = B.second;
            PabloBlock * const block = findInsertionPoint(operands, users);
            Variadic * factoring = nullptr;
            if (isa<And>(var)) {
                factoring = block->createAnd(operands.begin(), operands.end());
            } else if (isa<Or>(var)) {
                factoring = block->createOr(operands.begin(), operands.end());
            } else { // if (isa<Xor>(var)) {
                factoring = block->createXor(operands.begin(), operands.end());
            }
            for (PabloAST * user : users) {
                for (PabloAST * op : operands) {
                    cast<Variadic>(user)->deleteOperand(op);
                }
                cast<Variadic>(user)->addOperand(factoring);
            }
            if (factoring->getNumOperands() > 2) {
                if (LLVM_UNLIKELY(factors.full())) {
                    factors.set_capacity(factors.capacity() + factors.capacity() / 2);
                }
                factors.push_back(factoring);
            }
        }
        if (var->getNumOperands() > 2) {
            if (LLVM_UNLIKELY(factors.full())) {
                factors.set_capacity(factors.capacity() + factors.capacity() / 2);
            }
            factors.push_back(var);
        } else if (var->getNumOperands()  == 1) {
            var->replaceWith(var->getOperand(0));
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief factor
 *
 * Perform common subexpression elimination on the Variadic statements
 ** ------------------------------------------------------------------------------------------------------------- */
void FactorizeDFG::factor(PabloBlock * const block, Variadics & vars) const {
    Statement * stmt = block->front();
    while (stmt) {
        if (isa<If>(stmt) || isa<While>(stmt)) {
            factor(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody(), vars);
        } else if (stmt->getNumOperands() > 2 && (isa<And>(stmt) || isa<Or>(stmt) || isa<Xor>(stmt))) {
            assert (!vars.full());
            vars.push_back(cast<Variadic>(stmt));
        }
        stmt = stmt->getNextNode();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief factor
 ** ------------------------------------------------------------------------------------------------------------- */
inline void FactorizeDFG::factor(PabloFunction & function) const {
    Variadics vars(mNumOfVariadics);
    factor(function.getEntryBlock(), vars);
    while (vars.size() > 0) {
        Variadic * var = vars.front();
        vars.pop_front();
        factor(var, vars);
    }
}

#endif

#if 0

using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, PabloAST *>;
using Vertex = Graph::vertex_descriptor;
using Map = flat_map<const PabloAST *, Vertex>;

using VertexSet = std::vector<Vertex>;
using Biclique = std::pair<VertexSet, VertexSet>; // [{Operands}, {Users}]
using BicliqueSet = std::vector<Biclique>;
using TypeId = PabloAST::ClassTypeId;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enumerateBicliques
 *
 * Adaptation of the MICA algorithm as described in "Consensus algorithms for the generation of all maximal
 * bicliques" by Alexe et. al. (2003). Note: this implementation considers all verticies in set A to be in
 * bipartition A and their adjacencies to be in B.
  ** ------------------------------------------------------------------------------------------------------------- */
template <class Graph>
static BicliqueSet enumerateBicliques(const Graph & G, const VertexSet & A) {
    using IntersectionSets = std::set<VertexSet>;

    IntersectionSets B1;
    for (auto u : A) {
        if (out_degree(u, G) > 0) {
            VertexSet incomingAdjacencies;
            incomingAdjacencies.reserve(out_degree(u, G));
            for (auto e : make_iterator_range(out_edges(u, G))) {
                incomingAdjacencies.push_back(target(e, G));
            }
            std::sort(incomingAdjacencies.begin(), incomingAdjacencies.end());
            B1.insert(std::move(incomingAdjacencies));
        }
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

    for (;;) {
        if (Bi.empty()) {
            break;
        }
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

    BicliqueSet cliques;
    cliques.reserve(B.size());
    for (auto Bi = B.begin(); Bi != B.end(); ++Bi) {
        VertexSet Ai(A);
        for (const Vertex u : *Bi) {
            VertexSet Aj;
            Aj.reserve(in_degree(u, G));
            for (auto e : make_iterator_range(in_edges(u, G))) {
                Aj.push_back(source(e, G));
            }
            std::sort(Aj.begin(), Aj.end());
            VertexSet Ak;
            Ak.reserve(std::min(Ai.size(), Aj.size()));
            std::set_intersection(Ai.begin(), Ai.end(), Aj.begin(), Aj.end(), std::back_inserter(Ak));
            Ai.swap(Ak);
        }
        assert (Ai.size() > 0); // cannot happen if this algorithm is working correctly
        cliques.emplace_back(std::move(Ai), std::move(*Bi));
    }
    return std::move(cliques);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief intersects
 ** ------------------------------------------------------------------------------------------------------------- */
template <class Type>
inline bool intersects(const Type & A, const Type & B) {
    auto first1 = A.begin(), last1 = A.end();
    auto first2 = B.begin(), last2 = B.end();
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
 * @brief getMaximalIndependentBicliques
 ** ------------------------------------------------------------------------------------------------------------- */
template <unsigned side = 1>
inline static BicliqueSet && maximalIndependentBicliques(BicliqueSet && cliques, const unsigned minimum = 1) {
    using IndependentSetGraph = adjacency_list<hash_setS, vecS, undirectedS, unsigned>;

    static_assert((side == 0 || side == 1), "Side must be 0 (bipartition A) or 1 (bipartition B)");
    assert ("Minimum must be at least 1 or an infinite number of empty sets would be generated!" && minimum > 0);

    const auto l = cliques.size();
    IndependentSetGraph I(l);

    // Initialize our weights
    for (unsigned i = 0; i != l; ++i) {
        I[i] = std::pow(std::get<side>(cliques[i]).size(), 2);
    }

    // Determine our constraints
    for (unsigned i = 0; i != l; ++i) {
        for (unsigned j = i + 1; j != l; ++j) {
            if (intersects(std::get<side>(cliques[i]), std::get<side>(cliques[j]))) {
                add_edge(i, j, I);
            }
        }
    }

    // Use the greedy algorithm to choose our independent set
    VertexSet selected;
    for (;;) {
        unsigned w = 0;
        Vertex u = 0;
        for (unsigned i = 0; i != l; ++i) {
            if (I[i] > w) {
                w = I[i];
                u = i;
            }
        }
        if (w < minimum) break;
        selected.push_back(u);
        I[u] = 0;
        for (auto v : make_iterator_range(adjacent_vertices(u, I))) {
            I[v] = 0;
        }
    }

    // Sort the selected list and then remove the unselected cliques
    std::sort(selected.begin(), selected.end(), std::greater<Vertex>());
    auto end = cliques.end();
    for (const unsigned offset : selected) {
        end = cliques.erase(cliques.begin() + offset + 1, end) - 1;
    }
    cliques.erase(cliques.begin(), end);

    return std::move(cliques);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief factor
 ** ------------------------------------------------------------------------------------------------------------- */
template<class Type>
inline void factorAny(Graph & G, VertexSet & A, const Biclique & Si, PabloBlock * const block) {

    static_assert (std::is_same<Type, And>::value || std::is_same<Type, Or>::value || std::is_same<Type, Xor>::value, "Can only factor And, Or or Xor statements here!");

    flat_set<Statement *> S;
    for (auto u : Si.first) {
        if (isa<Type>(G[u])) {
            S.insert(cast<Type>(G[u]));
        }
    }
    if (S.empty()) {
        return;
    }
    // find the insertion point for this statement type
    for (Statement * ip : *block) {
        if (S.count(ip)) {
            block->setInsertPoint(ip->getPrevNode());
            break;
        }
    }
    Variadic * factored = nullptr;
    if (std::is_same<Type, And>::value) {
        factored = block->createAnd(Si.second.size());
    } else if (std::is_same<Type, Or>::value) {
        factored = block->createOr(Si.second.size());
    } else if (std::is_same<Type, Xor>::value) {
        factored = block->createXor(Si.second.size());
    }
    const auto u = add_vertex(factored, G);
    A.push_back(u);
    for (auto v : Si.second) {
        factored->addOperand(G[v]);
        add_edge(u, v, G);
    }
    const auto w = add_vertex(factored, G);
    for (auto u : Si.first) {
        if (isa<Type>(G[u])) {
            Type * factoring = cast<Type>(G[u]);
            for (auto v : Si.second) {
                factoring->deleteOperand(G[v]);
                remove_edge(u, v, G);
            }
            factoring->addOperand(factored);
            add_edge(u, w, G);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief eliminateCommonSubexpressions
 ** ------------------------------------------------------------------------------------------------------------- */
void eliminateCommonSubexpressions(Graph & G, VertexSet & A, PabloBlock * const block) {
    for (;;) {
        const auto S = maximalIndependentBicliques<1>(enumerateBicliques(G, A), 2);
        if (S.empty()) {
            break;
        }
        for (const Biclique Si : S) {
            factorAny<And>(G, A, Si, block);
            factorAny<Or>(G, A, Si, block);
            factorAny<Xor>(G, A, Si, block);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief factor
 *
 * Perform common subexpression elimination on the Variadic statements
 ** ------------------------------------------------------------------------------------------------------------- */
void FactorizeDFG::factor(PabloBlock * const block) {

    Graph G;
    VertexSet A;
    Map B;

    Statement * stmt = block->front();
    while (stmt) {
        if (isa<If>(stmt) || isa<While>(stmt)) {
            eliminateCommonSubexpressions(G, A, block);
            G.clear(); A.clear(); B.clear();
            factor(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody());
        } else if (isa<And>(stmt) || isa<Or>(stmt) || isa<Xor>(stmt)) {
            // When we encounter a reassociative statement, add it and its operands to our bigraph G.
            const auto u = add_vertex(stmt, G);
            A.push_back(u);
            for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
                auto f = B.find(stmt->getOperand(i));
                if (f == B.end()) {
                    f = B.emplace(stmt->getOperand(i), add_vertex(stmt->getOperand(i), G)).first;
                }
                add_edge(f->second, u, G);
            }
        }
        stmt = stmt->getNextNode();
    }
    eliminateCommonSubexpressions(G, A, block);
}
#endif

#if 0

using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, PabloAST *>;
using Matrix = adjacency_matrix<directedS>;
using Vertex = Graph::vertex_descriptor;
using Map = flat_map<const PabloAST *, Vertex>;
using TypeId = PabloAST::ClassTypeId;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getVertex
 ** ------------------------------------------------------------------------------------------------------------- */
static inline Vertex getVertex(PabloAST * expr, Graph & G, Map & M) {
    auto f = M.find(expr);
    if (LLVM_LIKELY(f != M.end())) {
        return f->second;
    } else {
        const Vertex u = add_vertex(expr, G);
        M.emplace(expr, u);
        return u;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief buildDependencyGraph
 ** ------------------------------------------------------------------------------------------------------------- */
static void buildDependencyGraph(PabloBlock * const block, Graph & G, Map & M) {
    Statement * stmt = block->front();
    while (stmt) {
        if (isa<If>(stmt) || isa<While>(stmt)) {
            buildDependencyGraph(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody(), G, M);
        } else if (isa<And>(stmt) || isa<Or>(stmt) || isa<Xor>(stmt)) {
            const Vertex u = getVertex(stmt, G, M);
            for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
                add_edge(getVertex(stmt->getOperand(i), G, M), u, G);
            }
        }
        stmt = stmt->getNextNode();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief transitiveClosure
 ** ------------------------------------------------------------------------------------------------------------- */
static Matrix transitiveClosureOf(const Graph & G) {
    std::vector<Vertex> rto; // reverse topological ordering
    rto.reserve(num_vertices(G));
    topological_sort(G, std::back_inserter(rto));
    Matrix M(num_vertices(G));
    for (auto u : rto) {
        for (auto ej : make_iterator_range(in_edges(u, G))) {
            add_edge(source(ej, G), target(ej, G), M);
        }
        for (auto ei : make_iterator_range(out_edges(u, M))) {
            for (auto ej : make_iterator_range(in_edges(u, G))) {
                add_edge(source(ej, G), target(ei, M), M);
            }
        }
        add_edge(u, u, M);
    }
    return std::move(M);
}

using VertexSet = std::vector<Vertex>;
using Biclique = std::pair<VertexSet, VertexSet>; // [{Operands}, {Users}]
using BicliqueSet = flat_set<Biclique>;
using IntersectionSets = std::set<VertexSet>;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enumerateBicliques
 *
 * Adaptation of the MICA algorithm as described in "Consensus algorithms for the generation of all maximal
 * bicliques" by Alexe et. al. (2003). Note: this implementation considers all verticies in set A to be in
 * bipartition A and their adjacencies to be in B.
  ** ------------------------------------------------------------------------------------------------------------- */
inline static void enumerateBicliques(VertexSet & A, VertexSet & B, const Graph & G, BicliqueSet & bicliques) {

    std::sort(A.begin(), A.end());
    std::sort(B.begin(), B.end());

    VertexSet S;
    IntersectionSets B1;
    for (auto u : A) {
        S.reserve(out_degree(u, G));
        for (auto e : make_iterator_range(out_edges(u, G))) {
            S.push_back(target(e, G));
        }
        assert (S.size() > 0);
        std::sort(S.begin(), S.end());
        VertexSet T;        
        std::set_intersection(B.begin(), B.end(), S.begin(), S.end(), std::back_inserter(T));
        assert (T.size() > 0);
        B1.emplace(std::move(T));
        S.clear();
    }

    IntersectionSets C(B1);
    IntersectionSets Bi;
    for (auto i = B1.begin(); i != B1.end(); ++i) {
        for (auto j = i; ++j != B1.end(); ) {
            std::set_intersection(i->begin(), i->end(), j->begin(), j->end(), std::back_inserter(S));
            if (S.size() > 0) {
                if (C.count(S) == 0) {
                    Bi.insert(S);
                }
                S.clear();
            }
        }
    }

    for (;;) {
        if (Bi.empty()) {
            break;
        }
        C.insert(Bi.begin(), Bi.end());
        IntersectionSets Bk;
        for (auto i = B1.begin(); i != B1.end(); ++i) {
            for (auto j = Bi.begin(); j != Bi.end(); ++j) {
                std::set_intersection(i->begin(), i->end(), j->begin(), j->end(), std::back_inserter(S));
                if (S.size() > 0) {
                    if (C.count(S) == 0) {
                        Bk.insert(S);
                    }
                    S.clear();
                }
            }
        }
        Bi.swap(Bk);
    }

    for (auto Bi = C.begin(); Bi != C.end(); ++Bi) {
        VertexSet Ai(A);
        for (const Vertex u : *Bi) {
            VertexSet Aj;
            Aj.reserve(in_degree(u, G));
            for (auto e : make_iterator_range(in_edges(u, G))) {
                Aj.push_back(source(e, G));
            }
            std::sort(Aj.begin(), Aj.end());
            VertexSet Ak;
            Ak.reserve(std::min(Ai.size(), Aj.size()));
            std::set_intersection(Ai.begin(), Ai.end(), Aj.begin(), Aj.end(), std::back_inserter(Ak));
            Ai.swap(Ak);
        }
        assert (Ai.size() > 0); // cannot happen if this algorithm is working correctly
        if (Ai.size() > 1 && Bi->size() > 1) {
            bicliques.emplace(std::move(Ai), std::move(*Bi));
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief analyzeGraph
 ** ------------------------------------------------------------------------------------------------------------- */
inline static void analyzeGraph(const Vertex u, Graph & G, const Matrix & M, BicliqueSet & bicliques, const unsigned traversalLimit = 10) {

    VertexSet A;
    VertexSet B;

    std::vector<bool> visited(num_vertices(G), false);
    circular_buffer<Vertex> Q(64);
    const TypeId typeId = G[u]->getClassTypeId();

    Vertex v = u;
    visited[v] = true;
    for (;;) {
        bool independent = true;
        for (auto w : B) {
            if (M.get_edge(v, w)) {
                independent = false;
                break;
            }
        }
        if (independent) {
            B.push_back(v);
            if (B.size() < traversalLimit) {
                for (auto e : make_iterator_range(in_edges(v, G))) {
                    const auto w = source(e, G);
                    if (visited[w]) {
                        continue;
                    }
                    bool independent = true;
                    for (auto x : A) {
                        if (M.get_edge(w, x)) {
                            independent = false;
                            break;
                        }
                    }
                    visited[w] = true;
                    if (independent) {
                        A.push_back(w);
                        for (auto e : make_iterator_range(out_edges(w, G))) {
                            const auto x = target(e, G);
                            if (visited[x]) {
                                continue;
                            }
                            visited[x] = true;
                            if (G[x]->getClassTypeId() == typeId) {
                                if (LLVM_UNLIKELY(Q.full())) {
                                    Q.set_capacity(Q.capacity() + (Q.capacity() / 2));
                                }
                                Q.push_back(x);
                            }
                        }
                    }
                }
            }
        }
        if (Q.empty()) {
            break;
        }
        v = Q.front();
        Q.pop_front();
    }

    enumerateBicliques(A, B, G, bicliques);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief intersects
 ** ------------------------------------------------------------------------------------------------------------- */
template <class Type>
inline bool intersects(const Type & A, const Type & B) {
    auto first1 = A.begin(), last1 = A.end();
    auto first2 = B.begin(), last2 = B.end();
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
inline static void independentCliqueSets(BicliqueSet & bicliques, const unsigned minimum) {
    using IndependentSetGraph = adjacency_list<hash_setS, vecS, undirectedS, unsigned>;

    const auto l = bicliques.size();
    IndependentSetGraph I(l);

    // Initialize our weights and determine the constraints
    for (auto i = bicliques.begin(); i != bicliques.end(); ++i) {
        I[std::distance(bicliques.begin(), i)] = std::pow(std::get<0>(*i).size(), 2);
        for (auto j = i; ++j != bicliques.end(); ) {
            if (intersects(std::get<1>(*i), std::get<1>(*j))) {
                add_edge(std::distance(bicliques.begin(), i), std::distance(bicliques.begin(), j), I);
            }
        }
    }

    // Use the greedy algorithm to choose our independent set
    VertexSet selected;
    for (;;) {
        unsigned w = 0;
        Vertex u = 0;
        for (unsigned i = 0; i != l; ++i) {
            if (I[i] > w) {
                w = I[i];
                u = i;
            }
        }
        if (w < minimum) break;
        selected.push_back(u);
        I[u] = 0;
        for (auto v : make_iterator_range(adjacent_vertices(u, I))) {
            I[v] = 0;
        }
    }

    // Sort the selected list and then remove the unselected cliques
    std::sort(selected.begin(), selected.end(), std::greater<Vertex>());
    auto end = bicliques.end();
    for (const unsigned offset : selected) {
        end = bicliques.erase(bicliques.begin() + offset + 1, end) - 1;
    }
    bicliques.erase(bicliques.begin(), end);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief chooseInsertionPoint
 ** ------------------------------------------------------------------------------------------------------------- */
inline PabloBlock * FactorizeDFG::chooseInsertionScope(const NodeSet & users) {

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
void FactorizeDFG::findInsertionPoint(const NodeSet & operands, PabloBlock * const scope) {
    Statement * stmt = scope->back();
    scope->setInsertPoint(nullptr);
    assert (std::is_sorted(operands.begin(), operands.end()));
    while (stmt) {
        if (isa<If>(stmt)) {
            for (Assign * def : cast<If>(stmt)->getDefined()) {
                if (std::binary_search(operands.begin(), operands.end(), def)) {
                    scope->setInsertPoint(stmt);
                    return;
                }
            }
        } else if (isa<While>(stmt)) {
            for (Next * var : cast<While>(stmt)->getVariants()) {
                if (std::binary_search(operands.begin(), operands.end(), var)) {
                    scope->setInsertPoint(stmt);
                    return;
                }
            }
        } else if (std::binary_search(operands.begin(), operands.end(), stmt)) {
            scope->setInsertPoint(stmt);
            break;
        }
        stmt = stmt->getPrevNode();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief factor
 ** ------------------------------------------------------------------------------------------------------------- */
inline void FactorizeDFG::factor(PabloFunction & function) {

//    raw_os_ostream out(std::cerr);

//    out << "BEFORE:\n\n";
//    PabloPrinter::print(function, out);
//    out << "******************************************\n";
//    out.flush();

    for (;;) {

        Graph G;
        {
            Map M;
            // Let G be a DAG representing each associative statement and its inputs
            buildDependencyGraph(function.getEntryBlock(), G, M);
        }

        BicliqueSet S;
        {
            const Matrix M = transitiveClosureOf(G);
            for (const Vertex u : make_iterator_range(vertices(M))) {
                if ((in_degree(u, G) > 2) && (isa<And>(G[u]) || isa<Or>(G[u]) || isa<Xor>(G[u]))) {
                    analyzeGraph(u, G, M, S);
                }
            }
        }

        independentCliqueSets(S, 2);

        if (S.empty()) {
            break;
        }

        std::vector<PabloAST *> operands;
        std::vector<PabloAST *> users;

        for (const Biclique & B : S) {
            for (auto u : std::get<0>(B)) {
                operands.push_back(G[u]);
            }
            std::sort(operands.begin(), operands.end());

            for (auto u : std::get<1>(B)) {
                users.push_back(G[u]);
            }
            std::sort(users.begin(), users.end());

//            out << " -- factoring {";
//            for (PabloAST * operand : operands) {
//                out << ' ';
//                PabloPrinter::print(operand, out);
//            }
//            out << " } from { ";
//            for (PabloAST * user : users) {
//                out << ' ';
//                PabloPrinter::print(user, out);
//            }
//            out << " } -> ";

            const TypeId typeId = users.front()->getClassTypeId();
            assert(typeId == TypeId::And || typeId == TypeId::Or || typeId == TypeId::Xor);
            #ifndef NDEBUG
            for (PabloAST * user : users) {
                assert(user->getClassTypeId() == typeId);
            }
            #endif
            PabloBlock * const block = chooseInsertionScope(users);
            findInsertionPoint(operands, block);
            Variadic * factored = nullptr;
            if (typeId == TypeId::And) {
                factored = block->createAnd(operands.begin(), operands.end());
            } else if (typeId == TypeId::Or) {
                factored = block->createOr(operands.begin(), operands.end());
            } else { // if (isa<Xor>(var)) {
                factored = block->createXor(operands.begin(), operands.end());
            }

//            PabloPrinter::print(factored, out);
//            out << '\n';
//            out.flush();

            for (PabloAST * user : users) {
                for (PabloAST * op : operands) {
                    cast<Variadic>(user)->deleteOperand(op);
                }
                cast<Variadic>(user)->addOperand(factored);
            }
            operands.clear();
            users.clear();
        }
//        out << "-----------------------------------------------------------------\n";
    }

//    out << "AFTER:\n\n";
//    PabloPrinter::print(function, out);
//    out << "******************************************\n";

}


#endif

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief deMorgansExpansion
 *
 * Apply the De Morgans' law to any negated And or Or statement with the intent of further coalescing its operands
 * thereby allowing the Simplifier to check for tautologies and contradictions.
 ** ------------------------------------------------------------------------------------------------------------- */
inline void FactorizeDFG::deMorgansExpansion(Not * const var, PabloBlock * const block) {
    PabloAST * const negatedVar = var->getOperand(0);
    if (isa<And>(negatedVar) || isa<Or>(negatedVar)) {
        const TypeId desiredTypeId = isa<And>(negatedVar) ? TypeId::Or : TypeId::And;
        bool canApplyDeMorgans = true;
        for (PabloAST * user : var->users()) {
            if (desiredTypeId != user->getClassTypeId()) {
                canApplyDeMorgans = false;
                break;
            }
        }
        if (canApplyDeMorgans) {
            const unsigned operands = cast<Variadic>(negatedVar)->getNumOperands();
            PabloAST * negations[operands];
            block->setInsertPoint(var);
            for (unsigned i = 0; i != operands; ++i) {
                negations[i] = block->createNot(cast<Variadic>(negatedVar)->getOperand(i));
            }
            const unsigned users = var->getNumUses();
            PabloAST * user[users];
            std::copy(var->user_begin(), var->user_end(), user);
            for (unsigned i = 0; i != users; ++i) {
                cast<Variadic>(user[i])->deleteOperand(var);
                for (unsigned j = 0; j != operands; ++j) {
                    cast<Variadic>(user[i])->addOperand(negations[j]);
                }
            }
            var->eraseFromParent(true);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief deMorgansExpansion
 ** ------------------------------------------------------------------------------------------------------------- */
void FactorizeDFG::deMorgansExpansion(PabloBlock * const block) {
    Statement * stmt = block->front();
    while (stmt) {
        if (isa<If>(stmt) || isa<While>(stmt)) {
            deMorgansExpansion(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody());
        } else if (isa<Not>(stmt)) {
            deMorgansExpansion(cast<Not>(stmt), block);
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
        } else if (stmt->getNumOperands() > 2 && (isa<And>(stmt) || isa<Or>(stmt) || isa<Xor>(stmt))) {
            ++mNumOfVariadics;
        }
        stmt = stmt->getNextNode();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enumerateScopeDepth
 ** ------------------------------------------------------------------------------------------------------------- */
inline void FactorizeDFG::enumerateScopeDepth(const PabloFunction & function) {
    mScopeDepth.emplace(nullptr, 0);
    mScopeDepth.emplace(function.getEntryBlock(), 1);
    enumerateScopeDepth(function.getEntryBlock(), 2);
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
 * @brief ensureLegality
 *
 * We don't want to run the simplifier after this as it might undo some of the ordering work we've done. Instead
 * just do the minimum possible to ensure that each variadic is legal prior to compilation.
 ** ------------------------------------------------------------------------------------------------------------- */
void FactorizeDFG::ensureLegality(PabloBlock * const block) {
    Statement * stmt = block->front();
    while (stmt) {
        if (isa<If>(stmt) || isa<While>(stmt)) {
            ensureLegality(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody());
        } else if (isa<And>(stmt) || isa<Or>(stmt) || isa<Xor>(stmt)) {
            assert (stmt->getNumOperands() <= 2);
            if (LLVM_UNLIKELY(stmt->getNumOperands() < 2)) {
                if (LLVM_UNLIKELY(stmt->getNumOperands() == 0)) {
                    stmt = stmt->eraseFromParent(true);
                } else {
                    stmt = stmt->replaceWith(stmt->getOperand(0), true, true);
                }
                continue;
            }
        }
        stmt = stmt->getNextNode();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief transform
 ** ------------------------------------------------------------------------------------------------------------- */
void FactorizeDFG::transform(PabloFunction & function) {
    FactorizeDFG ldfg;
//    ldfg.deMorgansExpansion(function.getEntryBlock());
//    #ifndef NDEBUG
//    PabloVerifier::verify(function, "post-demorgans-expansion");
//    #endif
    ldfg.enumerateScopeDepth(function);
    ldfg.factor(function);
    #ifndef NDEBUG
    PabloVerifier::verify(function, "post-factoring");
    #endif
    ldfg.lower(function);
    #ifndef NDEBUG
    PabloVerifier::verify(function, "post-lowering");
    #endif
    ldfg.ensureLegality(function.getEntryBlock());
}

}
