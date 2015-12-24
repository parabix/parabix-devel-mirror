#include "schedulingprepass.h"
#include <pablo/codegenstate.h>
#include <boost/circular_buffer.hpp>
#include <boost/container/flat_set.hpp>
#include <pablo/analysis/pabloverifier.hpp>

#include <pablo/printer_pablos.h>
#include <iostream>

using namespace boost;
using namespace boost::container;

namespace pablo {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief resolveNestedUsages
 ** ------------------------------------------------------------------------------------------------------------- */
void SchedulingPrePass::resolveNestedUsages(Statement * const root, Statement * const stmt, Graph & G, Map & M, PabloBlock * const block) {
    for (PabloAST * use : stmt->users()) {
        if (LLVM_LIKELY(isa<Statement>(use))) {
            const PabloBlock * scope = cast<Statement>(use)->getParent();
            if (scope != block) {
                for (PabloBlock * prior = scope->getParent(); prior; scope = prior, prior = prior->getParent()) {
                    if (prior == block) {
                        auto s = mResolvedScopes.find(scope);
                        assert (s != mResolvedScopes.end());
                        assert (s->second);
                        auto v = M.find(s->second);
                        assert (v != M.end());
                        auto u = M.find(root);
                        assert (u != M.end());
                        if (u->second != v->second) {
                            add_edge(u->second, v->second, G);
                        }
                        break;
                    }
                }
            }
        }
    }
}

///** ------------------------------------------------------------------------------------------------------------- *
// * @brief printGraph
// ** ------------------------------------------------------------------------------------------------------------- */
//template <typename Graph>
//static void printGraph(const Graph & G, const std::vector<unsigned> & ordering, const std::string name, raw_ostream & out) {
//    out << "*******************************************\n";
//    out << "digraph " << name << " {\n";
//    for (auto u : make_iterator_range(vertices(G))) {
//        out << "v" << u << " [label=\"" << ordering[u] << " : ";
//        PabloPrinter::print(G[u], out);
//        out << "\"];\n";
//    }
//    for (auto e : make_iterator_range(edges(G))) {
//        out << "v" << source(e, G) << " -> v" << target(e, G);
//        out << ";\n";
//    }

//    out << "}\n\n";
//    out.flush();
//}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief schedule
 ** ------------------------------------------------------------------------------------------------------------- */
void SchedulingPrePass::schedule(PabloBlock * const block) {
    Graph G;
    Map M;

    // Construct a use-def graph G representing the current scope block
    for (Statement * stmt : *block) {
        if (isa<If>(stmt) || isa<While>(stmt)) {
            PabloBlock * body = isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody();
            mResolvedScopes.emplace(body, stmt);
            schedule(body);
        }
        const auto u = add_vertex(stmt, G);
        M.insert(std::make_pair(stmt, u));
        for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
            const PabloAST * const op = stmt->getOperand(i);
            if (LLVM_LIKELY(isa<Statement>(op))) {
                const PabloBlock * scope = cast<Statement>(op)->getParent();
                // Was this instruction computed by this block?
                if (scope == block) {
                    auto v = M.find(op);
                    assert (v != M.end());
                    assert (v->second != u);
                    add_edge(v->second, u, G);
                } else if (isa<Assign>(op) || isa<Next>(op)) {
                    // if this statement isn't an Assign or Next node, it cannot come from a nested scope
                    // unless the function is invalid.
                    for (PabloBlock * prior = scope->getParent(); prior; scope = prior, prior = prior->getParent()) {
                        // Was this instruction computed by a nested block?
                        if (prior == block) {
                            auto s = mResolvedScopes.find(scope);
                            assert (s != mResolvedScopes.end());
                            assert (s->second);
                            auto v = M.find(s->second);
                            assert (v != M.end());
                            if (v->second != u) {
                                add_edge(v->second, u, G);
                            }
                            break;
                        }
                    }
                }
            }
        }
    }
    // Do a second pass to ensure that we've accounted for any nested usage of a statement
    for (Statement * stmt : *block) {
        if (LLVM_UNLIKELY(isa<If>(stmt))) {
            for (Assign * def : cast<If>(stmt)->getDefined()) {
                resolveNestedUsages(stmt, def, G, M, block);
            }
        } else if (LLVM_UNLIKELY(isa<While>(stmt))) {
            for (Next * var : cast<While>(stmt)->getVariants()) {
                resolveNestedUsages(stmt, var, G, M, block);
            }
        } else {
            resolveNestedUsages(stmt, stmt, G, M, block);
        }
    }

    // Determine the bottom-up ordering of G
    std::vector<unsigned> ordering(num_vertices(G));
    circular_buffer<Vertex> Q(num_vertices(G));
    for (const Vertex u : make_iterator_range(vertices(G))) {
        if (out_degree(u, G) == 0) {
            ordering[u] = 1;
            Q.push_back(u);
        }
    }

    while (!Q.empty()) {
        const Vertex u = Q.front();
        Q.pop_front();
        assert (ordering[u] > 0);
        for (const auto ei : make_iterator_range(in_edges(u, G))) {
            const Vertex v = source(ei, G);
            if (ordering[v] == 0) {
                unsigned weight = 0;
                bool ready = true;
                for (const auto ej : make_iterator_range(out_edges(v, G))) {
                    const Vertex w = target(ej, G);
                    const unsigned t = ordering[w];
                    if (t == 0) {
                        ready = false;
                        break;
                    }
                    weight = std::max(weight, t);
                }
                if (ready) {
                    assert (weight < std::numeric_limits<unsigned>::max());
                    assert (weight > 0);
                    ordering[v] = (weight + 1);
                    Q.push_back(v);
                }
            }
        }
    }

    // Compute the initial ready set
    ReadySet readySet;
    for (const Vertex u : make_iterator_range(vertices(G))) {
        if (in_degree(u, G) == 0) {
            readySet.emplace_back(ordering[u], u);
        }
    }

    struct {
        bool operator()(const ReadyPair a, const ReadyPair b) {
            return std::get<0>(a) > std::get<0>(b);
        }
    } readyComparator;

    std::sort(readySet.begin(), readySet.end(), readyComparator);

    block->setInsertPoint(nullptr);


    // Rewrite the AST using the bottom-up ordering
    while (!readySet.empty()) {

        // Scan through the ready set to determine which one 'kills' the greatest number of inputs
        // NOTE: the readySet is kept in sorted "distance to sink" order; thus those closest to a
        // sink will be evaluated first.
        double best = 0;
        auto chosen = readySet.begin();
        for (auto ri = readySet.begin(); ri != readySet.end(); ++ri) {
            const Vertex u = std::get<1>(*ri);
            const PabloAST * const expr = G[u];
            if (expr && (isa<Assign>(expr) || isa<Next>(expr))) {
                chosen = ri;
                break;
            }
            double progress = 0;
            for (auto ei : make_iterator_range(in_edges(u, G))) {
                const auto v = source(ei, G);
                const auto totalUsesOfIthOperand = out_degree(v, G);
                if (LLVM_UNLIKELY(totalUsesOfIthOperand == 0)) {
                    progress += 1.0;
                } else {
                    unsigned unscheduledUsesOfIthOperand = 0;
                    for (auto ej : make_iterator_range(out_edges(v, G))) {
                        if (ordering[target(ej, G)]) { // if this edge leads to an unscheduled statement
                            ++unscheduledUsesOfIthOperand;
                        }
                    }
                    progress += std::pow((double)(totalUsesOfIthOperand - unscheduledUsesOfIthOperand + 1) / (double)(totalUsesOfIthOperand), 2);
                }
            }
            if (progress > best) {
                chosen = ri;
                best = progress;
            }
        }

        Vertex u; unsigned weight;
        std::tie(weight, u) = *chosen;
        readySet.erase(chosen);

        assert ("Error: SchedulingPrePass is attempting to reschedule a statement!" && (weight > 0));

        // insert the statement then mark it as written ...
        block->insert(cast<Statement>(G[u]));
        ordering[u] = 0;
        // Now check whether any new instructions are ready
        for (auto ei : make_iterator_range(out_edges(u, G))) {
            bool ready = true;
            const auto v = target(ei, G);
            for (auto ej : make_iterator_range(in_edges(v, G))) {
                if (ordering[source(ej, G)]) {
                    ready = false;
                    break;
                }
            }
            if (ready) {
                auto entry = std::make_pair(ordering[v], v);
                auto position = std::lower_bound(readySet.begin(), readySet.end(), entry, readyComparator);
                readySet.insert(position, entry);
                assert (std::is_sorted(readySet.cbegin(), readySet.cend(), readyComparator));
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief schedule
 ** ------------------------------------------------------------------------------------------------------------- */
void SchedulingPrePass::schedule(PabloFunction & function) {
    mResolvedScopes.emplace(function.getEntryBlock(), nullptr);
    schedule(function.getEntryBlock());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief optimize
 ** ------------------------------------------------------------------------------------------------------------- */
bool SchedulingPrePass::optimize(PabloFunction & function) {
    SchedulingPrePass pp;
    pp.schedule(function);
    #ifndef NDEBUG
    PabloVerifier::verify(function, "post-scheduling-prepass");
    #endif
    return true;

}


}
