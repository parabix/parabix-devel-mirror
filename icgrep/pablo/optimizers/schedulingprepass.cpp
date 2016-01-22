#include "schedulingprepass.h"
#include <pablo/codegenstate.h>
#include <boost/circular_buffer.hpp>
#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <pablo/analysis/pabloverifier.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <unordered_map>

#include <pablo/printer_pablos.h>
#include <iostream>

using namespace boost;
using namespace boost::container;

namespace pablo {

using DependencyGraph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::bidirectionalS, std::vector<PabloAST *>>;
using Vertex = DependencyGraph::vertex_descriptor;
using Map = std::unordered_map<const PabloAST *, Vertex>;
using ReadyPair = std::pair<unsigned, Vertex>;
using ReadySet = std::vector<ReadyPair>;

using weight_t = unsigned;
using TypeId = PabloAST::ClassTypeId;
using LiveSet = flat_set<Vertex>;

void schedule(PabloBlock * const block);

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printGraph
 ** ------------------------------------------------------------------------------------------------------------- */
template <typename DependencyGraph>
static void printGraph(const DependencyGraph & G, const std::vector<unsigned> & ordering, const std::string name, raw_ostream & out) {
    out << "*******************************************\n";
    out << "digraph " << name << " {\n";
    for (auto u : make_iterator_range(vertices(G))) {
        if (G[u].empty()) {
            continue;
        }
        out << "v" << u << " [label=\"" << ordering[u] << " : ";
        bool newLine = false;
        for (PabloAST * expr : G[u]) {
            if (newLine) {
                out << '\n';
            }
            newLine = true;
            if (isa<Statement>(expr)) {
                PabloPrinter::print(cast<Statement>(expr), out);
            } else {
                PabloPrinter::print(expr, out);
            }
        }
        out << "\"];\n";
    }
    for (auto e : make_iterator_range(edges(G))) {
        out << "v" << source(e, G) << " -> v" << target(e, G);
        out << ";\n";
    }

    out << "}\n\n";
    out.flush();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief resolveNestedUsages
 ** ------------------------------------------------------------------------------------------------------------- */
static void resolveNestedUsages(Statement * const root, Statement * const stmt, DependencyGraph & G, Map & M, PabloBlock * const block) {
    for (PabloAST * use : stmt->users()) {
        if (LLVM_LIKELY(isa<Statement>(use))) {
            const PabloBlock * scope = cast<Statement>(use)->getParent();
            if (scope != block) {
                for (PabloBlock * prior = scope->getParent(); prior; scope = prior, prior = prior->getParent()) {
                    if (prior == block) {
                        assert (scope->getBranch());
                        auto v = M.find(scope->getBranch());
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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief resolveNestedUsages
 ** ------------------------------------------------------------------------------------------------------------- */
static void computeDependencyGraph(DependencyGraph & G, PabloBlock * const block) {
    Map M;
    // Construct a use-def graph G representing the current scope block
    for (Statement * stmt : *block) {
        if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
            schedule(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody());
        }
        const auto u = add_vertex({stmt}, G);
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
                            assert (scope->getBranch());
                            auto v = M.find(scope->getBranch());
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
    // Do a second pass to ensure that we've accounted for any nested usage of an If or While statement
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
    // Contract the graph
    for (;;) {
        bool done = true;
        for (const Vertex u : make_iterator_range(vertices(G))) {
            if (out_degree(u, G) == 1) {
                const Vertex v = target(*(out_edges(u, G).first), G);
                G[v].insert(G[v].begin(), G[u].begin(), G[u].end());
                for (auto e : make_iterator_range(in_edges(u, G))) {
                    add_edge(source(e, G), v, G);
                }
                G[u].clear();
                clear_vertex(u, G);
                done = false;
            }
        }
        if (done) {
            break;
        }
    }


}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeGraphOrdering
 ** ------------------------------------------------------------------------------------------------------------- */
std::vector<weight_t> computeGraphOrdering(const DependencyGraph & G) {
    // Determine the bottom-up ordering of G
    std::vector<weight_t> ordering(num_vertices(G));
    circular_buffer<Vertex> Q(num_vertices(G));
    for (const Vertex u : make_iterator_range(vertices(G))) {
        if (out_degree(u, G) == 0 && G[u].size() > 0) {
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
                weight_t weight = 0;
                bool ready = true;
                for (const auto ej : make_iterator_range(out_edges(v, G))) {
                    const Vertex w = target(ej, G);
                    const weight_t t = ordering[w];
                    if (t == 0) {
                        ready = false;
                        break;
                    }
                    weight = std::max(weight, t);
                }
                if (ready) {
                    assert (weight < std::numeric_limits<unsigned>::max());
                    assert (weight > 0);
                    ordering[v] = weight + 1;
                    Q.push_back(v);
                }
            }
        }
    }
    return ordering;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief schedule
 ** ------------------------------------------------------------------------------------------------------------- */
void schedule(PabloBlock * const block) {
    DependencyGraph G;
    computeDependencyGraph(G, block);
    std::vector<weight_t> ordering = computeGraphOrdering(G);

//    raw_os_ostream out(std::cerr);
//    printGraph(G, ordering, "G", out);

    // Compute the initial ready set
    ReadySet readySet;
    for (const Vertex u : make_iterator_range(vertices(G))) {
        if (in_degree(u, G) == 0 && G[u].size() > 0) {
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

    // Rewrite the AST
    while (!readySet.empty()) {
        DependencyGraph::degree_size_type killed = 0;
        auto chosen = readySet.begin();
        for (auto ri = readySet.begin(); ri != readySet.end(); ++ri) {
            DependencyGraph::degree_size_type kills = 0;
            for (auto e : make_iterator_range(in_edges(ri->first, G))) {
                if (out_degree(source(e, G), G) == 1) {
                    ++kills;
                }
            }
            if (kills > killed) {
                chosen = ri;
                killed = kills;
            }
        }

        Vertex u; unsigned weight;
        std::tie(weight, u) = *chosen;
        readySet.erase(chosen);
        clear_in_edges(u, G);

        assert ("Error: SchedulingPrePass is attempting to reschedule a statement!" && (weight > 0));

        // insert the statement then mark it as written ...
        for (PabloAST * expr : G[u]) {
            block->insert(cast<Statement>(expr));
        }

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
void schedule(PabloFunction & function) {
    schedule(function.getEntryBlock());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief optimize
 ** ------------------------------------------------------------------------------------------------------------- */
bool SchedulingPrePass::optimize(PabloFunction & function) {
    schedule(function);
    #ifndef NDEBUG
    PabloVerifier::verify(function, "post-scheduling-prepass");
    #endif
    return true;

}


}
