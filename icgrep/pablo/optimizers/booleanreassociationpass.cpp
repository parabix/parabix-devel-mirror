#include "booleanreassociationpass.h"
#include <pablo/printer_pablos.h>
#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <iostream>
#include <boost/circular_buffer.hpp>
#include <queue>
#include <pablo/builder.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>

using namespace boost;
using namespace boost::container;

namespace pablo {

bool BooleanReassociationPass::optimize(PabloFunction & function) {
    BooleanReassociationPass brp;
    brp.scan(function);
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief scan
 ** ------------------------------------------------------------------------------------------------------------- */
void BooleanReassociationPass::scan(PabloFunction & function) {
    Terminals terminals;
    for (unsigned i = 0; i != function.getNumOfResults(); ++i) {
        terminals.push_back(function.getResult(i));
    }
    scan(function.getEntryBlock(), std::move(terminals));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief is_power_of_2
 * @param n an integer
 ** ------------------------------------------------------------------------------------------------------------- */
static inline bool is_power_of_2(const size_t n) {
    return ((n & (n - 1)) == 0);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief log2_plus_one
 ** ------------------------------------------------------------------------------------------------------------- */
static inline size_t ceil_log2(const size_t n) {
    return std::log2<size_t>(n) + (is_power_of_2(n) ? 0 : 1);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isACD
 ** ------------------------------------------------------------------------------------------------------------- */
static inline bool isaBooleanOperation(const PabloAST * const expr) {
    switch (expr->getClassTypeId()) {
        case PabloAST::ClassTypeId::And:
        case PabloAST::ClassTypeId::Or:
        case PabloAST::ClassTypeId::Xor:
            return true;
        default:
            return false;
    }
}

using Graph = adjacency_list<hash_setS, vecS, bidirectionalS, PabloAST *>;
using Vertex = Graph::vertex_descriptor;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isCutNecessary
 ** ------------------------------------------------------------------------------------------------------------- */
static inline bool isCutNecessary(const Vertex u, const Vertex v, const Graph & G, const std::vector<unsigned> & component) {
    if (LLVM_UNLIKELY(component[u] != component[v])) {
        return true;
    } else if (LLVM_UNLIKELY(out_degree(v, G) && in_degree(u, G) && G[u]->getClassTypeId() != G[v]->getClassTypeId())) {
        return true;
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isCutNecessary
 ** ------------------------------------------------------------------------------------------------------------- */
static bool print(const Graph & G, const std::vector<unsigned> & component, raw_os_ostream & out) {
    bool hasError = false;
    out << "digraph G {\n";
    unsigned i = 0;
    for (auto u : make_iterator_range(vertices(G))) {
        out << "u" << u << " [label=\"";
        out << i++ << " : ";
        PabloPrinter::print(G[u], out);
        out << " (" << component[u] << ')';
        out << "\"";
        if (isaBooleanOperation(G[u]) && (in_degree(u, G) == 1 || in_degree(u, G) > 2)) {
            out << " color=red";
            hasError = true;
        }
        out << "];\n";
    }
    for (auto e : make_iterator_range(edges(G))) {
        Vertex u = source(e, G);
        Vertex v = target(e, G);
        out << "u" << u << " -> u" << v;
        if (isCutNecessary(u, v, G, component)) {
            out << " [color=red]";
            hasError = true;
        }
        out << ";\n";
    }
    out << "}\n";
    out.flush();
    return hasError;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief scan
 ** ------------------------------------------------------------------------------------------------------------- */
void BooleanReassociationPass::scan(PabloBlock & block, Terminals && terminals) {

    using Map = std::unordered_map<PabloAST *, Vertex>;
    using VertexQueue = std::queue<Vertex>;
    using EdgeQueue = std::queue<std::pair<Vertex, Vertex>>;

    for (Statement * stmt : block) {
        if (isa<If>(stmt)) {
            const auto & defs = cast<const If>(stmt)->getDefined();
            Terminals terminals(defs.begin(), defs.end());
            scan(cast<If>(stmt)->getBody(), std::move(terminals));
        } else if (isa<While>(stmt)) {
            const auto & vars = cast<const While>(stmt)->getVariants();
            Terminals terminals(vars.begin(), vars.end());
            scan(cast<While>(stmt)->getBody(), std::move(terminals));
        }
    }

    // And, Or and Xor instructions are all associative, commutative and distributive operations. Thus we can
    // safely rearrange expressions such as "((((a ∨ b) ∨ c) ∨ d) ∨ e) ∨ f" into "((a ∨ b) ∨ (c ∨ d)) ∨ (e ∨ f)".

    raw_os_ostream out(std::cerr);

    out << "=================================================\n";

    PabloBuilder builder(block);

    for (;;) {

        Graph G;
        Map M;

        // Generate a graph depicting the relationships between the terminals. If the original terminals
        // cannot be optimized with this algorithm bypass them in favour of their operands.

        VertexQueue Q;

        for (Statement * const term : terminals) {
            if (isaBooleanOperation(term)) {
                if (LLVM_LIKELY(M.count(term) == 0)) {
                    const auto v = add_vertex(term, G);
                    M.insert(std::make_pair(term, v));
                    Q.push(v);
                }
            } else {
                for (unsigned i = 0; i != term->getNumOperands(); ++i) {
                    PabloAST * const op = term->getOperand(i);
                    if (LLVM_LIKELY(isa<Statement>(op) && M.count(op) == 0)) {
                        const auto v = add_vertex(op, G);
                        M.insert(std::make_pair(op, v));
                        Q.push(v);
                    }
                }
            }

        }

        for (;;) {
            const Vertex u = Q.front(); Q.pop();
            if (isaBooleanOperation(G[u])) {
                // Scan through the use-def chains to locate any chains of rearrangable expressions and their inputs
                Statement * stmt = cast<Statement>(G[u]);
                for (unsigned i = 0; i != 2; ++i) {
                    PabloAST * op = stmt->getOperand(i);
                    auto f = M.find(op);
                    if (f == M.end()) {
                        const auto v = add_vertex(op, G);
                        f = M.insert(std::make_pair(op, v)).first;
                        if (op->getClassTypeId() == stmt->getClassTypeId() && cast<Statement>(op)->getParent() == &block) {
                            Q.push(v);
                        }
                    }
                    add_edge(f->second, u, G);
                }
            }
            if (Q.empty()) {
                break;
            }
        }

        // Generate a topological ordering for G; if one of our terminals happens to also be a partial computation of
        // another terminal, we need to make sure we compute it.
        std::vector<unsigned> ordering;
        ordering.reserve(num_vertices(G));
        topological_sort(G, std::back_inserter(ordering));
        std::vector<unsigned> component(num_vertices(G));

        for (;;) {

            // Mark which computation component these vertices are in based on their topological (occurence) order.
            unsigned count = 0;
            for (auto u : ordering) {
                unsigned id = 0;
                // If this one of our original terminals or a sink in G, it is the root of a new component.
                if (out_degree(u, G) == 0) {
                    id = ++count;
                } else {
                    for (auto e : make_iterator_range(out_edges(u, G))) {
                        id = std::max(id, component[target(e, G)]);
                    }
                }
                assert (id && "Topological ordering failed!");
                component[u] = id;
            }

            if (count > 1) {
                // Cut the graph wherever a computation crosses a component
                EdgeQueue Q;
                graph_traits<Graph>::edge_iterator ei, ei_end;

                for (std::tie(ei, ei_end) = edges(G); ei != ei_end; ) {
                    const Graph::edge_descriptor e = *ei++;
                    const Vertex u = source(e, G);
                    const Vertex v = target(e, G);
                    if (LLVM_UNLIKELY(isCutNecessary(u, v, G, component))) {
                        Q.push(std::make_pair(u, v));
                        remove_edge(u, v, G);
                    }
                }

                // If no edges cross a component, we're done.
                if (Q.empty()) {
                    break; // outer for loop
                }

                for (;;) {

                    Vertex u, v;
                    std::tie(u, v) = Q.front(); Q.pop();

                    // The vertex belonging to a component with a greater number must come "earlier"
                    // in the program. By replicating it, this ensures it's computed as an output of
                    // one component and used as an input of another.

                    if (component[u] < component[v]) {
                        std::swap(u, v);
                    }

                    // Replicate u and fix the ordering and component vectors to reflect the change in G.
                    Vertex w = add_vertex(G[u], G);
                    ordering.insert(std::find(ordering.begin(), ordering.end(), u), w);
                    assert (component.size() == w);
                    component.push_back(component[v]);
                    add_edge(w, v, G);

                    // However, after we do so, we need to make sure the original source vertex will be a
                    // sink in G unless it is also an input variable (in which case we'd simply end up with
                    // extraneous isolated vertex. Otherwise, we need to make further cuts and replications.

                    if (in_degree(u, G) != 0) {
                        for (auto e : make_iterator_range(out_edges(u, G))) {
                            Q.push(std::make_pair(source(e, G), target(e, G)));
                        }
                        clear_out_edges(u, G);
                    }

                    if (Q.empty()) {
                        break;
                    }

                }
                continue; // outer for loop
            }
            break; // outer for loop
        }

        if (print(G, component, out)) {
            PabloPrinter::print(block.statements(), out);
            out.flush();
            throw std::runtime_error("Illegal graph generated!");
        }

        // Scan through the graph in reverse order so that we find all subexpressions first
        for (auto ui = ordering.begin(); ui != ordering.end(); ++ui) {
            const Vertex u = *ui;
            if (out_degree(u, G) == 0 && in_degree(u, G) != 0) {

                out << " -- checking component " << component[u] << " : ";
                PabloPrinter::print(G[u], out);
                out << "\n";
                out.flush();

                // While we're collecting our variable set V, keep track of the maximum path length L.
                // If L == ceil(log2(|V|)), then this portion of the AST is already optimal.

                flat_map<Vertex, unsigned> L;
                flat_set<PabloAST *> V;
                VertexQueue Q;

                Vertex v = u;
                unsigned maxPathLength = 0;
                L.emplace(v, 0);
                for (;;) {
                    assert (isa<Statement>(G[v]) ? cast<Statement>(G[v])->getParent() != nullptr : true);
                    if (in_degree(v, G) == 0) {
                        V.insert(G[v]);
                    } else {
                        const auto l = L[v] + 1;
                        maxPathLength = std::max(maxPathLength, l);
                        for (auto e : make_iterator_range(in_edges(v, G))) {
                            const Vertex w = source(e, G);
                            auto f = L.find(w);
                            if (LLVM_LIKELY(f == L.end())) {
                                L.emplace(w, l);
                            } else {
                                f->second = std::max(f->second, l);
                            }
                            Q.push(w);
                        }
                    }
                    if (Q.empty()) {
                        break;
                    }
                    v = Q.front();
                    Q.pop();
                }

                // Should we optimize this portion of the AST?
                if (maxPathLength > ceil_log2(V.size())) {

                    out << " -- rewriting component " << component[u] << " : |P|=" << maxPathLength << ", |V|=" << V.size() << " (" << ceil_log2(V.size()) << ")\n";
                    out.flush();

                    circular_buffer<PabloAST *> Q(V.size());
                    for (PabloAST * var : V) {
                        Q.push_back(var);
                    }
                    Statement * stmt = cast<Statement>(G[u]);

                    block.setInsertPoint(stmt->getPrevNode());
                    if (isa<And>(stmt)) {
                        while (Q.size() > 1) {
                            PabloAST * e1 = Q.front(); Q.pop_front();
                            PabloAST * e2 = Q.front(); Q.pop_front();
                            Q.push_back(builder.createAnd(e1, e2));
                        }
                    } else if (isa<Or>(stmt)) {
                        while (Q.size() > 1) {
                            PabloAST * e1 = Q.front(); Q.pop_front();
                            PabloAST * e2 = Q.front(); Q.pop_front();
                            Q.push_back(builder.createOr(e1, e2));
                        }
                    } else { // if (isa<Xor>(stmt)) {
                        while (Q.size() > 1) {
                            PabloAST * e1 = Q.front(); Q.pop_front();
                            PabloAST * e2 = Q.front(); Q.pop_front();
                            Q.push_back(builder.createXor(e1, e2));
                        }
                    }
                    stmt->replaceWith(Q.front(), true, true);
                }

                for (auto uj = ui; ++uj != ordering.end(); ) {
                    assert (isa<Statement>(G[*uj]) ? cast<Statement>(G[*uj])->getParent() != nullptr : true);
                }

            }
        }

        // Determine the source variables of the next "layer" of the AST
        flat_set<Statement *> nextSet;
        for (auto u : ordering) {
            if (in_degree(u, G) == 0) {
                PabloAST * const var = G[u];
                if (LLVM_LIKELY(isa<Statement>(var) && cast<Statement>(var)->getParent() == &block)) {
                    nextSet.insert(cast<Statement>(var));
                }
            } else if (out_degree(u, G) == 0) { // an input may also be the output of some subgraph of G. We don't need to reevaluate it.
                PabloAST * const var = G[u];
                if (LLVM_LIKELY(isa<Statement>(var) && cast<Statement>(var)->getParent() == &block)) {
                    nextSet.erase(cast<Statement>(var));
                }
            }
        }

        if (nextSet.empty()) {
            break;
        }

        terminals.assign(nextSet.begin(), nextSet.end());

        out << "-------------------------------------------------\n";
    }
}

BooleanReassociationPass::BooleanReassociationPass()
{

}


}
