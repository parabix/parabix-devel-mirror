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
    return std::log2<size_t>(n) + (is_power_of_2(n) ? 1 : 0);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief scan
 ** ------------------------------------------------------------------------------------------------------------- */
void BooleanReassociationPass::scan(PabloBlock & block, Terminals && terminals) {

    using Graph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::bidirectionalS, PabloAST *>;
    using Vertex = Graph::vertex_descriptor;
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

    // PabloBuilder builder(block);

    out << "=================================================\n";

    for (;;) {

        Graph G;
        Map M;

        for (Statement * const term : terminals) {
            M.insert(std::make_pair(term, add_vertex(term, G)));
        }

        // Generate a graph depicting the relationships between the terminals
        for (Statement * const term : terminals) {
            const Vertex u = M.find(term)->second;
            for (unsigned i = 0; i != term->getNumOperands(); ++i) {
                PabloAST * const var = term->getOperand(i);
                if (LLVM_UNLIKELY(isa<Integer>(var) || isa<String>(var))) {
                    continue;
                }
                auto f = M.find(var);
                if (f == M.end()) {
                    Vertex v = add_vertex(var, G);
                    f = M.insert(std::make_pair(var, v)).first;
                    if ((isa<And>(var) || isa<Or>(var) || isa<Xor>(var))) {
                        VertexQueue Q;
                        // Scan through the use-def chains to locate any chains of rearrangable expressions and their inputs
                        for (Statement * stmt = cast<Statement>(var); ;) {
                            for (unsigned i = 0; i != 2; ++i) {
                                PabloAST * op = stmt->getOperand(i);
                                auto f = M.find(op);
                                if (f == M.end()) {
                                    const auto w = add_vertex(op, G);
                                    f = M.insert(std::make_pair(op, w)).first;
                                    if (op->getClassTypeId() == stmt->getClassTypeId() && cast<Statement>(op)->getParent() == &block) {
                                        Q.push(w);
                                    }
                                }
                                add_edge(f->second, v, G);
                            }
                            if (Q.empty()) break;
                            v = Q.front(); Q.pop();
                            stmt = cast<Statement>(G[v]);
                        }
                    }
                    add_edge(f->second, u, G);
                }
            }
        }

        // Generate a topological ordering for G; if one of our terminals happens to also be a partial computation of
        // another terminal, we need to make sure we compute it.
        std::vector<unsigned> ordering;
        ordering.reserve(num_vertices(G));
        topological_sort(G, std::back_inserter(ordering));
        std::vector<unsigned> component(num_vertices(G));

        // unsigned i = 0;

        for (;;) {

            // Mark which computation component these vertices are in based on their topological (occurence) order.
            unsigned count = 0;
            for (auto u : ordering) {
                unsigned id = 0;
                // If this one of our original terminals or a sink in G, it is the root of a new component.
                if (u < terminals.size() || out_degree(u, G) == 0) {
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

//                if (i++ == 0) {

//                    out << "digraph G_" << i << " {\n";
//                    unsigned i = 0;
//                    for (auto u : ordering) {
//                        out << "u" << u << " [label=\"";
//                        out << i++ << " : ";
//                        PabloPrinter::print(G[u], out);
//                        out << " (" << component[u] << ')';
//                        out << "\"];\n";
//                    }
//                    for (auto e : make_iterator_range(edges(G))) {
//                        out << "u" << source(e, G) << " -> u" << target(e, G) << ";\n";
//                    }
//                    out << "}\n";
//                    out.flush();

//                    out << "*************************************************\n";

//                }

                // Cut the graph wherever a computation crosses a component
                EdgeQueue Q;
                graph_traits<Graph>::edge_iterator ei, ei_end;

                for (std::tie(ei, ei_end) = edges(G); ei != ei_end; ) {
                    const Graph::edge_descriptor e = *ei++;
                    const Vertex u = source(e, G);
                    const Vertex v = target(e, G);
                    if (LLVM_UNLIKELY(component[u] != component[v])) {
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

//                    out << " -- replicating ";
//                    PabloPrinter::print(G[u], out);
//                    out << " for component " << component[v] << '\n';
//                    out.flush();

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

//                out << "*************************************************\n";

                continue; // outer for loop
            }
            break; // outer for loop
        }

        out << "digraph X {\n";
        unsigned i = 0;
        for (auto u : ordering) {
            out << "u" << u << " [label=\"";
            out << i++ << " : ";
            PabloPrinter::print(G[u], out);
            out << " (" << component[u] << ')';
            out << "\"];\n";
        }
        for (auto e : make_iterator_range(edges(G))) {
            out << "u" << source(e, G) << " -> u" << target(e, G) << ";\n";
        }
        out << "}\n";
        out.flush();

        // Scan through the graph in reverse order so that we find all subexpressions first
        for (auto ui = ordering.rbegin(), ui_end = ordering.rend(); ui != ui_end; ++ui) {
            const Vertex u = *ui;
            if (out_degree(u, G) == 0 && in_degree(u, G) != 0) {

                PabloAST * expr = G[u];


                std::array<Vertex, 3> root;
                unsigned count = 0;

                if (isa<And>(expr) || isa<Or>(expr) || isa<Xor>(expr)) {
                    root[count++] = u;
                } else {
                    for (auto e : make_iterator_range(in_edges(u, G))) {
                        root[count++] = source(e, G);
                    }
                }

                for (unsigned i = 0; i != count; ++i) {
                    // While we're collecting our variable set V, keep track of the maximum path length L.
                    // If L == ceil(log2(|V|)), then this portion of the AST is already optimal.

                    flat_map<Vertex, unsigned> L;
                    flat_set<PabloAST *> V;
                    VertexQueue Q;

                    Vertex v = root[i];
                    unsigned maxPathLength = 0;
                    L.emplace(v, 0);
                    for (;;) {
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
                    if (maxPathLength != ceil_log2(V.size())) {



                    }


                }





            }
        }

        // if (u < terminals.size() || out_degree(u, G) == 0) {

        for (auto e : make_iterator_range(edges(G))) {
            Vertex u = target(e, G);
            Vertex v = source(e, G);
            if (u >= terminals.size() && out_degree(v, G) != 0 && G[u]->getClassTypeId() != G[v]->getClassTypeId()) {
                throw std::runtime_error("Illegal graph generated!");
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





//        if (variables.size() > 3) {

//            circular_buffer<PabloAST *> Q(variables.size());

//            out << "-------------------------------------------------\n";

//            for (auto var : variables) {
//                Q.push_back(var);

//                out << " -> ";
//                PabloPrinter::print(var, out); out << '\n';



//                if (LLVM_LIKELY(isa<Statement>(var))) {
//                    nextSet.insert(cast<Statement>(var));
//                }
//            }



//            block.setInsertPoint(stmt->getPrevNode());
//            while (Q.size() > 1) {
//                PabloAST * e1 = Q.front(); Q.pop_front();
//                PabloAST * e2 = Q.front(); Q.pop_front();
//                PabloAST * r = nullptr;
//                switch (classTypeId) {
//                    case PabloAST::ClassTypeId::And:
//                        r = builder.createAnd(e1, e2);
//                        break;
//                    case PabloAST::ClassTypeId::Or:
//                        r = builder.createOr(e1, e2);
//                        break;
//                    case PabloAST::ClassTypeId::Xor:
//                        r = builder.createAnd(e1, e2);
//                        break;
//                    default: LLVM_BUILTIN_UNREACHABLE;
//                }
//                Q.push_back(r);
//            }
//            cast<Statement>(op)->replaceWith(Q.front(), true, true);

//            for (PabloAST * var : variables) {
//                if (isa<Statement>(var) && cast<Statement>(var)->getParent() == &block) {
//                    nextSet.insert(cast<Statement>(var));
//                }
//            }

//        }



//        if (nextSet.empty()) {
//            break;
//        }

//        out << "-------------------------------------------------\n";

//        terminals.assign(nextSet.begin(), nextSet.end());
    }

}

BooleanReassociationPass::BooleanReassociationPass()
{

}


}
