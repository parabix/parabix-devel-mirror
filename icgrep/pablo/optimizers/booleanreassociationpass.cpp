#include "booleanreassociationpass.h"
#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/topological_sort.hpp>
#include <pablo/optimizers/pablo_simplifier.hpp>
#include <algorithm>
#include <queue>
#include <set>
#include <iostream>
#include <pablo/printer_pablos.h>
#include "graph-facade.hpp"

using namespace boost;
using namespace boost::container;

namespace pablo {

using Graph = BooleanReassociationPass::Graph;
using Vertex = Graph::vertex_descriptor;
using VertexQueue = circular_buffer<Vertex>;
using Map = BooleanReassociationPass::Map;
using EdgeQueue = std::queue<std::pair<Vertex, Vertex>>;
using TypeId = PabloAST::ClassTypeId;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief optimize
 ** ------------------------------------------------------------------------------------------------------------- */
bool BooleanReassociationPass::optimize(PabloFunction & function) {
    BooleanReassociationPass brp;
    brp.resolveScopes(function);
    brp.processScopes(function);

    raw_os_ostream out(std::cerr);
    PabloPrinter::print(function.getEntryBlock().statements(), out);

    Simplifier::optimize(function);
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief resolveScopes
 ** ------------------------------------------------------------------------------------------------------------- */
inline void BooleanReassociationPass::resolveScopes(PabloFunction &function) {
    resolveScopes(function.getEntryBlock());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief resolveScopes
 ** ------------------------------------------------------------------------------------------------------------- */
void BooleanReassociationPass::resolveScopes(PabloBlock & block) {
    for (Statement * stmt : block) {
        if (isa<If>(stmt) || isa<While>(stmt)) {
            PabloBlock & nested = isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody();
            mResolvedScopes.emplace(&nested, stmt);
            resolveScopes(nested);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief processScopes
 ** ------------------------------------------------------------------------------------------------------------- */
inline void BooleanReassociationPass::processScopes(PabloFunction & function) {
    std::vector<Statement *> terminals;
    for (unsigned i = 0; i != function.getNumOfResults(); ++i) {
        terminals.push_back(function.getResult(i));
    }
    processScopes(function.getEntryBlock(), std::move(terminals));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief processScopes
 ** ------------------------------------------------------------------------------------------------------------- */
void BooleanReassociationPass::processScopes(PabloBlock & block, std::vector<Statement *> && terminals) {
    processScope(block, std::move(terminals));
    for (Statement * stmt : block) {
        if (isa<If>(stmt)) {
            const auto & defs = cast<const If>(stmt)->getDefined();
            std::vector<Statement *> terminals(defs.begin(), defs.end());
            processScopes(cast<If>(stmt)->getBody(), std::move(terminals));
        } else if (isa<While>(stmt)) {
            const auto & vars = cast<const While>(stmt)->getVariants();
            std::vector<Statement *> terminals(vars.begin(), vars.end());
            processScopes(cast<While>(stmt)->getBody(), std::move(terminals));
        }
    }
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
 * @brief isOptimizable
 *
 * And, Or and Xor instructions are all associative, commutative and distributive operations. Thus we can
 * safely rearrange expressions such as "((((a ∨ b) ∨ c) ∨ d) ∨ e) ∨ f" into "((a ∨ b) ∨ (c ∨ d)) ∨ (e ∨ f)".
 ** ------------------------------------------------------------------------------------------------------------- */
static inline bool isOptimizable(const PabloAST * const expr) {
    assert (expr);
    switch (expr->getClassTypeId()) {
        case PabloAST::ClassTypeId::And:
        case PabloAST::ClassTypeId::Or:
        case PabloAST::ClassTypeId::Xor:
            return true;
        default:
            return false;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief inCurrentBlock
 ** ------------------------------------------------------------------------------------------------------------- */
static inline bool inCurrentBlock(const Statement * stmt, const PabloBlock & block) {
    return stmt->getParent() == &block;
}

static inline bool inCurrentBlock(const PabloAST * expr, const PabloBlock & block) {
    return isa<Statement>(expr) && inCurrentBlock(cast<Statement>(expr), block);
}

///** ------------------------------------------------------------------------------------------------------------- *
// * @brief isCutNecessary
// ** ------------------------------------------------------------------------------------------------------------- */
//static inline bool isCutNecessary(const Vertex u, const Vertex v, const Graph & G, const std::vector<unsigned> & component) {
//    // Either this edge crosses a component boundary or the operations performed by the vertices differs, we need to cut
//    // the graph here and generate two partial equations.
//    if (LLVM_UNLIKELY(component[u] != component[v])) {
//        return true;
//    } else if (LLVM_UNLIKELY((in_degree(u, G) != 0) && (G[u]->getClassTypeId() != G[v]->getClassTypeId()))) {
//        return true;
//    }
//    return false;
//}

///** ------------------------------------------------------------------------------------------------------------- *
// * @brief push
// ** ------------------------------------------------------------------------------------------------------------- */
//static inline void push(const Vertex u, VertexQueue & Q) {
//    if (LLVM_UNLIKELY(Q.full())) {
//        Q.set_capacity(Q.capacity() * 2);
//    }
//    Q.push_back(u);
//    assert (Q.back() == u);
//}

///** ------------------------------------------------------------------------------------------------------------- *
// * @brief pop
// ** ------------------------------------------------------------------------------------------------------------- */
//static inline Vertex pop(VertexQueue & Q) {
//    assert (!Q.empty() && "Popping an empty vertex queue");
//    const Vertex u = Q.front();
//    Q.pop_front();
//    return u;
//}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getVertex
 ** ------------------------------------------------------------------------------------------------------------- */
template<typename ValueType, typename GraphType, typename MapType>
static inline Vertex getVertex(ValueType value, GraphType & G, MapType & M) {
    assert (value);
    const auto f = M.find(value);
    if (f != M.end()) {
        return f->second;
    }
    const auto u = add_vertex(value, G);
    M.insert(std::make_pair(value, u));
    return u;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief createTree
 ** ------------------------------------------------------------------------------------------------------------- */
static PabloAST * createTree(PabloBlock & block, const PabloAST::ClassTypeId typeId, circular_buffer<PabloAST *> & Q) {
    assert (!Q.empty());
    while (Q.size() > 1) {
        PabloAST * e1 = Q.front(); Q.pop_front();
        PabloAST * e2 = Q.front(); Q.pop_front();
        PabloAST * expr = nullptr;
        // Note: this switch ought to compile to an array of function pointers to the appropriate method.
        switch (typeId) {
            case PabloAST::ClassTypeId::And:
                expr = block.createAnd(e1, e2); break;
            case PabloAST::ClassTypeId::Or:
                expr = block.createOr(e1, e2); break;
            case PabloAST::ClassTypeId::Xor:
                expr = block.createXor(e1, e2); break;
            default: break;
        }
        Q.push_back(expr);
    }
    PabloAST * r = Q.front();
    Q.clear();
    return r;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printGraph
 ** ------------------------------------------------------------------------------------------------------------- */
static void printGraph(PabloBlock & block, const Graph & G, const std::string name) {
    raw_os_ostream out(std::cerr);
    out << "digraph " << name << " {\n";
    for (auto u : make_iterator_range(vertices(G))) {
        if (in_degree(u, G) == 0 && out_degree(u, G) == 0) {
            continue;
        }
        out << "v" << u << " [label=\"";
        PabloAST * expr = G[u];
        if (isa<Statement>(expr)) {
            if (LLVM_UNLIKELY(isa<If>(expr))) {
                out << "If ";
                PabloPrinter::print(cast<If>(expr)->getOperand(0), out);
                out << ":";
            } else if (LLVM_UNLIKELY(isa<While>(expr))) {
                out << "While ";
                PabloPrinter::print(cast<While>(expr)->getOperand(0), out);
                out << ":";
            } else {
                PabloPrinter::print(cast<Statement>(expr), "", out);
            }
        } else {
            PabloPrinter::print(expr, out);
        }
        out << "\"";
        if (!inCurrentBlock(expr, block)) {
            out << " style=dashed";
        }
        out << "];\n";
    }
    for (auto e : make_iterator_range(edges(G))) {
        out << "v" << source(e, G) << " -> v" << target(e, G) << ";\n";
    }

    if (num_vertices(G) > 0) {

        out << "{ rank=same;";
        for (auto u : make_iterator_range(vertices(G))) {
            if (in_degree(u, G) == 0 && out_degree(u, G) != 0) {
                out << " v" << u;
            }
        }
        out << "}\n";

        out << "{ rank=same;";
        for (auto u : make_iterator_range(vertices(G))) {
            if (out_degree(u, G) == 0 && in_degree(u, G) != 0) {
                out << " v" << u;
            }
        }
        out << "}\n";

    }

    out << "}\n\n";
    out.flush();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printGraph
 ** ------------------------------------------------------------------------------------------------------------- */
template<typename SubgraphType>
static void printGraph(PabloBlock & block, const SubgraphType & S, const Graph & G, const std::string name) {
    raw_os_ostream out(std::cerr);
    out << "digraph " << name << " {\n";
    for (auto u : make_iterator_range(vertices(S))) {
        if (in_degree(u, S) == 0 && out_degree(u, S) == 0) {
            continue;
        }
        out << "v" << u << " [label=\"";
        PabloAST * expr = G[S[u]];
        if (isa<Statement>(expr)) {
            if (LLVM_UNLIKELY(isa<If>(expr))) {
                out << "If ";
                PabloPrinter::print(cast<If>(expr)->getOperand(0), out);
                out << ":";
            } else if (LLVM_UNLIKELY(isa<While>(expr))) {
                out << "While ";
                PabloPrinter::print(cast<While>(expr)->getOperand(0), out);
                out << ":";
            } else {
                PabloPrinter::print(cast<Statement>(expr), "", out);
            }
        } else {
            PabloPrinter::print(expr, out);
        }
        out << "\"";
        if (!inCurrentBlock(expr, block)) {
            out << " style=dashed";
        }
        out << "];\n";
    }
    for (auto e : make_iterator_range(edges(S))) {
        out << "v" << source(e, S) << " -> v" << target(e, S) << ";\n";
    }

    if (num_vertices(S) > 0) {

        out << "{ rank=same;";
        for (auto u : make_iterator_range(vertices(S))) {
            if (in_degree(u, S) == 0 && out_degree(u, S) != 0) {
                out << " v" << u;
            }
        }
        out << "}\n";

        out << "{ rank=same;";
        for (auto u : make_iterator_range(vertices(S))) {
            if (out_degree(u, S) == 0 && in_degree(u, S) != 0) {
                out << " v" << u;
            }
        }
        out << "}\n";

    }

    out << "}\n\n";
    out.flush();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief processScope
 ** ------------------------------------------------------------------------------------------------------------- */
inline void BooleanReassociationPass::processScope(PabloBlock & block, std::vector<Statement *> && terminals) {
    Graph G;
    summarizeAST(block, G);
    redistributeAST(block, G);
    printGraph(block, G, "G");





    circular_buffer<Vertex> Q(num_vertices(G));
    topological_sort(G, std::front_inserter(Q));

    circular_buffer<PabloAST *> nodes;
    block.setInsertPoint(nullptr);

    while (!Q.empty()) {
        const Vertex u = Q.front();
        Q.pop_front();
        if (LLVM_LIKELY(in_degree(u, G) != 0 || out_degree(u, G) != 0)) {
            PabloAST * expr = G[u];
            if (LLVM_LIKELY(inCurrentBlock(expr, block))) {
                switch (expr->getClassTypeId()) {
                    case TypeId::And:
                    case TypeId::Or:
                case TypeId::Xor: {
                            nodes.set_capacity(in_degree(u, G));
                            for (auto e : make_iterator_range(in_edges(u, G))) {
                                nodes.push_back(G[source(e, G)]);
                            }
                            PabloAST * replacement = createTree(block, expr->getClassTypeId(), nodes);
                            cast<Statement>(expr)->replaceWith(replacement, true, true);
                            expr = replacement;
                        }
                    default:
                        block.insert(cast<Statement>(expr));
                }
            }
        }
    }


}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief summarizeAST
 *
 * This function scans through a basic block (starting by its terminals) and computes a DAG in which any sequences
 * of AND, OR or XOR functions are "flattened" (i.e., allowed to have any number of inputs.) This allows us to
 * reassociate them in the most efficient way possible.
 ** ------------------------------------------------------------------------------------------------------------- */
void BooleanReassociationPass::summarizeAST(PabloBlock & block, Graph & G) const {

    Map M;

    // Compute the base def-use graph ...
    for (Statement * stmt : block) {
        const Vertex u = getVertex(stmt, G, M);
        for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
            PabloAST * const op = stmt->getOperand(i);
            if (LLVM_UNLIKELY(isa<Integer>(op) || isa<String>(op))) {
                continue;
            }
            add_edge(getVertex(op, G, M), u, G);
        }
        if (isa<If>(stmt)) {
            for (Assign * def : cast<const If>(stmt)->getDefined()) {
                add_edge(u, getVertex(def, G, M), G);
            }
        } else if (isa<While>(stmt)) {
            for (Next * var : cast<const While>(stmt)->getVariants()) {
                add_edge(u, getVertex(var, G, M), G);
            }
        } else if (isOptimizable(stmt)) {
            for (PabloAST * user : stmt->users()) {
                if (LLVM_LIKELY(isa<Statement>(user))) {
                    PabloBlock * parent = cast<Statement>(user)->getParent();
                    if (LLVM_UNLIKELY(parent != &block)) {
                        while (parent->getParent() != &block) {
                            parent = parent->getParent();
                        }
                        if (LLVM_UNLIKELY(parent == nullptr)) {
                            throw std::runtime_error("Could not locate nested scope block!");
                        }
                        const auto f = mResolvedScopes.find(parent);
                        if (LLVM_UNLIKELY(f == mResolvedScopes.end())) {
                            throw std::runtime_error("Failed to resolve scope block!");
                        }
                        add_edge(u, getVertex(f->second, G, M), G);
                    }
                }
            }
        }
    }

    std::vector<Vertex> ordering;
    ordering.reserve(num_vertices(G));
    topological_sort(G, std::back_inserter(ordering));

    for (auto i = ordering.rbegin(); i != ordering.rend(); ++i) {
        const Vertex u = *i;
        if (isOptimizable(G[u])) {
            std::vector<Vertex> removable;
            for (auto e : make_iterator_range(in_edges(u, G))) {
                const Vertex v = source(e, G);
                if (out_degree(v, G) == 1 && in_degree(v, G) != 0 && G[u]->getClassTypeId() == G[v]->getClassTypeId()) {
                    removable.push_back(v);
                    for (auto e : make_iterator_range(in_edges(v, G))) {
                        add_edge(source(e, G), u, G);
                    }
                }
            }
            for (auto v : removable) {
                clear_vertex(v, G);
            }
        }
    }

}


///** ------------------------------------------------------------------------------------------------------------- *
// * @brief summarizeAST
// *
// * This function scans through a basic block (starting by its terminals) and computes a DAG in which any sequences
// * of AND, OR or XOR functions are "flattened" (i.e., allowed to have any number of inputs.) This allows us to
// * reassociate them in the most efficient way possible.
// ** ------------------------------------------------------------------------------------------------------------- */
//void BooleanReassociationPass::summarizeAST(PabloBlock & block, std::vector<Statement *> terminals, Graph & G) const {

//    VertexQueue Q(128);
//    EdgeQueue E;
//    Map M;



//    raw_os_ostream out(std::cerr);

//    out << "================================================================\n";

//    for (;;) {


//        Graph Gk;
//        Map Mk;

//        // Generate a graph depicting the relationships between the terminals. If the original terminals
//        // cannot be optimized with this algorithm bypass them in favour of their operands. If those cannot
//        // be optimized, they'll be left as the initial terminals for the next "layer" of the AST.

//        for (Statement * term : terminals) {
//            if (LLVM_LIKELY(Mk.count(term) == 0)) {
//                // add or find this terminal in our global graph
//                Vertex x = getVertex(term, G, M);

//                PabloPrinter::print(term, "term: ", out);
//                out << '\n';


//                if (inCurrentBlock(term, block)) {
//                    if (isOptimizable(term)) {
//                        const Vertex u = add_vertex(term, Gk);
//                        Mk.insert(std::make_pair(term, u));
//                        push(u, Q);
//                        continue;
//                    }
//                } else if (isa<Assign>(term) || isa<Next>(term)) {
//                    // If this is an Assign (Next) node whose operand does not originate from the current block
//                    // then check to see if there is an If (While) node that does.
//                    Statement * branch = nullptr;
//                    if (isa<Assign>(term)) {
//                        for (PabloAST * user : term->users()) {
//                            if (isa<If>(user)) {
//                                const If * ifNode = cast<If>(user);
//                                if (inCurrentBlock(ifNode, block)) {
//                                    const auto & defs = ifNode->getDefined();
//                                    if (LLVM_LIKELY(std::find(defs.begin(), defs.end(), cast<Assign>(term)) != defs.end())) {
//                                        branch = cast<Statement>(user);
//                                        break;
//                                    }
//                                }
//                            }
//                        }
//                    } else { // if (isa<Next>(term))
//                        for (PabloAST * user : term->users()) {
//                            if (isa<While>(user)) {
//                                const While * whileNode = cast<While>(user);
//                                if (inCurrentBlock(whileNode, block)) {
//                                    const auto & vars = whileNode->getVariants();
//                                    if (LLVM_LIKELY(std::find(vars.begin(), vars.end(), cast<Next>(term)) != vars.end())) {
//                                        branch = cast<Statement>(user);
//                                        break;
//                                    }
//                                }
//                            }
//                        }
//                    }

//                    // If we didn't find a branch, then the Assign (Next) node must have come from a preceeding
//                    // block. Just skip it for now.
//                    if (branch == nullptr) {
//                        continue;
//                    }

//                    // Otherwise add the branch to G and test its operands rather than the original terminal
//                    const Vertex z = getVertex(branch, G, M);
//                    add_edge(z, x, G);
//                    x = z;
//                    term = branch;

//                }

//                for (unsigned i = 0; i != term->getNumOperands(); ++i) {
//                    PabloAST * const op = term->getOperand(i);
//                    if (LLVM_LIKELY(inCurrentBlock(op, block))) {
//                        const Vertex y = getVertex(op, G, M);
//                        add_edge(y, x, G);

//                        if (LLVM_LIKELY(Mk.count(op) == 0)) {
//                            const Vertex v = add_vertex(op, Gk);
//                            Mk.insert(std::make_pair(op, v));
//                            push(v, Q);
//                        }
//                    }
//                }
//            }
//        }

//        out << "----------------------------------------------------------------\n";
//        out.flush();

//        if (LLVM_UNLIKELY(Q.empty())) {
//            break;
//        }

//        for (;;) {
//            const Vertex u = pop(Q);
//            if (isOptimizable(Gk[u])) {
//                Statement * stmt = cast<Statement>(Gk[u]);
//                // If any user of this statement is not in the current block, determine the outermost If/While node
//                // that contains this statement within and add an edge from this statement to it to denote both the
//                // topological ordering necessary and that this statement must be computed.
//                for (PabloAST * user : stmt->users()) {
//                    if (LLVM_LIKELY(isa<Statement>(user))) {
//                        PabloBlock * parent = cast<Statement>(user)->getParent();
//                        if (LLVM_UNLIKELY(parent != &block)) {
//                            while (parent->getParent() != &block) {
//                                parent = parent->getParent();
//                            }
//                            if (LLVM_UNLIKELY(parent == nullptr)) {
//                                throw std::runtime_error("Could not locate nested scope block!");
//                            }
//                            const auto f = mResolvedScopes.find(parent);
//                            if (LLVM_UNLIKELY(f == mResolvedScopes.end())) {
//                                throw std::runtime_error("Failed to resolve scope block!");
//                            }
//                            add_edge(u, getVertex(f->second, Gk, Mk), Gk);
//                        }
//                    }
//                }
//                // Scan through the use-def chains to locate any chains of rearrangable expressions and their inputs
//                for (unsigned i = 0; i != 2; ++i) {
//                    PabloAST * op = stmt->getOperand(i);
//                    auto f = Mk.find(op);
//                    if (f == Mk.end()) {
//                        const Vertex v = add_vertex(op, Gk);
//                        f = Mk.insert(std::make_pair(op, v)).first;
//                        if (op->getClassTypeId() == stmt->getClassTypeId() && inCurrentBlock(cast<Statement>(op), block)) {
//                            push(v, Q);
//                        }
//                    }
//                    add_edge(f->second, u, Gk);
//                }
//            }
//            if (Q.empty()) {
//                break;
//            }
//        }

//        printGraph(block, Gk, "Gk1");

//        // Generate a topological ordering for G; if one of our terminals happens to also be a partial computation of
//        // another terminal, we need to make sure we compute it as an independent subexpression.
//        std::vector<unsigned> ordering;
//        ordering.reserve(num_vertices(Gk));
//        topological_sort(Gk, std::back_inserter(ordering));
//        std::vector<unsigned> component(num_vertices(Gk), 0);

//        // Mark which computation component each terminal is in based on their topological (occurence) order.
//        unsigned components = 0;
//        std::sort(terminals.begin(), terminals.end());
//        for (auto u : ordering) {
//            unsigned id = 0;
//            // If this is a sink in G, it is the root of a new component.
//            if (out_degree(u, Gk) == 0 || std::binary_search(terminals.begin(), terminals.end(), Gk[u])) {
//                id = ++components;
//            } else { // otherwise it belongs to the outermost component.
//                for (auto e : make_iterator_range(out_edges(u, Gk))) {
//                    assert (component[target(e, Gk)] != 0);
//                    id = std::max(id, component[target(e, Gk)]);
//                }
//            }
//            assert (id && "Topological ordering failed!");
//            component[u] = id;
//        }

//        for (;;) {

//            // Cut the graph wherever a computation crosses a component or whenever we need to cut the graph because
//            // the instructions corresponding to the pair of nodes differs.
//            graph_traits<Graph>::edge_iterator ei, ei_end;
//            for (std::tie(ei, ei_end) = edges(Gk); ei != ei_end; ) {
//                const Graph::edge_descriptor e = *ei++;
//                const Vertex u = source(e, Gk);
//                const Vertex v = target(e, Gk);
//                if (LLVM_UNLIKELY(isCutNecessary(u, v, Gk, component))) {
//                    E.push(std::make_pair(u, v));
//                    remove_edge(u, v, Gk);
//                }
//            }

//            // If no cuts are necessary, we're done.
//            if (E.empty()) {
//                break;
//            }

//            for (;;) {

//                Vertex u, v;
//                std::tie(u, v) = E.front(); E.pop();

//                // The vertex belonging to a component with a greater number must come "earlier"
//                // in the program. By replicating it, this ensures it's computed as an output of
//                // one component and used as an input of another.
//                if (component[u] < component[v]) {
//                    std::swap(u, v);
//                }

//                // Replicate u and fix the ordering and component vectors to reflect the change in Gk.
//                const Vertex w = add_vertex(Gk[u], Gk);
//                ordering.insert(std::find(ordering.begin(), ordering.end(), u), w);
//                assert (component.size() == w);
//                component.push_back(component[v]);
//                add_edge(w, v, Gk);

//                // However, after we do so, we need to make sure the original source vertex will be a
//                // sink in Gk unless it is also an input variable (in which case we'd simply end up with
//                // extraneous isolated vertex. Otherwise, we need to make further cuts and replications.
//                if (in_degree(u, Gk) != 0) {
//                    for (auto e : make_iterator_range(out_edges(u, Gk))) {
//                        E.push(std::make_pair(source(e, Gk), target(e, Gk)));
//                    }
//                    clear_out_edges(u, Gk);
//                }

//                if (E.empty()) {
//                    break;
//                }
//            }

//            // Mark which computation component each sink is in based on their topological (occurence) order.
//            unsigned components = 0;
//            #ifndef NDEBUG
//            std::fill(component.begin(), component.end(), 0);
//            #endif
//            for (auto u : ordering) {
//                unsigned id = 0;
//                // If this is a sink in G, it is the root of a new component.
//                if (out_degree(u, Gk) == 0) {
//                    id = ++components;
//                } else { // otherwise it belongs to the outermost component.
//                    for (auto e : make_iterator_range(out_edges(u, Gk))) {
//                        assert (component[target(e, Gk)] != 0);
//                        id = std::max(id, component[target(e, Gk)]);
//                    }
//                }
//                assert (id && "Topological ordering failed!");
//                component[u] = id;
//            }
//        }

//        printGraph(block, Gk, "Gk2");

//        // Scan through the graph so that we process the outermost expressions first
//        for (const Vertex u : ordering) {
//            if (LLVM_UNLIKELY(out_degree(u, Gk) == 0)) {
//                // Create a vertex marking the output statement we may end up replacing
//                // and collect the set of source variables in the component
//                const Vertex x = getVertex(Gk[u], G, M);
//                if (LLVM_LIKELY(in_degree(u, Gk) > 0)) {
//                    flat_set<PabloAST *> vars;
//                    flat_set<Vertex> visited;
//                    for (Vertex v = u;;) {
//                        if (in_degree(v, Gk) == 0) {
//                            vars.insert(Gk[v]);
//                        } else {
//                            for (auto e : make_iterator_range(in_edges(v, Gk))) {
//                                const Vertex w = source(e, Gk);
//                                if (LLVM_LIKELY(visited.insert(w).second)) {
//                                    push(w, Q);
//                                }
//                            }
//                        }
//                        if (Q.empty()) {
//                            break;
//                        }
//                        v = pop(Q);
//                    }
//                    for (PabloAST * var : vars) {
//                        add_edge(getVertex(var, G, M), x, G);
//                    }
//                }
//            }
//        }

//        // Determine the source variables of the next "layer" of the AST
//        flat_set<Statement *> nextSet;
//        for (auto u : ordering) {
//            if (LLVM_UNLIKELY(in_degree(u, Gk) == 0 && isa<Statement>(Gk[u]))) {
//                nextSet.insert(cast<Statement>(Gk[u]));
//            } else if (LLVM_UNLIKELY(out_degree(u, Gk) == 0 && isa<Statement>(Gk[u]))) {
//                // some input will also be the output of some subgraph of Gk whenever we cut and
//                // replicated a vertex. We don't need to reevaluate it as part of the next layer.
//                nextSet.erase(cast<Statement>(Gk[u]));
//            }
//        }

//        if (LLVM_UNLIKELY(nextSet.empty())) {
//            break;
//        }

//        out << "----------------------------------------------------------------\n";

//        terminals.assign(nextSet.begin(), nextSet.end());
//    }



//}

using VertexSet = std::vector<Vertex>;
using VertexSets = std::vector<VertexSet>;

template <class Graph>
static VertexSet incomingVertexSet(const Vertex u, const Graph & G) {
    VertexSet V;
    V.reserve(G.in_degree(u));
    for (auto e : make_iterator_range(G.in_edges(u))) {
        V.push_back(G.source(e));
    }
    std::sort(V.begin(), V.end());
    return std::move(V);
}

template <class Graph>
static VertexSet outgoingVertexSet(const Vertex u, const Graph & G) {
    VertexSet V;
    V.reserve(G.out_degree(u));
    for (auto e : make_iterator_range(G.out_edges(u))) {
        V.push_back(G.target(e));
    }
    std::sort(V.begin(), V.end());
    return std::move(V);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enumerateBicliques
 *
 * Adaptation of the MICA algorithm as described in "Consensus algorithms for the generation of all maximal
 * bicliques" by Alexe et. al. (2003). Note: this implementation considers all verticies with an in-degree of 0
 * to be in bipartition A and their adjacencies to be in B.
  ** ------------------------------------------------------------------------------------------------------------- */
template <class Graph>
static VertexSets enumerateBicliques(const Graph & G) {
    using IntersectionSets = std::set<VertexSet>;

    IntersectionSets B1;
    for (auto u : make_iterator_range(G.vertices())) {
        if (G.in_degree(u) == 0) {
            B1.insert(std::move(outgoingVertexSet(u, G)));
        }
    }

    IntersectionSets C(B1.begin(), B1.end());

    IntersectionSets Bi;
    VertexSet clique;
    for (auto i = B1.begin(); i != B1.end(); ++i) {
        for (auto j = i; ++j != B1.end(); ) {
            std::set_intersection(i->begin(), i->end(), j->begin(), j->end(), std::back_inserter(clique));
            if (clique.size() > 0) {
                if (C.count(clique) == 0) {
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
        C.insert(Bi.begin(), Bi.end());
        IntersectionSets Bk;
        for (auto i = B1.begin(); i != B1.end(); ++i) {
            for (auto j = Bi.begin(); j != Bi.end(); ++j) {
                std::set_intersection(i->begin(), i->end(), j->begin(), j->end(), std::back_inserter(clique));
                if (clique.size() > 0) {
                    if (C.count(clique) == 0) {
                        Bk.insert(clique);
                    }
                    clique.clear();
                }
            }
        }
        Bi.swap(Bk);
    }
    return VertexSets(C.begin(), C.end());
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
 * @brief maximalIndependentSet
 ** ------------------------------------------------------------------------------------------------------------- */
static VertexSets && maximalIndependentSet(VertexSets && V) {
    using IndependentSetGraph = adjacency_list<hash_setS, vecS, undirectedS, unsigned>;
    const auto l = V.size();
    IndependentSetGraph I(l);
    // Initialize our weights
    for (unsigned i = 0; i != l; ++i) {
        I[i] = V[i].size();
    }
    // Determine our constraints
    for (unsigned i = 0; i != l; ++i) {
        for (unsigned j = i + 1; j != l; ++j) {
            if (intersects(V[i], V[j])) {
                add_edge(i, j, I);
            }
        }
    }
    // Use the greedy algorithm to choose our independent set (TODO: try the similar randomized approach)
    VertexSet selected;
    std::vector<bool> ignored(l, false);
    for (;;) {
        unsigned w = 0;
        Vertex u = 0;
        for (unsigned i = 0; i != l; ++i) {
            if (!ignored[i] && I[i] > w) {
                w = I[i];
                u = i;
            }
        }
        if (w == 0) break;
        selected.push_back(u);
        ignored[u] = true;
        for (auto v : make_iterator_range(adjacent_vertices(u, I))) {
            ignored[v] = true;
        }
    }
    // Sort the selected list and then remove the unselected sets from V
    std::sort(selected.begin(), selected.end(), std::greater<Vertex>());
    auto end = V.end();
    for (const unsigned offset : selected) {
        end = V.erase(V.begin() + offset + 1, end) - 1;
    }
    V.erase(V.begin(), end);
    return std::move(V);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief sinkIndependentMaximalBicliques
 ** ------------------------------------------------------------------------------------------------------------- */
template <class Graph>
static VertexSets sinkIndependentMaximalBicliques(Graph && G) {
    VertexSets B = maximalIndependentSet(std::move(enumerateBicliques(G)));
    VertexSets A;
    A.reserve(B.size());
    for (const VertexSet & Bi : B) {
        // Compute our A set
        auto bi = Bi.begin();
        VertexSet Ai = incomingVertexSet(*bi, G);
        while (++bi != Bi.end()) {
            VertexSet Ai = incomingVertexSet(*bi, G);
            VertexSet Ak;
            std::set_intersection(Ai.begin(), Ai.end(), Ai.begin(), Ai.end(), std::back_inserter(Ak));
            Ai.swap(Ak);
        }
        A.emplace_back(std::move(Ai));
    }
    std::vector<Vertex> sources;
    std::vector<Vertex> intermediary;
    for (auto u : make_iterator_range(G.vertices())) {
        if (G.in_degree(u) == 0) {
            sources.push_back(u);
        } else if (G.out_degree(u) != 0) {
            intermediary.push_back(u);
        }
    }
    for (auto u : sources) {
        G.clear_out_edges(u);
    }
    for (unsigned i = 0; i != B.size(); ++i) {
        for (auto u : A[i]) {
            for (auto v : B[i]) {
                G.add_edge(u, v);
            }
        }
    }
    for (auto u : intermediary) {
        if (G.in_degree(u) == 0) {
            G.clear_out_edges(u);
        }
    }
    return std::move(A);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief safeDistributionSets
 ** ------------------------------------------------------------------------------------------------------------- */
template <class Graph>
static VertexSets safeDistributionSets(Graph & G) {
    VertexSets sinks = sinkIndependentMaximalBicliques(makeTransposedGraphFacade(G));
    // Scan through G and replicate any source that has more than one sink until G is broken
    // into weakly connected components in which each has exactly one sink.
    if (sinks.size() > 1) {
        std::vector<unsigned> component(num_vertices(G), 0);
        unsigned components = 0;
        for (const VertexSet & S : sinks) {
            ++components;
            for (auto e : make_iterator_range(in_edges(S.front(), G))) {
                component[source(e, G)] = components;
            }
        }
        for (const Vertex u : make_iterator_range(vertices(G))) {
            if (LLVM_UNLIKELY(in_degree(u, G) == 0)) {
                flat_set<unsigned> membership;
                for (auto e : make_iterator_range(out_edges(u, G))) {
                    membership.insert(component[target(e, G)]);
                }
                if (LLVM_UNLIKELY(membership.size() > 1)) {
                    VertexSet adjacent;
                    adjacent.reserve(out_degree(u, G));
                    for (auto e : make_iterator_range(out_edges(u, G))) {
                        adjacent.push_back(target(e, G));
                    }
                    clear_out_edges(u, G);
                    auto mi = membership.begin();
                    for (Vertex w = u; ;) {
                        const unsigned m = *mi;
                        for (auto v : adjacent) {
                            if (component[v] == m) {
                                add_edge(w, v, G);
                            }
                        }
                        if (++mi == membership.end()) {
                            break;
                        }
                        w = add_vertex(G[u], G);
                    }
                }
            }
        }
    }
    return std::move(sinkIndependentMaximalBicliques(makeGraphFacade(G)));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief segmentInto3LevelGraph
 *
 * Ensure that every source-to-sink path in G has an edge-distance of exactly 2. The safeDistributionSets algorithm
 * expects that G exibits this property but if an input to a distributable function is also the output of another
 * distributable function, this complicates the analysis process. Thus method resolves that by replicating the
 * appropriate vertices into input-only and output-only vertices.
 ** ------------------------------------------------------------------------------------------------------------- */
template <class Graph>
Graph & segmentInto3LevelGraph(Graph & G) {
    std::vector<unsigned> distance(num_vertices(G), 0);
    std::queue<Vertex> Q;
    for (const Vertex u : make_iterator_range(vertices(G))) {
        if (in_degree(u, G) == 0) {
            distance[u] = 1;
            for (const auto e : make_iterator_range(out_edges(u, G))) {
                Q.push(target(e, G));
            }
        }
    }
    for (;;) {
        const Vertex u = Q.front(); Q.pop();
        unsigned dist = 0;
        bool ready = true;
        for (const auto e : make_iterator_range(in_edges(u, G))) {
            const Vertex v = source(e, G);
            if (LLVM_UNLIKELY(distance[v] == 0)) {
                ready = false;
                break;
            }
            dist = std::max(dist, distance[v]);
        }
        assert (dist <= 4);
        if (ready) {
            if (LLVM_UNLIKELY(dist == 4)) {
                for (const auto e : make_iterator_range(in_edges(u, G))) {
                    const Vertex v = source(e, G);
                    if (distance[v] == 3) {
                        const Vertex w = add_vertex(G[v], G);
                        for (const auto e : make_iterator_range(out_edges(v, G))) {
                            add_edge(w, target(e, G), G);
                        }
                        clear_out_edges(v, G);
                        assert (w == distance.size());
                        distance.push_back(0);
                        Q.push(w);
                    }
                }
            } else { // update the distance and add in the adjacent vertices to Q
                distance[u] = dist + 1;
                for (const auto e : make_iterator_range(out_edges(u, G))) {
                    Q.push(target(e, G));
                }
            }
        }
        if (Q.empty()) {
            break;
        }
    }
    return G;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief redistributeAST
 *
 * Apply the distribution law to reduce computations whenever possible.
 ** ------------------------------------------------------------------------------------------------------------- */
bool BooleanReassociationPass::redistributeAST(PabloBlock & block, Graph & G) const {
    bool anyChanges = false;
    for (;;) {

        adjacency_list<hash_setS, vecS, bidirectionalS, Vertex> H;
        flat_map<Vertex, Vertex> M;

        for (const Vertex u : make_iterator_range(vertices(G))) {
            const TypeId outerTypeId = G[u]->getClassTypeId();
            if (outerTypeId == TypeId::And || outerTypeId == TypeId::Or) {
                if (inCurrentBlock(cast<Statement>(G[u]), block)) {
                    const TypeId innerTypeId = (outerTypeId == TypeId::And) ? TypeId::Or : TypeId::And;
                    flat_set<Vertex> distributable;
                    for (auto e : make_iterator_range(in_edges(u, G))) {
                        const Vertex v = source(e, G);
                        if (LLVM_UNLIKELY(G[v]->getClassTypeId() == innerTypeId) && LLVM_LIKELY(inCurrentBlock(cast<Statement>(G[v]), block))) {
                            bool safe = true;
                            for (const auto e : make_iterator_range(out_edges(v, G))) {
                                if (G[target(e, G)]->getClassTypeId() != outerTypeId) {
                                    safe = false;
                                    break;
                                }
                            }
                            if (safe) {
                                distributable.insert(v);
                            }
                        }
                    }
                    if (LLVM_LIKELY(distributable.size() > 1)) {
                        // We're only interested in computing a subgraph of G in which every source has an out-degree
                        // greater than 1. Otherwise we'd only end up permuting the AND/OR sequence with no logical
                        // benefit. (Note: this does not consider register pressure / liveness.)
                        flat_map<Vertex, unsigned> observed;
                        bool anyOpportunities = false;
                        for (const Vertex v : distributable) {
                            for (auto e : make_iterator_range(in_edges(v, G))) {
                                const Vertex w = source(e, G);
                                const auto f = observed.find(w);
                                if (f == observed.end()) {
                                    observed.emplace(w, 0);
                                } else {
                                    f->second += 1;
                                    anyOpportunities = true;
                                }
                            }
                        }
                        if (anyOpportunities) {
                            for (auto ob : observed) {
                                if (std::get<1>(ob)) {
                                    const Vertex w = std::get<0>(ob);
                                    for (auto e : make_iterator_range(out_edges(w, G))) {
                                        const Vertex v = target(e, G);
                                        if (distributable.count(v)) {
                                            const Vertex y = getVertex(v, H, M);
                                            add_edge(y, getVertex(u, H, M), H);
                                            add_edge(getVertex(w, H, M), y, H);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // If we found no potential opportunities then we cannot apply the distribution law to any part of G.
        if (num_vertices(H) == 0) {
            break;
        }

        segmentInto3LevelGraph(H);

        const VertexSets distributionSets = safeDistributionSets(H);

        printGraph(block, H, G, "H");

        // If no sources remain, no bicliques were found that would have a meaningful impact on the AST.
        if (LLVM_UNLIKELY(distributionSets.size() == 0)) {
            break;
        }

        for (const VertexSet & sources : distributionSets) {
            assert (sources.size() > 0);
            const VertexSet intermediary = outgoingVertexSet(sources.front(), makeGraphFacade(H));
            assert (intermediary.size() > 0);
            const VertexSet sinks = outgoingVertexSet(intermediary.front(), makeGraphFacade(H));
            assert (sinks.size() > 0);

            // Now begin transforming the AST
            const TypeId typeId = G[H[sinks.front()]]->getClassTypeId();
            assert (typeId == TypeId::And || typeId == TypeId::Or);

            circular_buffer<PabloAST *> Q(std::max(sources.size(), intermediary.size() + 1));
            for (const Vertex u : intermediary) {
                Q.push_back(G[H[u]]);
            }
            PabloAST * merged = createTree(block, typeId, Q);
            for (const Vertex s : sources) {
                Q.push_back(G[H[s]]);
            }
            Q.push_back(merged);
            PabloAST * masked = createTree(block, typeId == TypeId::Or ? TypeId::And : TypeId::Or, Q);

            // Eliminate the edges from our original graph
            for (const Vertex u : intermediary) {
                for (const Vertex s : sources) {
                    remove_edge(H[s], H[u], G);
                }
                for (const Vertex t : sinks) {
                    remove_edge(H[u], H[t], G);
                }
            }

            // Finally update G to match the desired changes
            const Vertex x = add_vertex(merged, G);
            const Vertex y = add_vertex(masked, G);
            for (const Vertex u : intermediary) {
                add_edge(H[u], x, G);
            }
            add_edge(x, y, G);
            for (const Vertex s : sources) {
                add_edge(H[s], y, G);
            }
            for (const Vertex t : sinks) {
                add_edge(y, H[t], G);
            }
            for (const Vertex u : intermediary) {
                if (LLVM_UNLIKELY(in_degree(H[u], G) == 0)) {
                    clear_out_edges(H[u], G);
                }
            }
        }
        anyChanges = true;
    }
    return anyChanges;
}

}
