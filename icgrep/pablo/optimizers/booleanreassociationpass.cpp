#include "booleanreassociationpass.h"
#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>
#include <pablo/optimizers/pablo_simplifier.hpp>
#include <queue>
#include <iostream>
#include <pablo/printer_pablos.h>


using namespace boost;
using namespace boost::container;

namespace pablo {

using Graph = adjacency_list<hash_setS, vecS, bidirectionalS, PabloAST *>;
using Vertex = Graph::vertex_descriptor;
using VertexQueue = circular_buffer<Vertex>;
using Map = std::unordered_map<PabloAST *, Vertex>;
using EdgeQueue = std::queue<std::pair<Vertex, Vertex>>;

static void summarizeAST(PabloBlock & block, std::vector<Statement *> && terminals, Graph & G);

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief optimize
 ** ------------------------------------------------------------------------------------------------------------- */
bool BooleanReassociationPass::optimize(PabloFunction & function) {
    BooleanReassociationPass brp;
//    raw_os_ostream out(std::cerr);
//    out << "BEFORE:\n\n";
//    PabloPrinter::print(function.getEntryBlock().statements(), out);
    brp.scan(function);
    Simplifier::optimize(function);

//    out << "\n\nAFTER:\n\n";
//    PabloPrinter::print(function.getEntryBlock().statements(), out);
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief scan
 ** ------------------------------------------------------------------------------------------------------------- */
void BooleanReassociationPass::scan(PabloFunction & function) {
    std::vector<Statement *> terminals;
    for (unsigned i = 0; i != function.getNumOfResults(); ++i) {
        terminals.push_back(function.getResult(i));
    }
    scan(function.getEntryBlock(), std::move(terminals));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief scan
 ** ------------------------------------------------------------------------------------------------------------- */
void BooleanReassociationPass::scan(PabloBlock & block, std::vector<Statement *> && terminals) {

    processScope(block, std::move(terminals));

    for (Statement * stmt : block) {
        if (isa<If>(stmt)) {
            const auto & defs = cast<const If>(stmt)->getDefined();
            std::vector<Statement *> terminals(defs.begin(), defs.end());
            scan(cast<If>(stmt)->getBody(), std::move(terminals));
        } else if (isa<While>(stmt)) {
            const auto & vars = cast<const While>(stmt)->getVariants();
            std::vector<Statement *> terminals(vars.begin(), vars.end());
            scan(cast<While>(stmt)->getBody(), std::move(terminals));
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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isAnyUserNotInCurrentBlock
 ** ------------------------------------------------------------------------------------------------------------- */
static inline bool isAnyUserNotInCurrentBlock(const PabloAST * expr, const PabloBlock & block) {
    for (PabloAST * user : expr->users()) {
        if (!inCurrentBlock(cast<Statement>(user), block)) {
            return true;
        }
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isCutNecessary
 ** ------------------------------------------------------------------------------------------------------------- */
static inline bool isCutNecessary(const Vertex u, const Vertex v, const Graph & G, const std::vector<unsigned> & component) {
    // Either this edge crosses a component boundary or the operations performed by the vertices differs, we need to cut
    // the graph here and generate two partial equations.
    if (LLVM_UNLIKELY(component[u] != component[v])) {
        return true;
    } else if (LLVM_UNLIKELY((in_degree(u, G) != 0) && (G[u]->getClassTypeId() != G[v]->getClassTypeId()))) {
        return true;
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief push
 ** ------------------------------------------------------------------------------------------------------------- */
static inline void push(const Vertex u, VertexQueue & Q) {
    if (LLVM_UNLIKELY(Q.full())) {
        Q.set_capacity(Q.capacity() * 2);
    }
    Q.push_back(u);
    assert (Q.back() == u);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief pop
 ** ------------------------------------------------------------------------------------------------------------- */
static inline Vertex pop(VertexQueue & Q) {
    assert (!Q.empty() && "Popping an empty vertex queue");
    const Vertex u = Q.front();
    Q.pop_front();
    return u;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getVertex
 ** ------------------------------------------------------------------------------------------------------------- */
static inline Vertex getVertex(PabloAST * expr, Graph & G, Map & M) {
    const auto f = M.find(expr);
    if (f != M.end()) {
        return f->second;
    }
    const auto u = add_vertex(expr, G);
    M.insert(std::make_pair(expr, u));
    return u;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief createTree
 ** ------------------------------------------------------------------------------------------------------------- */
static PabloAST * createTree(PabloBlock & block, const PabloAST::ClassTypeId typeId, circular_buffer<PabloAST *> & Q) {
    while (Q.size() > 1) {
        PabloAST * e1 = Q.front(); Q.pop_front();
        PabloAST * e2 = Q.front(); Q.pop_front();
        PabloAST * expr = nullptr;
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
 * @brief applyDistributionLaw
 ** ------------------------------------------------------------------------------------------------------------- */
static bool applyDistributionLaw(PabloBlock & block, const PabloAST::ClassTypeId typeId, flat_set<PabloAST *> & vars) {
    circular_buffer<PabloAST *> Q0(vars.size());
    circular_buffer<PabloAST *> Q1(vars.size());
    std::vector<PabloAST *> distributedVars;

    for (auto vi = vars.begin(); vi != vars.end(); ) {
        PabloAST * const e0 = *vi;

        if (e0->getClassTypeId() == typeId) {
            Statement * const s0 = cast<Statement>(e0);

            for (auto vj = vi + 1; vj != vars.end(); ) {
                PabloAST * const e1 = *vj;

                if (e1->getClassTypeId() == typeId) {
                    Statement * const s1 = cast<Statement>(e1);
                    bool distributed = false;

                    if (s0->getOperand(0) == s1->getOperand(0)) {
                        Q0.push_back(s1->getOperand(1));
                        distributed = true;
                    } else if (s0->getOperand(0) == s1->getOperand(1)) {
                        Q0.push_back(s1->getOperand(0));
                        distributed = true;
                    }

                    if (s0->getOperand(1) == s1->getOperand(0)) {
                        Q1.push_back(s1->getOperand(1));
                        distributed = true;
                    } else if (s0->getOperand(1) == s1->getOperand(1)) {
                        Q1.push_back(s1->getOperand(0));
                        distributed = true;
                    }

                    if (distributed) {
                        vj = vars.erase(vj);
                        continue;
                    }
                }

                ++vj;
            }

            if (LLVM_UNLIKELY(Q0.size() > 0 || Q1.size() > 0)) {
                const PabloAST::ClassTypeId innerTypeId =
                        (typeId == PabloAST::ClassTypeId::Or) ? PabloAST::ClassTypeId::And : PabloAST::ClassTypeId::Or;

                vi = vars.erase(vi);
                if (Q0.size() > 0) {
                    Q0.push_back(s0->getOperand(1));
                    PabloAST * distributed = createTree(block, innerTypeId, Q0);
                    switch (typeId) {
                        case PabloAST::ClassTypeId::And:
                            distributed = block.createAnd(s0->getOperand(0), distributed); break;
                        case PabloAST::ClassTypeId::Or:
                            distributed = block.createOr(s0->getOperand(0), distributed); break;
                        default: break;
                    }
                    distributedVars.push_back(distributed);
                }
                if (Q1.size() > 0) {
                    Q1.push_front(s0->getOperand(0));
                    PabloAST * distributed = createTree(block, innerTypeId, Q1);
                    switch (typeId) {
                        case PabloAST::ClassTypeId::And:
                            distributed = block.createAnd(s0->getOperand(1), distributed); break;
                        case PabloAST::ClassTypeId::Or:
                            distributed = block.createOr(s0->getOperand(1), distributed); break;
                        default: break;
                    }
                    distributedVars.push_back(distributed);
                }
                continue;
            }
        }
        ++vi;
    }
    if (distributedVars.empty()) {
        return false;
    }
    for (PabloAST * var : distributedVars) {
        vars.insert(var);
    }
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief processScope
 ** ------------------------------------------------------------------------------------------------------------- */
void BooleanReassociationPass::processScope(PabloBlock & block, std::vector<Statement *> && terminals) {

    Graph G;
    summarizeAST(block, std::move(terminals), G);

    raw_os_ostream out(std::cerr);
    out << "digraph G {\n";
    for (auto u : make_iterator_range(vertices(G))) {
        out << "v" << u << " [label=\"";
        PabloAST * expr = G[u];
        if (isa<Statement>(expr)) {
            if (LLVM_UNLIKELY(isa<If>(expr))) {
                out << "if ";
                PabloPrinter::print(cast<If>(expr)->getOperand(0), out);
                out << ":";
            } else if (LLVM_UNLIKELY(isa<While>(expr))) {
                out << "while ";
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

    out << "}\n\n";
    out.flush();





}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief summarizeAST
 *
 * This function scans through a basic block (starting by its terminals) and computes a DAG in which any sequences
 * of AND, OR or XOR functions are "flattened" and allowed to have any number of inputs. This allows us to
 * reassociate them in the most efficient way possible.
 ** ------------------------------------------------------------------------------------------------------------- */
static void summarizeAST(PabloBlock & block, std::vector<Statement *> && terminals, Graph & G) {

    Map M;
    VertexQueue Q(128);
    EdgeQueue E;

    for (;;) {

        Graph Gk;
        Map Mk;

        // Generate a graph depicting the relationships between the terminals. If the original terminals
        // cannot be optimized with this algorithm bypass them in favour of their operands. If those cannot
        // be optimized, they'll be left as the initial terminals for the next "layer" of the AST.

        for (Statement * term : terminals) {
            if (LLVM_LIKELY(Mk.count(term) == 0)) {
                // add or find this terminal in our global graph
                Vertex x = getVertex(term, G, M);
                if (inCurrentBlock(term, block)) {
                    if (isOptimizable(term)) {
                        const Vertex u = add_vertex(term, Gk);
                        Mk.insert(std::make_pair(term, u));
                        push(u, Q);
                        continue;
                    }
                } else if (isa<Assign>(term) || isa<Next>(term)) {
                    // If this is an Assign (Next) node whose operand does not originate from the current block
                    // then check to see if there is an If (While) node that does.
                    Statement * branch = nullptr;
                    if (isa<Assign>(term)) {
                        for (PabloAST * user : term->users()) {
                            if (isa<If>(user)) {
                                const If * ifNode = cast<If>(user);
                                if (inCurrentBlock(ifNode, block)) {
                                    const auto & defs = ifNode->getDefined();
                                    if (LLVM_LIKELY(std::find(defs.begin(), defs.end(), cast<Assign>(term)) != defs.end())) {
                                        branch = cast<Statement>(user);
                                        break;
                                    }
                                }
                            }
                        }
                    } else { // if (isa<Next>(term))
                        for (PabloAST * user : term->users()) {
                            if (isa<While>(user)) {
                                const While * whileNode = cast<While>(user);
                                if (inCurrentBlock(whileNode, block)) {
                                    const auto & vars = whileNode->getVariants();
                                    if (LLVM_LIKELY(std::find(vars.begin(), vars.end(), cast<Next>(term)) != vars.end())) {
                                        branch = cast<Statement>(user);
                                        break;
                                    }
                                }
                            }
                        }
                    }

                    // If we didn't find a branch, then the Assign (Next) node must have come from a preceeding
                    // block. Just skip it for now.
                    if (branch == nullptr) {
                        continue;
                    }

                    // Otherwise add the branch to G and test its operands rather than the original terminal
                    const Vertex z = getVertex(branch, G, M);
                    add_edge(z, x, G);
                    x = z;
                    term = branch;
                }

                for (unsigned i = 0; i != term->getNumOperands(); ++i) {
                    PabloAST * const op = term->getOperand(i);
                    if (LLVM_LIKELY(inCurrentBlock(op, block))) {
                        const Vertex y = getVertex(op, G, M);
                        add_edge(y, x, G);
                        if (LLVM_LIKELY(Mk.count(op) == 0)) {
                            const Vertex v = add_vertex(op, Gk);
                            Mk.insert(std::make_pair(op, v));
                            push(v, Q);
                        }
                    }
                }
            }
        }

        if (LLVM_UNLIKELY(Q.empty())) {
            break;
        }

        for (;;) {
            const Vertex u = pop(Q);
            if (isOptimizable(Gk[u])) {
                Statement * stmt = cast<Statement>(Gk[u]);
                if (isAnyUserNotInCurrentBlock(stmt, block)) {
                    const Vertex v = add_vertex(block.createZeroes(), Gk);
                    add_edge(u, v, Gk);
                }
                // Scan through the use-def chains to locate any chains of rearrangable expressions and their inputs
                for (unsigned i = 0; i != 2; ++i) {
                    PabloAST * op = stmt->getOperand(i);
                    auto f = Mk.find(op);
                    if (f == Mk.end()) {
                        const Vertex v = add_vertex(op, Gk);
                        f = Mk.insert(std::make_pair(op, v)).first;
                        if (op->getClassTypeId() == stmt->getClassTypeId() && inCurrentBlock(cast<Statement>(op), block)) {
                            push(v, Q);
                        }
                    }
                    add_edge(f->second, u, Gk);
                }
            }
            if (Q.empty()) {
                break;
            }
        }

        // Generate a topological ordering for G; if one of our terminals happens to also be a partial computation of
        // another terminal, we need to make sure we compute it as an independent subexpression.
        std::vector<unsigned> ordering;
        ordering.reserve(num_vertices(Gk));
        topological_sort(Gk, std::back_inserter(ordering));
        std::vector<unsigned> component(num_vertices(Gk));

        for (;;) {

            // Mark which computation component these vertices are in based on their topological (occurence) order.
            unsigned components = 0;
            for (auto u : ordering) {
                unsigned id = 0;
                // If this is a sink in G, it is the root of a new component.
                if (out_degree(u, Gk) == 0) {
                    id = ++components;
                } else {
                    for (auto e : make_iterator_range(out_edges(u, Gk))) {
                        id = std::max(id, component[target(e, Gk)]);
                    }
                }
                assert (id && "Topological ordering failed!");
                component[u] = id;
            }

            // Cut the graph wherever a computation crosses a component or whenever we need to cut the graph because
            // the instructions corresponding to the pair of nodes differs.
            graph_traits<Graph>::edge_iterator ei, ei_end;
            for (std::tie(ei, ei_end) = edges(Gk); ei != ei_end; ) {
                const Graph::edge_descriptor e = *ei++;
                const Vertex u = source(e, Gk);
                const Vertex v = target(e, Gk);
                if (LLVM_UNLIKELY(isCutNecessary(u, v, Gk, component))) {
                    E.push(std::make_pair(u, v));
                    remove_edge(u, v, Gk);
                }
            }

            // If no cuts are necessary, we're done.
            if (E.empty()) {
                break;
            }

            for (;;) {

                Vertex u, v;
                std::tie(u, v) = E.front(); E.pop();

                // The vertex belonging to a component with a greater number must come "earlier"
                // in the program. By replicating it, this ensures it's computed as an output of
                // one component and used as an input of another.

                if (component[u] < component[v]) {
                    std::swap(u, v);
                }

                // Replicate u and fix the ordering and component vectors to reflect the change in G.
                Vertex w = add_vertex(Gk[u], Gk);
                ordering.insert(std::find(ordering.begin(), ordering.end(), u), w);
                assert (component.size() == w);
                component.push_back(component[v]);
                add_edge(w, v, Gk);

                // However, after we do so, we need to make sure the original source vertex will be a
                // sink in G unless it is also an input variable (in which case we'd simply end up with
                // extraneous isolated vertex. Otherwise, we need to make further cuts and replications.

                if (in_degree(u, Gk) != 0) {
                    for (auto e : make_iterator_range(out_edges(u, Gk))) {
                        E.push(std::make_pair(source(e, Gk), target(e, Gk)));
                    }
                    clear_out_edges(u, Gk);
                }

                if (E.empty()) {
                    break;
                }
            }
        }

        // Scan through the graph so that we process the outermost expressions first
        for (const Vertex u : ordering) {
            if (LLVM_UNLIKELY(out_degree(u, Gk) == 0)) {
                if (LLVM_UNLIKELY(isa<Zeroes>(Gk[u]))) {
                    continue;
                }
                const Vertex x = getVertex(Gk[u], G, M);
                if (LLVM_LIKELY(in_degree(u, Gk) > 0)) {
                    flat_set<PabloAST *> vars;
                    flat_set<Vertex> visited;
                    for (Vertex v = u;;) {
                        if (in_degree(v, Gk) == 0) {
                            vars.insert(Gk[v]);
                        } else {
                            for (auto e : make_iterator_range(in_edges(v, Gk))) {
                                const Vertex w = source(e, Gk);
                                if (LLVM_LIKELY(visited.insert(w).second)) {
                                    push(w, Q);
                                }
                            }
                        }
                        if (Q.empty()) {
                            break;
                        }
                        v = pop(Q);
                    }
                    for (PabloAST * var : vars) {
                        add_edge(getVertex(var, G, M), x, G);
                    }
                }
            }
        }

        // Determine the source variables of the next "layer" of the AST
        flat_set<Statement *> nextSet;
        for (auto u : ordering) {
            if (LLVM_UNLIKELY(in_degree(u, Gk) == 0 && isa<Statement>(Gk[u]))) {
                nextSet.insert(cast<Statement>(Gk[u]));
            } else if (LLVM_UNLIKELY(out_degree(u, Gk) == 0 && isa<Statement>(Gk[u]))) { // an input may also be the output of a subgraph of G. We don't need to reevaluate it.
                nextSet.erase(cast<Statement>(Gk[u]));
            }
        }

        if (LLVM_UNLIKELY(nextSet.empty())) {
            break;
        }

        terminals.assign(nextSet.begin(), nextSet.end());
    }

}

BooleanReassociationPass::BooleanReassociationPass()
{

}


}
