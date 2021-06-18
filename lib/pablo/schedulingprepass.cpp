#include <pablo/schedulingprepass.h>

#include <pablo/pablo_kernel.h>
#include <pablo/codegenstate.h>
#include <pablo/ps_assign.h>
#include <pablo/pe_var.h>
#include <pablo/branch.h>
#ifndef NDEBUG
#include <pablo/pabloverifier.hpp>
#else
#include <llvm/Support/ErrorHandling.h>
#endif
#include <boost/container/flat_set.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <random>
#include <unordered_map>

#include <llvm/Support/raw_ostream.h>




using namespace llvm;
using namespace boost;
using namespace boost::container;

namespace pablo {

using IndexPair = std::pair<unsigned, Statement *>;
using Graph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::bidirectionalS>;
using Vertex = Graph::vertex_descriptor;
using Map = std::unordered_map<const PabloAST *, Vertex>;
using ReadyPair = std::pair<Vertex, size_t>;

struct SchedulingPrePassContainer {

    void run(PabloBlock * const block) {
        // traverse to the inner-most branch first
        for (Statement * stmt : *block) {
            if (isa<Branch>(stmt)) {
                run(cast<Branch>(stmt)->getBody());
            }
        }
        // starting with the inner-most scope(s) first, attempt to schedule the statements to
        // minimize the number of simulaneously live variables.
        schedule(block);
    }

protected:

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief schedule
     ** ------------------------------------------------------------------------------------------------------------- */
    void schedule(PabloBlock * const block) {
        build_data_dependency_dag(block);
        initialize_process();
        initialize_probabilities();

        unsigned count = 0;
        std::vector<size_t> W;
        W.reserve(num_vertices(G));
        for (;;) {
            ready_sort();
            const auto u = choose_ready(++count);
            queue_to_ready(u);
            if (LLVM_UNLIKELY(R.empty())) {
                break;
            }
            update_liveness(u);
            W.push_back(get_live_weight());
        }

        const auto m = W.size() / 2;
        std::nth_element(W.begin(), W.begin() + m, W.end());

        errs() << "median_live_weight: " << W[m] << "\n";



//        std::sort(I.begin(), I.end(), [](const IndexPair & A, const IndexPair & B){
//            return std::get<0>(A) < std::get<0>(B);
//        });

//        block->setInsertPoint(nullptr);
//        for (const auto & p : I) {
//            Statement * stmt = std::get<1>(p);
//            assert (std::get<0>(p) > 0);
//            assert (stmt->getParent() == block);
//            block->insert(stmt);
//        }
//        assert (block->getInsertPoint() == block->back());

        I.clear();
        G.clear();
        L.clear();
    }

    void build_data_dependency_dag(PabloBlock * const block) {
        assert (M.empty());
        for (Statement * stmt : *block) {
            const auto u = find(stmt);
            if (isa<Assign>(stmt)) {
                addDominatedUsers(u, cast<Assign>(stmt), block);
            } else if (isa<Branch>(stmt)) {
                addEscapedUsers(u, cast<Branch>(stmt), block);
            } else {
                addUsers(u, stmt, block);
            }
        }
        M.clear();
    }

    void addUsers(const Vertex u, Statement * const stmt, PabloBlock * const block) {
        for (PabloAST * use : stmt->users()) {
            if (LLVM_LIKELY(isa<Statement>(use))) {
                Statement * user = cast<Statement>(use);
                while (user->getParent() != block) {
                    PabloBlock * const parent = user->getParent();
                    assert (parent);
                    user = parent->getBranch();
                    assert (user);
                }
                assert (strictly_dominates(stmt, user));
                add_edge(u, find(user), G);
            }
        }
    }

    void addEscapedUsers(const Vertex u, Branch * const br, PabloBlock * const block) {
        for (Var * var : br->getEscaped()) {
            for (PabloAST * use : var->users()) {
                if (LLVM_LIKELY(isa<Statement>(use))) {
                    Statement * user = cast<Statement>(use);
                    for (;;) {
                        if (user->getParent() == block) {
                            if (LLVM_LIKELY(isa<Assign>(user) ? dominates(br, user) : (br != user))) {
                                add_edge(u, find(user), G);
                            }
                            break;
                        }
                        PabloBlock * const parent = user->getParent();
                        assert (parent);
                        user = parent->getBranch();
                        if (LLVM_UNLIKELY(user == nullptr)) {
                            break;
                        }
                    }
                }
            }
        }
    }

    void addDominatedUsers(const Vertex u, Assign * const assignment, PabloBlock * const block) {
        for (PabloAST * use : assignment->getVariable()->users()) {
            if (LLVM_LIKELY(isa<Statement>(use))) {
                Statement * user = cast<Statement>(use);
                for (;;) {
                    if (user->getParent() == block) {
                        if (user != assignment) {
                            const auto v = find(user);
                            if (strictly_dominates(assignment, user)) {
                                add_edge(u, v, G);
                            } else {
                                assert (strictly_dominates(user, assignment));
                                // although there is not actually a dependency here, a prior value
                                // must be read by statement v prior to assignment u
                                add_edge(v, u, G);
                            }
                        }
                        break;
                    }
                    PabloBlock * const parent = user->getParent();
                    assert (parent);
                    user = parent->getBranch();
                    if (LLVM_UNLIKELY(user == nullptr)) {
                        break;
                    }
                }
            }
        }
    }

    void printGraph(const std::string name, raw_ostream & out) {
        out << "\ndigraph " << name << " {\n";
        for (auto u : make_iterator_range(vertices(G))) {
            out << "v" << u << " [label=\"" << u << " : ";
            std::get<1>(I[u])->print(out);
            out << "  " << std::get<0>(I[u]) << "\"];\n";
        }
        for (auto e : make_iterator_range(edges(G))) {
            out << "v" << source(e, G) << " -> v" << target(e, G) << ";\n";
        }
        out << "}\n\n";
        out.flush();
    }

    void initialize_process() {
        assert ("Ready set was not cleared prior to initializing!" && R.empty());
        std::vector<Vertex> Q;
        Q.reserve(64);
        for (auto u : make_iterator_range(vertices(G))) {
            if (out_degree(u, G) == 0) {
                Q.push_back(u);
            } else if (in_degree(u, G) == 0) {
                R.push_back(u);
            }
        }

        // perform a transitive reduction of G to remove the unnecessary dependency analysis work
        // within the genetic algorithm search process

        const auto n = num_vertices(G);

        std::vector<flat_set<Vertex>> reachability(n);
        flat_set<Vertex> pending;

        for (;;) {
            const auto u = Q.back();
            Q.pop_back();

            flat_set<Vertex> & D = reachability[u];

            assert (D.empty());

            // initially we do not immediately consider any adjacent vertices reachable
            for (auto e : make_iterator_range(out_edges(u, G))) {
                const auto v = target(e, G);
                const auto & R = reachability[v];
                assert (R.count(v) == 0);
                D.insert(R.begin(), R.end());
            }

            // so that if one of our outgoing targets is already reachable, we can remove it.
            remove_out_edge_if(u, [&D, this](Graph::edge_descriptor e) {

                // TODO: if we remove this edge, add the liveness weight of statement u to the
                // target since the "liveness" of u is transitively implied.

                return D.count(target(e, G)) != 0;
            }, G);

            // afterwards we insert any remaining outgoing targets to finalize our reachable set
            for (auto e : make_iterator_range(out_edges(u, G))) {
                D.insert(target(e, G));
            }

            // add any incoming source to our pending set
            for (auto e : make_iterator_range(in_edges(u, G))) {
                pending.insert(source(e, G));
            }

            // so that when we exhaust the queue, we can repopulate it with the next 'layer' of the graph
            if (LLVM_UNLIKELY(Q.empty())) {
                if (LLVM_UNLIKELY(pending.empty())) {
                    break;
                }
                for (auto v : pending) {
                    bool ready = true;
                    for (auto e : make_iterator_range(out_edges(v, G))) {
                        const auto w = target(e, G);
                        if (LLVM_UNLIKELY(out_degree(w, G) != 0 && reachability[w].empty())) {
                            ready = false;
                            break;
                        }
                    }
                    if (ready) {
                        Q.push_back(v);
                    }
                }
                pending.clear();
                assert (!Q.empty());
            }

        }
    }

    void initialize_probabilities() {
        std::random_device rd;
        std::default_random_engine rand(rd());
        std::uniform_int_distribution<size_t> dist(std::numeric_limits<size_t>::min(), std::numeric_limits<size_t>::max());
        const auto n = num_vertices(G);
        P.resize(n);
        std::generate_n(P.begin(), n, [&dist, &rand](){ return dist(rand); });
    }

    void ready_sort() {
        std::sort(R.begin(), R.end(), [this](const Vertex u, const Vertex v){ return P[u] < P[v]; });
    }

    Vertex choose_ready(const unsigned index) {
        assert (!R.empty());
        const auto u = R.back();
        assert (std::get<0>(I[u]) == 0);
        std::get<0>(I[u]) = index;
        R.pop_back();
        return u;
    }

    void update_liveness(const Vertex u) {
        // determine which statements were killed (i.e., u was its last unscheduled user) or
        // killable (i.e., u is the penultimate unscheduled user)
        for (auto ei : make_iterator_range(in_edges(u, G))) {
            const auto v = source(ei, G);
            const auto f = L.find(v);
            if (f != L.end()) {
                bool killed = true;
                for (auto ej : make_iterator_range(out_edges(v, G))) {
                    if (std::get<0>(I[target(ej, G)]) == 0) {
                        killed = false;
                        break;
                    }
                }
                if (killed) {
                    L.erase(f);
                }
            }
        }
        L.insert(u);
    }

    size_t get_live_weight() {
        return L.size();
    }

    void queue_to_ready(const Vertex u) {
        // determine which statements are now ready (i.e., u was its last unscheduled input)
        for (auto ei : make_iterator_range(out_edges(u, G))) {
            const auto v = target(ei, G);
            bool ready = true;
            for (auto ej : make_iterator_range(in_edges(v, G))) {
                if (std::get<0>(I[source(ej, G)]) == 0) {
                    ready = false;
                    break;
                }
            }
            if (ready) {
                R.push_back(v);
            }
        }
    }



private:

    Vertex find(Statement * expr) {
        const auto f = M.find(expr);
        if (f == M.end()) {
            const auto u = add_vertex(G);
            assert (u == I.size());
            I.emplace_back(0, expr);
            M.emplace(expr, u);
            return u;
        } else {
            return f->second;
        }
    }

private:

    std::vector<Vertex> R;      // ready
    flat_set<Vertex> L;         // live variables
    std::vector<IndexPair> I;   // index
    std::vector<size_t> P;

    Graph G;
    Map M;
};


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief optimize
 ** ------------------------------------------------------------------------------------------------------------- */
bool SchedulingPrePass::optimize(PabloKernel * const kernel) {
    #ifdef NDEBUG
    llvm::report_fatal_error("DistributivePass is unsupported");
    #else
    SchedulingPrePassContainer S;
    S.run(kernel->getEntryScope());
    PabloVerifier::verify(kernel, "post-scheduling-prepass");
    return true;
    #endif
}


}
