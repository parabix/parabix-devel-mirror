#ifndef LEXOGRAPHIC_ORDERING_HPP
#define LEXOGRAPHIC_ORDERING_HPP

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/dynamic_bitset.hpp>
#include <llvm/Support/ErrorHandling.h>
#include <llvm/ADT/SmallVector.h>
#include <vector>
#include <queue>
#include <numeric>

using namespace boost;
using namespace llvm;

template <typename Graph, typename Vec>
void lexical_ordering(const Graph & G, Vec & L, const llvm::Twine error) {

    using Vertex = typename graph_traits<Graph>::vertex_descriptor;
    using Queue = std::priority_queue<Vertex, std::vector<Vertex>, std::greater<Vertex>>;

    const auto count = num_vertices(G);

    std::vector<unsigned> encountered(count, 0);
    L.reserve(count);
    Queue Q;

    for (unsigned i = 0; i < count; ++i) {
        if (LLVM_LIKELY(in_degree(i, G) == 0)) {
            Q.push(i);
        }
    }

    while (!Q.empty()) {
        const auto n = Q.top();
        Q.pop();
        L.push_back(n);
        for (auto e : make_iterator_range(out_edges(n, G))) {
            const auto m = target(e, G);
            assert (encountered[m] < in_degree(m, G));
            encountered[m]++;
            if (in_degree(m, G) == encountered[m]) {
                Q.push(m);
            }
        }
    }

    const auto traversed = std::accumulate(encountered.begin(), encountered.end(), 0);
    if (LLVM_UNLIKELY(traversed != num_edges(G))) {
        llvm::report_fatal_error(error);
    }
    assert ("not all vertices were reached?" && L.size() == count);
}

template <typename Graph>
inline void transitive_closure_dag(Graph & G) {
    // Simple topological closure for DAGs
    SmallVector<unsigned, 256> ordering;
    ordering.reserve(num_vertices(G));
    topological_sort(G, std::back_inserter(ordering));
    for (unsigned u : ordering) {
        for (auto e : make_iterator_range(in_edges(u, G))) {
            const auto s = source(e, G);
            for (auto f : make_iterator_range(out_edges(u, G))) {
                add_edge(s, target(f, G), G);
            }
        }
    }
}

template <typename Graph>
bool add_edge_if_no_induced_cycle(const typename graph_traits<Graph>::vertex_descriptor s,
                                  const typename graph_traits<Graph>::vertex_descriptor t,
                                  Graph & G) {
    // If s-t exists, skip adding this edge
    if (edge(s, t, G).second || s == t) {
        return true;
    }
    // If G is a DAG and there is a t-s path, adding s-t will induce a cycle.
    const auto d = in_degree(s, G);
    if (d != 0) {
        dynamic_bitset<> V(num_vertices(G));
        std::queue<typename graph_traits<Graph>::vertex_descriptor> Q;
        // do a BFS to search for a t-s path
        Q.push(t);
        for (;;) {
            const auto u = Q.front();
            Q.pop();
            for (auto e : make_iterator_range(out_edges(u, G))) {
                const auto v = target(e, G);
                if (LLVM_UNLIKELY(v == s)) {
                    // we found a t-s path
                    return false;
                }
                assert ("G was not initially acyclic!" && v != s);
                if (LLVM_LIKELY(V.test(v))) {
                    V.set(v);
                    Q.push(v);
                }
            }
            if (Q.empty()) {
                break;
            }
        }
    }
    add_edge(s, t, G);
    return true;
}

#endif // LEXOGRAPHIC_ORDERING_HPP
