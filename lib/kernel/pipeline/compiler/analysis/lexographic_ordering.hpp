#ifndef LEXOGRAPHIC_ORDERING_HPP
#define LEXOGRAPHIC_ORDERING_HPP

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/dynamic_bitset.hpp>
#include <llvm/Support/ErrorHandling.h>
#include <llvm/ADT/SmallVector.h>
#include <llvm/ADT/BitVector.h>
#include <vector>
#include <queue>
#include <numeric>

using namespace boost;
using namespace llvm;

template <typename Graph, typename Vec>
bool lexical_ordering(const Graph & G, Vec & L) {

    using Vertex = typename graph_traits<Graph>::vertex_descriptor;
    using Queue = std::priority_queue<Vertex, std::vector<Vertex>, std::greater<Vertex>>;

    const auto count = num_vertices(G);

    SmallVector<unsigned, 4> isolated;
    SmallVector<unsigned, 256> encountered(count, 0);
    L.reserve(count);
    Queue Q;

    for (unsigned i = 0; i < count; ++i) {
        if (LLVM_UNLIKELY(in_degree(i, G) == 0)) {
            if (LLVM_LIKELY(out_degree(i, G) != 0)) {
                Q.push(i);
            } else {
                isolated.push_back(i);
            }
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

    // leave all isolated vertices at the end of the list
    L.insert(L.end(), isolated.begin(), isolated.end());
    if (LLVM_UNLIKELY(L.size() != count)) {
        return false;
    }
    const size_t traversed = std::accumulate(encountered.begin(), encountered.end(), 0);
    return (traversed == num_edges(G));
}

using ReverseTopologicalOrdering = SmallVector<unsigned, 256>;

template <typename Vector, typename Graph>
void transitive_closure_dag(const Vector & ordering, Graph & G) {
    // Simple transitive closure for DAGs
    for (unsigned u : ordering) {
        for (const auto e : make_iterator_range(in_edges(u, G))) {
            const auto s = source(e, G);
            for (const auto f : make_iterator_range(out_edges(u, G))) {
                const auto t = target(f, G);
                if (edge(s, t, G).second) continue;
                typename Graph::edge_property_type p{};
                add_edge(s, t, p, G);
            }
        }
    }
}

template <typename Vector, typename Graph>
void transitive_reduction_dag(const Vector & ordering, Graph & G) {
    using Edge = typename graph_traits<Graph>::edge_descriptor;
    BitVector sources(num_vertices(G), false);
    for (unsigned u : ordering ) {
        for (auto e : make_iterator_range(in_edges(u, G))) {
            sources.set(source(e, G));
        }
        for (auto e : make_iterator_range(out_edges(u, G))) {
            remove_in_edge_if(target(e, G), [&G, &sources](const Edge & f) {
                return sources.test(source(f, G));
            }, G);
        }
        sources.reset();
    }
}

template <typename Graph>
inline void transitive_closure_dag(Graph & G) {
    ReverseTopologicalOrdering ordering;
    ordering.reserve(num_vertices(G));
    topological_sort(G, std::back_inserter(ordering));
    transitive_closure_dag(ordering, G);
}

template <typename Graph>
inline void transitive_reduction_dag(Graph & G) {
    ReverseTopologicalOrdering ordering;
    ordering.reserve(num_vertices(G));
    topological_sort(G, std::back_inserter(ordering));
    transitive_closure_dag(ordering, G);
    transitive_reduction_dag(ordering, G);
}

#endif // LEXOGRAPHIC_ORDERING_HPP
