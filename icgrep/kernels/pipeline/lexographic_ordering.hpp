#ifndef LEXOGRAPHIC_ORDERING_HPP
#define LEXOGRAPHIC_ORDERING_HPP

#include <boost/graph/adjacency_list.hpp>
#include <llvm/Support/ErrorHandling.h>
#include <vector>
#include <queue>
#include <iostream>

template <typename Graph, typename Vec>
inline void lexicalOrdering(Graph && G, Vec & L, const llvm::Twine error) {

    using Vertex = typename boost::graph_traits<Graph>::vertex_descriptor;
    using Queue = std::priority_queue<Vertex, std::vector<Vertex>, std::greater<Vertex>>;

    // Use a Kahn's algorithm with a priority queue to get a lexographic ordering.
    const auto count = boost::num_vertices(G);
    L.reserve(count);
    Queue Q;
    // build our initial set
    for (unsigned i = 0; i < count; ++i) {
        if (LLVM_LIKELY(in_degree(i, G) == 0)) {
            Q.push(i);
        }
    }

    while (!Q.empty()) {
        const auto n = Q.top(); Q.pop();
        L.push_back(n);
        for (auto e : boost::make_iterator_range(out_edges(n, G))) {
            const auto m = boost::target(e, G);
            if (boost::in_degree(m, G) == 1) {
                Q.push(m);
            }
        }
        boost::clear_out_edges(n, G);
    }
    if (LLVM_UNLIKELY(boost::num_edges(G) != 0)) {
        llvm::report_fatal_error(error);
    }
}

#endif // LEXOGRAPHIC_ORDERING_HPP
