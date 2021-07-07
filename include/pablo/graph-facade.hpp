#ifndef GRAPHFACADE_HPP
#define GRAPHFACADE_HPP

#include <boost/graph/adjacency_list.hpp>
#include <vector>

template <class Graph, bool transposed = false>
struct DirectedGraphFacade;

template <class Graph>
struct DirectedGraphFacade<Graph, false> {

    using Vertex = typename boost::graph_traits<Graph>::vertex_descriptor;
    using VertexIterator = typename boost::graph_traits<Graph>::vertex_iterator;
    using Degree = typename boost::graph_traits<Graph>::degree_size_type;
    using Edge = typename boost::graph_traits<Graph>::edge_descriptor;
    using InEdgeIterator = typename boost::graph_traits<Graph>::in_edge_iterator;
    using OutEdgeIterator = typename boost::graph_traits<Graph>::out_edge_iterator;

    DirectedGraphFacade(Graph & G) : mG(G) {}
    inline Degree in_degree(const Vertex u) const {
        return boost::in_degree(u, mG);
    }
    inline Degree out_degree(const Vertex u) const {
        return boost::out_degree(u, mG);
    }
    inline std::pair<VertexIterator, VertexIterator> vertices() const {
        return boost::vertices(mG);
    }
    inline std::pair<InEdgeIterator, InEdgeIterator> in_edges(const Vertex u) const {
        return boost::in_edges(u, mG);
    }
    inline std::pair<OutEdgeIterator, OutEdgeIterator> out_edges(const Vertex u) const {
        return boost::out_edges(u, mG);
    }
    inline void add_edge(const Vertex u, const Vertex v) {
        boost::add_edge(u, v, mG);
    }
    inline void remove_edge(const Vertex u, const Vertex v) {
        boost::remove_edge(u, v, mG);
    }
    inline void clear_in_edges(const Vertex u) {
        boost::clear_in_edges(u, mG);
    }
    inline void clear_out_edges(const Vertex u) {
        boost::clear_out_edges(u, mG);
    }
    inline Vertex source(const Edge & e) const {
        return boost::source(e, mG);
    }
    inline Vertex target(const Edge & e) const {
        return boost::target(e, mG);
    }
private:
    Graph & mG;
};

template <class Graph>
struct DirectedGraphFacade<Graph, true> {

    using Vertex = typename boost::graph_traits<Graph>::vertex_descriptor;
    using VertexIterator = typename boost::graph_traits<Graph>::vertex_iterator;
    using Degree = typename boost::graph_traits<Graph>::degree_size_type;
    using Edge = typename boost::graph_traits<Graph>::edge_descriptor;
    using InEdgeIterator = typename boost::graph_traits<Graph>::out_edge_iterator;
    using OutEdgeIterator = typename boost::graph_traits<Graph>::in_edge_iterator;

    DirectedGraphFacade(Graph & G) : mG(G) {}
    inline Degree in_degree(const Vertex u) const {
        return boost::out_degree(u, mG);
    }
    inline Degree out_degree(const Vertex u) const {
        return boost::in_degree(u, mG);
    }
    inline std::pair<VertexIterator, VertexIterator> vertices() const {
        return boost::vertices(mG);
    }
    inline std::pair<InEdgeIterator, InEdgeIterator> in_edges(const Vertex u) const {
        return boost::out_edges(u, mG);
    }
    inline std::pair<OutEdgeIterator, OutEdgeIterator> out_edges(const Vertex u) const {
        return boost::in_edges(u, mG);
    }
    inline void add_edge(const Vertex u, const Vertex v) {
        boost::add_edge(v, u, mG);
    }
    inline void remove_edge(const Vertex u, const Vertex v) {
        boost::remove_edge(v, u, mG);
    }
    inline void clear_in_edges(const Vertex u) {
        boost::clear_out_edges(u, mG);
    }
    inline void clear_out_edges(const Vertex u) {
        boost::clear_in_edges(u, mG);
    }
    inline Vertex source(const Edge & e) const {
        return boost::target(e, mG);
    }
    inline Vertex target(const Edge & e) const {
        return boost::source(e, mG);
    }
private:
    Graph & mG;
};

template<class Graph>
DirectedGraphFacade<Graph, false> makeGraphFacade(Graph & G) {
    return DirectedGraphFacade<Graph, false>(G);
}

template<class Graph>
DirectedGraphFacade<Graph, true> makeTransposedGraphFacade(Graph & G) {
    return DirectedGraphFacade<Graph, true>(G);
}

#endif // GRAPHFACADE_HPP
