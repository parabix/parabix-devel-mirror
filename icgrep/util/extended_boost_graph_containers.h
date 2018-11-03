#ifndef EXTENDED_BOOST_GRAPH_CONTAINERS_H
#define EXTENDED_BOOST_GRAPH_CONTAINERS_H

#include <boost/graph/adjacency_list.hpp>
#include <boost/container/flat_set.hpp>

// sorted vector
struct svecS{};
// flat (ordered) set
struct flat_setS{};

namespace {

template<class T>
struct __sorted_edge_vector : public std::vector<T> {
    using iterator = typename std::vector<T>::iterator;
    void push_back(const T & item) {
        const auto p = std::upper_bound(std::vector<T>::begin(), std::vector<T>::end(), item);
        std::vector<T>::insert(p, item);
    }
};

}

namespace boost {

    template <class ValueType> struct container_gen<svecS, ValueType> {
        typedef __sorted_edge_vector<ValueType> type;
    };

    template <> struct parallel_edge_traits<svecS> {
        typedef allow_parallel_edge_tag type;
    };

    template<class T> graph_detail::vector_tag container_category(const __sorted_edge_vector<T>&) {
        return graph_detail::vector_tag();
    }

    template <class T> graph_detail::unstable_tag iterator_stability(const __sorted_edge_vector<T>&) {
        return graph_detail::unstable_tag();
    }

    template <class T> struct container_gen<flat_setS, T> {
        typedef boost::container::flat_set<T>  type;
    };

    template <> struct parallel_edge_traits<flat_setS> {
        typedef disallow_parallel_edge_tag type;
    };

}

#endif // EXTENDED_BOOST_GRAPH_CONTAINERS_H
