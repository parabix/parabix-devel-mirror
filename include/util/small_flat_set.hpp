#ifndef SMALL_FLAT_SET_HPP
#define SMALL_FLAT_SET_HPP

#include <boost/container/flat_set.hpp>

#if BOOST_VERSION < 106600
// Versions of boost prior to 1.66 could not directly use the small_vector as a flat set's
// backing buffer. Fall back to the original flat_set.

template <typename T, unsigned /* N */>
using SmallFlatSet = boost::container::flat_set<T, std::less<T>>;

#else

#include <boost/container/small_vector.hpp>

template <typename T, unsigned N>
using SmallFlatSet = boost::container::flat_set<T, std::less<T>, boost::container::small_vector<T, N>>;

#endif

#endif // SMALL_FLAT_SET_HPP
