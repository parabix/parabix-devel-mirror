/*
 * Copyright (c) 2019 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

#pragma once

#include <numeric>
#include <vector>

namespace meta {

/**
 * Fills a `Container` with `count` items using `std::iota` and an `initial_val`.
 * 
 * Examples:
 * 
 *  Fill a `std::vector` with 100 integers starting at 0:
 * 
 *      auto vec = meta::iota_fill<int>(100, 0);
 *      // vec: 0, 1, 2, 3, ..., 98, 99
 * 
 *  Using a difference container:
 * 
 *      std::list<int> l = meta::iota_fill<int, std::list>(100, 0);
 */
template<typename T, template<typename, typename...> class Container = std::vector>
inline Container<T> iota_fill(std::size_t count, T initial_val) {
    Container<T> v(count);
    std::iota(v.begin(), v.end(), initial_val);
    return v;
}

}
