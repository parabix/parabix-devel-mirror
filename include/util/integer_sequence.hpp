/*
 * Copyright (c) 2019 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

// C++11 compatable version of integer_sequence.

#pragma once

#include <cinttypes>
#include <type_traits>
#include <utility>

namespace meta {

#if CXX_STANDARD >= 14
// use std::integer_sequence if it is available

template<typename T, T... Ints>
using integer_sequence = std::integer_sequence<T, Ints...>;

template<std::size_t... Ints>
using index_sequence = std::index_sequence<Ints...>;

template<typename T, std::size_t N>
using make_integer_sequence = std::make_integer_sequence<T, N>;

template<std::size_t N>
using make_index_sequence = std::make_index_sequence<N>;

template<typename... T>
using index_sequence_for = std::index_sequence_for<T...>;

#else
// otherwise use custom implementation

// note: this implemenation fails to compile using C++17

template<typename T, T... Ints>
struct integer_sequence {
    static constexpr std::size_t size() noexcept { return sizeof...(Ints); }
};

template<std::size_t... Ints>
using index_sequence = integer_sequence<std::size_t, Ints...>;

template<typename T, uint64_t N, T... Ints>
struct __sequence_generator : __sequence_generator<T, N - 1, static_cast<T>(N) - 1, Ints...> {};

template<typename T, T... Ints>
struct __sequence_generator<T, 0, Ints...> {
    using type = integer_sequence<T, Ints...>;
};

template<typename T, uint64_t N>
using make_integer_sequence = typename __sequence_generator<T, N>::type;

template<uint64_t N>
using make_index_sequence = make_integer_sequence<std::size_t, N>;

template<typename... Ts>
using index_sequence_for = make_index_sequence<sizeof...(Ts)>;

#endif

}
