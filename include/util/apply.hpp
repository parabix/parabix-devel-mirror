/*
 * Copyright (c) 2019 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

// Rudimentary implementation of C++14's std::integer_sequence and C++17's std::apply in C++11.

#pragma once

#include <cinttypes>
#include <type_traits>

namespace meta {

template<typename T, T... Ints>
struct integer_sequence {
    static constexpr std::size_t size() noexcept { return sizeof...(Ints); }
};

template<std::size_t... Ints>
using index_sequence = integer_sequence<std::size_t, Ints...>;

template<typename T, typename C, T... Ints>
struct __sequence_generator {};

template<typename T, T N, T... Ints>
struct __sequence_generator<T, std::integral_constant<T, N>, Ints...>
    : __sequence_generator<T, std::integral_constant<T, N - 1>, N - 1, Ints...>
{};

template<typename T, T... Ints>
struct __sequence_generator<T, std::integral_constant<T, 0>, Ints...> {
    using type = integer_sequence<T, Ints...>;
};

template<typename T, T N>
using make_integer_sequence = typename __sequence_generator<T, std::integral_constant<T, N>>::type;

template<std::size_t N>
using make_index_sequence = make_integer_sequence<std::size_t, N>;

template<typename... Ts>
using index_sequence_for = make_index_sequence<sizeof...(Ts)>;


template<
    typename Rt,
    typename... Params,
    template<typename...> class Tuple,
    std::size_t... I
>
inline Rt apply_impl(Rt(*fn)(Params...), Tuple<Params...> && tuple, index_sequence<I...> seq) {
    return fn(std::get<I>(std::forward<Tuple<Params...>>(tuple))...);
}

template<typename Rt, typename... Params, template<typename...> class Tuple>
inline Rt apply(Rt(*fn)(Params...), Tuple<Params...> && tuple) {
    return apply_impl(fn, std::forward<Tuple<Params...>>(tuple), index_sequence_for<Params...>());
}

}
