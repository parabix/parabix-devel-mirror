/*
 * Copyright (c) 2019 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

// Calls a function pointer using values stored in a `std::tuple` for parameters.

#pragma once

#include <cinttypes>
#include <type_traits>
#include <util/integer_sequence.hpp>

namespace meta {

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
