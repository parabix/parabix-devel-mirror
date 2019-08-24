/*
 * Copyright (c) 2019 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

#pragma once

namespace meta {

/// Concats two typelists together.
template<template<typename...> class List, typename T, typename U>
struct typelist_cat {};

template<
    template<typename...> class List,
    typename... LHS,
    typename... RHS
>
struct typelist_cat<List, List<LHS...>, List<RHS...>> {
    using type = List<LHS..., RHS...>;
};

template<template<typename...> class List, typename T, typename U>
using typelist_cat_t = typename typelist_cat<List, T, U>::type;

/// Prepends a type to the beinging of a typelist.
template<template<typename...> class List, typename T, typename U>
struct typelist_prepend {};

template<
    template<typename...> class List,
    typename T,
    typename... RHS
>
struct typelist_prepend<List, T, List<RHS...>> {
    using type = List<T, RHS...>;
};

template<template<typename...> class List, typename T, typename U>
using typelist_prepend_t = typename typelist_prepend<List, T, U>::type;

/// Appends a type to the end of a typelist.
template<template<typename...> class List, typename T, typename U>
struct typelist_append {};

template<
    template<typename...> class List,
    typename... LHS,
    typename T
>
struct typelist_append<List, List<LHS...>, T> {
    using type = List<LHS..., T>;
};

template<template<typename...> class List, typename T, typename U>
using typelist_append_t = typename typelist_append<List, T, U>::type;

/// Swaps a type list container with another.
template<class From>
struct swap_container {};

template<template<typename...> class From, typename... Ts>
struct swap_container<From<Ts...>> {
    template<template<typename...> class To>
    using type = To<Ts...>;
};

template<typename From, template<typename...> class To>
using swap_container_t = typename swap_container<From>::template type<To>;

}
