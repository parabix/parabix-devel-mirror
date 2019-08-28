/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

/*
    Multicharacter sequence matching using compile-time constants.

    Supports sequences upto, and including, 8 characters in length.

    Ported from old parabix2 source, reworked to use template specialization
    instead of multiple class types for different sequence lengths.
 */

#pragma once

#include <cinttypes>
#include <limits>
#include <type_traits>
#include <util/lookup_table.hpp>

#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
#define LOW_BYTE_SHIFT 8
#define HIGH_BYTE_SHIFT 0
#else
#define LOW_BYTE_SHIFT 0
#define HIGH_BYTE_SHIFT 8
#endif

using char_t = uint8_t;

template<char_t... Cs>
struct multiliteral {};

template<char_t C1>
struct multiliteral<C1> {
    using value_type = uint8_t;
    static const value_type value = C1;
    static const value_type mask = std::numeric_limits<value_type>::max();
};

template<char_t C1, char_t C2>
struct multiliteral<C1, C2> {
    using value_type = uint16_t;
    static const value_type value = ((value_type) C1 << LOW_BYTE_SHIFT) 
                                  | ((value_type) C2 << HIGH_BYTE_SHIFT);
    static const value_type mask = std::numeric_limits<value_type>::max();
};

template<char_t C1, char_t C2, char_t C3, char_t C4>
struct multiliteral<C1, C2, C3, C4> {
    using value_type = uint32_t;
    static const value_type value = ((value_type) multiliteral<C1, C2>::value << 2 * LOW_BYTE_SHIFT)
                                  | ((value_type) multiliteral<C3, C4>::value << 2 * HIGH_BYTE_SHIFT);
    static const value_type mask = std::numeric_limits<value_type>::max();
};

template<char_t C1, char_t C2, char_t C3>
struct multiliteral<C1, C2, C3> {
    using super_t = multiliteral<C1, C2, C3, 0>;
    using value_type = typename super_t::value_type;
    static const value_type value = super_t::value;
    static const value_type mask = 0xFFFFFF << LOW_BYTE_SHIFT;
};

template<char_t C1, char_t C2, char_t C3, char_t C4, char_t C5, char_t C6, char_t C7, char_t C8>
struct multiliteral<C1, C2, C3, C4, C5, C6, C7, C8> {
    using value_type = uint64_t;
    static const value_type value = ((value_type) multiliteral<C1, C2, C3, C4>::value << 4 * LOW_BYTE_SHIFT)
                                  | ((value_type) multiliteral<C5, C6, C7, C8>::value << 4 * HIGH_BYTE_SHIFT);
    static const value_type mask = std::numeric_limits<value_type>::max();
};

template<char_t C1, char_t C2, char_t C3, char_t C4, char_t C5>
struct multiliteral<C1, C2, C3, C4, C5> {
    using super_t = multiliteral<C1, C2, C3, C4, C5, 0, 0, 0>;
    using value_type = typename super_t::value_type;
    static const value_type value = super_t::value;
    static const value_type mask = 0xFFFFFFFFFFULL << (3 * LOW_BYTE_SHIFT);
};

template<char_t C1, char_t C2, char_t C3, char_t C4, char_t C5, char_t C6>
struct multiliteral<C1, C2, C3, C4, C5, C6> {
    using super_t = multiliteral<C1, C2, C3, C4, C5, C6, 0, 0>;
    using value_type = typename super_t::value_type;
    static const value_type value = super_t::value;
    static const value_type mask = 0xFFFFFFFFFFFFULL << (2 * LOW_BYTE_SHIFT);
};

template<char_t C1, char_t C2, char_t C3, char_t C4, char_t C5, char_t C6, char_t C7>
struct multiliteral<C1, C2, C3, C4, C5, C6, C7> {
    using super_t = multiliteral<C1, C2, C3, C4, C5, C6, C7, 0>;
    using value_type = typename super_t::value_type;
    static const value_type value = super_t::value;
    static const value_type mask = 0xFFFFFFFFFFFFFFULL << LOW_BYTE_SHIFT;
};

constexpr auto ml_to_upper_table = lut::char_table(lut::to_upper);
constexpr auto ml_to_lower_table = lut::char_table(lut::to_lower);

template<typename M>
struct caseless_multiliteral {};

template<char_t... Cs>
struct caseless_multiliteral<multiliteral<Cs...>> {
    using ml_t = multiliteral<Cs...>;
    using upper_ml_t = multiliteral<ml_to_upper_table[Cs]...>;
    using lower_ml_t = multiliteral<ml_to_lower_table[Cs]...>;
    using value_type = typename ml_t::value_type;
    
    static const value_type lower = lower_ml_t::value;
    static const value_type upper = upper_ml_t::value;
    
    static const value_type case_mask = lower ^ upper;
    static const value_type cannon = lower & ~case_mask;
};

/**
 * Returns `true` if a given `MultiLiteral` sequence matches the first `N`
 * characters in `str`. Where `N` is the length of the multiliteral.
 * 
 * Examples:
 * 
 *      using hello_ml = multiliteral<'h', 'e', 'l', 'l', 'o'>;
 *      bool b = matches<hello_ml>("hello world");
 *      // b == true
 */
template<typename MultiLiteral>
inline bool matches(const char_t * str) {
    auto const numeral = *reinterpret_cast<typename MultiLiteral::value_type const *>(str) & MultiLiteral::mask;
    return MultiLiteral::value == numeral;
}

/**
 * Returns `true` if a given `MultiLiteral` sequence matches the first `N`
 * characters in `str` not accounting for case. Where `N` is the length of
 * the multiliteral.
 * 
 * Examples:
 * 
 *      using hello_ml = multiliteral<'h', 'e', 'l', 'l', 'o'>;
 *      bool b = matches<hello_ml>("HELLO WORLD");
 *      // b == true
 */
template<typename MultiLiteral>
inline bool caseless_matches(const char_t * str) {
    using caseless_ml_t = caseless_multiliteral<MultiLiteral>;
    auto const numeral = *reinterpret_cast<typename MultiLiteral::value_type const *>(str) & MultiLiteral::mask;
    return caseless_ml_t::cannon == (numeral & ~caseless_ml_t::case_mask);
}
