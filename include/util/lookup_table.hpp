/*
 * Copyright (c) 2019 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

#pragma once

#include <limits>
#include <type_traits>
#include <util/integer_sequence.hpp>

namespace lut {

template<typename ItemType, std::size_t N>
struct lookup_table {
    ItemType table[N];
    constexpr const ItemType & operator [] (size_t i) const { return table[i]; }
};

/// Lookup table generator typealias.
template<typename ItemType, typename IndexType>
using lut_generator_t = ItemType(*)(IndexType);

/// The size of a lookup table for a given `IndexType`.
template<typename IndexType>
struct lut_size {
    static const std::size_t value = std::numeric_limits<IndexType>::max();
};

template<typename ItemType, typename IndexType, IndexType... Ints>
constexpr
lookup_table<ItemType, sizeof...(Ints)>
__generate_lookup_table(lut_generator_t<ItemType, IndexType> generator, meta::integer_sequence<IndexType, Ints...> seq) {
    return lookup_table<ItemType, sizeof...(Ints)>({generator(Ints)...});
}

/// Constructs a lookup table mapping values of `IndexType` to values of `ItemType`.
template<typename ItemType, typename IndexType>
constexpr
lookup_table<ItemType, lut_size<IndexType>::value> 
make_lookup_table(lut_generator_t<ItemType, IndexType> generator) {
    return __generate_lookup_table<ItemType, IndexType>(
        generator,
        meta::make_integer_sequence<IndexType, lut_size<IndexType>::value>{}
    );
}

/// Character lookup table - maps chars to values of type `T`.
template<typename T>
constexpr lookup_table<T, lut_size<char>::value> char_table(lut_generator_t<T, char> generator) {
    return make_lookup_table<T, char>(generator);
}

/// Compile-time character range.
template<char LB, char UB>
struct char_range {
    /// Returns true if `c` is within the inclusive range [LB, UB]
    constexpr static bool contains(char c) {
        return (c >= LB) && (c <= UB);
    }
};

/// Whitespace table generator. Whitespace characters map to true, others to false.
constexpr bool is_whitespace(char c) {
    return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

/// Hex digit table generator. Hex digits map to true, others to false.
constexpr bool is_hex_digit(char c) {
    return char_range<'0', '9'>::contains(c)
        || char_range<'a', 'f'>::contains(c)
        || char_range<'A', 'F'>::contains(c);
}

/// Decimal digit table generator. Decimal digits map to true, others to false.
constexpr bool is_dec_digit(char c) {
    return char_range<'0', '9'>::contains(c);
}

/// Hex digit value table generator.
/// Maps a hex digit character to its integer value or 0xff if it's not a hex digit.
constexpr uint8_t hex_digit_val(char c) {
    using digit = char_range<'0', '9'>;
    using lc = char_range<'a', 'f'>;
    using uc = char_range<'A', 'F'>;
    return digit::contains(c) ? c - '0' 
         : (lc::contains(c) ? (c - 'a') + 10
          : (uc::contains(c) ? (c - 'A') + 10
           : 0xff));
}

/// Decimal digit value table generator.
/// Maps a decimal digit character to its integer value or 0xff if it's not a
/// valid digit.
constexpr uint8_t dec_digit_val(char c) {
    return char_range<'0', '9'>::contains(c) ? c - '0' : 0xff;
}

/// Converts uppercase characters to lowercase.
constexpr uint8_t to_lower(char c) {
    return char_range<'A', 'F'>::contains(c) ? c + 0x20 : c;
}

/// Converts lowercase characters to uppercase.
constexpr uint8_t to_upper(char c) {
    return char_range<'a', 'f'>::contains(c) ? c - 0x20 : c;
}

} // namespace lut
