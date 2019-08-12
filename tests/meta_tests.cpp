/*
 * Copyright (c) 2019 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

// Tests for the testing framework

#include <testing/testing.h>

using namespace testing;

auto simple_eq_i = HexStream("0123 4567 89ab cdef");
auto simple_eq_e = HexStream("0123 4567 89ab cdef");

TEST_CASE(simple_eq, simple_eq_i, simple_eq_e) {
    AssertEQ(T, Input, Expected);
}


auto simple_ne_i = HexStream("0123 4567 89ab cdef");
auto simple_ne_e = HexStream("0123 4567 89ab cdee");

TEST_CASE(simple_ne, simple_ne_i, simple_ne_e) {
    AssertNE(T, Input, Expected);
}


auto hex_binary_eq_i = HexStream("0123 4567 89ab cdef");
auto hex_binary_eq_e = BinaryStream(
    ".... ...1 ..1. ..11"
    ".1.. .1.1 .11. .111"
    "1... 1..1 1.1. 1.11"
    "11.. 11.1 111. 1111");

TEST_CASE(hex_binary_equivalence, hex_binary_eq_i, hex_binary_eq_e) {
    AssertEQ(T, Input, Expected);
}


auto dot_zero_eq_i = BinaryStream("1... .... .... ...1");
auto dot_zero_eq_e = BinaryStream("1000 0000 0000 0001");

TEST_CASE(dot_zero_equivalence, dot_zero_eq_i, dot_zero_eq_e) {
    AssertEQ(T, Input, Expected);
}


auto len_mismatch_i = BinaryStream("1111 1111");
auto len_mismatch_e = BinaryStream("1111 111");

TEST_CASE(len_mismatch, len_mismatch_i, len_mismatch_e) {
    AssertNE(T, Input, Expected);
}


auto len_mismatch_int_i = IntStream<int64_t>({1, 2, 3, 4, 5, 6, 7, 8});
auto len_mismatch_int_e = IntStream<int64_t>({1, 2, 3, 4, 5});

TEST_CASE(len_mismatch_int, len_mismatch_int_i, len_mismatch_int_e) {
    AssertNE(T, Input, Expected);
}


auto len_mismatch_int_block_boundary_i = IntStream<int64_t>({1, 2, 3, 4, 5});
auto len_mismatch_int_block_boundary_e = IntStream<int64_t>({1, 2, 3, 4});

TEST_CASE(len_mismatch_int_block_boundary, len_mismatch_int_block_boundary_i, len_mismatch_int_block_boundary_e) {
    AssertNE(T, Input, Expected);
}


auto commutative_ne_a = IntStream<int64_t>({1, 2, 3, 4, 5});
auto commutative_ne_b = IntStream<int64_t>({1, 2, 3, 4});

TEST_CASE(commutative_ne, commutative_ne_a, commutative_ne_b) {
    AssertNE(T, Input, Expected);
    AssertNE(T, Expected, Input);
}


RUN_TESTS(
    CASE(simple_eq),
    CASE(simple_ne),
    CASE(hex_binary_equivalence),
    CASE(dot_zero_equivalence),
    CASE(len_mismatch),
    CASE(len_mismatch_int),
    CASE(len_mismatch_int_block_boundary),
    CASE(commutative_ne)
)
