/*
 * Copyright (c) 2019 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

// Tests for the testing framework

#include <testing/testing.h>
#include <kernel/streamutils/stream_select.h>
#include <kernel/util/debug_display.h>

using namespace testing;
using namespace kernel;
namespace su = kernel::streamutils;

auto simple_eq_i = HexStream("0123 4567 89ab cdef");
auto simple_eq_e = HexStream("0123 4567 89ab cdef");

TEST_CASE(simple_eq, simple_eq_i, simple_eq_e) {
    AssertEQ(T, Input<0>(T), Input<1>(T));
}


auto simple_ne_i = HexStream("0123 4567 89ab cdef");
auto simple_ne_e = HexStream("0123 4567 89ab cdee");

TEST_CASE(simple_ne, simple_ne_i, simple_ne_e) {
    AssertNE(T, Input<0>(T), Input<1>(T));
}


auto long_hex_ne_i = HexStream("1{200}");
auto long_hex_ne_e = HexStream("1{199}2");

TEST_CASE(long_hex_ne, long_hex_ne_i, long_hex_ne_e) {
    AssertNE(T, Input<0>(T), Input<1>(T));
}


auto hex_binary_eq_i = HexStream("0123 4567 89ab cdef");
auto hex_binary_eq_e = BinaryStream(
    ".... ...1 ..1. ..11"
    ".1.. .1.1 .11. .111"
    "1... 1..1 1.1. 1.11"
    "11.. 11.1 111. 1111");

TEST_CASE(hex_binary_equivalence, hex_binary_eq_i, hex_binary_eq_e) {
    AssertEQ(T, Input<0>(T), Input<1>(T));
}


auto dot_zero_eq_i = BinaryStream("1... .... .... ...1");
auto dot_zero_eq_e = BinaryStream("1000 0000 0000 0001");

TEST_CASE(dot_zero_equivalence, dot_zero_eq_i, dot_zero_eq_e) {
    AssertEQ(T, Input<0>(T), Input<1>(T));
}


auto len_mismatch_i = BinaryStream("1111 1111");
auto len_mismatch_e = BinaryStream("1111 111");

TEST_CASE(len_mismatch, len_mismatch_i, len_mismatch_e) {
    AssertNE(T, Input<0>(T), Input<1>(T));
}


auto len_mismatch_int_i = IntStream<int64_t>({1, 2, 3, 4, 5, 6, 7, 8});
auto len_mismatch_int_e = IntStream<int64_t>({1, 2, 3, 4, 5});

TEST_CASE(len_mismatch_int, len_mismatch_int_i, len_mismatch_int_e) {
    AssertNE(T, Input<0>(T), Input<1>(T));
}


auto len_mismatch_int_block_boundary_i = IntStream<int64_t>({1, 2, 3, 4, 5});
auto len_mismatch_int_block_boundary_e = IntStream<int64_t>({1, 2, 3, 4});

TEST_CASE(len_mismatch_int_block_boundary, len_mismatch_int_block_boundary_i, len_mismatch_int_block_boundary_e) {
    AssertNE(T, Input<0>(T), Input<1>(T));
}


auto commutative_ne_a = IntStream<int64_t>({1, 2, 3, 4, 5});
auto commutative_ne_b = IntStream<int64_t>({1, 2, 3, 4});

TEST_CASE(commutative_ne, commutative_ne_a, commutative_ne_b) {
    AssertNE(T, Input<0>(T), Input<1>(T));
    AssertNE(T, Input<1>(T), Input<0>(T));
}


auto single_rep_i = HexStream("a{10}");
auto single_rep_e = HexStream("aaaaaaaaaa");

TEST_CASE(single_rep, single_rep_i, single_rep_e) {
    AssertEQ(T, Input<0>(T), Input<1>(T));
}


auto grouped_rep_i = HexStream("(123abc){3}");
auto grouped_rep_e = HexStream("123abc 123abc 123abc");

TEST_CASE(grouped_rep, grouped_rep_i, grouped_rep_e) {
    AssertEQ(T, Input<0>(T), Input<1>(T));
}


auto rep_of_rep_i = HexStream("((123){2} (abc){2}){2}");
auto rep_of_rep_e = HexStream("123123abcabc123123abcabc");

TEST_CASE(rep_of_rep, rep_of_rep_i, rep_of_rep_e) {
    AssertEQ(T, Input<0>(T), Input<1>(T));
}


auto odd_bin_i = BinaryStream("(1.1){3}");
auto odd_bin_e = BinaryStream("1.1 1.1 1.1");

TEST_CASE(odd_bin, odd_bin_i, odd_bin_e) {
    AssertEQ(T, Input<0>(T), Input<1>(T));
}


auto hex_set_i = HexStreamSet({
    "a{300}",
    "((ab){25} (01){25}){3}"
});
auto hex_set_e = HexStreamSet({
    "((ab){25} (01){25}){3}",
    "a{300}"
});

TEST_CASE(hex_set, hex_set_i, hex_set_e) {
    auto A = Input<0>(T);
    auto B = Input<1>(T);
    AssertEQ(T, su::Select(T, A, 0), su::Select(T, B, 1));
    AssertEQ(T, su::Select(T, A, 1), su::Select(T, B, 0));
}


auto hex_set_ne_i = HexStreamSet({
    "a{65}",
    "f{65}"
});
auto hex_set_ne_e = HexStreamSet({
    "a{64}b",
    "f{65}"
});

TEST_CASE(hex_set_ne, hex_set_ne_i, hex_set_ne_e) {
    auto A = Input<0>(T);
    auto B = Input<1>(T);
    AssertNE(T, A, B);
}


auto bin_set_i = BinaryStreamSet({
    "(1 .{254} 1){4}",
    "(.1 .{252} 1.){4}"
});

auto bin_set_e = BinaryStreamSet({
    "(.1 .{252} 1.){4}",
    "(1 .{254} 1){4}"
});

TEST_CASE(bin_set, bin_set_i, bin_set_e) {
    auto A = Input<0>(T);
    auto B = Input<1>(T);

    auto a = su::Select(T, A, 0);
    auto b = su::Select(T, B, 1);
    auto c = su::Select(T, A, 1);
    auto d = su::Select(T, B, 0);

    AssertEQ(T, a, b);
    AssertEQ(T, c, d);
}


auto small_set_select_i = BinaryStreamSet({"1.1", ".1."});

auto small_set_select_e = BinaryStream(".1.");

TEST_CASE(small_set_select, small_set_select_i, small_set_select_e) {
    AssertEQ(T, su::Select(T, Input<0>(T), 1), Input<1>(T));
}


auto int_set_i = IntStreamSet<uint32_t>({
    { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19 },
    { 9, 8, 7, 6, 5, 4, 3, 2, 1, 90, 80, 70, 60, 50, 40, 30, 20, 10, 0 }
});

auto int_set_e = IntStreamSet<uint32_t>({
    { 9, 8, 7, 6, 5, 4, 3, 2, 1, 90, 80, 70, 60, 50, 40, 30, 20, 10, 0 },
    { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19 }
});

TEST_CASE(int_set, int_set_i, int_set_e) {
    auto a = su::Select(T, Input<0>(T), 0);
    auto b = su::Select(T, Input<1>(T), 1);
    auto c = su::Select(T, Input<0>(T), 1);
    auto d = su::Select(T, Input<1>(T), 0);
    AssertEQ(T, a, b);
    AssertEQ(T, c, d);
}


auto int_set_select_i = IntStreamSet<uint16_t>({
    { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19 },
    { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19 },
    { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19 }
});

auto int_set_select_e = IntStream<uint16_t>(
    { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19 }
);

TEST_CASE(int_set_select, int_set_select_i, int_set_select_e) {
    AssertEQ(T, su::Select(T, Input<0>(T), 1), Input<1>(T));
}



auto int_set_ne_i = IntStreamSet<uint32_t>({
    { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19 },
    { 9, 8, 7, 6, 5, 4, 3, 2, 1, 90, 80, 70, 60, 50, 40, 30, 20, 10, 100 }
});

auto int_set_ne_e = IntStreamSet<uint32_t>({
    { 9, 8, 7, 6, 5, 4, 3, 2, 1, 90, 80, 70, 60, 50, 40, 30, 20, 10, 0 },
    { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19 }
});

TEST_CASE(int_set_ne, int_set_ne_i, int_set_ne_e) {
    auto a = su::Select(T, Input<0>(T), 0);
    auto b = su::Select(T, Input<1>(T), 1);
    auto c = su::Select(T, Input<0>(T), 1);
    auto d = su::Select(T, Input<1>(T), 0);
    AssertEQ(T, a, b);
    AssertNE(T, c, d);
}


auto multi_input_a = HexStream("0123 abcd");
auto multi_input_b = HexStream("0123 abcd");
auto multi_input_c = HexStream("abcd 0123");

TEST_CASE(multi_input, multi_input_a, multi_input_b, multi_input_c) {
    AssertEQ(T, Input<0>(T), Input<1>(T));
    AssertNE(T, Input<1>(T), Input<2>(T));
}


RUN_TESTS(
    /*CASE(simple_eq),
    CASE(simple_ne),
    CASE(long_hex_ne),
    CASE(hex_binary_equivalence),
    CASE(dot_zero_equivalence),
    CASE(len_mismatch),
    CASE(len_mismatch_int),
    CASE(len_mismatch_int_block_boundary),
    CASE(commutative_ne),
    CASE(single_rep),
    CASE(grouped_rep),
    CASE(rep_of_rep),
    CASE(odd_bin),*/
    CASE(hex_set),
    /*CASE(hex_set_ne),
    CASE(bin_set),
    CASE(small_set_select),
    CASE(int_set),
    CASE(int_set_select),
    CASE(int_set_ne),
    CASE(multi_input),*/
)
