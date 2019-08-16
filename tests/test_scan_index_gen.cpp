/*
 * Copyright (c) 2019 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

#include <testing/testing.h>
#include <kernel/scan/index_generator.h>
#include <kernel/util/debug_display.h>

using namespace kernel;
using namespace testing;

auto tiny_scan_i = BinaryStream("(1... ....){3}");
auto tiny_scan_e = IntStream<uint64_t>({0, 8, 16});

TEST_CASE(tiny_scan, tiny_scan_i, tiny_scan_e) {
    auto Result = scan::ToIndices(T, Input);
    AssertEQ(T, Result, Expected);
}


auto no_bits_i = HexStream("0{10000}");
auto no_bits_e = IntStream<uint64_t>({});

TEST_CASE(no_bits, no_bits_i, no_bits_e) {
    auto Result = scan::ToIndices(T, Input);
    AssertEQ(T, Result, Expected);
}


auto long_scan_i = BinaryStream(".{105123} 1 .{3000}");
auto long_scan_e = IntStream<uint64_t>({105123});

TEST_CASE(long_scan, long_scan_i, long_scan_e) {
    auto Result = scan::ToIndices(T, Input);
    AssertEQ(T, Result, Expected);
}

RUN_TESTS(
    CASE(tiny_scan),
    CASE(no_bits),
    CASE(long_scan),
)
