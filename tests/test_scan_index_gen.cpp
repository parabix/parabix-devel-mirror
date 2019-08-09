/*
 * Copyright (c) 2019 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

#include <testing/testing.h>
#include <kernel/scan/index_generator.h>
#include <kernel/util/debug_display.h>

using namespace kernel;
using namespace testing;

auto scanbits0 = BinaryStream("1... .... 1... ....");
auto expected0 = IntStream<uint64_t>({0, 8});

TEST_CASE(tiny_scan, scanbits0, expected0) {
    auto Result = scan::ToIndices(T, Input);
    auto rt = AssertEQ(T, Result, Expected);
    T.setReturnValue(rt);
}


auto scanbits1 = HexStream("000000000000000000000000000000000000000000000000");
auto expected1 = IntStream<uint64_t>({});

TEST_CASE(no_bits, scanbits1, expected1) {
    auto Result = scan::ToIndices(T, Input);
    auto rt = AssertEQ(T, Result, Expected);
    T.setReturnValue(rt);
}

RUN_TESTS(
    CASE(tiny_scan),
    CASE(no_bits)
);
