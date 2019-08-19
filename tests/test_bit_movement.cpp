/*
 * Copyright (c) 2019 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

#include <testing/testing.h>
#include <kernel/streamutils/deletion.h>
#include <kernel/streamutils/pdep_kernel.h>
#include <kernel/util/debug_display.h>

using namespace kernel;
using namespace testing;

auto insert_mask_i = BinaryStream({"11100001"});
auto insert_before_e = BinaryStream({"010101111101"});

TEST_CASE(insert_before1, insert_mask_i, insert_before_e) {
    auto Result = UnitInsertionSpreadMask(T, Input, InsertPosition::Before);
    AssertEQ(T, Result, Expected);
}

auto insert_after_e = BinaryStream({"101010111110"});

TEST_CASE(insert_after1, insert_mask_i, insert_after_e) {
    auto Result = UnitInsertionSpreadMask(T, Input, InsertPosition::After);
    AssertEQ(T, Result, Expected);
}

auto insert_counts = BinaryStreamSet({"00100010", "00100000", "10000000"});
auto insert_mult_e = BinaryStream("0000110001111011");

TEST_CASE(insert_mult, insert_counts, insert_mult_e) {
    auto Result = InsertionSpreadMask(T, Input, InsertPosition::Before);
    AssertEQ(T, Result, Expected);
}

RUN_TESTS(
          CASE(insert_before1),
          CASE(insert_after1),
          CASE(insert_mult)
)
