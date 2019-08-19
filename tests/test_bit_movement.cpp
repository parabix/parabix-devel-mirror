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

auto insert_mask_i = BinaryStream({"1"});
auto insert_before_e = BinaryStream({"01"});

TEST_CASE(insert_before1, insert_mask_i, insert_before_e) {
    auto Result = UnitInsertionSpreadMask(T, Input, InsertPosition::Before);
    AssertEQ(T, Result, Expected);
}

auto insert_after_e = BinaryStream({"10"});

TEST_CASE(insert_after1, insert_mask_i, insert_after_e) {
    auto Result = UnitInsertionSpreadMask(T, Input, InsertPosition::After);
    AssertEQ(T, Result, Expected);
}

RUN_TESTS(
          CASE(insert_before1),
          CASE(insert_after1)
)
