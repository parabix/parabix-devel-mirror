/*
 * Copyright (c) 2019 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

#include <testing/testing.h>
#include <kernel/streamutils/run_index.h>
#include <kernel/util/debug_display.h>

using namespace kernel;
using namespace testing;


auto no_runs = BinaryStream({"0{1245}"});
auto no_run_idx_e = BinaryStreamSet({"0{1245}", "0{1245}", "0{1245}"});

TEST_CASE(index_no_runs, no_runs, no_run_idx_e) {
    auto Result = T->CreateStreamSet(3);
    P->CreateKernelCall<RunIndex>(Input<0>(T), Result);
    AssertEQ(T, Result, Input<1>(T));
}

auto runs_i =       BinaryStream({"..111111.111111111.1..."});
auto run_idx_e = BinaryStreamSet({"..010101.010101010.0...",
                                  "..001100.001100110.0...",
                                  "..000011.000011110.0..."});
auto overflow_e =   BinaryStream({".................1....."});

TEST_CASE(runs_1, runs_i, run_idx_e) {
    auto Result = T->CreateStreamSet(3);
    P->CreateKernelCall<RunIndex>(Input<0>(T), Result);
    AssertEQ(T, Result, Input<1>(T));
}

TEST_CASE(runs_1overflow, runs_i, overflow_e) {
    auto Result = T->CreateStreamSet(3);
    auto OverFlow = T->CreateStreamSet(1);
    P->CreateKernelCall<RunIndex>(Input<0>(T), Result, OverFlow);
    AssertEQ(T, OverFlow, Input<1>(T));
}
RUN_TESTS(
          CASE(index_no_runs),
          CASE(runs_1),
          CASE(runs_1overflow)
)
