/*
 * Copyright (c) 2019 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

#include <testing/testing.h>
#include <kernel/util/debug_display.h>

using namespace testing;
using namespace kernel;

auto A = IntStream<uint64_t>({1, 2});
auto B = IntStream<uint64_t>({1});

TEST_CASE(mismatch_should_fail, A, B) {
    StreamSet * a = Input;
    StreamSet * b = Expected;
    
    auto carryIn = T.driver().CreateConstant(T.driver().getBuilder()->getInt32(0));
    //                                                       v is zero extended
    auto k = T->CreateKernelCall<StreamEquivalenceKernel>(a, b, carryIn);
    T.setReturnValue(k->getOutputScalar("result"));
}


auto H0 = HexStream("ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff");
auto H1 = HexStream("fffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff");

TEST_CASE(mismatch_should_fail_bitstream, H0, H1) {
    StreamSet * h0 = Input;
    StreamSet * h1 = Expected;

    auto carryIn = T.driver().CreateConstant(T.driver().getBuilder()->getInt32(0));
    //                                                        vv is zero extended
    auto k = T->CreateKernelCall<StreamEquivalenceKernel>(h0, h1, carryIn);
    T.setReturnValue(k->getOutputScalar("result"));
}


//                                                                                    v AVX-256 new block
auto H2 = HexStream("ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff ffff");
auto H3 = HexStream("ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff");

TEST_CASE(mismatch_accross_block_boundary_should_fail_bitstream, H2, H3) {
    StreamSet * h2 = Input;
    // util::DebugDisplay(T, "h2", h2);
    StreamSet * h3 = Expected;
    // util::DebugDisplay(T, "h3", h3);

    auto carryIn = T.driver().CreateConstant(T.driver().getBuilder()->getInt32(0));
    //                                                        vv is zero extended
    auto k = T->CreateKernelCall<StreamEquivalenceKernel>(h2, h3, carryIn);
    T.setReturnValue(k->getOutputScalar("result"));
}


auto C = IntStream<uint64_t>({1, 2, 3, 4, 5});
auto D = IntStream<uint64_t>({1, 2, 3, 4});

TEST_CASE(mismatch_does_fail, C, D) {
    StreamSet * c = Input;
    StreamSet * d = Expected;

    auto carryIn = T.driver().CreateConstant(T.driver().getBuilder()->getInt32(0));
    //                                                       v is zero extended
    auto k = T->CreateKernelCall<StreamEquivalenceKernel>(c, d, carryIn);
    T.setReturnValue(k->getOutputScalar("result"));
}


TEST_CASE(scalar_carry_doesnt_work, C, D) {
    StreamSet * c = Input;
    StreamSet * d = Expected;

    auto carryIn = T.driver().CreateConstant(T.driver().getBuilder()->getInt32(0));
    //                                                        v is zero extended
    auto k0 = T->CreateKernelCall<StreamEquivalenceKernel>(c, d, carryIn);
    // This equivalence fails so carry should be 1
    auto carry = k0->getOutputScalar("result");

    //                                                        v is zero extended
    auto k1 = T->CreateKernelCall<StreamEquivalenceKernel>(d, c, carry);
    // This equivalence passes, but they carry in of 1 should give a result of 1
    // failing this test, but that doesn't happen
    T.setReturnValue(k1->getOutputScalar("result"));
}


//                                                                                    v AVX256 new block
auto H4 = HexStream("ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff ffff");
auto H5 = HexStream("ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff f");

TEST_CASE(mismatch_does_fail_bitstream, H4, H5) {
    StreamSet * h4 = Input;
    // util::DebugDisplay(T, "h4", h4);
    StreamSet * h5 = Expected;
    // util::DebugDisplay(T, "h5", h5);

    auto carryIn = T.driver().CreateConstant(T.driver().getBuilder()->getInt32(0));
    //                                                        vv is zero extended
    auto k = T->CreateKernelCall<StreamEquivalenceKernel>(h4, h5, carryIn);
    T.setReturnValue(k->getOutputScalar("result"));
}


// all of these should fail
RUN_TESTS(
    CASE(mismatch_should_fail),
    CASE(mismatch_should_fail_bitstream),
    CASE(mismatch_accross_block_boundary_should_fail_bitstream),
    CASE(mismatch_does_fail),
    CASE(mismatch_does_fail_bitstream),
    CASE(scalar_carry_doesnt_work)
)
