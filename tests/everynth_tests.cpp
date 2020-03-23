/*
 * Copyright (c) 2020 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

#include <testing/testing.h>
#include <kernel/core/kernel_builder.h>
#include <pablo/pablo_kernel.h>
#include <pablo/builder.hpp>
#include <pablo/pe_everynth.h>

using namespace kernel;
using namespace testing;

namespace kernel {

using namespace pablo;

class EveryNthKernel : public PabloKernel {
public:
    EveryNthKernel(const std::unique_ptr<KernelBuilder> & b, StreamSet * const input, StreamSet * output, uint64_t n)
    : PabloKernel(b,
                  "everyNthKernel" + std::to_string(n),
                  {Binding{"input", input}},
                  {Binding{"output", output}}),
      n{n} {}
protected:
    uint64_t n;
    void generatePabloMethod() override {
        PabloBuilder pb(getEntryScope());
        PabloAST * const input = getInputStreamSet("input")[0];
        Var * const output = getOutputStreamVar("output");
        PabloAST * const out = pb.createEveryNth(input, pb.getInteger(n));
        pb.createAssign(pb.createExtract(output, pb.getInteger(0)), out);
    }
};

}

auto every1in  = BinaryStream({".... ...1 .... 1... ...1"});
auto every1out = BinaryStream({".... ...1 .... 1... ...1"});

TEST_CASE(every1, every1in, every1out) {
    auto const Result = T->CreateStreamSet(1);
    P->CreateKernelCall<EveryNthKernel>(Input<0>(T), Result, 1);
    AssertEQ(T, Result, Input<1>(T));
}

auto every2in  = BinaryStream({".... ...1 .... 1... ...1"});
auto every2out = BinaryStream({".... ...1 .... 0... ...1"});

TEST_CASE(every2, every2in, every2out) {
    auto const Result = T->CreateStreamSet(1);
    P->CreateKernelCall<EveryNthKernel>(Input<0>(T), Result, 2);
    AssertEQ(T, Result, Input<1>(T));
}

auto every7in  = BinaryStream({".... ...1 .... 1... ...1 .... 1..1 ...1 ..1. 1.1. 1... ...."});
auto every7out = BinaryStream({".... ...1 .... 0... ...0 .... 0..0 ...0 ..0. 1.0. 0... ...."});

TEST_CASE(every7, every7in, every7out) {
    auto const Result = T->CreateStreamSet(1);
    P->CreateKernelCall<EveryNthKernel>(Input<0>(T), Result, 7);
    AssertEQ(T, Result, Input<1>(T));
}

auto every4in  = HexStream({"ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff"});
auto every4out = HexStream({"1111 1111 1111 1111 1111 1111 1111 1111 1111 1111 1111 1111 1111 1111 1111 1111"});

TEST_CASE(every4, every4in, every4out) {
    auto const Result = T->CreateStreamSet(1);
    P->CreateKernelCall<EveryNthKernel>(Input<0>(T), Result, 4);
    AssertEQ(T, Result, Input<1>(T));
}

auto every5in  = HexStream({"ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff 0000 0000 0000 0000"});
auto every5out = HexStream({"1248 0124 8012 4801 2480 1248 0124 8012 4801 2480 1248 0124 8012 4801 2480 1248 0124 8012 4801 2480 0000 0000 0000 0000"});

TEST_CASE(every5, every5in, every5out) {
    auto const Result = T->CreateStreamSet(1);
    P->CreateKernelCall<EveryNthKernel>(Input<0>(T), Result, 5);
    AssertEQ(T, Result, Input<1>(T));
}

auto every256in  = HexStream({"ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff"});
auto every256out = HexStream({"1000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 1000"});

TEST_CASE(every256, every256in, every256out) {
    auto const Result = T->CreateStreamSet(1);
    P->CreateKernelCall<EveryNthKernel>(Input<0>(T), Result, 256);
    AssertEQ(T, Result, Input<1>(T));
}

RUN_TESTS(
          CASE(every1),
          CASE(every2),
          CASE(every7),
          CASE(every4),
          CASE(every5),
          CASE(every256)
)
