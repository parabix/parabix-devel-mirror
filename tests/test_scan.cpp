/*
 * Copyright (c) 2019 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

#include <testing/testing.h>
#include <kernel/scan/index_generator.h>
#include <kernel/scan/line_span_generator.h>
#include <kernel/util/debug_display.h>
#include <kernel/util/linebreak_kernel.h>
#include <kernel/streamutils/deletion.h>

using namespace kernel;
using namespace testing;

auto tiny_scan_i = BinaryStream("(1... ....){3}");
auto tiny_scan_e = IntStream<uint64_t>({0, 8, 16});

TEST_CASE(tiny_scan, tiny_scan_i, tiny_scan_e) {
    auto Result = scan::ToIndices(T, Input<0>(T));
    AssertEQ(T, Result, Input<1>(T));
}


auto no_bits_i = HexStream("0{10000}");
auto no_bits_e = IntStream<uint64_t>({});

TEST_CASE(no_bits, no_bits_i, no_bits_e) {
    auto Result = scan::ToIndices(T, Input<0>(T));
    AssertEQ(T, Result, Input<1>(T));
}


auto long_scan_i = BinaryStream(".{105123} 1 .{3000}");
auto long_scan_e = IntStream<uint64_t>({105123});

TEST_CASE(long_scan, long_scan_i, long_scan_e) {
    auto Result = scan::ToIndices(T, Input<0>(T));
    AssertEQ(T, Result, Input<1>(T));
}


auto simple_line_span_i = BinaryStream(".{12} 1 .{2} 1");
auto simple_line_span_e = IntStreamSet<uint64_t>({
    {  0, 13 },
    { 12, 15 }
});

TEST_CASE(simple_line_span, simple_line_span_i, simple_line_span_e) {
    auto Result = scan::LineSpans(T, Input<0>(T));
    // util::DebugDisplay(T, "result", Result);
    AssertEQ(T, Result, Input<1>(T));
}


auto text_line_span_source = TextStream(
    "abc\n"
    "123"
);
auto text_line_span_e = IntStreamSet<uint64_t>({
    { 0, 4 },
    { 5, 7 }
});

TEST_CASE(text_line_span, text_line_span_source, text_line_span_e) {
    auto const LineBreaks = T->CreateStreamSet();
    P->CreateKernelCall<UnixLinesKernelBuilder>(Input<0>(T), LineBreaks, UnterminatedLineAtEOF::Add1);
    util::DebugDisplay(T, "linebreaks", LineBreaks);
    auto const Result = scan::LineSpans(T, LineBreaks);
    util::DebugDisplay(T, "result", Result);
    AssertEQ(T, Result, Input<1>(T));
}


RUN_TESTS(
    CASE(tiny_scan),
    CASE(no_bits),
    CASE(long_scan),
    CASE(simple_line_span),
    // CASE(text_line_span),
)
