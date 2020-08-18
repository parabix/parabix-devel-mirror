/*
 * Copyright (c) 2019 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

#include <testing/testing.h>
#include <kernel/basis/s2p_kernel.h>
#include <kernel/scan/index_generator.h>
#include <kernel/scan/line_span_generator.h>
#include <kernel/scan/line_number_generator.h>
#include <kernel/streamutils/collapse.h>
#include <kernel/streamutils/deletion.h>
#include <kernel/util/debug_display.h>
#include <kernel/util/linebreak_kernel.h>
#include <kernel/unicode/charclasses.h>
#include <re/adt/re_cc.h>
#include <util/iota_fill.hpp>

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


// 445 characters
auto scan_index_integration_text = TextStream(
"Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor "
"incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis "
"nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. "
"Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore "
"eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt "
"in culpa qui officia deserunt mollit anim id est laborum."
);
auto scan_index_integration_expected = IntStream<uint64_t>({122, 230, 333, 444});

TEST_CASE(scan_index_integration, 
    scan_index_integration_text,
    scan_index_integration_expected)
{
    auto Basis = P->CreateStreamSet(8, 1);
    P->CreateKernelCall<S2PKernel>(Input<0>(T), Basis);
    auto Marker = P->CreateStreamSet(1, 1);
    P->CreateKernelCall<ByteClassesKernel>(std::vector<re::CC *>{re::makeByte('.')}, Basis, Marker);
    auto Collapsed = streamutils::Collapse(P, Marker);
    auto Indices = scan::ToIndices(P, Collapsed);
    AssertEQ(P, Indices, Input<1>(T));
}


auto simple_line_span_i = BinaryStream(".{12} 1 .{2} 1");
auto simple_line_span_e = IntStreamSet<uint64_t>({
    {  0, 13 },
    { 12, 15 }
});

TEST_CASE(simple_line_span, simple_line_span_i, simple_line_span_e) {
    auto Result = scan::LineSpans(T, Input<0>(T));
    AssertEQ(T, Result, Input<1>(T));
}


auto text_line_span_source = TextStream(
    "abc\n"
    "123"
);
auto text_line_span_e = IntStreamSet<uint64_t>({
    { 0, 4 },
    { 3, 7 }
});

TEST_CASE(text_line_span, text_line_span_source, text_line_span_e) {
    auto const LineBreaks = T->CreateStreamSet();
    P->CreateKernelCall<UnixLinesKernelBuilder>(Input<0>(T), LineBreaks, UnterminatedLineAtEOF::Add1);
    auto const Result = scan::LineSpans(T, LineBreaks);
    AssertEQ(T, Result, Input<1>(T));
}


auto long_spans_i = BinaryStream(".{1000} 1 .{512} 1");
auto long_spans_e = IntStreamSet<uint64_t>({
    {   0, 1001},
    {1000, 1513}
});

TEST_CASE(long_spans, long_spans_i, long_spans_e) {
    auto Result = scan::LineSpans(T, Input<0>(T));
    AssertEQ(T, Result, Input<1>(T));
}


auto filter_spans_spans = IntStreamSet<uint64_t>({
    { 0, 12, 19, 24, 56, 62, 70},
    {11, 18, 23, 55, 61, 69, 74}
});
auto filter_spans_filter = IntStream<uint64_t>(
    {0, 2, 3, 5}
);
auto filter_spans_e = IntStreamSet<uint64_t>({
    { 0, 19, 24, 62},
    {11, 23, 55, 69}
});

TEST_CASE(filter_spans, filter_spans_spans, filter_spans_filter, filter_spans_e) {
    auto Result = scan::FilterLineSpans(T, Input<1>(T), Input<0>(T));
    AssertEQ(T, Result, Input<2>(T));
}


auto filter_no_spans_spans = IntStreamSet<uint64_t>({
    { 0, 12, 19, 24, 56, 62, 70},
    {11, 18, 23, 55, 61, 69, 74}
});
auto filter_no_spans_filter = IntStream<uint64_t>({});
auto filter_no_spans_e = IntStreamSet<uint64_t>({{}, {}});

TEST_CASE(filter_no_spans, filter_no_spans_spans, filter_no_spans_filter, filter_no_spans_e) {
    auto Result = scan::FilterLineSpans(T, Input<1>(T), Input<0>(T));
    AssertEQ(T, Result, Input<2>(T));
}


auto one_per_line_markers    = BinaryStream("(.1...){20} (1.1.){10}");
auto one_per_line_linebreaks = BinaryStream("(...1.){20} (.11.){10}");
auto one_per_line_e = IntStream<uint64_t>(meta::iota_fill<uint64_t>(40, 0));

TEST_CASE(one_per_line, one_per_line_markers, one_per_line_linebreaks, one_per_line_e) {
    auto Result = scan::LineNumbers(T, Input<0>(T), Input<1>(T));
    AssertEQ(T, Result, Input<2>(T));
}


RUN_TESTS(
    CASE(tiny_scan),
    CASE(no_bits),
    CASE(long_scan),
    CASE(scan_index_integration),
    CASE(simple_line_span),
    CASE(text_line_span),
    CASE(long_spans),
    CASE(filter_spans),
    CASE(filter_no_spans),
    CASE(one_per_line),
)
