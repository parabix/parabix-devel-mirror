/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#pragma once

#include <kernel/scan/base.h>
#include <kernel/pipeline/pipeline_builder.h>

namespace kernel {

/**
 * Converts a linebreak stream into start and end indices.
 *
 * The production rate for the output stream is PopcountOf(linebreaks).
 *
 * The `linebreaks` stream must be terminated by a 1 bit. If the input is a
 * `UnixLinesKernelBuilder`, then it should use `UnterminatedLineAtEOF::Add1`
 * to ensure that the stream is terminated by a 1 bit.
 *
 * Signature:
 *  kernel LineSpanGenerator :: [<i1>[1] linebreaks] -> [<i64>[2] output]
 *
 * Example:
 *  linebreaks  : ...1....1......1
 *  output   [0]: 0, 4, 9
 *           [1]: 3, 8, 15
 */
class LineSpanGenerator : public SingleStreamScanKernelTemplate {
public:
    LineSpanGenerator(BuilderRef b, StreamSet * linebreaks, StreamSet * output);
protected:
    void generateProcessingLogic(BuilderRef b, llvm::Value * absoluteIndex) override;
private:
};


/**
 * Filters a stream of line spans based on a stream of line numbers.
 * 
 * The production rate for `output` is equivalent to the consumption rate of
 * `lineNumbers`.
 * 
 * Signature:
 *  kernel LineSpanFilterKernel :: [<i64>[1] linenums, <i64>[2] spans] -> [<i64>[2] output]
 * 
 * Example:
 *  lineNumbers:    0, 2, 3
 *  spans[0]:       0, 4,  8, 12, 16
 *  spans[1]:       2, 6, 10, 14, 18
 * 
 *  output[0]:      0,  8, 12
 *  output[1]:      2, 10, 14
 * 
 */
class LineSpanFilterKernel : public MultiBlockKernel {
    using BuilderRef = const std::unique_ptr<KernelBuilder> &;
public:
    LineSpanFilterKernel(BuilderRef b, StreamSet * lineNumbers, StreamSet * spans, StreamSet * output);
protected:
    void generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfStrides) override;
};

namespace scan {

/**
 * Converts a stream of linebreaks into two index streams where each pair of
 * indices is the start and end position of each line.
 * 
 * The output index streams are intended to be converted into start and end 
 * pointers via a `scan::Reader` kernel call.
 * 
 * The end index will be the location of bit it `linebreaks` with the start
 * index being the location just after.
 * 
 * Preconditions:
 *  - linebreaks must be of type <i1>[1]
 * 
 * Returns:
 *  - A set of 2 i64 streams (i.e., <i64>[2]) with stream [0] being the start
 *    indices and stream [1] being the end indices.
 * 
 * Example:
 *  linebreaks[0]:  ...1...1
 * 
 *  output[0]:      0, 3
 *  output[1]:      4, 7
 */
inline StreamSet * LineSpans(const std::unique_ptr<ProgramBuilder> & P, StreamSet * linebreaks) {
    assert(linebreaks->getFieldWidth() == 1);
    assert(linebreaks->getNumElements() == 1);
    StreamSet * const out = P->CreateStreamSet(2, 64);
    P->CreateKernelCall<LineSpanGenerator>(linebreaks, out);
    return out;
}


/**
 * Filters a stream set of line spans according to a stream of line numbers.
 * Only the spans for the given line numbers will be present in the ouput
 * stream set.
 * 
 * This function is used to filter the output of `span::LineSpans` so that
 * it may be passed to a `span::Reader` call (see usage example).
 * 
 * This function can be viewed as a 1-to-1 mapping of `lineNumbers` to start
 * and end indicies for those lines.
 * 
 * Preconditions:
 *  - `lineNumbers` must be of type <i64>[1]
 *  - `spans` must be of type <i64>[2]
 * 
 * Returns:
 *  - A set of 2 i64 streams (i.e., <i64>[2]) with stream [0] being the start
 *    indicies and stream [1] being the end indices.
 * 
 * Example:
 *  lineNumbers:    0, 2, 3
 *  spans[0]:       0, 4,  8, 12, 16
 *  spans[1]:       2, 6, 10, 14, 18
 * 
 *  output[0]:      0,  8, 12
 *  output[1]:      2, 10, 14
 * 
 * Usage Example:
 *  This example shows how to use `FilterLineSpans` to pass begin and end 
 *  pointers into a call to `scan::Reader`.
 * 
 *      extern "C" callback(const uint8_t * lineBegin, const uint8_t * lineEnd);
 *      // -- snip --
 *      using namespace kernel;
 *      // -- snip -- 
 *      std::unique_ptr<ProgramBuilder> P = ...;
 *      CPUDriver driver = ...;
 *      StreamSet * Source = ...; // <i8>[1]
 *      StreamSet * Linebreaks = ...; // <i1>[1]
 *      StreamSet * Scan = ...; // <i1>[1]
 *      StreamSet * LineNumbers = scan::LineNumbers(P, Scan, Linebreaks); // <i64>[1]
 *      // `SourceLineSpans` contains spans for every line in `Source`
 *      StreamSet * SourceLineSpans = scan::LineSpans(P, Linebreaks); // <i64>[2]
 * 
 *      // Filter `SourceLineSpans` to only the lines where there is a bit in `Scan`
 *      StreamSet * Spans = scan::FilterLineSpans(P, LineNumbers, SourceLineSpans); // <i64>[2]
 * 
 *      // `Spans` are converted to pointers in `Source` and passed to `callback`
 *      scan::Reader(P, driver, SCAN_CALLBACK(callback), Source, Spans);
 */
inline StreamSet * FilterLineSpans(const std::unique_ptr<ProgramBuilder> & P, StreamSet * lineNumbers, StreamSet * spans) {
    assert(lineNumbers->getFieldWidth() == 64 && lineNumbers->getNumElements() == 1);
    assert(spans->getFieldWidth() == 64 && spans->getNumElements() == 2);
    StreamSet * const out = P->CreateStreamSet(2, 64);
    P->CreateKernelCall<LineSpanFilterKernel>(lineNumbers, spans, out);
    return out;
}

} // namespace kernel::scan

} // namespace kernel
