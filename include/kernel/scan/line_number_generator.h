/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#pragma once

#include <kernel/scan/base.h>
#include <kernel/pipeline/pipeline_builder.h>

namespace kernel {

/**
 * Generates a stream of line numbers corresponding to each bit in a given scan
 * stream. Resultant line numbers are zero-indexed.
 *
 * The production rate for the output stream is PopcountOf(scan).
 *
 * Signature:
 *  kernel LineNumberGenerator :: [<i1>[1] scan, <i1>[1] linebreaks] -> [<i64>[1] output]
 *
 * Example:
 *  scan        : .1...... ....1...
 *  linebreaks  : ...1.... 1..1...1
 *  output      : 0, 3
 */
class LineNumberGenerator : public SingleStreamScanKernelTemplate {
public:
    LineNumberGenerator(BuilderRef b, StreamSet * scan, StreamSet * linebreaks, StreamSet * output);
protected:
    void initialize(BuilderRef b) override;
    void willProcessStride(BuilderRef b, llvm::Value * const strideNo) override;
    void maskBuildingIterationHead(BuilderRef b) override;
    void maskBuildingIterationBody(BuilderRef b, llvm::Value * const blockIndex) override;
    void generateProcessingLogic(
        BuilderRef b, 
        llvm::Value * const absoluteIndex, 
        llvm::Value * const blockIndex, 
        llvm::Value * const bitOffset) override;
    void didProcessStride(BuilderRef b, llvm::Value * const strideNo) override;
private:
    llvm::Value *   mLineCountArrayBlockPtr = nullptr;
    llvm::Value *   mInitialLineNum = nullptr;
    llvm::Value *   mHighestLineCount = nullptr;
    llvm::PHINode * mBaseCounts = nullptr;
};

namespace scan {

/**
 * Generates an i64 stream where each item is the zero-indexed line number of
 * a bit in the `scan` bitstream.
 * 
 * The output stream will contain PopcountOf(`scan`) items.
 * 
 * Preconditions:
 *  - `scan` is of type <i1>[1]
 *  - `linebreaks` is of type <i1>[1]
 * 
 * Returns:
 *  A single i64 stream (i.e., <i64>[1])
 * 
 * Example:
 *  Consider the following sample XML document, with spaces and '\n' characters
 *  marked by '∙' amd '⏎', where `scan` marks the location of each '<'.
 * 
 *      <doc>⏎
 *      ∙∙<node><i/></node>⏎
 *      </doc>⏎
 * 
 *  doc:        <doc>...<node><i/></node>.</doc>.
 *  scan:       1.......1.....1...1.......1......
 *  linebreaks: .....1...................1......1
 *  output:     0, 1, 1, 1, 2
 */
inline StreamSet * LineNumbers(const std::unique_ptr<ProgramBuilder> & P, StreamSet * scan, StreamSet * linebreaks) {
    assert(scan->getFieldWidth() == 1);
    assert(scan->getNumElements() == 1);
    assert(linebreaks->getFieldWidth() == 1);
    assert(linebreaks->getNumElements() == 1);
    StreamSet * const out = P->CreateStreamSet(1, 64);
    P->CreateKernelCall<LineNumberGenerator>(scan, linebreaks, out);
    return out;
}

} // namespace kernel::scan

} // namespace kernel
