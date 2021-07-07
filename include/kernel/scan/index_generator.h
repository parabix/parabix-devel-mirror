/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#pragma once

#include <kernel/scan/base.h>
#include <kernel/pipeline/pipeline_builder.h>

namespace kernel {

/**
 * Generates a stream of indices corresponding to the offset of each bit in a
 * given scan stream.
 *
 * The production rate for the ouput stream is PopcountOf(scan).
 *
 * Signature:
 *  kernel ScanIndexGenerator :: [<i1>[1] scan] -> [<i64>[1] output]
 *
 * Example:
 *  scan        : 11...... ....1...
 *  output      : 0, 1, 12
 */
class ScanIndexGenerator : public SingleStreamScanKernelTemplate {
public:
    ScanIndexGenerator(BuilderRef b, StreamSet * scan, StreamSet * output);
protected:
    void generateProcessingLogic(
        BuilderRef b, 
        llvm::Value * const absoluteIndex, 
        llvm::Value * const blockIndex, 
        llvm::Value * const bitOffset) override;
};

namespace scan {

/**
 * Converts a bitsteram into a i64 stream where each i64 is the index of a 1 bit
 * in the input stream `s`.
 * 
 * Preconditions:
 *  - `s` is of type <i1>[1] (i.e., field width == 1 and stream count == 1)
 * 
 * Returns:
 *  - A single i64 stream (i.e., <i64>[1])
 * 
 * Example:
 *  Input:
 *      s[0]:   11..1..1
 * 
 *  Output:
 *      out[0]: 0, 1, 4, 7
 * 
 * Use:
 *  `ToIndices` is intended to be used in conjunction with `kernel::scan::Read`
 *  to perform callbacks at each bit location in a given scan stream. `ToIndices`
 *  transforms bit locations into integers which are futher transformed to 
 *  pointers in a source stream by the `Reader` kernel.
 * 
 *  This example shows how `ToIndices` may be used, along with `Read` to call a
 *  C function `callback_fn` with a pointer to a location in the `Source` stream
 *  at locations specified by the bits in the `Scan` stream.
 *  
 *      using namespace kernel;
 *      // -- snip --
 *      extern "C" void callback_fn(const uint8_t * ptr) { ... }
 *      // -- snip --
 *      std::unique_ptr<ProgramBuilder> P = ...;
 *      CPUDriver & Driver = ...;
 *      StreamSet * Source = ...; // <i8>[1]
 *      StreamSet * Scan = ...; // <i1>[1]
 *      StreamSet * Indices = scan::ToIndices(P, Scan); 
 *      scan::Read(P, Driver, Source, Indices, SCAN_CALLBACK(callback_fn));
 */
inline StreamSet * ToIndices(const std::unique_ptr<ProgramBuilder> & P, StreamSet * s) {
    assert(s->getFieldWidth() == 1);
    assert(s->getNumElements() == 1);
    StreamSet * const out = P->CreateStreamSet(1, 64);
    P->CreateKernelCall<ScanIndexGenerator>(s, out);
    return out;
}

} // namespace kernel::scan

} // namespace kernel
