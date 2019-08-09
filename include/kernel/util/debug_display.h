/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#pragma once

#include <kernel/core/kernel.h>
#include <kernel/pipeline/pipeline_builder.h>

namespace kernel {

/**
 * A debug kernel which displays the contents of a streamset to stderr using
 * either `PrintRegister` or `PrintInt` depending of the field width of `s`.
 */
class DebugDisplayKernel : public MultiBlockKernel {
    using BuilderRef = const std::unique_ptr<KernelBuilder> &;
public:
    DebugDisplayKernel(BuilderRef b, llvm::StringRef name, StreamSet * s);
    void generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfStrides) override;
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
    llvm::StringRef mName;
    uint32_t        mFW;
    uint32_t        mSCount;
};

namespace util {

/**
 * Displays the contents of an arbitrary streamset to stderr.
 * 
 * If the field width of the stream is 1, then stream stream will be printed as
 * bitblocks using `PrintRegister`. Otherwise, the stream will be printed as a
 * sequences of integers using `PrintInt`. For streamsets with more than one
 * stream, the index of the stream being shown will be displayed along with the
 * provided name.
 * 
 * Usage:
 *  The primary use for this kernel is to display the stream outputs of kernels
 *  when building pipelines. See also stream_select.h for extracting specific
 *  streams from stream sets.
 * 
 *      using namespace kernel;
 *      // -- snip --
 *      std::unique_ptr<ProgramBuilder> P = ...;
 *      StreamSet * SomeStream = ...;
 *      util::DebugDisplay(P, "some_stream", SomeStream);
 */
inline void DebugDisplay(const std::unique_ptr<ProgramBuilder> & P, llvm::StringRef name, StreamSet * s) {
    P->CreateKernelCall<DebugDisplayKernel>(name, s);
}

} // namespace kernel::util

} // namespace kernel
