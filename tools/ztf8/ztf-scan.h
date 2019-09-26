/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#ifndef ZTF_SCAN_H
#define ZTF_SCAN_H

#include <pablo/pablo_kernel.h>
#include <kernel/core/kernel_builder.h>

namespace kernel {

struct LengthGroup {unsigned lo; unsigned hi;};

class LengthGroupCompressionMask : public MultiBlockKernel {
public:
    LengthGroupCompressionMask(const std::unique_ptr<kernel::KernelBuilder> & b,
                               LengthGroup lengthGroup,
                               StreamSet * symbolMarks,
                               StreamSet * symbolLengths,
                               StreamSet * const byteData, StreamSet * const hashValues, StreamSet * compressionMask, unsigned strideBlocks = 8);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
    void generateMultiBlockLogic(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::Value * const numOfStrides) override;
    LengthGroup mLengthGroup;
};

class LengthGroupDecompression : public MultiBlockKernel {
public:
    LengthGroupDecompression(const std::unique_ptr<kernel::KernelBuilder> & b,
                             LengthGroup lengthGroup,
                             StreamSet * keyMarks,
                             StreamSet * symbolLengths,
                             StreamSet * const hashMarks, StreamSet * const byteData,
                             StreamSet * const hashValues,
                             StreamSet * const result, unsigned strideBlocks = 8);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
    void generateMultiBlockLogic(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::Value * const numOfStrides) override;
    LengthGroup mLengthGroup;
};

class FixedLengthDecompression : public MultiBlockKernel {
public:
    FixedLengthDecompression(const std::unique_ptr<kernel::KernelBuilder> & b,
                             unsigned length,
                             StreamSet * keyMarks,
                             StreamSet * const hashMarks, StreamSet * const byteData,
                             StreamSet * const hashValues,
                             StreamSet * const result, unsigned strideBlocks = 8);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
    void generateMultiBlockLogic(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::Value * const numOfStrides) override;
    unsigned mLength;
};

}
#endif

