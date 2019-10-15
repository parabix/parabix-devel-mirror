/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#ifndef ZTF_SCAN_H
#define ZTF_SCAN_H

#include <pablo/pablo_kernel.h>
#include <kernel/core/kernel_builder.h>
#include "ztf-logic.h"

namespace kernel {

class LengthGroupCompression : public MultiBlockKernel {
public:
    LengthGroupCompression(const std::unique_ptr<kernel::KernelBuilder> & b,
                           EncodingInfo encodingScheme,
                           unsigned groupNo,
                           StreamSet * symbolMarks,
                           StreamSet * hashValues,
                           StreamSet * const byteData,
                           StreamSet * compressionMask,
                           StreamSet * encodedBytes,
                           unsigned strideBlocks = 8);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
    void generateMultiBlockLogic(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::Value * const numOfStrides) override;
    EncodingInfo mEncodingScheme;
    unsigned mGroupNo;
};

class LengthGroupDecompression : public MultiBlockKernel {
public:
    LengthGroupDecompression(const std::unique_ptr<kernel::KernelBuilder> & b,
                             EncodingInfo encodingScheme,
                             unsigned groupNo,
                             StreamSet * keyMarks,
                             StreamSet * hashValues,
                             StreamSet * const hashMarks,
                             StreamSet * const byteData,
                             StreamSet * const result,
                             unsigned strideBlocks = 8);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
    void generateMultiBlockLogic(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::Value * const numOfStrides) override;
    EncodingInfo mEncodingScheme;
    unsigned mGroupNo;
};

class FixedLengthCompression : public MultiBlockKernel {
public:
    FixedLengthCompression(const std::unique_ptr<kernel::KernelBuilder> & b,
                           EncodingInfo encodingScheme,
                           unsigned length,
                           StreamSet * const byteData,
                           StreamSet * hashValues,
                           std::vector<StreamSet *> symbolMarks,
                           StreamSet * compressionMask,
                           StreamSet * encodedBytes,
                           unsigned strideBlocks = 8);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
    void generateMultiBlockLogic(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::Value * const numOfStrides) override;
    EncodingInfo mEncodingScheme;
    unsigned mLo;
    unsigned mHi;
    size_t mSubTableSize;
};

class FixedLengthDecompression : public MultiBlockKernel {
public:
    FixedLengthDecompression(const std::unique_ptr<kernel::KernelBuilder> & b,
                             EncodingInfo encodingScheme,
                             unsigned lo,
                             StreamSet * const byteData,
                             StreamSet * const hashValues,
                             std::vector<StreamSet *> keyMarks,
                             std::vector<StreamSet *> hashMarks,
                             StreamSet * const result, unsigned strideBlocks = 8);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
    void generateMultiBlockLogic(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::Value * const numOfStrides) override;
    EncodingInfo mEncodingScheme;
    unsigned mLo;
    unsigned mHi;
    size_t mSubTableSize;
};
}
#endif
