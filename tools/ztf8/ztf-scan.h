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

class LengthGroupCompression final : public MultiBlockKernel {
public:
    LengthGroupCompression(BuilderRef b,
                           EncodingInfo encodingScheme,
                           unsigned groupNo,
                           StreamSet * symbolMarks,
                           StreamSet * hashValues,
                           StreamSet * const byteData,
                           StreamSet * compressionMask,
                           StreamSet * encodedBytes,
                           unsigned strideBlocks = 8);
private:
    void generateMultiBlockLogic(BuilderRef iBuilder, llvm::Value * const numOfStrides) override;

    const EncodingInfo mEncodingScheme;
    const unsigned mGroupNo;
};

class LengthGroupDecompression final : public MultiBlockKernel {
public:
    LengthGroupDecompression(BuilderRef b,
                             EncodingInfo encodingScheme,
                             unsigned groupNo,
                             StreamSet * keyMarks,
                             StreamSet * hashValues,
                             StreamSet * const hashMarks,
                             StreamSet * const byteData,
                             StreamSet * const result,
                             unsigned strideBlocks = 8);
private:
    void generateMultiBlockLogic(BuilderRef iBuilder, llvm::Value * const numOfStrides) override;

    const EncodingInfo mEncodingScheme;
    const unsigned mGroupNo;
};

class FixedLengthCompression final : public MultiBlockKernel {
public:
    FixedLengthCompression(BuilderRef b,
                           EncodingInfo encodingScheme,
                           unsigned length,
                           StreamSet * const byteData,
                           StreamSet * hashValues,
                           std::vector<StreamSet *> symbolMarks,
                           StreamSet * compressionMask,
                           StreamSet * encodedBytes,
                           unsigned strideBlocks = 8);
private:
    void generateMultiBlockLogic(BuilderRef iBuilder, llvm::Value * const numOfStrides) override;

    const EncodingInfo mEncodingScheme;
    const unsigned mLo;
    const unsigned mHi;
    size_t mSubTableSize;
};

class FixedLengthDecompression final : public MultiBlockKernel {
public:
    FixedLengthDecompression(BuilderRef b,
                             EncodingInfo encodingScheme,
                             unsigned lo,
                             StreamSet * const byteData,
                             StreamSet * const hashValues,
                             std::vector<StreamSet *> keyMarks,
                             std::vector<StreamSet *> hashMarks,
                             StreamSet * const result, unsigned strideBlocks = 8);
private:
    void generateMultiBlockLogic(BuilderRef iBuilder, llvm::Value * const numOfStrides) override;

    const EncodingInfo mEncodingScheme;
    const unsigned mLo;
    const unsigned mHi;
    size_t mSubTableSize;
};
}
#endif
