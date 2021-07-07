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



/*
1. Kernel is executed for numOfStrides strides.
2. Every stride is comprised of b number of blocks. Each block is the width of the SIMD register.
3. A given stride may contain more than one keyword/phrases that are to be considered for compression.
4. At a time, mNumSym such strides are analyzed to check for non-overlapping keyword/phrases.
5. One keyword/phrase from a stride is looked at every time which may be overlapping.
    Eg: aAbcd eEfgh iIjkl mMnop qQrstu vVwxyz ABCDE
    4 word phrases: aAbcd eEfgh iIjkl mMnop, eEfgh iIjkl mMnop qQrstu, iIjkl mMnop qQrstu vVwxyz, mMnop qQrstu vVwxyz ABCDE
                          len = 20                   len = 20                len = 20                   len = 20
    all the 4 phrases are in the same length group and are overlapping.
    Case 1: If none of the phrases have an entry in the scalar hashtable -> save all the 4 phrases in the hashTable.
    Case 2: If one of the phrases has an entry in the hashTable -> try to compress that phrase and skip storing phrases in hashTable
            If not possible, store all 4 entries in the scalar HashTable.
    Case 3: If more than one phrases has an entry in the hashTable. Try compressing the first phrase.
            (Extend to check all the phrases for compression that have an entry)
6. Mark the current phrase as analyzed and move to the next phrase in all mNumSym strides
7. Once all the phrases in the current stride are analyzed, move to the next stride
8. Once all the strides are analyzed, the kernel may execute next set of strides
*/
class PhraseCompression final : public MultiBlockKernel {
public:
    PhraseCompression(BuilderRef b,
                           EncodingInfo encodingScheme,
                           unsigned groupNo,
                           std::vector<StreamSet *> symbolMarks,
                           std::vector<StreamSet *> hashValues,
                           StreamSet * const byteData,
                           StreamSet * compressionMask,
                           StreamSet * encodedBytes,
                           StreamSet * compSymSeq,
                           unsigned strideBlocks = 8);
private:
    void generateMultiBlockLogic(BuilderRef iBuilder, llvm::Value * const numOfStrides) override;

    const EncodingInfo mEncodingScheme;
    const unsigned mGroupNo;
    const unsigned mNumSym;
};

}
#endif
