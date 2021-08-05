/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef DELETION_H
#define DELETION_H

#include <kernel/core/kernel.h>
#include <llvm/IR/Value.h>
#include <kernel/pipeline/driver/driver.h>
#include <kernel/streamutils/stream_select.h>

namespace IDISA { class IDISA_Builder; }

namespace kernel {

/*  FilterByMask - extract selected bits of input streams according to a mask.

    One output bit is produced for every 1 bit in the mask stream.

    Input stream:    abcdefghjklmnpqra
    Mask stream:     ...1.1...111..1..     . represents 0
    Output stream:   dfjklmq

    The output stream is produced at the Popcount rate of the mask stream.

    The number of streams to process is governed by the size of the output stream set.
    The input streams to process are selected sequentially from the stream set,
    starting from the position indicated by the streamOffset value.

*/
    
void FilterByMask(const std::unique_ptr<ProgramBuilder> & P,
                  StreamSet * mask, StreamSet * inputs, StreamSet * outputs,
                  unsigned streamOffset = 0,
                  unsigned extractionFieldWidth = 64);

//
// Parallel Prefix Deletion Kernel
// see Parallel Prefix Compress in Henry S. Warren, Hacker's Delight, Chapter 7
//
// Given that we want to delete bits within fields of width fw, moving
// nondeleted bits to the right, the parallel prefix compress method can
// be applied.   This requires a preprocessing step to compute log2(fw)
// masks that can be used to select bits to be moved in each step of the
// algorithm.
//
class DeletionKernel final : public BlockOrientedKernel {
public:
    DeletionKernel(BuilderRef b, StreamSet * input, StreamSet * delMask, StreamSet * output, StreamSet * unitCounts);
protected:
    void generateDoBlockMethod(BuilderRef iBuilder) override;
    void generateFinalBlockMethod(BuilderRef iBuilder, llvm::Value * remainingBytes) override;
private:
    const unsigned mDeletionFieldWidth;
    const unsigned mStreamCount;
};

// Compress within fields of size fieldWidth.
class FieldCompressKernel final : public MultiBlockKernel {
public:
    FieldCompressKernel(BuilderRef b,
                        SelectOperation const & maskOp, SelectOperationList const & inputOps, StreamSet * outputStreamSet,
                        unsigned fieldWidth = 64);
protected:
    void generateMultiBlockLogic(BuilderRef kb, llvm::Value * const numOfStrides) override;
private:
    const unsigned mCompressFieldWidth;
    SelectedInput mMaskOp;
    SelectedInputList mInputOps;

};

class PEXTFieldCompressKernel final : public MultiBlockKernel {
public:
    PEXTFieldCompressKernel(BuilderRef b, unsigned fw, unsigned streamCount);
protected:
    void generateMultiBlockLogic(BuilderRef kb, llvm::Value * const numOfStrides) override;
private:
    const unsigned mPEXTWidth;
    const unsigned mStreamCount;
};

//
//  Given streams that are compressed within fields, produced fully
//  compressed streams.
class StreamCompressKernel final : public MultiBlockKernel {
public:
    StreamCompressKernel(BuilderRef b
                         , StreamSet * extractionMask
                         , StreamSet * source
                         , StreamSet * compressedOutput
                         , const unsigned FieldWidth = sizeof(size_t) * 8);
protected:
    void generateMultiBlockLogic(BuilderRef kb, llvm::Value * const numOfBlocks) override;
private:
    const unsigned mCompressedFieldWidth;
    const unsigned mStreamCount;
};

/*
Input: a set of bitstreams
Output: swizzles containing the input bitstreams with the specified bits deleted
*/
class SwizzledDeleteByPEXTkernel final : public MultiBlockKernel {
public:
    using SwizzleSets = std::vector<std::vector<llvm::Value *>>;
    SwizzledDeleteByPEXTkernel(BuilderRef b
                               , StreamSet * selectors, StreamSet * inputStreamSet
                               , const std::vector<StreamSet *> & outputs
                               , unsigned PEXTWidth = sizeof(size_t) * 8);
protected:
    void generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfBlocks) override;
private:
    SwizzleSets makeSwizzleSets(BuilderRef b, llvm::Value * selectors, llvm::Value * const strideIndex);
private:
    const unsigned mStreamCount;
    const unsigned mSwizzleFactor;
    const unsigned mSwizzleSetCount;
    const unsigned mPEXTWidth;
};

class DeleteByPEXTkernel final : public BlockOrientedKernel {
public:
    DeleteByPEXTkernel(BuilderRef b, unsigned fw, unsigned streamCount, unsigned PEXT_width = sizeof(size_t) * 8);
protected:
    void generateDoBlockMethod(BuilderRef iBuilder) override;
    void generateFinalBlockMethod(BuilderRef iBuilder, llvm::Value * remainingBytes) override;
    void generateProcessingLoop(BuilderRef iBuilder, llvm::Value * delMask);
private:
    const unsigned mDelCountFieldWidth;
    const unsigned mStreamCount;
    const unsigned mSwizzleFactor;
    const unsigned mPEXTWidth;
};

class SwizzledBitstreamCompressByCount final : public BlockOrientedKernel {
public:
    SwizzledBitstreamCompressByCount(BuilderRef b, unsigned bitStreamCount, unsigned fieldWidth = sizeof(size_t) * 8);
protected:
    void generateDoBlockMethod(BuilderRef iBuilder) override;
    void generateFinalBlockMethod(BuilderRef iBuilder, llvm::Value * remainingBytes) override;
private:
    const unsigned mBitStreamCount;
    const unsigned mFieldWidth;
    const unsigned mSwizzleFactor;
    const unsigned mSwizzleSetCount;
};


}

#endif

