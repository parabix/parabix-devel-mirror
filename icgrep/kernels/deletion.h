/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef DELETION_H
#define DELETION_H

#include "kernel.h"
#include <llvm/IR/Value.h>
namespace IDISA { class IDISA_Builder; }

namespace kernel {

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
    DeletionKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned fw, unsigned streamCount);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder) override;
    void generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder, llvm::Value * remainingBytes) override;
private:
    const unsigned mDeletionFieldWidth;
    const unsigned mStreamCount;
};

    class FieldCompressKernel final : public MultiBlockKernel {
    public:
        FieldCompressKernel(const std::unique_ptr<kernel::KernelBuilder> & b, unsigned fw, unsigned streamCount);
        bool isCachable() const override { return true; }
        bool hasSignature() const override { return false; }
    protected:
        void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & kb, llvm::Value * const numOfStrides) override;
    private:
        const unsigned mCompressFieldWidth;
        const unsigned mStreamCount;
    };
    

/*
Input: a set of bitstreams
Output: swizzles containing the input bitstreams with the specified bits deleted
*/
class SwizzledDeleteByPEXTkernel final : public BlockOrientedKernel {
public:
    SwizzledDeleteByPEXTkernel(const std::unique_ptr<kernel::KernelBuilder> & b, unsigned streamCount, unsigned PEXT_width = 64);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & b) override;
    void generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> & b, llvm::Value * remainingBytes) override;
private:
    void generateProcessingLoop(const std::unique_ptr<KernelBuilder> & b, llvm::Value * delMask, const bool flush);
    std::vector<std::vector<llvm::Value *>> makeSwizzleSets(const std::unique_ptr<KernelBuilder> & b, llvm::Value * delMask);
private:
    const unsigned mStreamCount;
    const unsigned mSwizzleFactor;
    const unsigned mSwizzleSetCount;
    const unsigned mPEXTWidth;
};

class DeleteByPEXTkernel final : public BlockOrientedKernel {
public:
    DeleteByPEXTkernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned fw, unsigned streamCount, unsigned PEXT_width = 64);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder) override;
    void generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder, llvm::Value * remainingBytes) override;
    void generateProcessingLoop(const std::unique_ptr<KernelBuilder> & iBuilder, llvm::Value * delMask);
private:
    const unsigned mDelCountFieldWidth;
    const unsigned mStreamCount;
    const unsigned mSwizzleFactor;
    const unsigned mPEXTWidth;
};
    
class SwizzledBitstreamCompressByCount final : public BlockOrientedKernel {
public:
    SwizzledBitstreamCompressByCount(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned bitStreamCount, unsigned fieldWidth = 64);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder) override;
    void generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder, llvm::Value * remainingBytes) override;
private:
    const unsigned mBitStreamCount;
    const unsigned mFieldWidth;
    const unsigned mSwizzleFactor;
    const unsigned mSwizzleSetCount;
};

    
}
    
#endif

