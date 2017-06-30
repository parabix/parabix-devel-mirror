/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef DELETION_H
#define DELETION_H

#include "kernel.h"
#include <llvm/IR/Value.h>
namespace IDISA { class IDISA_Builder; }

//
// Parallel Prefix Deletion 
// see Parallel Prefix Compress in Henry S. Warren, Hacker's Delight, Chapter 7
// 
// Given that we want to delete bits within fields of width fw, moving
// nondeleted bits to the right, the parallel prefix compress method can
// be applied.   This requires a preprocessing step to compute log2(fw) 
// masks that can be used to select bits to be moved in each step of the
// algorithm.
//

namespace kernel {

/*
Input: a set of bitstreams
Output: swizzles containing the input bitstreams with the specified bits deleted
*/
class SwizzledDeleteByPEXTkernel final : public BlockOrientedKernel {
public:
    SwizzledDeleteByPEXTkernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned fw, unsigned streamCount, unsigned PEXT_width = 64);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder) override;
    void generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder, llvm::Value * remainingBytes) override;
    std::vector<llvm::Value *> get_PEXT_masks(const std::unique_ptr<KernelBuilder> & iBuilder, llvm::Value * del_mask);
    void generateProcessingLoop(const std::unique_ptr<KernelBuilder> & iBuilder, const std::vector<llvm::Value *> & masks, 
                                llvm::Value * delMask);
    void generatePEXTAndSwizzleLoop(const std::unique_ptr<KernelBuilder> & iBuilder, const std::vector<llvm::Value *> & masks, std::vector<llvm::Value *> counts);
    std::vector<llvm::Value *> apply_PEXT_deletion_with_swizzle(const std::unique_ptr<KernelBuilder> & iBuilder, 
                                                                const std::vector<llvm::Value *> & masks, std::vector<llvm::Value *> strms);
    llvm::Value * apply_PEXT_deletion(const std::unique_ptr<KernelBuilder> & iBuilder, const std::vector<llvm::Value *> & masks,
                                      llvm::Value * strm);
private:
    const unsigned mDelCountFieldWidth;
    const unsigned mStreamCount;
    const unsigned mSwizzleFactor;
    const unsigned mSwizzleSetCount;
    const unsigned mPEXTWidth;
    static constexpr const char* mOutputSwizzleNameBase = "outputStreamSet";
};

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

class DeleteByPEXTkernel final : public BlockOrientedKernel {
public:
    DeleteByPEXTkernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned fw, unsigned streamCount, bool shouldSwizzle);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder) override;
    void generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder, llvm::Value * remainingBytes) override;
    void generatePEXTAndSwizzleLoop(const std::unique_ptr<KernelBuilder> & iBuilder, const std::vector<llvm::Value *> & masks);
    void generatePEXTLoop(const std::unique_ptr<KernelBuilder> & iBuilder, const std::vector<llvm::Value *> & masks);
    void generateProcessingLoop(const std::unique_ptr<KernelBuilder> & iBuilder, const std::vector<llvm::Value *> & masks, llvm::Value * delMask);
private:
    const unsigned mDelCountFieldWidth;
    const unsigned mStreamCount;
    const unsigned mSwizzleFactor;
    const bool mShouldSwizzle;
    static constexpr const char* mOutputSwizzleNameBase = "outputStreamSet";
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

