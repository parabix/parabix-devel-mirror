/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef SCAN_KERNEL_H
#define SCAN_KERNEL_H

#include <kernels/core/kernel.h>

namespace kernel {

class ScanKernelBase {
public:

    using BuilderRef = const std::unique_ptr<KernelBuilder> &;

protected:

    struct ScanWordContext {
        static const unsigned maxStrideWidth /* = 4096; for width of 64-bits */;

        const unsigned strideMaskWidth = 64;
        const unsigned minScanWordWidth = 8;

        const unsigned width;
        const unsigned wordsPerBlock;
        const unsigned wordsPerStride;
        const unsigned fieldWidth;

        llvm::Type * const Ty;
        llvm::Type * const PointerTy;
        llvm::Type * const StrideMaskTy;

        llvm::Constant * const WIDTH;
        llvm::Constant * const WORDS_PER_BLOCK;
        llvm::Constant * const WORDS_PER_STRIDE;

        ScanWordContext(BuilderRef b, unsigned strideWidth);
    };

    ScanKernelBase(BuilderRef b, unsigned strideWidth, llvm::StringRef scanStreamSetName);

    void initializeBase(BuilderRef b);

    llvm::Value * computeStridePosition(BuilderRef b, llvm::Value * strideNumber) const;

    llvm::Value * computeStrideBlockOffset(BuilderRef b, llvm::Value * strideNo) const;

    llvm::Value * loadScanStreamBitBlock(BuilderRef b ,llvm::Value * strideNo, llvm::Value * blockNo, llvm::Value * streamIndex = nullptr);

    llvm::Value * orBlockIntoMask(BuilderRef b, ScanWordContext const & sw, llvm::Value * maskAccum, llvm::Value * block, llvm::Value * blockNo);

    llvm::Value * loadScanWord(BuilderRef b, ScanWordContext const & sw, llvm::Value * wordOffset, llvm::Value * strideNo, llvm::Value * streamIndex = nullptr);

protected:
    unsigned        mStrideWidth;
    llvm::StringRef mScanStreamSetName;
    llvm::Value *   mInitialPos;

    llvm::Constant * const sz_STRIDE_WIDTH;
    llvm::Constant * const sz_NUM_BLOCKS_PER_STRIDE;
};

// Callback must be of type: 
//          void(*)(const char * ptr, uint64_t pos)
// where `ptr` is a pointer to the chacater at the callback position in the 
// source stream and `pos` is the callback position 

class ScanKernel : protected ScanKernelBase, public MultiBlockKernel {
public:

    enum class OptimizeMode { Sparse, Dense };

    ScanKernel(BuilderRef b, StreamSet * scanStream, StreamSet * sourceStream, llvm::StringRef callbackName, OptimizeMode optimizeModes = OptimizeMode::Sparse);

protected:
    void generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfStrides) override;

    void createOptimizedContinueProcessingBr(BuilderRef b, llvm::Value * value, llvm::BasicBlock * trueBlock, llvm::BasicBlock * falseBlock);

    llvm::StringRef mCallbackName;
    OptimizeMode    mOptimizeMode;
};

}

#endif
