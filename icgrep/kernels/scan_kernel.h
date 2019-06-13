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
        static const size_t maxStrideWidth /* = 4096; for width of 64-bits */;

        const size_t strideMaskWidth = 64;
        const size_t minScanWordWidth = 8;

        const size_t width;
        const size_t wordsPerBlock;
        const size_t wordsPerStride;
        const size_t fieldWidth;

        llvm::Type * const Ty;
        llvm::Type * const PointerTy;
        llvm::Type * const StrideMaskTy;

        llvm::Constant * const WIDTH;
        llvm::Constant * const WORDS_PER_BLOCK;
        llvm::Constant * const WORDS_PER_STRIDE;

        ScanWordContext(BuilderRef b, size_t strideWidth);
    };

    ScanKernelBase(BuilderRef b, size_t strideWidth, llvm::StringRef scanStreamSetName);

    void initializeBase(BuilderRef b);

    llvm::Value * computeStridePosition(BuilderRef b, llvm::Value * strideNumber) const;

    llvm::Value * computeStrideBlockOffset(BuilderRef b, llvm::Value * strideNo) const;

    llvm::Value * loadScanStreamBitBlock(BuilderRef b ,llvm::Value * strideNo, llvm::Value * blockNo);

    llvm::Value * orBlockIntoMask(BuilderRef b, ScanWordContext const & sw, llvm::Value * maskAccum, llvm::Value * block, llvm::Value * blockNo);

protected:
    llvm::StringRef mScanStreamSetName;
    llvm::Value *   mInitialPos;

    llvm::Constant * const sz_STRIDE_WIDTH;
    llvm::Constant * const sz_NUM_BLOCKS_PER_STRIDE;
};

// Callback must be of type: 
//          void(*)(const char * ptr, uint64_t pos)
// where `ptr` is a pointer to the chacater at the callback position in the 
// source stream and `pos` is the callback position 

class ScanKernel : public MultiBlockKernel, protected ScanKernelBase {
public:

    ScanKernel(BuilderRef b, StreamSet * scanStream, StreamSet * sourceStream, llvm::StringRef callbackName);

protected:
    void generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfStrides) override;

    llvm::StringRef mCallbackName;
};

}

#endif
