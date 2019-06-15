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

    enum class OptimizeMode { Sparse, Dense };

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

    ScanKernelBase(BuilderRef b, unsigned strideWidth, llvm::StringRef scanStreamSetName, OptimizeMode optimizeMode);

    void initializeBase(BuilderRef b);

    llvm::Value * computeStridePosition(BuilderRef b, llvm::Value * strideNumber) const;

    llvm::Value * computeStrideBlockOffset(BuilderRef b, llvm::Value * strideNo) const;

    llvm::Value * loadScanStreamBitBlock(BuilderRef b ,llvm::Value * strideNo, llvm::Value * blockNo, llvm::Value * streamIndex = nullptr);

    llvm::Value * orBlockIntoMask(BuilderRef b, ScanWordContext const & sw, llvm::Value * maskAccum, llvm::Value * block, llvm::Value * blockNo);

    llvm::Value * loadScanWord(BuilderRef b, ScanWordContext const & sw, llvm::Value * wordOffset, llvm::Value * strideNo, llvm::Value * streamIndex = nullptr);

    void createOptimizedContinueProcessingBr(BuilderRef b, llvm::Value * value, llvm::BasicBlock * trueBlock, llvm::BasicBlock * falseBlock);

protected:
    unsigned        mStrideWidth;
    llvm::StringRef mScanStreamSetName;
    llvm::Value *   mInitialPos;

    llvm::Constant * const sz_STRIDE_WIDTH;
    llvm::Constant * const sz_NUM_BLOCKS_PER_STRIDE;
    OptimizeMode    mOptimizeMode;
};

/**
 * A scan kernel which accepts a single scanning stream.
 * 
 * Callback must be of type: 
 *      void(*)(const char * ptr, size_t pos)
 * where `ptr` is a pointer to the chacater at the callback position in the 
 * source stream and `pos` is the callback position  
 */ 
class ScanKernel : protected ScanKernelBase, public MultiBlockKernel {
public:

    ScanKernel(BuilderRef b, StreamSet * scanStream, StreamSet * sourceStream, llvm::StringRef callbackName, OptimizeMode optimizeModes = OptimizeMode::Sparse);

    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }

protected:
    void generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfStrides) override;

    llvm::StringRef mCallbackName;
};


/**
 * A scan kernel which accepts multiple scanning streams. All scanning streams
 * are processed in parallel to ensure that the order in which callbacks are
 * triggered directly correlates to the locations of bits in the scan streams.
 * 
 * For example, consider the 3 scanning streams X, Y, Z with callbacks triggered
 * at locations A, B, and C:
 * 
 *         ---A--B----C----
 *      X: ......1....1....
 *      Y: ...1............
 *      Z: ...........1....
 * 
 * The order of callouts will be: Y@A, X@B, X@C, Z@C
 * 
 * In the event of 2 or more streams having bits in the same location, callouts
 * are performed in the streams are indexed in the scan streamset.
 * 
 * Significant overhead is introduced in order to preserve the ordering of the
 * callouts. Therefore, if ordering does not matter, a series of single stream
 * scan kernels is recommended over the use of this kernel.
 * 
 * Callback must be of type:
 *      void(*)(const char * ptr, size_t pos, size_t idx)
 * where `ptr` is a pointer to the character at the callback position in the
 * source stream, `pos` is the callback position, and `idx` is the index of the
 * scan stream which triggered the callback.
 */
class MultiStreamScanKernel : protected ScanKernelBase, public MultiBlockKernel {
public:

    MultiStreamScanKernel(BuilderRef b, StreamSet * scanStream, StreamSet * sourceStream, llvm::StringRef callbackName, OptimizeMode optimizeMode = OptimizeMode::Sparse);

    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }

protected:
    
    void generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfStrides) override;

    llvm::StringRef mCallbackName;
};

}

#endif
