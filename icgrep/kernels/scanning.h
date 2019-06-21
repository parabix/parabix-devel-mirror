/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef LINE_BASED_SCAN_KERNEL_H
#define LINE_BASED_SCAN_KERNEL_H

#include <kernels/core/kernel.h>

#define CACHEABLE \
bool isCachable() const override { return true; } \
bool hasSignature() const override { return false; }

namespace kernel {

namespace generic {

class SingleStreamScanKernelTemplate : public MultiBlockKernel {
public:
    using BuilderRef = const std::unique_ptr<KernelBuilder> &;

    SingleStreamScanKernelTemplate(BuilderRef b, std::string && name, StreamSet * scan, StreamSet * source);

    virtual ~SingleStreamScanKernelTemplate() {}

    static const uint32_t MaxStrideWidth;

protected:

    struct ScanWordContext {
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
        llvm::Constant * const NUM_BLOCKS_PER_STRIDE;

        ScanWordContext(BuilderRef b, unsigned strideWidth);
    };

    CACHEABLE

    void generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfStrides) final override;

    virtual void initialize(BuilderRef b) { }

    virtual void willProcessStride(BuilderRef b, llvm::Value * const strideNo) { }

    virtual void maskBuildingIterationHead(BuilderRef b) { }

    virtual void maskBuildingIterationBody(BuilderRef b, llvm::Value * const blockIndex) { }

    virtual void didBuildMask(BuilderRef b, llvm::Value * const mask) { }

    virtual void willProcessWord(BuilderRef b, llvm::Value * const word) { }

    virtual void generateProcessingLogic(BuilderRef b, llvm::Value * const absoluteIndex) { }

    virtual void generateProcessingLogic(BuilderRef b, llvm::Value * const absoluteIndex, llvm::Value * const blockIndex, llvm::Value * const bitOffset) { 
        generateProcessingLogic(b, absoluteIndex);
    }

    virtual void didProcessWord(BuilderRef b) { }

    virtual void didProcessStride(BuilderRef b, llvm::Value * strideNo) { }

    virtual void finalize(BuilderRef b) { }

    ScanWordContext mSW;
    llvm::BasicBlock * mEntryBlock = nullptr;
    llvm::BasicBlock * mStrideStart = nullptr;
    llvm::BasicBlock * mBuildStrideMask = nullptr;
    llvm::BasicBlock * mMaskReady = nullptr;
    llvm::BasicBlock * mProcessMask = nullptr;
    llvm::BasicBlock * mProcessWord = nullptr;
    llvm::BasicBlock * mWordDone = nullptr;
    llvm::BasicBlock * mStrideDone = nullptr;
    llvm::BasicBlock * mExitBlock = nullptr;
    llvm::Value * mStrideNo = nullptr;
    llvm::Value * mBlockNo = nullptr;
    llvm::Value * mWordOffset = nullptr;
    llvm::Value * mProcessingMask = nullptr;
    llvm::Value * mProcessingWord = nullptr;
};

} // namespace kernel::generic

/**
 * Converts a stream of callback positions to pointer, line # pairs based on a
 * given source stream.
 * 
 * The output streamset is indexed as follows:
 *  [0]: source pointers
 *  [1]: line numbers
 * 
 * kernel ScanPositionGenerator 
 * :: [<i1>[1] scan, <i1>[1] linebreaks, <i8>[1] source]
 * -> [<i64>[2] output]
 */
class ScanPositionGenerator : public generic::SingleStreamScanKernelTemplate {
public:
    ScanPositionGenerator(BuilderRef b, StreamSet * scan, StreamSet * linebreakStream, StreamSet * source, StreamSet * output);
protected:
    void initialize(BuilderRef b) override;
    void willProcessStride(BuilderRef b, llvm::Value * const strideNo) override;
    void maskBuildingIterationHead(BuilderRef b) override;
    void maskBuildingIterationBody(BuilderRef b, llvm::Value * const blockIndex) override;
    void generateProcessingLogic(BuilderRef b, llvm::Value * const absoluteIndex, llvm::Value * const blockIndex, llvm::Value * const bitOffset) override;
    void didProcessStride(BuilderRef b, llvm::Value * const strideNo) override;
private:
    llvm::Value *   mLineCountArrayBlockPtr = nullptr;
    llvm::Value *   mInitialLineNum = nullptr;
    llvm::Value *   mHighestLineCount = nullptr;
    llvm::PHINode * mBaseCounts = nullptr;
};

/**
 * Converts a linebreak stream into start and end pointers in a source stream.
 * 
 * kernel LineSpanGenerator :: [<i1>[1] lbs, <i8>[1] src] -> [<i64>[2] ptrs]
 */
class LineSpanGenerator : public generic::SingleStreamScanKernelTemplate {
public:
    LineSpanGenerator(BuilderRef b, StreamSet * linebreakStream, StreamSet * source, StreamSet * output);
protected:
    void initialize(BuilderRef b) override;
    void generateProcessingLogic(BuilderRef b, llvm::Value * absoluteIndex) override;
};

/**
 * Reads from a stream of scan positions and linebreak spans to trigger a
 * registered callback function.
 * 
 * Callback function must take the form:
 *  void (*) (const uint8_t * pos, const uint8_t * line_begin, const uint8_t * line_end, uint64_t line_num);
 */
class LineBasedScanPositionReader : public MultiBlockKernel {
public:
    using BuilderRef = const std::unique_ptr<KernelBuilder> &;
    LineBasedScanPositionReader(BuilderRef b, StreamSet * scanPositions, StreamSet * lineSpans, llvm::StringRef callbackName);
protected:
    CACHEABLE
    void generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfStrides) override;
private:
    llvm::StringRef mCallbackName;
};

} // namespace kernel

#undef CACHEABLE

#endif
