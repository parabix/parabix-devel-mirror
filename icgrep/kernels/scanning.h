/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef LINE_BASED_SCAN_KERNEL_H
#define LINE_BASED_SCAN_KERNEL_H

#include <kernels/core/kernel.h>

namespace kernel {

namespace generic {

/**
 * Abstract scanning kernel class. Implements a basic scanning framework,
 * delegating procssing logic on scan bits to subclasses.
 * 
 * Accepts only a single scanning stream (i.e., <i1>[1]).
 */
class SingleStreamScanKernelTemplate : public MultiBlockKernel {
public:
    using BuilderRef = const std::unique_ptr<KernelBuilder> &;

    SingleStreamScanKernelTemplate(BuilderRef b, std::string && name, StreamSet * scan);

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

    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }

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
 * Generates a stream of indices corresponding to the offset of each bit in a
 * given scan stream.
 *
 * The production rate for the ouput stream is PopcountOf(scan).
 *
 * Signature:
 *  kernel ScanIndexGenerator :: [<i1>[1] scan] -> [<i64>[1] output]
 *
 * Example:
 *  scan        : 11...... ....1...
 *  output      : 0, 1, 12
 */
class ScanIndexGenerator : public generic::SingleStreamScanKernelTemplate {
public:
    ScanIndexGenerator(BuilderRef b, StreamSet * scan, StreamSet * output);
protected:
    void generateProcessingLogic(BuilderRef b, llvm::Value * const absoluteIndex, llvm::Value * const blockIndex, llvm::Value * const bitOffset) override;
};

/**
 * Generates a stream of line numbers corresponding to each bit in a given scan
 * stream. Resultant line numbers are zero-indexed.
 *
 * The production rate for the output stream is PopcountOf(scan).
 *
 * Signature:
 *  kernel LineNumberGenerator :: [<i1>[1] scan, <i1>[1] linebreaks] -> [<i64>[1] output]
 *
 * Example:
 *  scan        : .1...... ....1...
 *  linebreaks  : ...1.... 1..1...1
 *  output      : 0, 3
 */
class LineNumberGenerator : public generic::SingleStreamScanKernelTemplate {
public:
    LineNumberGenerator(BuilderRef b, StreamSet * scan, StreamSet * linebreaks, StreamSet * output);
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
 * Converts a linebreak stream into start and end indices.
 *
 * The production rate for the output stream is PopcountOf(linebreaks).
 *
 * Signature:
 *  kernel LineSpanGenerator :: [<i1>[1] linebreaks, <i8>[1] src] -> [<i64>[2] output]
 *
 * Example:
 *  linebreaks  : ...1.... 1......1
 *  output   [0]: 0, 4, 9
 *           [1]: 3, 8, 15
 */
class LineSpanGenerator : public generic::SingleStreamScanKernelTemplate {
public:
    LineSpanGenerator(BuilderRef b, StreamSet * linebreaks, StreamSet * output);
protected:
    void initialize(BuilderRef b) override;
    void generateProcessingLogic(BuilderRef b, llvm::Value * absoluteIndex) override;
    void finalize(BuilderRef b) override;
private:
    llvm::BasicBlock * mFinalBlock = nullptr;
    llvm::BasicBlock * mLineSpanExit = nullptr;
};


using AdditionalStreams = std::initializer_list<StreamSet *>;

/**
 * Reads a stream of scan indices to trigger a callback function.
 *
 * The scan indices are converted to pointers to elements in a source stream.
 *
 * Optionally, additional streams may be suplied to supplement the scan pointer
 * when performing the callback. A single element will be extracted from each
 * stream and passed to the callback as a parameter. For example, adding a
 * stream of line numbers (generated via the LineNumberGenerator kernel) will
 * pass the line number of of each scan position to the callback.
 *
 * Any additional streams will be processed at a rate equal to the scan stream.
 * Items from additional streams will be passed to the callback in the same
 * order that the streams appear in the kernel's constructor.
 *
 * Signature:
 *  kernel ScanReader :: [<i8>[1] source, <i64>[1] scanIndices, func(i8*, ...), ...] -> []
 * 
 * Callback Signature:
 *  void (*) (const uint8_t * scanPtr, ...);
 *
 * Code Example:
 *  Supplying line numbers to a callback:
 *
 *      extern "C" void callback(const uint8_t * ptr, uint64_t linenum) { ... }
 *      // ...
 *      StreamSet * const source = ...;
 *      StreamSet * const scanbits = ...;
 *      StreamSet * const linebreaks = ...;
 *
 *      StreamSet * const scanIndices = P->CreateStreamSet(1, 64);
 *      P->CreateKernelCall<ScanIndexGenerator>(scanbits, scanIndices);
 *      StreamSet * const linenums = P->CreateStreamSet(1, 64);
 *      P->CreateKernelCall<LineNumberGenerator>(scanbits, linebreaks, linenums);
 *      Kernel * const reader = P->CreateKernelCall<ScanReader>(source, scanIndices, "callback", AdditionalStreams{linenums});
 *      pxDriver.LinkFunction(reader, "callback", callback);
 * 
 *  Note that in this example, the line numbers are zero-indexed when passed to
 *  the callout. This differs from the LineBasedKernel which internally converts
 *  line numbers to one-indexed before passing them to the callback.
 */
class ScanReader : public MultiBlockKernel {
public:
    using BuilderRef = const std::unique_ptr<KernelBuilder> &;
    ScanReader(BuilderRef b, StreamSet * source, StreamSet * scanIndices, llvm::StringRef callbackName);
    ScanReader(BuilderRef b, StreamSet * source, StreamSet * scanIndices, llvm::StringRef callbackName, AdditionalStreams additionalStreams);
protected:
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
    void generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfStrides) override;
private:
    llvm::StringRef          mCallbackName;
    std::vector<std::string> mAdditionalStreamNames;
};

/**
 * Reads a stream of scan indices, line numbers, and line spans to trigger
 * callbacks parameterized with pointers to scan positions as well as pointers
 * to the beinging and ends of lines.
 * 
 * Optionally, additional streams may be added which will be consumed at the
 * same rate as the scan positions, one item per callback. Items from additional
 * streams will be passed to the callback in the same order that the streams
 * appear in the kernel's constructor.
 * 
 * Signature:
 *  kernel LineBasedReader :: [<i64>[1] scanIndices,
 *                             <i64>[1] lineNumbers,
 *                             <i64>[2] lineSpans,
 *                             <i8>[1] source,
 *                             func(i8*, i8*, i8*, i64, ...),
 *                             ...] -> []
 * 
 * Callback Signature:
 *  void (*) (const uint8_t * scanPtr,
 *            const uint8_t * lineBegin,
 *            const uint8_t * lineEnd,
 *            uint64_t lineNumber,
 *            ...);
 */
class LineBasedReader : public MultiBlockKernel {
public:
    using BuilderRef = const std::unique_ptr<KernelBuilder> &;
    LineBasedReader(BuilderRef b, 
                    StreamSet * source, 
                    StreamSet * scanIndices, 
                    StreamSet * lineNumbers, 
                    StreamSet * lineSpans, 
                    llvm::StringRef callbackName);

    LineBasedReader(BuilderRef b, 
                    StreamSet * source, 
                    StreamSet * scanIndices, 
                    StreamSet * lineNumbers, 
                    StreamSet * lineSpans, 
                    llvm::StringRef callbackName, 
                    AdditionalStreams additionalStreams);
protected:
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
    void generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfStrides) override;
private:
    llvm::StringRef          mCallbackName;
    std::vector<std::string> mAdditionalStreamNames;
};

} // namespace kernel

#endif
