/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef SCANMATCHGEN_H
#define SCANMATCHGEN_H

#include <kernel/core/kernel.h>
namespace IDISA { class IDISA_Builder; }
namespace llvm { class Function; }
namespace llvm { class Module; }

namespace kernel {


class ScanMatchKernel : public MultiBlockKernel {
public:
    ScanMatchKernel(BuilderRef b, StreamSet * const Matches, StreamSet * const LineBreakStream, StreamSet * const ByteStream, Scalar * const callbackObject, unsigned strideBlocks = 1);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
    void generateMultiBlockLogic(BuilderRef iBuilder, llvm::Value * const numOfStrides) override;
};

class MatchCoordinatesKernel : public MultiBlockKernel {
public:
    MatchCoordinatesKernel(BuilderRef b,
                           StreamSet * const Matches, StreamSet * const LineBreakStream,
                           StreamSet * const Coordinates, unsigned strideBlocks = 1);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
    void generateMultiBlockLogic(BuilderRef iBuilder, llvm::Value * const numOfStrides) override;
};

class MatchReporter : public SegmentOrientedKernel {
public:
    MatchReporter(BuilderRef b,
                  StreamSet * ByteStream, StreamSet * const Coordinates, Scalar * const callbackObject);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
    void generateDoSegmentMethod(BuilderRef iBuilder) override;
};

class MatchFilterKernel : public MultiBlockKernel {
public:
    MatchFilterKernel(BuilderRef b, StreamSet * const MatchStarts, StreamSet * const LineBreaks,
                      StreamSet * const ByteStream, StreamSet * Output, unsigned strideBlocks = 1);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
    void generateMultiBlockLogic(BuilderRef iBuilder, llvm::Value * const numOfStrides) override;
};
}
#endif // SCANMATCHGEN_H
