/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef STDIN_KERNEL_H
#define STDIN_KERNEL_H

#include "streamset.h"
#include "kernel.h"
#include <llvm/IR/Type.h>

namespace IDISA { class IDISA_Builder; }

namespace kernel {

class StdInKernel final : public SegmentOrientedKernel {
public:
    StdInKernel(IDISA::IDISA_Builder * iBuilder, unsigned blocksPerSegment = 1, unsigned codeUnitWidth = 8);
    bool moduleIDisSignature() override { return true; }
protected:
    void generateDoSegmentMethod(llvm::Value * doFinal, const std::vector<llvm::Value *> & producerPos) override;
private:
    unsigned mSegmentBlocks;
    unsigned mCodeUnitWidth;
};

class FileSourceKernel final : public SegmentOrientedKernel {
public:
    FileSourceKernel(IDISA::IDISA_Builder * iBuilder, llvm::Type * fileSourceTy, unsigned blocksPerSegment = 1, unsigned codeUnitWidth = 8);
    bool moduleIDisSignature() override { return true; }
protected:
    void generateInitMethod() override;
    void generateDoSegmentMethod(llvm::Value * doFinal, const std::vector<llvm::Value *> & producerPos) override;
private:
    unsigned mSegmentBlocks;
    unsigned mCodeUnitWidth;
  };

}

#endif
