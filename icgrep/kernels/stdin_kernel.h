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

class StdInKernel : public SegmentOrientedKernel {
public:
    StdInKernel(IDISA::IDISA_Builder * iBuilder, unsigned blocksPerSegment = 1, unsigned codeUnitWidth = 8);
protected:
    void generateDoSegmentMethod() const override final;
private:
    unsigned mSegmentBlocks;
    unsigned mCodeUnitWidth;
};
    

class FileSource : public SegmentOrientedKernel {
public:
    FileSource(IDISA::IDISA_Builder * iBuilder, unsigned blocksPerSegment = 1, unsigned codeUnitWidth = 8);
protected:
    void generateInitMethod() const override final;
    void generateDoSegmentMethod() const override final;
private:
    unsigned mSegmentBlocks;
    unsigned mCodeUnitWidth;
  
};

}

#endif
