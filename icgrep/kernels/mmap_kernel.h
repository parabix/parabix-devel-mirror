/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef MMAP_KERNEL_H
#define MMAP_KERNEL_H

#include "kernel.h"
namespace IDISA { class IDISA_Builder; }

namespace kernel {

/* The MMapSourceKernel is a simple wrapper for an external MMap file buffer.
   The doSegment method of this kernel feeds one segment at a time to a 
   pipeline. */
    
class MMapSourceKernel : public SegmentOrientedKernel {
public:
    MMapSourceKernel(IDISA::IDISA_Builder * iBuilder, unsigned blocksPerSegment = 1, unsigned codeUnitWidth = 8);  
private:
    void generateDoSegmentMethod(llvm::Function * function, llvm::Value * self, llvm::Value * doFinal, const std::vector<llvm::Value *> & producerPos) const override final;
private:
    const unsigned mSegmentBlocks;
    const unsigned mCodeUnitWidth;
};
}

#endif
