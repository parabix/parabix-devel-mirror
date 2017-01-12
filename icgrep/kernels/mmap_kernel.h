/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef MMAP_KERNEL_H
#define MMAP_KERNEL_H

#include "streamset.h"
#include "kernel.h"
#include <llvm/IR/Type.h>

namespace IDISA { class IDISA_Builder; }

namespace kernel {

/* The MMapSourceKernel is a simple wrapper for an external MMap file buffer.
   The doSegment method of this kernel feeds one segment at a time to a 
   pipeline. */
    
class MMapSourceKernel : public KernelBuilder {
public:
    MMapSourceKernel(IDISA::IDISA_Builder * iBuilder, unsigned blocksPerSegment = 1, unsigned codeUnitWidth = 8) :
    KernelBuilder(iBuilder, "mmap_source",
                  {}, {Binding{iBuilder->getStreamSetTy(1, codeUnitWidth), "sourceBuffer"}}, 
                  {Binding{iBuilder->getSizeTy(), "fileSize"}}, {}, {}),
    mSegmentBlocks(blocksPerSegment),
    mCodeUnitWidth(codeUnitWidth) {}
    
private:
    unsigned mSegmentBlocks;
    unsigned mCodeUnitWidth;
  
    void generateDoBlockMethod() const override;
    void generateDoSegmentMethod() const override;
    
};
}

#endif
