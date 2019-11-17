/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef EDITD_GPU_KERNEL_H
#define EDITD_GPU_KERNEL_H

#include <kernel/core/kernel.h>

namespace llvm { class Module; }

namespace IDISA { class IDISA_Builder; }

namespace kernel {

class editdGPUKernel : public BlockOrientedKernel {
public:

    editdGPUKernel(BuilderRef b, unsigned dist, unsigned pattLen, unsigned groupSize);


private:
    void generateDoBlockMethod(BuilderRef idb) override;
    void generateFinalBlockMethod(BuilderRef idb, llvm::Value * remainingBytes) override;
    unsigned mEditDistance;
    unsigned mPatternLen;
    unsigned mGroupSize;

};

}
#endif
