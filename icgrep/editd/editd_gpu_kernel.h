/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef EDITD_GPU_KERNEL_H
#define EDITD_GPU_KERNEL_H

#include <kernels/kernel.h>

namespace llvm { class Module; }

namespace IDISA { class IDISA_Builder; }

namespace kernel {

class editdGPUKernel : public BlockOrientedKernel {
public:
    
    editdGPUKernel(const std::unique_ptr<kernel::KernelBuilder> & b, unsigned dist, unsigned pattLen);
    
    
private:
    void generateDoBlockMethod() override;
    void generateFinalBlockMethod(llvm::Value * remainingBytes) override;
    unsigned mEditDistance;
    unsigned mPatternLen;
    
};   

}
#endif
