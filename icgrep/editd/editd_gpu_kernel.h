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

class editdGPUKernel : public KernelBuilder {
public:
    
    editdGPUKernel(IDISA::IDISA_Builder * b, unsigned dist, unsigned pattLen);
    
    
private:
    void generateDoBlockMethod() const override;
    void generateFinalBlockMethod() const override;
    unsigned mEditDistance;
    unsigned mPatternLen;
    
};   

}
#endif
