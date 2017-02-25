/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef EDITD_CPU_KERNEL_H
#define EDITD_CPU_KERNEL_H

#include <kernels/kernel.h>

namespace llvm { class Module; }

namespace IDISA { class IDISA_Builder; }

namespace kernel {

class editdCPUKernel : public BlockOrientedKernel {
public:

    editdCPUKernel(IDISA::IDISA_Builder * b, unsigned dist, unsigned pattLen);
    

private:
    void generateDoBlockMethod() override;
    void generateFinalBlockMethod(llvm::Value * remainingBytes) override;
    void bitblock_advance_ci_co(llvm::Value * val, unsigned shift, llvm::Value * stideCarryArr, unsigned carryIdx, std::vector<std::vector<llvm::Value *>> & adv, std::vector<std::vector<int>> & calculated, int i, int j) const;
    unsigned mEditDistance;
    unsigned mPatternLen;
    
};

    

}
#endif
