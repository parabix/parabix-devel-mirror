/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef EDITD_CPU_KERNEL_H
#define EDITD_CPU_KERNEL_H

#include <kernels/streamset.h>
#include <kernels/interface.h>
#include <kernels/kernel.h>

namespace llvm { class Module; }

namespace IDISA { class IDISA_Builder; }

namespace kernel {

class editdCPUKernel : public KernelBuilder {
public:
    
    editdCPUKernel(IDISA::IDISA_Builder * b, unsigned dist, unsigned pattLen) :
    KernelBuilder(b, "editd_cpu",
                  {Binding{b->getStreamSetTy(4), "CCStream"}},
                  {Binding{b->getStreamSetTy(dist + 1), "ResultStream"}},
                  {Binding{PointerType::get(b->getInt8Ty(), 1), "pattStream"},
                  Binding{PointerType::get(ArrayType::get(b->getBitBlockType(), pattLen * (dist + 1) * 4), 0), "srideCarry"}},
                  {},
                  {Binding{b->getBitBlockType(), "EOFmask"}}),
    mEditDistance(dist),
    mPatternLen(pattLen){}
    
    
private:
    void generateDoBlockMethod() const override;
    void generateFinalBlockMethod() const override;
    void bitblock_advance_ci_co(Value * val, unsigned shift, Value * stideCarryArr, unsigned carryIdx, std::vector<std::vector<Value *>> & adv, std::vector<std::vector<int>> & calculated, int i, int j) const;
    unsigned mEditDistance;
    unsigned mPatternLen;
    
};

    

}
#endif
