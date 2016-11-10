/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef EDITD_GPU_KERNEL_H
#define EDITD_GPU_KERNEL_H

#include <kernels/streamset.h>
#include <kernels/interface.h>
#include <kernels/kernel.h>

namespace llvm { class Module; }

namespace IDISA { class IDISA_Builder; }

namespace kernel {

class editdGPUKernel : public KernelBuilder {
public:
    
    editdGPUKernel(IDISA::IDISA_Builder * iBuilder, unsigned dist, unsigned pattLen) :
    KernelBuilder(iBuilder, "editd_gpu",
                  {Binding{parabix::StreamSetType(iBuilder, 4, 1), "CCStream"}},
                  {Binding{parabix::StreamSetType(iBuilder, dist+1, 1), "ResultStream"}},
                  {Binding{PointerType::get(iBuilder->getInt8Ty(), 1), "pattStream"}, 
                  Binding{PointerType::get(ArrayType::get(iBuilder->getBitBlockType(), pattLen * (dist + 1) * 4), 0), "srideCarry"}},
                  {},
                  {Binding{iBuilder->getBitBlockType(), "EOFmask"}}),
    mEditDistance(dist),
    mPatternLen(pattLen){}
    
    
private:
    void generateDoBlockMethod() override;
    void generateFinalBlockMethod() override;
    unsigned mEditDistance;
    unsigned mPatternLen;
    
};   

}
#endif
