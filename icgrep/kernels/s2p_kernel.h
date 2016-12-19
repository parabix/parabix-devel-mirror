/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef S2P_KERNEL_H
#define S2P_KERNEL_H

#include "streamset.h"
#include "interface.h"
#include "kernel.h"

namespace llvm { class Module; }

namespace IDISA { class IDISA_Builder; }

namespace kernel {

class KernelBuilder;

//void generateS2P_16Kernel(llvm::Module *, IDISA::IDISA_Builder * iBuilder, KernelBuilder * kBuilder);



class S2PKernel : public KernelBuilder {
public:
    
    S2PKernel(IDISA::IDISA_Builder * iBuilder) :
    KernelBuilder(iBuilder, "s2p",
                  {Binding{iBuilder->getStreamSetTy(1, 8), "byteStream"}},
                  {Binding{iBuilder->getStreamSetTy(8, 1), "basisBits"}},
                  {}, {}, {}) {}
    
    
private:
    void generateDoBlockLogic(Value * self, Value * blockNo) override;
    void generateDoBlockMethod() override;
    void generateFinalBlockMethod() override;
    
};

    

}
#endif
