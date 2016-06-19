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



class s2pKernel : public KernelBuilder {
public:
    s2pKernel(IDISA::IDISA_Builder * iBuilder) :
    KernelBuilder(iBuilder, "s2p",
                  {StreamSetBinding{StreamSetType(1, 8), "byteStream"}},
                  {StreamSetBinding{StreamSetType(8, 1), "basisBits"}},
                  {}, {}, {}) {}
    
    void generateFinalBlockMethod();
    void generateKernel() override;
    
};

    

}
#endif
