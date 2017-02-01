/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef P2S_KERNEL_H
#define P2S_KERNEL_H

#include "kernel.h"  // for KernelBuilder
namespace IDISA { class IDISA_Builder; }

namespace kernel {

   
class P2SKernel : public BlockOrientedKernel {
public:
    P2SKernel(IDISA::IDISA_Builder * iBuilder);
private:
    void generateDoBlockMethod(llvm::Value * blockNo) override;
};

class P2SKernelWithCompressedOutput : public BlockOrientedKernel {
public:
    P2SKernelWithCompressedOutput(IDISA::IDISA_Builder * iBuilder);    
private:
    void generateDoBlockMethod(llvm::Value * blockNo) override;
};

class P2S16Kernel : public BlockOrientedKernel {
public:
    P2S16Kernel(IDISA::IDISA_Builder * iBuilder);    
private:
    void generateDoBlockMethod(llvm::Value * blockNo) override;
};
    
class P2S16KernelWithCompressedOutput : public BlockOrientedKernel {
public:
    P2S16KernelWithCompressedOutput(IDISA::IDISA_Builder * iBuilder);
private:
    void generateDoBlockMethod(llvm::Value * blockNo) override;
};
    
}

#endif
