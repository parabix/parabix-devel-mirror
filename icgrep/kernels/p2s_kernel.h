/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef P2S_KERNEL_H
#define P2S_KERNEL_H

#include "streamset.h"
#include "interface.h"
#include "kernel.h"

namespace llvm { class Module; }

namespace IDISA { class IDISA_Builder; }

namespace kernel {

   
class P2SKernel : public KernelBuilder {
public:
    P2SKernel(IDISA::IDISA_Builder * iBuilder);
    
private:
    void generateDoBlockMethod() const override;
    
};

class P2SKernelWithCompressedOutput : public KernelBuilder {
public:
    P2SKernelWithCompressedOutput(IDISA::IDISA_Builder * iBuilder);    
private:
    void generateDoBlockMethod() const override;
};
    

class P2S16Kernel : public KernelBuilder {
public:
    P2S16Kernel(IDISA::IDISA_Builder * iBuilder);    
private:
    void generateDoBlockMethod() const override;
    
};

    
class P2S16KernelWithCompressedOutput : public KernelBuilder {
public:
    P2S16KernelWithCompressedOutput(IDISA::IDISA_Builder * iBuilder);
        
private:
    void generateDoBlockMethod() const override;
    void generateFinalBlockMethod() const override;
};
    
}

#endif
