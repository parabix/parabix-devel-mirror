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

   
class p2sKernel : public KernelBuilder {
public:
    p2sKernel(IDISA::IDISA_Builder * iBuilder) :
    KernelBuilder(iBuilder, "p2s",
                  {Binding{StreamSetType(iBuilder,8, 1), "basisBits"}},
                  {Binding{StreamSetType(iBuilder,1, 8), "byteStream"}},
                  {}, {}, {}) {}
    
private:
    void generateDoBlockMethod() override;
    
};

class p2sKernel_withCompressedOutput : public KernelBuilder {
public:
    p2sKernel_withCompressedOutput(IDISA::IDISA_Builder * iBuilder) :
    KernelBuilder(iBuilder, "p2s_compress",
                  {Binding{StreamSetType(iBuilder,8, 1), "basisBits"}, Binding{StreamSetType(iBuilder,1, 1), "deletionCounts"}},
                  {Binding{StreamSetType(iBuilder,1, 8), "byteStream"}},
                  {}, {}, {}) {}
    
private:
    void prepareKernel() override;
    void generateDoBlockMethod() override;
};
    

class p2s_16Kernel : public KernelBuilder {
public:
    p2s_16Kernel(IDISA::IDISA_Builder * iBuilder) :
    KernelBuilder(iBuilder, "p2s_16",
                  {Binding{StreamSetType(iBuilder,16, 1), "basisBits"}},
                  {Binding{StreamSetType(iBuilder,1, 16), "i16Stream"}},
                  {}, {}, {}) {}
    
private:
    void generateDoBlockMethod() override;
    
};

    
class p2s_16Kernel_withCompressedOutput : public KernelBuilder {
public:
    p2s_16Kernel_withCompressedOutput(IDISA::IDISA_Builder * iBuilder) :
    KernelBuilder(iBuilder, "p2s_16_compress",
                  {Binding{StreamSetType(iBuilder,16, 1), "basisBits"}, Binding{StreamSetType(iBuilder,1, 1), "deletionCounts"}},
                  {Binding{StreamSetType(iBuilder,1, 16), "i16Stream"}},
                  {},
                  {},
                  {Binding{iBuilder->getSizeTy(), "unitsGenerated"}, Binding{iBuilder->getSizeTy(), "unitsWritten"}}) {}
        
private:
    void prepareKernel() override;
    void generateDoBlockMethod() override;
    void generateFinalBlockMethod() override;
};
    
}

#endif
