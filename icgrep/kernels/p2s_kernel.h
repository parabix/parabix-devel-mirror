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
                  {Binding{iBuilder->getStreamSetTy(8, 1), "basisBits"}},
                  {Binding{iBuilder->getStreamSetTy(1, 8), "byteStream"}},
                  {}, {}, {}) {}
    
private:
    void generateDoBlockMethod() const override;
    
};

class p2sKernel_withCompressedOutput : public KernelBuilder {
public:
    p2sKernel_withCompressedOutput(IDISA::IDISA_Builder * iBuilder) :
    KernelBuilder(iBuilder, "p2s_compress",
                  {Binding{iBuilder->getStreamSetTy(8, 1), "basisBits"}, Binding{iBuilder->getStreamSetTy(1, 1), "deletionCounts"}},
                  {Binding{iBuilder->getStreamSetTy(1, 8), "byteStream"}},
                  {}, {}, {}) {}
    
private:
    void generateDoBlockMethod() const override;
};
    

class p2s_16Kernel : public KernelBuilder {
public:
    p2s_16Kernel(IDISA::IDISA_Builder * iBuilder) :
    KernelBuilder(iBuilder, "p2s_16",
                  {Binding{iBuilder->getStreamSetTy(16, 1), "basisBits"}},
                  {Binding{iBuilder->getStreamSetTy(1, 16), "i16Stream"}},
                  {}, {}, {}) {}
    
private:
    void generateDoBlockMethod() const override;
    
};

    
class p2s_16Kernel_withCompressedOutput : public KernelBuilder {
public:
    p2s_16Kernel_withCompressedOutput(IDISA::IDISA_Builder * iBuilder) :
    KernelBuilder(iBuilder, "p2s_16_compress",
                  {Binding{iBuilder->getStreamSetTy(16, 1), "basisBits"}, Binding{iBuilder->getStreamSetTy(1, 1), "deletionCounts"}},
                  {Binding{iBuilder->getStreamSetTy(1, 16), "i16Stream"}},
                  {},
                  {},
                  {Binding{iBuilder->getSizeTy(), "unitsGenerated"}, Binding{iBuilder->getSizeTy(), "unitsWritten"}}) {}
        
private:
    void generateDoBlockMethod() const override;
    void generateFinalBlockMethod() const override;
};
    
}

#endif
