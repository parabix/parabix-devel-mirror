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
                  {StreamSetBinding{StreamSetType(8, 1), "basisBits"}},
                  {StreamSetBinding{StreamSetType(1, 8), "byteStream"}},
                  {}, {}, {}) {}
    
private:
    void generateDoBlockMethod() override;
    
};

class p2sKernel_withCompressedOutput : public KernelBuilder {
public:
    p2sKernel_withCompressedOutput(IDISA::IDISA_Builder * iBuilder) :
    KernelBuilder(iBuilder, "p2s_compress",
                  {StreamSetBinding{StreamSetType(8, 1), "basisBits"}, StreamSetBinding{StreamSetType(1, 1), "deletionCounts"}},
                  {StreamSetBinding{StreamSetType(1, 8), "byteStream"}},
                  {}, {}, {}) {}
    
private:
    void prepareKernel() override;
    void generateDoBlockMethod() override;
};
    

class p2s_16Kernel : public KernelBuilder {
public:
    p2s_16Kernel(IDISA::IDISA_Builder * iBuilder) :
    KernelBuilder(iBuilder, "p2s_16",
                  {StreamSetBinding{StreamSetType(16, 1), "basisBits"}},
                  {StreamSetBinding{StreamSetType(1, 16), "i16Stream"}},
                  {}, {}, {}) {}
    
private:
    void generateDoBlockMethod() override;
    
};

    
class p2s_16Kernel_withCompressedOutput : public KernelBuilder {
public:
    p2s_16Kernel_withCompressedOutput(IDISA::IDISA_Builder * iBuilder) :
    KernelBuilder(iBuilder, "p2s_16_compress",
                  {StreamSetBinding{StreamSetType(16, 1), "basisBits"}, StreamSetBinding{StreamSetType(1, 1), "deletionCounts"}},
                  {StreamSetBinding{StreamSetType(1, 16), "i16Stream"}},
                  {},
                  {},
                  {ScalarBinding{iBuilder->getSizeTy(), "unitsGenerated"}, ScalarBinding{iBuilder->getSizeTy(), "unitsWritten"}}) {}
        
private:
    void prepareKernel() override;
    void generateDoBlockMethod() override;
    void generateFinalBlockMethod() override;
};
    
}

#endif
