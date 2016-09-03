/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef CC_KERNEL_H
#define CC_KERNEL_H

#include "streamset.h"
#include "interface.h"
#include "kernel.h"
#include <re/re_cc.h>
#include <pablo/pablo_kernel.h>
#include <cc/cc_compiler.h>

namespace kernel {

class KernelBuilder;



class DirectCharacterClassKernelBuilder : public KernelBuilder {
public:
    
    DirectCharacterClassKernelBuilder(IDISA::IDISA_Builder * iBuilder, std::string ccSetName, std::vector<re::CC *> charClasses, unsigned codeUnitSize) :
    KernelBuilder(iBuilder, "cc",
                  {StreamSetBinding{parabix::StreamSetType(1, 8 * codeUnitSize), "codeUnitStream"}},
                  {StreamSetBinding{parabix::StreamSetType(charClasses.size(), parabix::i1), "ccStream"}},
                  {}, {}, {}), mCharClasses(charClasses), mCodeUnitSize(codeUnitSize) {}
    
    
private:
    void generateDoBlockMethod() override;
    std::vector<re::CC *> mCharClasses;
    unsigned mCodeUnitSize;
    
};
    
    

class ParabixCharacterClassKernelBuilder: public pablo::PabloKernel {
public:
    ParabixCharacterClassKernelBuilder(IDISA::IDISA_Builder * iBuilder, std::string ccSetName, std::vector<re::CC *> charClasses, unsigned basisBitsCount) :
        PabloKernel(iBuilder, ccSetName +"_kernel", cc::ParabixCharacterClassFunction(ccSetName, charClasses, basisBitsCount), {}) {}
    
};
  

}
#endif
