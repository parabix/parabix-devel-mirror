/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef CC_KERNEL_H
#define CC_KERNEL_H

#include "streamset.h"
#include "interface.h"
#include "kernel.h"
#include <pablo/pablo_kernel.h>

namespace re {
    class CC;
}

namespace kernel {

class KernelBuilder;

class DirectCharacterClassKernelBuilder : public KernelBuilder {
public:
    
    DirectCharacterClassKernelBuilder(IDISA::IDISA_Builder * iBuilder, std::string ccSetName, std::vector<re::CC *> charClasses, unsigned codeUnitSize) :
    KernelBuilder(iBuilder, "cc",
                  {Binding{iBuilder->getStreamSetTy(1, 8 * codeUnitSize), "codeUnitStream"}},
                  {Binding{iBuilder->getStreamSetTy(charClasses.size(), 1), "ccStream"}},
                  {}, {}, {}), mCharClasses(charClasses), mCodeUnitSize(codeUnitSize) {}
    
    
private:
    void generateDoBlockMethod() override;
    std::vector<re::CC *> mCharClasses;
    unsigned mCodeUnitSize;
    
};

class ParabixCharacterClassKernelBuilder: public pablo::PabloKernel {
public:
    ParabixCharacterClassKernelBuilder(IDISA::IDISA_Builder * iBuilder, std::string ccSetName, const std::vector<re::CC *> & charClasses, unsigned basisBitsCount);
};

}
#endif
