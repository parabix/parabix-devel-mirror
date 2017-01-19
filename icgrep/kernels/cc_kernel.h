/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef CC_KERNEL_H
#define CC_KERNEL_H

#include <pablo/pablo_kernel.h>  // for PabloKernel
#include "kernel.h"              // for KernelBuilder
#include <vector>                // for vector
#include <string>                // for string
namespace IDISA { class IDISA_Builder; }
namespace re { class CC; }

namespace kernel {

class KernelBuilder;

class DirectCharacterClassKernelBuilder : public KernelBuilder {
public:
    
    DirectCharacterClassKernelBuilder(IDISA::IDISA_Builder * iBuilder, std::string ccSetName, std::vector<re::CC *> charClasses, unsigned codeUnitSize)
    : KernelBuilder(iBuilder, std::move(ccSetName),
                  {Binding{iBuilder->getStreamSetTy(1, 8 * codeUnitSize), "codeUnitStream"}},
                  {Binding{iBuilder->getStreamSetTy(charClasses.size(), 1), "ccStream"}},
                  {}, {}, {})
    , mCharClasses(charClasses)
    , mCodeUnitSize(codeUnitSize) {
    }
    
    
private:
    void generateDoBlockMethod() const override;
    std::vector<re::CC *> mCharClasses;
    unsigned mCodeUnitSize;
    
};

class ParabixCharacterClassKernelBuilder: public pablo::PabloKernel {
public:
    ParabixCharacterClassKernelBuilder(IDISA::IDISA_Builder * iBuilder, std::string ccSetName, const std::vector<re::CC *> & charClasses, unsigned basisBitsCount);
};

}
#endif
