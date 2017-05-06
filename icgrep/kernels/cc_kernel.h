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

class DirectCharacterClassKernelBuilder final : public BlockOrientedKernel {
public:    
    DirectCharacterClassKernelBuilder(const std::unique_ptr<IDISA::IDISA_Builder> & b, std::string ccSetName, std::vector<re::CC *> charClasses, unsigned codeUnitSize);
    void generateDoBlockMethod() override;
private:
    const std::vector<re::CC *> mCharClasses;
    const unsigned              mCodeUnitSize;
    
};

class ParabixCharacterClassKernelBuilder final : public pablo::PabloKernel {
public:
    ParabixCharacterClassKernelBuilder(const std::unique_ptr<IDISA::IDISA_Builder> & b, std::string ccSetName, const std::vector<re::CC *> & charClasses, unsigned codeUnitSize);
protected:
    void prepareKernel() override;
private:
    const std::vector<re::CC *> mCharClasses;
};

}
#endif
