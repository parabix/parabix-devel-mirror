/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef GREP_KERNEL_H
#define GREP_KERNEL_H

#include <pablo/pablo_kernel.h>  // for PabloKernel

namespace IDISA { class IDISA_Builder; }
namespace re { class RE; }
namespace kernel {

class ICgrepKernelBuilder: public pablo::PabloKernel {
public:
    ICgrepKernelBuilder(IDISA::IDISA_Builder * const iBuilder, re::RE * const re_ast);
    
    std::string generateKernelSignature(std::string moduleId) override;
    
    void prepareKernel() override;

private:
    re::RE * const  mRE;
    std::string     mSignature;
};

class InvertMatchesKernel : public BlockOrientedKernel {
public:
    InvertMatchesKernel(IDISA::IDISA_Builder * builder);
private:
    void generateDoBlockMethod() override;
};


class PopcountKernel : public pablo::PabloKernel {
public:
    PopcountKernel(IDISA::IDISA_Builder * builder);
};

}
#endif
