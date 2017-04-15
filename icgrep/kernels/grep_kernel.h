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
    ICgrepKernelBuilder(IDISA::IDISA_Builder * const iBuilder, re::RE * const re_ast, const bool CountOnly = false);
    
    std::string generateKernelSignature(std::string moduleId) override;
    
    void prepareKernel() override;

private:
    const bool      mCountOnly;
    re::RE * const  mRE;
    std::string     mSignature;
};

}
#endif
