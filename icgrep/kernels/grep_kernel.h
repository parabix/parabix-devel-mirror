/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef GREP_KERNEL_H
#define GREP_KERNEL_H

#include <pablo/pablo_kernel.h>  // for PabloKernel
#include "kernel.h"              // for KernelBuilder
#include <re/re_re.h>
#include <vector>                // for vector
#include <string>                // for string
namespace IDISA { class IDISA_Builder; }

namespace kernel {

class ICgrepKernelBuilder: public pablo::PabloKernel {
public:
    ICgrepKernelBuilder(IDISA::IDISA_Builder * iBuilder, re::RE * re_ast, bool CountOnly = false);
    
    std::string generateKernelSignature(std::string moduleId) override;
    
    void prepareKernel() override;

private:
    bool mCountOnly;
    re::RE * mRE;
    std::string mSignature;
};

}
#endif
