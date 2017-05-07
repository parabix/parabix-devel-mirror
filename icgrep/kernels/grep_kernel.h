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
    ICgrepKernelBuilder(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, re::RE * const re_ast);    
    std::string makeSignature() override;
    bool isCachable() const override { return true; }
protected:
    void generatePabloMethod() override;
private:
    re::RE * const  mRE;
    std::string     mSignature;
};

class InvertMatchesKernel : public BlockOrientedKernel {
public:
    InvertMatchesKernel(const std::unique_ptr<kernel::KernelBuilder> & builder);
private:
    void generateDoBlockMethod() override;
};


class PopcountKernel : public pablo::PabloKernel {
public:
    PopcountKernel(const std::unique_ptr<kernel::KernelBuilder> & builder);
};

}
#endif
