/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef CHARCLASSES_H
#define CHARCLASSES_H

#include <pablo/pablo_kernel.h>  // for PabloKernel

namespace kernel { class KernelBuilder; }
namespace IDISA { class IDISA_Builder; }
namespace re { class RE; class CC; }
namespace kernel {

struct CharClassesSignature {
    CharClassesSignature(const std::vector<re::CC *> & ccs);
protected:
    const std::string mSignature;
};

class CharClassesKernel : public CharClassesSignature, public pablo::PabloKernel {
public:
    CharClassesKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, std::vector<re::CC *> && ccs);
    bool hasSignature() const override { return true; }
    std::string makeSignature(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) override;
    bool isCachable() const override { return true; }
protected:
    void generatePabloMethod() override;
protected:
    std::vector<re::CC *> mCCs;
};

}
#endif
