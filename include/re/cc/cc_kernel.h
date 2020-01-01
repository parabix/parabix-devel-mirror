/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef CC_KERNEL_H
#define CC_KERNEL_H

#include <pablo/pablo_kernel.h>
// #include <kernel/util/callback.h>

namespace IDISA { class IDISA_Builder; }
namespace re { class CC; }

namespace kernel {

struct CharacterClassesSignature {
    CharacterClassesSignature(const std::vector<re::CC *> & ccs, StreamSet * source, Scalar * signal);
protected:
    const std::string mSignature;
};

class CharacterClassKernelBuilder final : public CharacterClassesSignature, public pablo::PabloKernel {
public:    
    CharacterClassKernelBuilder(BuilderRef b, std::vector<re::CC *> charClasses, StreamSet * source, StreamSet * ccStream, Scalar * signalNullObject = nullptr);
protected:
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return true; }
    std::string makeSignature(BuilderRef) const override;
    void generatePabloMethod() override;
    Bindings makeInputScalarBindings(Scalar * signalNullObject);
private:
    const std::vector<re::CC *> mCharClasses;
    bool mAbortOnNull;
};

}
#endif
