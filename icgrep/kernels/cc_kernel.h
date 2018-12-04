/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef CC_KERNEL_H
#define CC_KERNEL_H

#include <pablo/pablo_kernel.h>
#include <kernels/callback.h>

namespace IDISA { class IDISA_Builder; }
namespace re { class CC; }

namespace kernel {

class DirectCharacterClassKernelBuilder final : public pablo::PabloKernel {
public:    
    DirectCharacterClassKernelBuilder(const std::unique_ptr<KernelBuilder> & b, std::string ccSetName, std::vector<re::CC *> charClasses, StreamSet * byteStream, StreamSet * ccStream, Scalar * signalNullObject = nullptr);
protected:
    void generatePabloMethod() override;
    Bindings makeInputScalarBindings(Scalar * signalNullObject);
private:
    const std::vector<re::CC *> mCharClasses;
    bool mAbortOnNull;
};

class ParabixCharacterClassKernelBuilder final : public pablo::PabloKernel {
public:
    ParabixCharacterClassKernelBuilder(const std::unique_ptr<KernelBuilder> & b, std::string ccSetName, const std::vector<re::CC *> & charClasses, StreamSet * basisStream, StreamSet * outputStream);
protected:
    void generatePabloMethod() override;
private:
    const std::vector<re::CC *> mCharClasses;
};

}
#endif
