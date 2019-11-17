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

class CharacterClassKernelBuilder final : public pablo::PabloKernel {
public:    
    CharacterClassKernelBuilder(BuilderRef b, std::string ccSetName, std::vector<re::CC *> charClasses, StreamSet * byteStream, StreamSet * ccStream, Scalar * signalNullObject = nullptr);
protected:
    void generatePabloMethod() override;
    Bindings makeInputScalarBindings(Scalar * signalNullObject);
private:
    const std::vector<re::CC *> mCharClasses;
    bool mAbortOnNull;
};

}
#endif
