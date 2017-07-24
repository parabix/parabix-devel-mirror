/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef CHARCLASSES_H
#define CHARCLASSES_H

#include <pablo/pablo_kernel.h>  // for PabloKernel
#include <UCD/resolve_properties.h>

namespace kernel { class KernelBuilder; }
namespace IDISA { class IDISA_Builder; }
namespace re { class RE; }
namespace kernel {

class CharClassesKernel : public pablo::PabloKernel {
public:
    CharClassesKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, std::vector<UCD::UnicodeSet> multiplexedCCs);
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
protected:
    std::vector<UCD::UnicodeSet> mMultiplexedCCs;
};

}
#endif
