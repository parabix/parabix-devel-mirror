/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef LINEBREAK_KERNEL_H
#define LINEBREAK_KERNEL_H

#include <pablo/pablo_kernel.h>  // for PabloKernel

namespace kernel { class KernelBuilder; }

namespace kernel {

class LineBreakKernelBuilder final : public pablo::PabloKernel {
public:
    LineBreakKernelBuilder(const std::unique_ptr<KernelBuilder> & b, unsigned basisBitsCount);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

}
#endif
