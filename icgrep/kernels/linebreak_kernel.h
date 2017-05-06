/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef LINEBREAK_KERNEL_H
#define LINEBREAK_KERNEL_H

#include <pablo/pablo_kernel.h>  // for PabloKernel

namespace IDISA { class IDISA_Builder; }

namespace kernel {

class LineBreakKernelBuilder final : public pablo::PabloKernel {
public:
    LineBreakKernelBuilder(const std::unique_ptr<IDISA::IDISA_Builder> & b, unsigned basisBitsCount);
    bool isCachable() const override { return true; }
    bool moduleIDisSignature() const override { return true; }
protected:
    void prepareKernel() override;
};

}
#endif
