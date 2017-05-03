/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef LINEBREAK_KERNEL_H
#define LINEBREAK_KERNEL_H

#include <pablo/pablo_kernel.h>  // for PabloKernel

namespace IDISA { class IDISA_Builder; }

namespace kernel {

class LineBreakKernelBuilder: public pablo::PabloKernel {
public:
    LineBreakKernelBuilder(IDISA::IDISA_Builder * iBuilder, unsigned basisBitsCount);
    bool isCachable() const override { return true; }
    bool moduleIDisSignature() const override { return true; }
};

}
#endif
