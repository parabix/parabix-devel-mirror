/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef DELMASK_KERNEL_H
#define DELMASK_KERNEL_H

#include <pablo/pablo_kernel.h>  // for PabloKernel
#include <string>                // for string
#include <kernel/core/kernel_builder.h>

namespace kernel {

class DelMaskKernelBuilder final: public pablo::PabloKernel {
public:

    DelMaskKernelBuilder (BuilderRef iBuilder)
    : PabloKernel(iBuilder, "delmask_kernel", {Binding{iBuilder->getStreamSetTy(8, 1), "u8bit"}},
                       {Binding{iBuilder->getStreamSetTy(1, 1), "delMask"},
                        Binding{iBuilder->getStreamSetTy(1, 1), "neg_delMask"},
                        Binding{iBuilder->getStreamSetTy(1, 1), "errMask"}}, {}) {

    }
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
    void generatePabloMethod() override;

};

}
#endif
