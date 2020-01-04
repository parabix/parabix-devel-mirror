/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef U8U32_KERNEL_H
#define U8U32_KERNEL_H

#include <pablo/pablo_kernel.h>  // for PabloKernel
#include <string>                // for string
#include <kernel/core/kernel_builder.h>

namespace kernel {

class U8U32KernelBuilder final: public pablo::PabloKernel {
public:

    U8U32KernelBuilder (BuilderRef iBuilder, std::string u8u32)
    : PabloKernel(iBuilder, u8u32 +"_kernel", {Binding{iBuilder->getStreamSetTy(8, 1), "u8bit"}},
                       {Binding{iBuilder->getStreamSetTy(24, 1), "u32bit"},
                        Binding{iBuilder->getStreamSetTy(1, 1), "delMask"},
                        Binding{iBuilder->getStreamSetTy(1, 1), "errMask"}}, {}) {

    }

    void generatePabloMethod() override;

};

}
#endif
