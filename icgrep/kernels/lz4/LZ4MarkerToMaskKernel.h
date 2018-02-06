//
// Created by wxy325 on 2017/8/15.
//

#ifndef ICGREP_LZ4MARKERTOMASKKERNEL_H
#define ICGREP_LZ4MARKERTOMASKKERNEL_H

#include <pablo/pablo_compiler.h>
#include <pablo/pablo_toolchain.h>
#include <pablo/pablo_kernel.h>
#include "kernels/kernel.h"


namespace kernel {
    class LZ4MarkerToMaskKernel final : public pablo::PabloKernel {
    public:
        LZ4MarkerToMaskKernel(std::string kernelName, const std::unique_ptr<kernel::KernelBuilder> & b);
        bool isCachable() const override { return true; }
        bool hasSignature() const override { return false; }

    protected:
        void generatePabloMethod() override;

    };
}


#endif //ICGREP_LZ4MARKERTOMASKKERNEL_H
