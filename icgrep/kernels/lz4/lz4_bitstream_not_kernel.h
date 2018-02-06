//
//

#ifndef ICGREP_LZ4_E1_TO_E_KERNEL_H
#define ICGREP_LZ4_E1_TO_E_KERNEL_H

#include <pablo/pablo_compiler.h>
#include <pablo/pablo_toolchain.h>
#include <pablo/pablo_kernel.h>
#include "kernels/kernel.h"

namespace kernel {

    class LZ4BitStreamNotKernel final : public pablo::PabloKernel {
    public:
        LZ4BitStreamNotKernel(const std::unique_ptr<kernel::KernelBuilder> &b);

        bool isCachable() const override { return true; }

        bool hasSignature() const override { return false; }

    protected:
        void generatePabloMethod() override;
    };
}

#endif //ICGREP_LZ4_E1_TO_E_KERNEL_H
