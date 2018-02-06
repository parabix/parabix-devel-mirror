
#ifndef ICGREP_LZ4_GENERATE_DEPOSIT_STREAM_H
#define ICGREP_LZ4_GENERATE_DEPOSIT_STREAM_H
#include <pablo/pablo_compiler.h>
#include <pablo/pablo_toolchain.h>
#include <pablo/pablo_kernel.h>
#include "kernels/kernel.h"


namespace kernel {
    class LZ4GenerateDepositStreamKernel final : public pablo::PabloKernel {
    public:
        LZ4GenerateDepositStreamKernel(const std::unique_ptr<kernel::KernelBuilder> & b);
        bool isCachable() const override { return true; }
        bool hasSignature() const override { return false; }

    protected:
        void generatePabloMethod() override;

    };
}

#endif //ICGREP_LZ4_GENERATE_DEPOSIT_STREAM_H
