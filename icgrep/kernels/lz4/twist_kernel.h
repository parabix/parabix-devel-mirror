

#ifndef ICGREP_TWIST_KERNEL_H
#define ICGREP_TWIST_KERNEL_H


#include "kernels/kernel.h"

namespace IDISA { class IDISA_Builder; }

namespace kernel {

    class TwistByPDEPKernel final : public BlockOrientedKernel {
    public:
        TwistByPDEPKernel(const std::unique_ptr <kernel::KernelBuilder> &b, unsigned numberOfInputStream, unsigned twistWidth);

    private:
        const unsigned mNumberOfInputStream;
        const unsigned mTwistWidth;

        void generateDoBlockMethod(const std::unique_ptr <kernel::KernelBuilder> &b) override;
    };


    class TwistMultipleByPDEPKernel final : public BlockOrientedKernel {
    public:
        TwistMultipleByPDEPKernel(const std::unique_ptr <kernel::KernelBuilder> &b, unsigned numberOfInputStreamSet, unsigned twistWidth);

    private:
        const unsigned mNumberOfInputStreamSet;
        const unsigned mTwistWidth;

        void generateDoBlockMethod(const std::unique_ptr <kernel::KernelBuilder> &b) override;
    };


}


#endif //ICGREP_TWIST_KERNEL_H
