

#ifndef ICGREP_UNTWIST_KERNEL_H
#define ICGREP_UNTWIST_KERNEL_H

#include <kernels/kernel.h>

namespace IDISA { class IDISA_Builder; }
namespace llvm { class Value; }

namespace kernel {

    class UntwistByPEXTKernel final : public BlockOrientedKernel{
    public:
        UntwistByPEXTKernel(const std::unique_ptr<kernel::KernelBuilder> & b, unsigned numberOfOutputStream = 4, unsigned twistWidth = 2);
    protected:
        const size_t mNumberOfOutputStream;
        const size_t mTwistWidth;
        void generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & b) override;
    };

    class StreamCompareKernel final : public BlockOrientedKernel{
    public:
        StreamCompareKernel(const std::unique_ptr<kernel::KernelBuilder> & b, unsigned numberOfStream = 1);
    protected:
        const unsigned mNumberOfStream;
        void generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & b) override;
    };
}


#endif //ICGREP_UNTWIST_KERNEL_H
