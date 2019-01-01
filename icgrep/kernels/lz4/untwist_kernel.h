

#ifndef ICGREP_UNTWIST_KERNEL_H
#define ICGREP_UNTWIST_KERNEL_H

#include <kernels/kernel.h>

namespace IDISA { class IDISA_Builder; }
namespace llvm { class Value; }

namespace kernel {

    class UntwistByPEXTKernel final : public BlockOrientedKernel{
    public:
        UntwistByPEXTKernel(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet * inputStream, StreamSet * outputStream);
    protected:
        const size_t mNumberOfOutputStream;
        const size_t mTwistWidth;
        void generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & b) override;
    };

    class UntwistMultipleByPEXTKernel final : public BlockOrientedKernel{
    public:
        UntwistMultipleByPEXTKernel(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet * inputStream, const StreamSets & outputStreams);
    protected:
        const size_t mTwistWidth;
        void generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & b) override;
    };
}


#endif //ICGREP_UNTWIST_KERNEL_H
