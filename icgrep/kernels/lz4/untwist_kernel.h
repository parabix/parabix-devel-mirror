

#ifndef ICGREP_UNTWIST_KERNEL_H
#define ICGREP_UNTWIST_KERNEL_H

#include <kernels/kernel.h>

namespace IDISA { class IDISA_Builder; }
namespace llvm { class Value; }

namespace kernel {

    class UntwistByPEXTKernel final : public BlockOrientedKernel{
    public:
        UntwistByPEXTKernel(const std::unique_ptr<kernel::KernelBuilder> & b, unsigned numberOfOutputStream, unsigned twistWidth);
    protected:
        const size_t mNumberOfOutputStream;
        const size_t mTwistWidth;
        void generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & b) override;
    };

    class UntwistMultipleByPEXTKernel final : public BlockOrientedKernel{
    public:
        UntwistMultipleByPEXTKernel(const std::unique_ptr<kernel::KernelBuilder> & b, std::vector<unsigned> numberOfOutputStreams, unsigned twistWidth);
    protected:
        const std::vector<unsigned> mNumberOfOutputStreams;
        const size_t mTwistWidth;
        void generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & b) override;
    };
}


#endif //ICGREP_UNTWIST_KERNEL_H
