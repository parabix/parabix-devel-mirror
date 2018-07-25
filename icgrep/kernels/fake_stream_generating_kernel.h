
#ifndef ICGREP_LZ4_FAKE_STREAM_GENERATING_KERNEL_H
#define ICGREP_LZ4_FAKE_STREAM_GENERATING_KERNEL_H

#include <kernels/kernel.h>
#include <llvm/IR/Value.h>
#include <string>
#include <vector>

namespace IDISA { class IDISA_Builder; }

namespace kernel {

    class FakeStreamGeneratingKernel final : public MultiBlockKernel {
    public:
        FakeStreamGeneratingKernel(const std::unique_ptr<kernel::KernelBuilder> & b, const unsigned numberOfInputStream = 1, const unsigned numberOfOutputStreams = 1, std::string name = "LZ4FakeStreamGeneratingKernel");
        FakeStreamGeneratingKernel(const std::unique_ptr<kernel::KernelBuilder> & b, const unsigned numberOfInputStream, std::vector<unsigned> numbersOfOutputStreams, std::string name = "LZ4FakeStreamGeneratingKernel");
        bool isCachable() const override { return true; }
        bool hasSignature() const override { return false; }

    protected:
        void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, llvm::Value * const numOfStrides) final;
    };
}


#endif //ICGREP_LZ4_FAKE_STREAM_GENERATING_KERNEL_H
