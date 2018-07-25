
#include "fake_stream_generating_kernel.h"
#include <kernels/kernel_builder.h>
#include <llvm/Support/raw_ostream.h>
#include <toolchain/toolchain.h>

using namespace llvm;

namespace kernel {

    FakeStreamGeneratingKernel::FakeStreamGeneratingKernel(const std::unique_ptr<kernel::KernelBuilder> &b,
                                                                 const unsigned numberOfInputStream,
                                                                 const unsigned numberOfOutputStream,
                                                                 std::string name)
            : FakeStreamGeneratingKernel(b, numberOfInputStream, std::vector<unsigned>({numberOfOutputStream}), name) {

    }

    FakeStreamGeneratingKernel::FakeStreamGeneratingKernel(const std::unique_ptr<kernel::KernelBuilder> &b,
                                                           const unsigned numberOfInputStream,
                                                           std::vector<unsigned> numbersOfOutputStreams,
                                                           std::string name)
            : MultiBlockKernel(std::move(name),
// input stream sets
                               {Binding{b->getStreamSetTy(numberOfInputStream), "inputStream", FixedRate(), Principal()}},
// output stream set
                               {},
                               {}, {}, {}) {
        for (unsigned i = 0; i < numbersOfOutputStreams.size(); i++) {
            mStreamSetOutputs.push_back(Binding{b->getStreamSetTy(numbersOfOutputStreams[i]), "outputStream" + std::to_string(i), RateEqualTo("inputStream")});
        }
    }

    void FakeStreamGeneratingKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> &b,
                                                             llvm::Value *const numOfStrides) {
    }

}