
#include "lz4_fake_stream_generating_kernel.h"
#include <kernels/kernel_builder.h>
#include <llvm/Support/raw_ostream.h>
#include <toolchain/toolchain.h>

using namespace llvm;

namespace kernel {


    LZ4FakeStreamGeneratingKernel::LZ4FakeStreamGeneratingKernel(const std::unique_ptr<kernel::KernelBuilder> &b,
                                                                 const unsigned int numberOfInputStream,
                                                                 const unsigned int numberOfOutputStream,
                                                                 std::string name)
            : MultiBlockKernel(std::move(name),
// input stream sets
                               {Binding{b->getStreamSetTy(numberOfInputStream), "inputStream", FixedRate(), Principal()}},
// output stream set
                               {Binding{b->getStreamSetTy(numberOfOutputStream), "outputStream", RateEqualTo("inputStream")}},
                               {}, {}, {}) {

    }

    void LZ4FakeStreamGeneratingKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> &b,
                                                                llvm::Value *const numOfStrides) {
    }
}