
#include "fake_stream_generating_kernel.h"
#include <kernels/kernel_builder.h>
#include <llvm/Support/raw_ostream.h>
#include <toolchain/toolchain.h>

using namespace llvm;

namespace kernel {

FakeStreamGeneratingKernel::FakeStreamGeneratingKernel(const std::unique_ptr<kernel::KernelBuilder> &b,
                                                       StreamSet * refStream,
                                                       StreamSet * outputStream)
: SegmentOrientedKernel(b, "FakeStream",
// input stream sets
{Binding{"inputStream", refStream}},
// output stream set
{Binding{"outputStream", outputStream}},
{},{},{}) {

}

FakeStreamGeneratingKernel::FakeStreamGeneratingKernel(const std::unique_ptr<kernel::KernelBuilder> &b,
                                                       StreamSet * refStream,
                                                       const StreamSets & outputStreams)
: SegmentOrientedKernel(b, "FakeStream",
// input stream sets
{Binding{"inputStream", refStream}},
{},{},{},{}
) {
    // output stream sets
    for (unsigned i = 0; i < outputStreams.size(); i++) {
        mOutputStreamSets.push_back(Binding{"outputStream" + std::to_string(i), outputStreams[i]});
    }
}

void FakeStreamGeneratingKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &) {
    // does nothing
}

}
