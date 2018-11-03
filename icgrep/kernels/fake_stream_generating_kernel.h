
#ifndef ICGREP_LZ4_FAKE_STREAM_GENERATING_KERNEL_H
#define ICGREP_LZ4_FAKE_STREAM_GENERATING_KERNEL_H

#include <kernels/kernel.h>

namespace IDISA { class IDISA_Builder; }

namespace kernel {

    class FakeStreamGeneratingKernel final : public SegmentOrientedKernel {
    public:
        FakeStreamGeneratingKernel(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet * refStream, StreamSet * outputStream);
        FakeStreamGeneratingKernel(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet * refStream, const StreamSets & outputStreams);
        bool isCachable() const override { return true; }
        bool hasSignature() const override { return false; }
    protected:
        void generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &) final;
    };
}


#endif //ICGREP_LZ4_FAKE_STREAM_GENERATING_KERNEL_H
