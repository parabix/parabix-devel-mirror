
#ifndef ICGREP_LZ4_FAKE_STREAM_GENERATING_KERNEL_H
#define ICGREP_LZ4_FAKE_STREAM_GENERATING_KERNEL_H

#include <kernel/core/kernel.h>

namespace IDISA { class IDISA_Builder; }

namespace kernel {

    class FakeStreamGeneratingKernel final : public SegmentOrientedKernel {
    public:
        FakeStreamGeneratingKernel(BuilderRef b, StreamSet * refStream, StreamSet * outputStream);
        FakeStreamGeneratingKernel(BuilderRef b, StreamSet * refStream, const StreamSets & outputStreams);
        bool isCachable() const override { return true; }
        bool hasSignature() const override { return false; }
    protected:
        void generateDoSegmentMethod(BuilderRef) final;
    };
}


#endif //ICGREP_LZ4_FAKE_STREAM_GENERATING_KERNEL_H
