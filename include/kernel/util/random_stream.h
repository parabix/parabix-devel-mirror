#ifndef RANDOM_STREAM_H
#define RANDOM_STREAM_H

#include <kernels/core/kernel.h>
namespace kernel { class KernelBuilder; }

namespace kernel {

class RandomStreamKernel final : public SegmentOrientedKernel {
public:
    RandomStreamKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned seed, unsigned valueWidth, size_t streamLength);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
    void generateInitializeMethod(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) override;
    void generateDoSegmentMethod(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) override;
protected:
    const unsigned mSeed;
    const unsigned mValueWidth;
    const size_t mStreamLength;
};

}

#endif // RANDOM_STREAM_H
