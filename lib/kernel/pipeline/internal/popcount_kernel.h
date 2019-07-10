#ifndef INTERNAL_POPCOUNT_KERNEL_H
#define INTERNAL_POPCOUNT_KERNEL_H

#include <kernels/core/kernel.h>

namespace kernel {

class PopCountKernel final : public MultiBlockKernel {
public:

    enum PopCountType { POSITIVE, NEGATIVE, BOTH };

    explicit PopCountKernel(const std::unique_ptr<kernel::KernelBuilder> & b, const PopCountType type, const unsigned stepFactor, StreamSet * input, StreamSet * const output);

    explicit PopCountKernel(const std::unique_ptr<kernel::KernelBuilder> & b, const PopCountType type, const unsigned stepFactor, StreamSet * input, StreamSet * const positive, StreamSet * negative);

    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }

    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, llvm::Value * const numOfStrides) final;

private:

    const PopCountType mType;
};

}

#endif // REGIONSELECTIONKERNEL_H
