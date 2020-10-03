#ifndef INTERNAL_POPCOUNT_KERNEL_H
#define INTERNAL_POPCOUNT_KERNEL_H

#include <kernel/core/kernel.h>

namespace kernel {

class PopCountKernel final : public MultiBlockKernel {
public:

    static bool classof(const Kernel * const k) {
        return k->getTypeId() == TypeId::PopCountKernel;
    }

    enum PopCountType { POSITIVE, NEGATIVE, BOTH };

    explicit PopCountKernel(BuilderRef b, const PopCountType type, const unsigned stepFactor, StreamSet * input, StreamSet * const output);

    explicit PopCountKernel(BuilderRef b, const PopCountType type, const unsigned stepFactor, StreamSet * input, StreamSet * const positive, StreamSet * negative);

    void generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfStrides) final;

private:

    const PopCountType mType;
};

}

#endif // REGIONSELECTIONKERNEL_H
