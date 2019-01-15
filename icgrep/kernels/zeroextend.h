#ifndef ZEROEXTEND_H
#define ZEROEXTEND_H

#include "kernel.h"

namespace kernel {

class ZeroExtend final : public MultiBlockKernel {
public:
    ZeroExtend(const std::unique_ptr<KernelBuilder> &b,
               StreamSet * const input, StreamSet * const output);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, llvm::Value * const numOfStrides) override;
};

}

#endif // ZEROEXTEND_H
