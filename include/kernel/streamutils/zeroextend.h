#ifndef ZEROEXTEND_H
#define ZEROEXTEND_H

#include <kernel/core/kernel.h>

namespace kernel {

class ZeroExtend final : public MultiBlockKernel {
public:
    ZeroExtend(BuilderRef b,
               StreamSet * const input, StreamSet * const output);
    void generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfStrides) override;
};

}

#endif // ZEROEXTEND_H
