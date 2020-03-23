
#ifndef ICGREP_BITSTREAM_GATHER_PDEP_KERNEL_H
#define ICGREP_BITSTREAM_GATHER_PDEP_KERNEL_H

#include <kernel/core/kernel.h>
#include <llvm/IR/Value.h>
#include <string>

namespace kernel {

class BitStreamGatherPDEPKernel final : public MultiBlockKernel {
public:
    BitStreamGatherPDEPKernel(BuilderRef b, const unsigned numberOfStream = 8, std::string name = "BitStreamGatherPDEPKernel");
private:
    void generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfStrides) final;
private:
    const unsigned mNumberOfStream;

    llvm::Value* fill_address(BuilderRef b, unsigned fw, unsigned field_count, llvm::Value* a);
};

}

#endif //ICGREP_BITSTREAM_GATHER_PDEP_KERNEL_H
