
#ifndef ICGREP_LZ4_MULTIPLE_PDEP_KERNEL_H
#define ICGREP_LZ4_MULTIPLE_PDEP_KERNEL_H

#include "../kernel.h"
#include <llvm/IR/Value.h>
#include <string>
namespace IDISA { class IDISA_Builder; }

namespace kernel {
class LZ4MultiplePDEPkernel : public MultiBlockKernel {
public:
    LZ4MultiplePDEPkernel(const std::unique_ptr<kernel::KernelBuilder> & kb, unsigned streamCount, unsigned streamSize, unsigned swizzleFactor, unsigned PDEP_width = 64, std::string name = "PDEPdel");
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
    const unsigned mSwizzleFactor;
    const unsigned mPDEPWidth;
    const unsigned mStreamSize;

    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & kb, llvm::Value * const numOfStrides) override;
    std::vector<llvm::Value *> get_PDEP_masks(const std::unique_ptr<KernelBuilder> & kb, llvm::Value * PDEP_ms_blk,
                                              const unsigned mask_width);
    std::vector<llvm::Value *> get_block_popcounts(const std::unique_ptr<KernelBuilder> & kb, llvm::Value * blk,
                                                   const unsigned field_width);
};
}

#endif //ICGREP_LZ4_MULTIPLE_PDEP_KERNEL_H
