//
//

#ifndef ICGREP_LZ4_MATCH_COPY_KERNEL_H
#define ICGREP_LZ4_MATCH_COPY_KERNEL_H

#include "kernels/kernel.h"

namespace IDISA { class IDISA_Builder; }

namespace kernel {
    class LZ4MatchCopyKernel: public MultiBlockKernel {
    public:
        LZ4MatchCopyKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    protected:
        void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value * const numOfStrides) override;
    private:
        void generateOutputCopy(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value* outputBlocks);

        llvm::Value* getMaximumMatchCopyBlock(const std::unique_ptr<KernelBuilder> &iBuilder);
        llvm::Value* mIsFinalBlock;
    };
}



#endif //ICGREP_LZ4_MATCH_COPY_KERNEL_H
