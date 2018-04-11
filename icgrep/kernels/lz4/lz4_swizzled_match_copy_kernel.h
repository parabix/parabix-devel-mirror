//
// Created by wxy325 on 2018/3/9.
//

#ifndef ICGREP_LZ4_SWIZZLED_MATCH_COPY_KERNEL_H
#define ICGREP_LZ4_SWIZZLED_MATCH_COPY_KERNEL_H

#include "kernels/kernel.h"

namespace IDISA { class IDISA_Builder; }


namespace kernel {
    class LZ4SwizzledMatchCopyKernel final: public SegmentOrientedKernel {
    public:
        LZ4SwizzledMatchCopyKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned streamCount, unsigned streamSize, unsigned swizzleFactor, unsigned PDEP_width = 64);
    protected:
//        void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value * const numOfStrides) override;
        void generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & b) override;
    private:

        const unsigned mSwizzleFactor;
        const unsigned mPDEPWidth;
        const unsigned mStreamSize;
        const unsigned mStreamCount;



        void generateOutputCopy(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value* outputBlocks);

        llvm::Value* mIsFinalBlock;
        llvm::Value* loadInt64NumberInput(const std::unique_ptr<KernelBuilder> &iBuilder, std::string bufferName, llvm::Value* offset);
    };
}




#endif //ICGREP_LZ4_SWIZZLED_MATCH_COPY_KERNEL_H
