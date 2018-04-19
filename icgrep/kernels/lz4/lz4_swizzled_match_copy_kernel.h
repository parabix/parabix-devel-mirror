//
// Created by wxy325 on 2018/3/9.
//

#ifndef ICGREP_LZ4_SWIZZLED_MATCH_COPY_KERNEL_H
#define ICGREP_LZ4_SWIZZLED_MATCH_COPY_KERNEL_H

#include "kernels/kernel.h"

namespace IDISA { class IDISA_Builder; }


namespace kernel {
    class LZ4SwizzledMatchCopyKernel: public SegmentOrientedKernel {
    public:
        LZ4SwizzledMatchCopyKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned streamCount, unsigned streamSize, unsigned swizzleFactor, unsigned PDEP_width = 64);
    protected:
        void generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & b) override;
        void generateOutputCopy(const std::unique_ptr<KernelBuilder> & iBuilder);

    private:

        const unsigned mSwizzleFactor;
        const unsigned mPDEPWidth;
        const unsigned mStreamSize;
        const unsigned mStreamCount;
        std::pair<llvm::Value*, llvm::Value*> loadNextMatchOffset(const std::unique_ptr<KernelBuilder> &iBuilder);
        std::pair<llvm::Value*, llvm::Value*> loadNextM0StartEnd(const std::unique_ptr<KernelBuilder> &iBuilder);
        llvm::Value *advanceUntilNextBit(const std::unique_ptr<KernelBuilder> &iBuilder, std::string inputName,
                                          llvm::Value *startPos, bool isNextOne);

        llvm::Value* doMatchCopy(const std::unique_ptr<KernelBuilder> & iBuilder, llvm::Value* matchPos, llvm::Value* matchOffset, llvm::Value* matchLength);

    };
}




#endif //ICGREP_LZ4_SWIZZLED_MATCH_COPY_KERNEL_H
