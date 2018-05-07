
#ifndef ICGREP_LZ4_SWIZZLED_MATCH_COPY_KERNEL2_H
#define ICGREP_LZ4_SWIZZLED_MATCH_COPY_KERNEL2_H


#include "kernels/kernel.h"

namespace IDISA { class IDISA_Builder; }


namespace kernel {
    class LZ4SwizzledMatchCopyKernel: public SegmentOrientedKernel {
    public:
        LZ4SwizzledMatchCopyKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned streamCount, unsigned streamSize, unsigned swizzleFactor, unsigned PDEP_width = 64);
    protected:
        void generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & b) override;

    private:

        const unsigned mSwizzleFactor;
        const unsigned mPDEPWidth;
        const unsigned mStreamSize;
        const unsigned mStreamCount;
        std::pair<llvm::Value*, llvm::Value*> loadNextMatchOffset(const std::unique_ptr<KernelBuilder> &iBuilder);
        llvm::Value *advanceUntilNextBit(const std::unique_ptr<KernelBuilder> &iBuilder, std::string inputName,
                                          llvm::Value *startPos, bool isNextOne);
    };
}

#endif //ICGREP_LZ4_SWIZZLED_MATCH_COPY_KERNEL2_H
