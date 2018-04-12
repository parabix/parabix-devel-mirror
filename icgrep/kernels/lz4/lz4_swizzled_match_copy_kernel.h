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

        llvm::Value * loadOffset(const std::unique_ptr<KernelBuilder> &iBuilder, const std::string & bufferName, llvm::Value* offset);

    private:

        const unsigned mSwizzleFactor;
        const unsigned mPDEPWidth;
        const unsigned mStreamSize;
        const unsigned mStreamCount;
    };
}




#endif //ICGREP_LZ4_SWIZZLED_MATCH_COPY_KERNEL_H
