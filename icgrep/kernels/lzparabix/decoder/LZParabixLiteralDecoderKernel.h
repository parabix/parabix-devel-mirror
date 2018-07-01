//
// Created by wxy325 on 2018/6/30.
//

#ifndef ICGREP_LZPARABIXLITERALDECODERKERNEL_H
#define ICGREP_LZPARABIXLITERALDECODERKERNEL_H

#include "kernels/kernel.h"
#include <string>
#include <map>
#include <vector>

namespace llvm {
    class Module;
    class Function;
    class BasicBlock;
    class Value;
}

namespace IDISA { class IDISA_Builder; }

namespace kernel{
    class LZParabixLiteralDecoderKernel : public SegmentOrientedKernel {
    public:
        LZParabixLiteralDecoderKernel(const std::unique_ptr<kernel::KernelBuilder> &b);
    protected:
        void generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &b) override;

        llvm::Value* generateLoadInt64NumberInput(const std::unique_ptr<KernelBuilder> &iBuilder,
                                                                      std::string inputBufferName, llvm::Value *globalOffset);
        llvm::Value* processBlock(const std::unique_ptr<KernelBuilder> &b, llvm::Value* blockStart);
    };
}



#endif //ICGREP_LZPARABIXLITERALDECODERKERNEL_H
