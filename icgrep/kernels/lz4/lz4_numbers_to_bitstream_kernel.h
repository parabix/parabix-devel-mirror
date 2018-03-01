
#ifndef ICGREP_LZ4_NUMBERS_TO_BITSTREAM_KERNEL2_H
#define ICGREP_LZ4_NUMBERS_TO_BITSTREAM_KERNEL2_H
#include "kernels/kernel.h"

namespace llvm {
    class Module;
    class Function;
    class BasicBlock;
    class Value;
}

namespace IDISA { class IDISA_Builder; }

namespace kernel {
class LZ4NumbersToBitstreamKernel final : public MultiBlockKernel {
    public:
        LZ4NumbersToBitstreamKernel(std::string kernelName, const std::unique_ptr<kernel::KernelBuilder> &iBuilder);
    protected:
        void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value * const numOfStrides) override;
    private:
        inline size_t getAnyBufferSize(const std::unique_ptr<KernelBuilder> &iBuilder, std::string bufferName);
        llvm::Value* setIntVectorBitOne(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value* intVec, llvm::Value* pos, llvm::Value* isSet);
        inline llvm::Value* intVecGT(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value* intVec1, llvm::Value* intVec2);
    };
}


#endif //ICGREP_LZ4_NUMBERS_TO_BITSTREAM_KERNEL2_H
