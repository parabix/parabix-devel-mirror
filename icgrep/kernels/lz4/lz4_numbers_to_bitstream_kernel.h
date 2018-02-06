//
// Created by wxy325 on 2017/8/9.
//

#ifndef ICGREP_LZ4_NUMBERS_TO_BITSTREAM_KERNEL_H
#define ICGREP_LZ4_NUMBERS_TO_BITSTREAM_KERNEL_H
#include <string>

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
//        void generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &iBuilder) override;
    private:
        inline llvm::Value* generateLoadCircularInput(const std::unique_ptr<KernelBuilder> &iBuilder, std::string inputBufferName, llvm::Value* offset, llvm::Type* pointerType);
        inline size_t getInputBufferSize(const std::unique_ptr<KernelBuilder> &iBuilder, std::string bufferName);
        inline size_t getOutputBufferSize(const std::unique_ptr<KernelBuilder> &iBuilder, std::string bufferName);
        inline llvm::Value* getPackOutputPtr(const std::unique_ptr<KernelBuilder> & iBuilder, llvm::Value* packIndex);
    };
}



#endif //ICGREP_LZ4_NUMBERS_TO_BITSTREAM_KERNEL_H
