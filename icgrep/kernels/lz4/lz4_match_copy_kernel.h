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
//        void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) override;
    private:
        llvm::Value* generateLoadCircularInput(const std::unique_ptr<KernelBuilder> &iBuilder, std::string inputBufferName, llvm::Value* offset, llvm::Type* pointerType);
        llvm::Value* generateLoadCircularOutput(const std::unique_ptr<KernelBuilder> &iBuilder, std::string inputBufferName, llvm::Value* offset, llvm::Type* pointerType);
        void generateStoreCircularOutput(const std::unique_ptr<KernelBuilder> &iBuilder, std::string outputBufferName, llvm::Value* offset, llvm::Type* pointerType, llvm::Value* value);

        size_t getInputBufferSize(const std::unique_ptr<KernelBuilder> &iBuilder, std::string bufferName);
        size_t getOutputBufferSize(const std::unique_ptr<KernelBuilder> &iBuilder, std::string bufferName);
        void generateOutputCopy(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value* outputBlocks);

        llvm::Value* getMaximumMatchCopyBlock(const std::unique_ptr<KernelBuilder> &iBuilder);
        llvm::Value* mIsFinalBlock;
    };
}



#endif //ICGREP_LZ4_MATCH_COPY_KERNEL_H
