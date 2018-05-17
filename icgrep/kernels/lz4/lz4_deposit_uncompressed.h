
#ifndef ICGREP_LZ4_DEPOSIT_UNCOMPRESSED_H
#define ICGREP_LZ4_DEPOSIT_UNCOMPRESSED_H

#include "kernels/kernel.h"

namespace llvm {
    class Module;
    class Function;
    class BasicBlock;
    class Value;
}

namespace IDISA { class IDISA_Builder; }

namespace kernel {
    class LZ4DepositUncompressedKernel : SegmentOrientedKernel {
    public:
        LZ4DepositUncompressedKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    protected:
        virtual void generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &iBuilder) override;
    private:
        inline llvm::Value* loadCurrentUncompressedData(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const std::string & name);
        inline llvm::Value* generateLoadCircularInput(const std::unique_ptr<KernelBuilder> &iBuilder, const std::string & bufferName, llvm::Value* offset, llvm::Type* pointerType);
        inline void increaseCurrentUncompressedDataIndex(const std::unique_ptr<KernelBuilder> &iBuilder);
        inline void generateDepositUncompressed(const std::unique_ptr<KernelBuilder> &iBuilder);
    };
}




#endif //ICGREP_LZ4_DEPOSIT_UNCOMPRESSED_H
