
#ifndef ICGREP_LZ4_AIO_H
#define ICGREP_LZ4_AIO_H

#include "kernels/lz4/decompression/lz4_sequential_decompression_base.h"

namespace kernel {

    class LZ4ByteStreamDecompressionKernel : public LZ4SequentialDecompressionKernel {
    public:
        LZ4ByteStreamDecompressionKernel(const std::unique_ptr<kernel::KernelBuilder> &b, bool copyOtherByteStream = false, unsigned blockSize = 4 * 1024 * 1024, bool conditionalDecompression = false);


    protected:
        virtual void doLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *literalStart,
                                   llvm::Value *literalLength, llvm::Value* blockStart) override;
        virtual void doMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *matchOffset,
                                 llvm::Value *matchLength) override;
        virtual void setProducedOutputItemCount(const std::unique_ptr<KernelBuilder> &b, llvm::Value* produced) override;

        virtual void initializationMethod(const std::unique_ptr<KernelBuilder> &b) override;
        virtual void prepareProcessBlock(const std::unique_ptr<KernelBuilder> &b, llvm::Value* blockStart, llvm::Value* blockEnd) override;
        virtual void beforeTermination(const std::unique_ptr<KernelBuilder> &b) override;

    private:
        inline std::string getCopyByteStreamName();
        bool mCopyOtherByteStream;
        llvm::Value* oldOutputExceedFwData;
    };

}




#endif //ICGREP_LZ4_AIO_H
