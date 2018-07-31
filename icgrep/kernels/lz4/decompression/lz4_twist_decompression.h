

#ifndef ICGREP_LZ4_I4_BYTESTREAM_AIO_H
#define ICGREP_LZ4_I4_BYTESTREAM_AIO_H

#include "kernels/lz4/decompression/lz4_sequential_decompression_base.h"

namespace kernel{
    class LZ4TwistDecompressionKernel : public LZ4SequentialDecompressionKernel {
    public:
        LZ4TwistDecompressionKernel(const std::unique_ptr<kernel::KernelBuilder> &b, unsigned twistWidth, unsigned blockSize = 4 * 1024 * 1024);

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
        void doShortMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *matchOffset,
                                 llvm::Value *matchLength);
        void doLongMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *matchOffset,
                                 llvm::Value *matchLength);

        inline size_t getNormalCopyLength();
        inline llvm::Value* getNormalCopyLengthValue(const std::unique_ptr<KernelBuilder> &b);
        inline llvm::Value* getOutputMask(const std::unique_ptr<KernelBuilder> &b, llvm::Value* outputPosRem4);


        const unsigned mTwistWidth;
        const unsigned mItemsPerByte;
    };
}

#endif //ICGREP_LZ4_I4_BYTESTREAM_AIO_H
