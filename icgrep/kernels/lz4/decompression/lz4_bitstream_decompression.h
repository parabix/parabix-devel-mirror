
#ifndef ICGREP_LZ4_BITSTREAM_AIO_H
#define ICGREP_LZ4_BITSTREAM_AIO_H

#include <kernels/lz4/decompression/lz4_sequential_decompression_base.h>
#include <vector>

namespace kernel {
    class LZ4BitStreamDecompressionKernel : public LZ4SequentialDecompressionKernel {
    public:
        LZ4BitStreamDecompressionKernel(const std::unique_ptr<kernel::KernelBuilder> &b,
                                        Scalar * fileSize,
                                        StreamSet * inputStream,
                                        const LZ4BlockInfo & blockInfo,
                                        StreamSets inputStreams,
                                        StreamSets outputStreams);
    protected:
        virtual void doLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *literalStart,
                                   llvm::Value *literalLength, llvm::Value* blockStart) override;
        virtual void doMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *matchOffset,
                                 llvm::Value *matchLength) override;
        virtual void setProducedOutputItemCount(const std::unique_ptr<KernelBuilder> &b, llvm::Value* produced) override;
        virtual void storePendingOutput(const std::unique_ptr<KernelBuilder> &b) override;

    private:

        std::vector<unsigned> mNumsOfBitStreams;
        void initPendingOutputScalar(const std::unique_ptr<kernel::KernelBuilder> &b);
        void appendBitStreamOutput(const std::unique_ptr<KernelBuilder> &b, std::vector<llvm::Value*>& extractedValues, llvm::Value* valueLength);


        void storePendingOutput_BitStream(const std::unique_ptr<KernelBuilder> &b);
    };
}




#endif //ICGREP_LZ4_BITSTREAM_AIO_H
