
#ifndef ICGREP_LZ4_BITSTREAM_AIO_H
#define ICGREP_LZ4_BITSTREAM_AIO_H

#include "kernels/lz4/aio/lz4_sequential_aio_base.h"
#include <vector>

namespace kernel {
    class LZ4BitStreamAioKernel : public LZ4SequentialAioBaseKernel {
    public:
        LZ4BitStreamAioKernel(const std::unique_ptr<kernel::KernelBuilder> &b, std::vector<unsigned> numsOfBitStreams = {8}, unsigned blockSize = 4 * 1024 * 1024);
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
