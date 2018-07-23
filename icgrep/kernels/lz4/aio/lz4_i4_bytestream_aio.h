//
// Created by wxy325 on 2018/7/19.
//

#ifndef ICGREP_LZ4_I4_BYTESTREAM_AIO_H
#define ICGREP_LZ4_I4_BYTESTREAM_AIO_H

#include "kernels/lz4/aio/lz4_sequential_aio_base.h"

namespace kernel{
    class LZ4I4ByteStreamAioKernel : public LZ4SequentialAioBaseKernel {
    public:
        LZ4I4ByteStreamAioKernel(const std::unique_ptr<kernel::KernelBuilder> &b, unsigned blockSize = 4 * 1024 * 1024);

    protected:
        virtual void doLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *literalStart,
                                   llvm::Value *literalLength, llvm::Value* blockStart) override;
        virtual void doMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *matchOffset,
                                 llvm::Value *matchLength) override;
        virtual void setProducedOutputItemCount(const std::unique_ptr<KernelBuilder> &b, llvm::Value* produced) override;
    };

}

#endif //ICGREP_LZ4_I4_BYTESTREAM_AIO_H
