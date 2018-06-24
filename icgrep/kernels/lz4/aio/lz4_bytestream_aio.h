
#ifndef ICGREP_LZ4_AIO_H
#define ICGREP_LZ4_AIO_H

#include "kernels/lz4/aio/lz4_sequential_aio_base.h"

namespace kernel {

    class LZ4ByteStreamAioKernel : public LZ4SequentialAioBaseKernel {
    public:
        LZ4ByteStreamAioKernel(const std::unique_ptr<kernel::KernelBuilder> &b, unsigned blockSize = 4 * 1024 * 1024);

    protected:
        virtual void doLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *literalStart,
                                   llvm::Value *literalLength) override;
        virtual void doMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *matchOffset,
                                 llvm::Value *matchLength) override;
        virtual void setProducedOutputItemCount(const std::unique_ptr<KernelBuilder> &b, llvm::Value* produced) override;
    };

}




#endif //ICGREP_LZ4_AIO_H
