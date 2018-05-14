
#ifndef ICGREP_LZ4_BITSTREAM_MATCH_COPY_KERNEL_H
#define ICGREP_LZ4_BITSTREAM_MATCH_COPY_KERNEL_H

#include "kernels/kernel.h"

namespace IDISA { class IDISA_Builder; }

namespace kernel {
    class LZ4BitStreamMatchCopyKernel : public SegmentOrientedKernel {
    public:
        LZ4BitStreamMatchCopyKernel(const std::unique_ptr<kernel::KernelBuilder> &iBuilder, unsigned numberOfStreams,
                                    std::string name = "LZ4BitStreamMatchCopyKernel");

    protected:
        void generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &b) override;

    private:
        unsigned mNumberOfStreams;

        std::pair<llvm::Value *, llvm::Value *> loadNextMatchOffset(const std::unique_ptr<KernelBuilder> &iBuilder);

        llvm::Value *advanceUntilNextBit(const std::unique_ptr<KernelBuilder> &iBuilder, std::string inputName,
                                         llvm::Value *startPos, bool isNextOne);

        std::vector<llvm::Value *> loadAllI64BitStreamValues(const std::unique_ptr<KernelBuilder> &b,
                                                             llvm::Value *basePtr,
                                                             llvm::Value *i64PackIndex);
        void storeAllI64BitStreamValues(const std::unique_ptr<KernelBuilder> &b,
                                                                     llvm::Value *basePtr, llvm::Value *i64PackIndex,
                                                                     const std::vector<llvm::PHINode *> &values);
    };
}

#endif //ICGREP_LZ4_BITSTREAM_MATCH_COPY_KERNEL_H
