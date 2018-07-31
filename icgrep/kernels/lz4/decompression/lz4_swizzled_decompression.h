
#ifndef ICGREP_LZ4_SWIZZLED_AIO_H
#define ICGREP_LZ4_SWIZZLED_AIO_H

#include "kernels/lz4/decompression/lz4_sequential_decompression_base.h"
#include "kernels/kernel.h"
#include <string>
#include <map>
#include <vector>


namespace llvm {
    class Module;
    class Function;
    class BasicBlock;
    class Value;
}

namespace IDISA { class IDISA_Builder; }

namespace kernel {
    class LZ4SwizzledDecompressionKernel : public LZ4SequentialDecompressionKernel {

    public:
        LZ4SwizzledDecompressionKernel(const std::unique_ptr<kernel::KernelBuilder> &b, unsigned streamCount, unsigned streamSize, unsigned swizzleFactor, unsigned blockSize = 4 * 1024 * 1024);

    protected:
        unsigned mStreamCount;
        unsigned mStreamSize;
        unsigned mSwizzleFactor;
        unsigned mPDEPWidth;


        void handleAccelerationLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *literalStart,
                                                                 llvm::Value *literalLength, const std::vector<llvm::Value*>& inputLiteralValues);
        void handleAccelerationPdepOutput(
                const std::unique_ptr<KernelBuilder> &b,
                llvm::Value *literalBlockIndex,
                llvm::Value *literalMasks,
                llvm::Value *literalLengthArray,
                llvm::Value *matchOffsetArray,
                llvm::Value *matchLengthArray,
                llvm::Value *numOfElements
        );

        void handleAccelerationMatchCopyOutput(
                const std::unique_ptr<KernelBuilder> &b,
                llvm::Value *literalLengthArray,
                llvm::Value *matchOffsetArray,
                llvm::Value *matchLengthArray,
                llvm::Value *numOfElements
        );

        void handleMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value* matchPos, llvm::Value* matchOffset, llvm::Value* matchLength, bool clearBuffer = true);

        virtual void doLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *literalStart,
                                   llvm::Value *literalLength, llvm::Value* blockStart) override;
        virtual void doMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *matchOffset,
                                 llvm::Value *matchLength) override;
        virtual void setProducedOutputItemCount(const std::unique_ptr<KernelBuilder> &b, llvm::Value* produced) override;


        virtual void prepareAcceleration(const std::unique_ptr<KernelBuilder> &b, llvm::Value* beginTokenPos) override;
        virtual void doAccelerationLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *literalStart,
                                                             llvm::Value *literalLength, llvm::Value* blockStart) override;
        virtual void doAccelerationMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *matchOffset,
                                                           llvm::Value *matchLength) override;
        virtual void finishAcceleration(const std::unique_ptr<KernelBuilder> &b, llvm::Value* beginTokenPos, llvm::Value* literalMask) override;
    private:

    };
};


#endif //ICGREP_LZ4_SWIZZLED_AIO_H
