
#ifndef ICGREP_LZ4_SWIZZLED_AIO_H
#define ICGREP_LZ4_SWIZZLED_AIO_H

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
    class LZ4SwizzledAioKernel : public SegmentOrientedKernel {

    public:
        LZ4SwizzledAioKernel(const std::unique_ptr<kernel::KernelBuilder> &b, unsigned streamCount, unsigned streamSize, unsigned swizzleFactor);

    protected:
        void generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &b) override;

    private:
        unsigned mStreamCount;
        unsigned mStreamSize;
        unsigned mSwizzleFactor;
        unsigned mPDEPWidth;

        const static unsigned int ACCELERATION_WIDTH = 64; // TODO for now, we only consider 64 or 32 since cttz only have i64 or i32 version

        llvm::Value *
        generateLoadInt64NumberInput(const std::unique_ptr<KernelBuilder> &iBuilder, std::string inputBufferName,
                                     llvm::Value *globalOffset);

        void generateProcessCompressedBlock(const std::unique_ptr<KernelBuilder> &b, llvm::Value *lz4BlockStart,
                                            llvm::Value *lz4BlockEnd);

        std::pair<std::pair<llvm::Value *, llvm::Value *>, llvm::Value *>
        generateAcceleration(const std::unique_ptr<KernelBuilder> &b, llvm::Value *beginTokenPos,
                             llvm::Value *blockEnd);

        llvm::Value *processBlockBoundary(const std::unique_ptr<KernelBuilder> &b,
                                          llvm::Value *beginTokenPos, llvm::Value *lz4BlockEnd);




        inline std::pair<llvm::Value *, llvm::Value *> scanThruLiteralLength(const std::unique_ptr<KernelBuilder> &b,
                                                                             llvm::Value *currentTokenMarker,
                                                                             llvm::Value *currentExtenderValue,
                                                                             llvm::Value *tokenValue,
                                                                             llvm::Value *blockPosBase,
                                                                             llvm::Value *currentTokenLocalPos
        );

        inline std::pair<llvm::Value *, llvm::Value *> scanThruMatchLength(const std::unique_ptr<KernelBuilder> &b,
                                                                           llvm::Value *matchOffsetEndMarker,
                                                                           llvm::Value *currentExtenderValue,
                                                                           llvm::Value *tokenValue,
                                                                           llvm::Value *blockPosBase
        );
        inline llvm::Value *
        scanThru(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value *from, llvm::Value *thru);



        void handleLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value* literalStart, llvm::Value* literalLength);
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
                llvm::Value *literalBlockIndex,
                llvm::Value *literalMasks,
                llvm::Value *literalLengthArray,
                llvm::Value *matchOffsetArray,
                llvm::Value *matchLengthArray,
                llvm::Value *numOfElements
        );

        void handleMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value* matchPos, llvm::Value* matchOffset, llvm::Value* matchLength, bool clearBuffer = true);
        void handleMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value* matchOffset, llvm::Value* matchLength);

    };
};


#endif //ICGREP_LZ4_SWIZZLED_AIO_H
