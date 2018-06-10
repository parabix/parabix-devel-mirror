
#ifndef ICGREP_LZ4_PARALLEL_BYTESTREAM_AIO_H
#define ICGREP_LZ4_PARALLEL_BYTESTREAM_AIO_H

#include "kernels/kernel.h"
#include <string>
#include <map>
#include <vector>
#include <llvm/IR/DerivedTypes.h>

namespace llvm {
    class Module;
    class Function;
    class BasicBlock;
    class Value;
}

namespace IDISA { class IDISA_Builder; }


namespace kernel {

    class LZ4ParallelByteStreamAioKernel : public SegmentOrientedKernel {

    public:
        // By default, output block size in LZ4 is 4MB
        LZ4ParallelByteStreamAioKernel(const std::unique_ptr<kernel::KernelBuilder> &b, size_t outputBlockSize = 4 * 1024 * 1024 );

    protected:
        void generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &b) override;

    private:
        void generateSimdDecompression(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value* blockDataIndex);

        llvm::Value *
        generateLoadSimdInt64NumberInput(const std::unique_ptr<KernelBuilder> &iBuilder, std::string inputBufferName,
                                     llvm::Value *globalOffset);

        llvm::Value *
        generateLoadInt64NumberInput(const std::unique_ptr<KernelBuilder> &iBuilder, std::string inputBufferName,
                                     llvm::Value *globalOffset);

        llvm::Value* generateProcessCompressedBlock(const std::unique_ptr<KernelBuilder> &b, llvm::Value *lz4BlockStart,
                                            llvm::Value *lz4BlockEnd, llvm::Value* initOutputPos);

        llvm::Value* generateSimdAcceleration(const std::unique_ptr<KernelBuilder> &b, llvm::Value *beginTokenPosVec,
                                      llvm::Value *blockEndVec);

        std::pair<llvm::Value*, llvm::Value*> simdProcessBlockBoundary(const std::unique_ptr<KernelBuilder> &b,
                                          llvm::Value *beginTokenPos, llvm::Value *lz4BlockEnd, llvm::Value* outputPosVec);

        std::pair<llvm::Value*, llvm::Value*> processBlockBoundary(const std::unique_ptr<KernelBuilder> &b,
                                          llvm::Value *beginTokenPos, llvm::Value *lz4BlockEnd, llvm::Value* outputPos);



        void handleSimdLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value* literalStartVec, llvm::Value* literalLengthVec, llvm::Value* outputPosVec);
        void generateSimdSequentialLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *literalStartVec,
                                               llvm::Value *literalLengthVec, llvm::Value *outputPosVec);

        void generateSimdSequentialLiteralCopyWithSimdCalculation(const std::unique_ptr<KernelBuilder> &b,
                                                                  llvm::Value *literalStartVec,
                                                                  llvm::Value *literalLengthVec,
                                                                  llvm::Value *outputPosVec);
        void generateSimdLiteralCopyByScatter(const std::unique_ptr<KernelBuilder> &b, llvm::Value *literalStartVec,
                                              llvm::Value *literalLengthVec, llvm::Value *outputPosVec);
        void generateSimdLiteralCopyByMemcpy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *literalStartVec,
                                             llvm::Value *literalLengthVec, llvm::Value *outputPosVec);

        void generateOverwritingMemcpy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *inputBasePtr,
                                       llvm::Value *outputBasePtr, llvm::Value *copyBytes, llvm::PointerType *targetPtrTy,
                                       size_t stepSize);
        void generateOverwritingMemcpy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *inputBasePtr,
                                       llvm::Value *outputBasePtr, llvm::Value *copyBytes, llvm::PointerType *targetPtrTy,
                                       llvm::Value* stepSize);

        void handleSimdMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value* matchOffsetVec, llvm::Value* matchLengthVec, llvm::Value* outputPosVec);
        void generateSimdMatchCopyByMemcpy(const std::unique_ptr<KernelBuilder> &b, llvm::Value* matchOffsetVec, llvm::Value* matchLengthVec, llvm::Value* outputPosVec);
        void generateSimdSequentialMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value* matchOffsetVec, llvm::Value* matchLengthVec, llvm::Value* outputPosVec);

        void handleLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value* literalStart, llvm::Value* literalLength, llvm::Value* outputPos);
        void handleMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value* matchOffset, llvm::Value* matchLength, llvm::Value* outputPos);

        void generateSequentialDecompression(const std::unique_ptr<KernelBuilder> &b, llvm::Value* startBlockDataIndex, llvm::Value* endBlockDataIndex);


        llvm::Value* simdFetchData(const std::unique_ptr<KernelBuilder> &b, llvm::Value* basePtr, llvm::Value* offsetVec, llvm::Value* mask);
        llvm::Value* simdFetchByteData(const std::unique_ptr<KernelBuilder> &b, llvm::Value* basePtr, llvm::Value* offsetVec, llvm::Value* mask);
        llvm::Value* simdFetchDataByGather(const std::unique_ptr<KernelBuilder> &b, llvm::Value *basePtr,
                                           llvm::Value *offsetVec, llvm::Value *mask);
        llvm::Value* simdFetchDataByLoop(const std::unique_ptr<KernelBuilder> &b, llvm::Value *basePtr,
                                         llvm::Value *offsetVec, llvm::Value *mask);

        size_t mOutputBlockSize;
    };

}


#endif //ICGREP_LZ4_PARALLEL_BYTESTREAM_AIO_H
