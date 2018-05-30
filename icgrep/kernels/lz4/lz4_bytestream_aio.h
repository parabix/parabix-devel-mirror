
#ifndef ICGREP_LZ4_AIO_H
#define ICGREP_LZ4_AIO_H

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

    class LZ4ByteStreamAioKernel : public SegmentOrientedKernel {

    public:
        LZ4ByteStreamAioKernel(const std::unique_ptr<kernel::KernelBuilder> &b);

    protected:
        void generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &b) override;

    private:
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


        inline std::pair<llvm::Value *, llvm::Value *> noExtensionLiteralLength(const std::unique_ptr<KernelBuilder> &b,
                                                                             llvm::Value *currentTokenMarker,
                                                                             llvm::Value *currentExtenderValue,
                                                                             llvm::Value *tokenValue,
                                                                             llvm::Value *blockPosBase,
                                                                             llvm::Value *currentTokenLocalPos
        );

        inline std::pair<llvm::Value *, llvm::Value *> scanThruLiteralLength(const std::unique_ptr<KernelBuilder> &b,
                                                                             llvm::Value *currentTokenMarker,
                                                                             llvm::Value *currentExtenderValue,
                                                                             llvm::Value *tokenValue,
                                                                             llvm::Value *blockPosBase,
                                                                             llvm::Value *currentTokenLocalPos
        );

        inline std::pair<llvm::Value *, llvm::Value *> noExtensionMatchLength(const std::unique_ptr<KernelBuilder> &b,
                                                                           llvm::Value *matchOffsetEndMarker,
                                                                           llvm::Value *currentExtenderValue,
                                                                           llvm::Value *tokenValue,
                                                                           llvm::Value *blockPosBase
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
        void handleMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value* matchOffset, llvm::Value* matchLength);
    };

}




#endif //ICGREP_LZ4_AIO_H
