
#ifndef ICGREP_LZ4_INDEX_BUILDER_NEW_H
#define ICGREP_LZ4_INDEX_BUILDER_NEW_H

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
    class LZ4IndexBuilderNewKernel final : public SegmentOrientedKernel {
    public:
        LZ4IndexBuilderNewKernel(const std::unique_ptr<kernel::KernelBuilder> &b);

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

        inline llvm::Value *
        scanThru(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value *from, llvm::Value *thru);

        llvm::Value *processBlockBoundary(const std::unique_ptr<KernelBuilder> &b,
                                          llvm::Value *beginTokenPos, llvm::Value *lz4BlockEnd);

        /**
         * scanThru the LiteralLength part of an LZ4 sequence
         * @param b
         * @param currentTokenMarker
         * @param currentExtenderValue
         * @param tokenValue
         * @param blockPosBase
         * @param currentTokenLocalPos
         * @return
         *   literalLength
         *   literalMarker  it will be the same as currentTokenMarker if there is no extend literal length, and will be the
         *                  marker of last byte of LiteralLength part.
         */
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

        // Deletion Marker Output
        void storePendingLiteralMasksUntilPos(const std::unique_ptr<KernelBuilder> &iBuilder,
                                              llvm::Value *targetGlobalPos);
        void appendBoundaryLiteralMaskOutput(const std::unique_ptr<KernelBuilder> &b,
                                             llvm::Value *globalLiteralStartPos, llvm::Value *globalLiteralEndPos,
                                             llvm::Value *outputUntilPos);
        void storeLiteralMask(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value *blockIndex,
                              llvm::Value *value);
        void storePendingLiteralMask(const std::unique_ptr<KernelBuilder> &iBuilder);

/*
        llvm::Value *generateLoadSourceInputByte(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value *offset);

        void increaseScalarField(const std::unique_ptr<KernelBuilder> &iBuilder, const std::string &fieldName,
                                 llvm::Value *value);

        llvm::Value *
        processLiteral(const std::unique_ptr<KernelBuilder> &b, llvm::Value *token, llvm::Value *tokenPos,
                       llvm::Value *blockEnd);

        llvm::Value *
        processMatch(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value *offsetPos, llvm::Value *token,
                     llvm::Value *blockEnd);

        // MatchOffset Marker Output
        void appendMatchOffsetMarkerOutput(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value *position);
        void storeMatchOffsetMarker(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value* blockIndex, llvm::Value* value);
        void storePendingMatchOffsetMarker(const std::unique_ptr<KernelBuilder> &iBuilder);



        // M0 Output
        void appendM0Output(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value *start, llvm::Value *end);
        void storeM0(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value* blockIndex, llvm::Value* value);
        void storePendingM0(const std::unique_ptr<KernelBuilder> &iBuilder);
         */
    };
}

#endif //ICGREP_LZ4_INDEX_BUILDER_NEW_H
