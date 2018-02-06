//
//

#ifndef ICGREP_SEQUENTIAL_SEGMENT_ORIENTED_KERNEL_H
#define ICGREP_SEQUENTIAL_SEGMENT_ORIENTED_KERNEL_H

#include "kernel.h"
#include <vector>
#include <map>

namespace llvm {
    class Module;
    class Function;
    class BasicBlock;
    class Value;
}

namespace IDISA { class IDISA_Builder; }

namespace kernel {
    class SequentialKernel : public MultiBlockKernel {
    protected:
        std::map<std::string, size_t> inputStreamIndexMap;
        std::map<std::string, std::string> clearBufferMap;

        SequentialKernel(
                const std::unique_ptr<kernel::KernelBuilder> & iBuilder,
                std::string && kernelName,
                std::vector<Binding> && stream_inputs,
                std::vector<Binding> && stream_outputs,
                std::vector<Binding> && scalar_parameters,
                std::vector<Binding> && scalar_outputs,
                std::vector<Binding> && internal_scalars);

        virtual void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value * const numOfStrides) override;
//        virtual void generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &iBuilder) override;


        virtual void generateDoSequentialSegmentMethod(const std::unique_ptr<KernelBuilder> &iBuilder);

        //Index Bit
        void configIndexBits(const std::unique_ptr<KernelBuilder> &iBuilder, const std::map<std::string, size_t>& inputStreamMap);

        void configOutputBufferToBeClear(const std::map<std::string, std::string>& clearMap);


        //Cursor
        void initBufferCursor(const std::unique_ptr<KernelBuilder> &iBuilder, std::vector<std::string> cursorNames);

        llvm::Value* getCursorValue(const std::unique_ptr<KernelBuilder> &iBuilder, std::string cursorName);
        void advanceCursor(const std::unique_ptr<KernelBuilder> &iBuilder, std::string cursorName, llvm::Value* nums);
        void advanceCursorUntilPos(const std::unique_ptr<KernelBuilder> &iBuilder, std::string cursorName, llvm::Value* position);
        llvm::BasicBlock* advanceCursorUntilNextOne(const std::unique_ptr<KernelBuilder> &iBuilder, std::string cursorName, std::string inputStreamBufferName, llvm::Value* maxPos);
        llvm::BasicBlock* advanceCursorUntilNextZero(const std::unique_ptr<KernelBuilder> &iBuilder,
                                                     std::string cursorName, std::string inputStreamBufferName, llvm::Value* maxPos);

        llvm::BasicBlock* memcpy2CursorsUntilNextZero(
                const std::unique_ptr<KernelBuilder> &iBuilder,
                std::string sourceBufferName,
                std::string sourceCursorName,
                std::string dstBufferName,
                std::string dstCursorName,
                std::string sourceMarkerName,
                llvm::Value* maxPos);
        llvm::BasicBlock* memcpyOutputDstCursorUntilNextZero(
                const std::unique_ptr<KernelBuilder> &iBuilder,
                std::string outputBufferName,
                llvm::Value* copyOffset,
                std::string dstCursorName,
                std::string dstMarkerName,
                llvm::Value* maxPos
        );

        llvm::BasicBlock* memcpyOutputDst(
                const std::unique_ptr<KernelBuilder> &iBuilder,
                std::string outputBufferName,
                llvm::Value* copyOffset,
                llvm::Value* copyLength

        );

        void memcpyCircularBuffer(
                const std::unique_ptr<KernelBuilder> &iBuilder,
                std::string sourceBufferName,
                llvm::Value* sourceOffset,
                std::string dstBufferName,
                llvm::Value* dstOffset,
                llvm::Value* distance
        );

        // Helper Functions
        void markCircularOutputBitstreamOnePack(const std::unique_ptr<KernelBuilder> &iBuilder, const std::string& bitstreamName, llvm::Value* start, llvm::Value* end, bool isOne);
        llvm::BasicBlock* markCircularOutputBitstream(const std::unique_ptr<KernelBuilder> &iBuilder, const std::string& bitstreamName, llvm::Value* start, llvm::Value* end, bool isOne, bool setProduced = true);

        llvm::Value* generateLoadCircularInputPack(const std::unique_ptr<KernelBuilder> &iBuilder, std::string inputBufferName, llvm::Value* offset);
        llvm::Value* generateLoadCircularInput(const std::unique_ptr<KernelBuilder> &iBuilder, std::string inputBufferName, llvm::Value* offset, llvm::Type* pointerType);
        llvm::Value* generateLoadSourceInputByte(const std::unique_ptr<KernelBuilder> &iBuilder, std::string sourceBuffername, llvm::Value* offset);
        void generateStoreCircularOutput(const std::unique_ptr<KernelBuilder> &iBuilder, std::string outputBufferName, llvm::Type* pointerType, llvm::Value* value);
        llvm::Value* generateLoadCircularOutput(const std::unique_ptr<KernelBuilder> &iBuilder, std::string inputBufferName, llvm::Value* offset, llvm::Type* pointerType);
        void increaseScalarField(const std::unique_ptr<KernelBuilder> &iBuilder, const std::string& fieldName, llvm::Value* value);

//        void generateStoreCircularOutput1(const std::unique_ptr<KernelBuilder> &iBuilder, std::string outputBufferName, llvm::Type* pointerType, llvm::Value* value);

        llvm::Value* offsetToActualBufferOffset(const std::unique_ptr<KernelBuilder> &iBuilder, std::string inputBufferName, llvm::Value* offset);
        llvm::Value* offsetToPackOffset(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value* offset);
        llvm::Value* offsetToPackIndex(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value* offset);
        llvm::Value* offsetToPackBaseOffset(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value* offset);

        llvm::BasicBlock* waitCursorUntilInputAvailable(const std::unique_ptr<KernelBuilder> &iBuilder, std::string cursorName, std::string inputStreamBufferName);


        size_t getInputBufferSize(const std::unique_ptr<KernelBuilder> &iBuilder, std::string bufferName);
        size_t getOutputBufferSize(const std::unique_ptr<KernelBuilder> &iBuilder, std::string bufferName);

    private:

        // forwardBits, packEnd, exceedAvailable
        std::pair<llvm::Value*, std::pair<llvm::Value*, llvm::Value*>> genereateCountForwardBitsOnePack(const std::unique_ptr<KernelBuilder> &iBuilder, std::string inputStreamBufferName, llvm::Value* beginOffset, bool isZero = true);
        llvm::Value* generateCountIndexBit(const std::unique_ptr<KernelBuilder> &iBuilder, std::string bufferName, bool isZero, llvm::Value* beginBitIndex);

        // forwardBits, isFinished
        // maxCount for final bits
        std::pair<llvm::Value*, llvm::Value*> generateCountForwardBits(const std::unique_ptr<KernelBuilder> &iBuilder, std::string inputStreamBufferName, llvm::Value* beginOffset, bool isZero = true, llvm::Value* maxPos = NULL);
        std::pair<llvm::Value*, llvm::Value*> generateCountForwardZeros(const std::unique_ptr<KernelBuilder> &iBuilder, std::string inputStreamBufferName, llvm::Value* beginOffset, llvm::Value* maxPos = NULL);
        std::pair<llvm::Value*, llvm::Value*> generateCountForwardOnes(const std::unique_ptr<KernelBuilder> &iBuilder, std::string inputStreamBufferName, llvm::Value* beginOffset, llvm::Value* maxPos = NULL);

        void generateDstMatchCopy(const std::unique_ptr<KernelBuilder> & iBuilder, llvm::BasicBlock* entry, llvm::BasicBlock* exit, std::string outputBufferName, llvm::Value* matchOffset, llvm::Value* matchLength, llvm::Value* outputOffset);

        void generateBuildIndexBits(const std::unique_ptr<KernelBuilder> &iBuilder);
        inline bool hasIndexBits(const std::string& streamName);
        void generateClearBuffer(const std::unique_ptr<KernelBuilder> &iBuilder);

        std::vector<llvm::BasicBlock*> stateBlocks;

        inline std::string generateCursorFullname(std::string cursorName);
        inline std::string generateInputZeroIndexName(std::string inputStreamName);
        inline std::string generateInputOneIndexName(std::string inputStreamName);
        inline std::string generateInputPreviousAvailableName(std::string inputStreamName);



        // CursorValue should not be set directly in user implemented kernel
        void setCursorValue(const std::unique_ptr<KernelBuilder> &iBuilder, std::string cursorName, llvm::Value* value);

        llvm::BasicBlock* exitBlock;


        inline void recordCountForwardTempMaxPos(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value* maxPos);
        inline llvm::Value* restoreCountForwardTempMaxPos(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value* currentMaxPos);
    };
}

#endif //ICGREP_SEQUENTIAL_SEGMENT_ORIENTED_KERNEL_H
