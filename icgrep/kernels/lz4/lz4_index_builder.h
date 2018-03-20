//
// Created by wxy325 on 2018/3/16.
//

#ifndef ICGREP_LZ4_INDEX_BUILDER_H
#define ICGREP_LZ4_INDEX_BUILDER_H

#include "kernels/kernel.h"
#include <string>

namespace llvm {
    class Module;
    class Function;
    class BasicBlock;
    class Value;
}

namespace IDISA { class IDISA_Builder; }

namespace kernel {
    class LZ4IndexBuilderKernel final : public MultiBlockKernel {
    public:
        LZ4IndexBuilderKernel(const std::unique_ptr<kernel::KernelBuilder> &iBuilder);

    protected:
        void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> &iBuilder,
                                     llvm::Value *const numOfStrides) override;

    private:
        llvm::Value * generateLoadCircularInput(const std::unique_ptr<KernelBuilder> &iBuilder, std::string inputBufferName,
                                                         llvm::Value *offset, llvm::Type *pointerType);
        void generateProcessCompressedBlock(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value* blockStart, llvm::Value* blockEnd);

        llvm::Value *generateLoadSourceInputByte(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value *offset);

        llvm::Value *advanceUntilNextZero(const std::unique_ptr<KernelBuilder> &iBuilder, std::string inputName, llvm::Value* startPos, llvm::Value* maxPos = nullptr);
        llvm::Value *advanceUntilNextOne(const std::unique_ptr<KernelBuilder> &iBuilder, std::string inputName, llvm::Value* startPos, llvm::Value* maxPos = nullptr);
        llvm::Value *advanceUntilNextValue(const std::unique_ptr<KernelBuilder> &iBuilder, std::string inputName, llvm::Value* startPos, bool isNextZero, llvm::Value* maxPos = nullptr);
        void increaseScalarField(const std::unique_ptr<KernelBuilder> &iBuilder, const std::string &fieldName, llvm::Value *value);
        llvm::Value* processLiteral(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value* token, llvm::Value* tokenPos, llvm::Value* blockEnd);
        llvm::Value* processMatch(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value* offsetPos, llvm::Value* token, llvm::Value* blockEnd);


        size_t getOutputBufferSize(const std::unique_ptr<KernelBuilder> &iBuilder, std::string bufferName);
        llvm::BasicBlock *markCircularOutputBitstream(const std::unique_ptr<KernelBuilder> &iBuilder,
                                                                             const std::string &bitstreamName,
                                                                             llvm::Value *start, llvm::Value *end, bool isOne,
                                                                             bool setProduced = true);
        void generateStoreCircularOutput(const std::unique_ptr<KernelBuilder> &iBuilder, std::string outputBufferName, llvm::Type* pointerType, llvm::Value* value);

        llvm::Value* previousE1Produced ;
    };
}


#endif //ICGREP_LZ4_INDEX_BUILDER_H
