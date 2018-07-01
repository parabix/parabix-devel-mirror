//
// Created by wxy325 on 2018/6/18.
//

#ifndef ICGREP_LZPARABIXAIOKERNEL_H
#define ICGREP_LZPARABIXAIOKERNEL_H

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

    class LZParabixAioKernel : public SegmentOrientedKernel {

    public:
        LZParabixAioKernel(const std::unique_ptr<kernel::KernelBuilder> &b, std::vector<unsigned> numsOfBitStreams = {8});

    protected:
        void generateProcessCompressedBlock(const std::unique_ptr<KernelBuilder> &b, llvm::Value *lz4BlockStart,
                                            llvm::Value *lz4BlockEnd);
        void generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &b) override;
        llvm::Value *generateLoadInt64NumberInput(const std::unique_ptr<KernelBuilder> &iBuilder,
                                                  std::string inputBufferName, llvm::Value *globalOffset);
        std::pair<llvm::Value*, llvm::Value*> processSequence(const std::unique_ptr<KernelBuilder> &b, llvm::Value* beginLiteralPos, llvm::Value *beginTokenPos,
                                                         llvm::Value *lz4BlockEnd);
        void processLiteral(const std::unique_ptr<KernelBuilder> &b, llvm::Value* cursorPos, llvm::Value* literalLength);
        llvm::Value *processMatch(const std::unique_ptr<KernelBuilder> &b, llvm::Value* cursorPos, llvm::Value* matchOffset, llvm::Value* sequenceBasePtr);


        std::vector<unsigned> mNumsOfBitStreams;


        // ---- Output
        void initPendingOutputScalar(const std::unique_ptr<KernelBuilder> &b);
        void appendBitStreamOutput(const std::unique_ptr<KernelBuilder> &b, std::vector<llvm::Value*>& extractedValues, llvm::Value* valueLength);
        void storePendingOutput(const std::unique_ptr<KernelBuilder> &b);


        void initPendingOutputScalar_BitStream(const std::unique_ptr<KernelBuilder> &b);
        void appendBitStreamOutput_BitStream(const std::unique_ptr<KernelBuilder> &b, std::vector<llvm::Value*>& extractedValues, llvm::Value* valueLength);
        void storePendingOutput_BitStream(const std::unique_ptr<KernelBuilder> &b);


        void initPendingOutputScalar_Swizzled(const std::unique_ptr<KernelBuilder> &b);
        void appendBitStreamOutput_Swizzled(const std::unique_ptr<KernelBuilder> &b, std::vector<llvm::Value*>& extractedValues, llvm::Value* valueLength);
        void storePendingOutput_Swizzled(const std::unique_ptr<KernelBuilder> &b);
    };

}




#endif //ICGREP_LZPARABIXAIOKERNEL_H
