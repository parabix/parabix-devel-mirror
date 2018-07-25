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

    class LZParabixAioBaseKernel : public SegmentOrientedKernel {
    public:
        LZParabixAioBaseKernel(const std::unique_ptr<kernel::KernelBuilder> &b, std::string&& name = "LZParabixAioKernel");

    protected:
        void generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &b) override;

        virtual void doLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *cursorPos, llvm::Value *literalLength) = 0;
        virtual void doMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value* cursorPos, llvm::Value* matchOffset, llvm::Value* matchMask) = 0;
        virtual void initDoSegmentMethod(const std::unique_ptr<KernelBuilder> &b) {};
        virtual llvm::Value* isAllItemAvailable(const std::unique_ptr<KernelBuilder> &b, llvm::Value* literalLength);
        virtual void setProducedOutputItemCount(const std::unique_ptr<KernelBuilder> &b, llvm::Value* c) = 0;
        virtual llvm::Value* getProcessedInputItemCount(const std::unique_ptr<KernelBuilder> &b) = 0;
        virtual void setProcessedInputItemCount(const std::unique_ptr<KernelBuilder> &b, llvm::Value* c) = 0;

    private:
        void generateProcessCompressedBlock(const std::unique_ptr<KernelBuilder> &b, llvm::Value *lz4BlockStart,
                                            llvm::Value *lz4BlockEnd);
        llvm::Value *generateLoadInt64NumberInput(const std::unique_ptr<KernelBuilder> &iBuilder,
                                                  std::string inputBufferName, llvm::Value *globalOffset);
        std::pair<llvm::Value*, llvm::Value*> processSequence(const std::unique_ptr<KernelBuilder> &b, llvm::Value* beginLiteralPos, llvm::Value *beginTokenPos,
                                                              llvm::Value *lz4BlockEnd);
        llvm::Value *processMatch(const std::unique_ptr<KernelBuilder> &b, llvm::Value* cursorPos, llvm::Value* matchOffset, llvm::Value* sequenceBasePtr);
    };
}




#endif //ICGREP_LZPARABIXAIOKERNEL_H
