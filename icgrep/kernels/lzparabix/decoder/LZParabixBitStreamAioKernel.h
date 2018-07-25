
#ifndef ICGREP_LZPARABIXBITSTREAMAIOKERNEL_H
#define ICGREP_LZPARABIXBITSTREAMAIOKERNEL_H

#include "kernels/lzparabix/decoder/LZParabixAioBaseKernel.h"
namespace llvm {
    class Module;
    class Function;
    class BasicBlock;
    class Value;
}

namespace IDISA { class IDISA_Builder; }

namespace kernel {
    class LZParabixBitStreamAioKernel: public LZParabixAioBaseKernel {
    public:
        LZParabixBitStreamAioKernel(const std::unique_ptr<kernel::KernelBuilder> &b, std::vector<unsigned> numsOfBitStreams = {8});
    protected:
        virtual void initDoSegmentMethod(const std::unique_ptr<KernelBuilder> &b) override;
        virtual llvm::Value* isAllItemAvailable(const std::unique_ptr<KernelBuilder> &b, llvm::Value* literalLength) override;
        virtual void doLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *cursorPos, llvm::Value *literalLength) override;
        virtual void doMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value* cursorPos, llvm::Value* matchOffset, llvm::Value* matchMask) override;
        virtual void setProducedOutputItemCount(const std::unique_ptr<KernelBuilder> &b, llvm::Value* c) override;

        virtual llvm::Value* getProcessedInputItemCount(const std::unique_ptr<KernelBuilder> &b) override;
        virtual void setProcessedInputItemCount(const std::unique_ptr<KernelBuilder> &b, llvm::Value* c) override;

    private:
        std::vector<unsigned> mNumsOfBitStreams;

        void appendBitStreamOutput(const std::unique_ptr<KernelBuilder> &b, std::vector<llvm::Value*>& extractedValues, llvm::Value* valueLength);
    };
}



#endif //ICGREP_LZPARABIXBITSTREAMAIOKERNEL_H
