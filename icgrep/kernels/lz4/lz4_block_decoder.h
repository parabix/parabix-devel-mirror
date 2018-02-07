//
// Created by wxy325 on 2017/6/25.
//

#ifndef ICGREP_LZ4D_E_D_H
#define ICGREP_LZ4D_E_D_H

#include "kernels/kernel.h"

namespace llvm {
    class Module;
    class Function;
    class BasicBlock;
    class Value;
}

namespace IDISA { class IDISA_Builder; }

namespace kernel {

class LZ4BlockDecoderKernel final : public MultiBlockKernel {

public:
    LZ4BlockDecoderKernel(const std::unique_ptr<kernel::KernelBuilder> &iBuilder);

protected:
    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value * const numOfStrides) override;
//    virtual void generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder) override;
//    void generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &iBuilder) override;

private:
    const size_t wordWidth;

    llvm::Value *generateLoadInput(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value *offset);

    void appendOutput(const std::unique_ptr<KernelBuilder> & iBuilder, llvm::Value *isCompressed, llvm::Value *blockStart, llvm::Value *blockEnd);

    void generateStoreCircularOutput(const std::unique_ptr<KernelBuilder> &iBuilder, const std::string& outputBufferName,
                                     llvm::Type *pointerType, llvm::Value *value);
    size_t getOutputBufferSize(const std::unique_ptr<KernelBuilder> &iBuilder, const std::string& bufferName);
};

}



#endif //ICGREP_LZ4D_E_D_H
