
#ifndef ICGREP_LZ4_BLOCK_DECODER_NEW_H
#define ICGREP_LZ4_BLOCK_DECODER_NEW_H


#include "kernels/kernel.h"
#include <map>
#include <vector>
#include <string>

namespace llvm {
    class Module;
    class Function;
    class BasicBlock;
    class Value;
}

namespace IDISA { class IDISA_Builder; }

namespace kernel {

class LZ4BlockDecoderNewKernel final : public MultiBlockKernel {

public:
    LZ4BlockDecoderNewKernel(const std::unique_ptr<kernel::KernelBuilder> &iBuilder);

protected:
    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value * const numOfStrides) override;

private:
    llvm::Value *generateLoadInput(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value *offset);

    void appendOutput(const std::unique_ptr<KernelBuilder> & iBuilder, llvm::Value *isCompressed, llvm::Value *blockStart, llvm::Value *blockEnd);

    void generateStoreNumberOutput(const std::unique_ptr<KernelBuilder> &iBuilder, const std::string &outputBufferName,
                                   llvm::Type *pointerType, llvm::Value *value);
    size_t getOutputBufferSize(const std::unique_ptr<KernelBuilder> &iBuilder, const std::string& bufferName);

    std::map<std::string, llvm::Value*> previousProducedMap;

    void resetPreviousProducedMap(const std::unique_ptr<KernelBuilder> &iBuilder, std::vector<std::string> outputList);
};

}


#endif //ICGREP_LZ4_BLOCK_DECODER_NEW_H
