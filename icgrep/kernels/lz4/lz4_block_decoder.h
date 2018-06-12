
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

class LZ4BlockDecoderNewKernel : public SegmentOrientedKernel {
public:
    LZ4BlockDecoderNewKernel(const std::unique_ptr<kernel::KernelBuilder> &iBuilder, std::string&& kernelName = "LZ4BlockDecoderKernel");
protected:
    void generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & b) override;
private:
    llvm::Value *generateLoadInput(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value *offset);

    void appendOutput(const std::unique_ptr<KernelBuilder> & iBuilder, llvm::Value *isCompressed, llvm::Value *blockStart, llvm::Value *blockEnd);

    void generateStoreNumberOutput(const std::unique_ptr<KernelBuilder> &iBuilder, const std::string &outputBufferName, llvm::Value *offset, llvm::Value *value);

};

}


#endif //ICGREP_LZ4_BLOCK_DECODER_NEW_H
