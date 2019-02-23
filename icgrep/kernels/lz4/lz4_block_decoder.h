
#ifndef ICGREP_LZ4_BLOCK_DECODER_NEW_H
#define ICGREP_LZ4_BLOCK_DECODER_NEW_H


#include <kernels/core/kernel.h>
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

class LZ4BlockDecoderKernel : public SegmentOrientedKernel {
public:
    LZ4BlockDecoderKernel(const std::unique_ptr<kernel::KernelBuilder> &b,
                          // arguments
                          Scalar * hasBlockChecksum, Scalar * headerSize, Scalar * fileSize,
                          // inputs
                          StreamSet * byteStream,
                          // outputs
                          StreamSet * isCompressed, StreamSet * blockStart, StreamSet * blockEnd);
protected:
    void generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & b) override;
private:
    llvm::Value *generateLoadInput(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value *offset);

    void appendOutput(const std::unique_ptr<KernelBuilder> & iBuilder, llvm::Value *isCompressed, llvm::Value *blockStart, llvm::Value *blockEnd);
    void generateStoreNumberOutput(const std::unique_ptr<KernelBuilder> &iBuilder, const std::string &outputBufferName, llvm::Value *offset, llvm::Value *value);

};

}


#endif //ICGREP_LZ4_BLOCK_DECODER_NEW_H
