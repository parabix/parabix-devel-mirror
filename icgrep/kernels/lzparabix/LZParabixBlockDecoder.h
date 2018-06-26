//
// Created by wxy325 on 2018/6/18.
//

#ifndef ICGREP_LZPARABIXBLOCKDECODER_H
#define ICGREP_LZPARABIXBLOCKDECODER_H

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

class LZParabixBlockDecoderKernel : public SegmentOrientedKernel  {
public:
    LZParabixBlockDecoderKernel(const std::unique_ptr<kernel::KernelBuilder> &iBuilder, std::string&& kernelName = "LZParabixBlockDecoderKernel");
protected:
    void generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & b) override;
private:
    llvm::Value *generateLoadInput(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value *offset);

    void appendOutput(const std::unique_ptr<KernelBuilder> & iBuilder, llvm::Value *blockStart, llvm::Value *blockEnd);

    void generateStoreNumberOutput(const std::unique_ptr<KernelBuilder> &iBuilder, const std::string &outputBufferName, llvm::Value *offset, llvm::Value *value);

};

}




#endif //ICGREP_LZPARABIXBLOCKDECODER_H
