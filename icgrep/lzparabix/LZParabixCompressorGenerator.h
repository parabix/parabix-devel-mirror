
#ifndef ICGREP_LZPARABIXCOMPRESSORGENERATOR_H
#define ICGREP_LZPARABIXCOMPRESSORGENERATOR_H
#include <llvm/IR/Module.h>
#include <llvm/IR/Function.h>

#include <llvm/Support/Signals.h>
#include <llvm/Support/ManagedStatic.h>
#include <IR_Gen/idisa_target.h>

#include <toolchain/toolchain.h>

#include <toolchain/cpudriver.h>
#include <string>
#include <vector>

typedef void (*CompressorMainFunctionType)(char * byte_data, size_t filesize);


class LZParabixCompressorGenerator {
public:
    LZParabixCompressorGenerator();

    CompressorMainFunctionType getMainFunc();

    void generatePipeline(const std::string &outputFile);

protected:
    //// Protected Method
    void generateMainFunc(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    virtual void generateLoadByteStreamAndBitStream(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);

    // BufferSize related Helper Function
    virtual size_t getInputBufferBlocks(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);


    //// Data Member
    // Driver
    ParabixDriver mPxDriver;
    size_t mLzParabixBlockSize;

    // Runtime Arguments
    llvm::Value * mInputStream;
    llvm::Value * mFileSize;

    // StreamSetBuffers
    parabix::StreamSetBuffer * mCompressedByteStream;
    parabix::StreamSetBuffer * mCompressedBasisBits;
};


#endif //ICGREP_LZPARABIXCOMPRESSORGENERATOR_H
