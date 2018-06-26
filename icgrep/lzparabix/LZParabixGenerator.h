
#ifndef ICGREP_LZPARABIXGENERATOR_H
#define ICGREP_LZPARABIXGENERATOR_H
#include <llvm/IR/Module.h>
#include <llvm/IR/Function.h>

#include <llvm/Support/Signals.h>
#include <llvm/Support/ManagedStatic.h>
#include <IR_Gen/idisa_target.h>

#include <toolchain/toolchain.h>

#include <toolchain/cpudriver.h>
#include <string>
#include <vector>

typedef void (*MainFunctionType)(char * byte_data, size_t headerSize, size_t filesize, bool hasBlockChecksum);

class LZParabixGenerator {
public:
    LZParabixGenerator();

    MainFunctionType getMainFunc();

    void generatePipeline(const std::string &outputFile);

protected:
    //// Protected Method
    void generateMainFunc(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    virtual void generateLoadByteStreamAndBitStream(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);

    std::vector<parabix::StreamSetBuffer*> generateAioBitStreamDecompressoin(
            const std::unique_ptr<kernel::KernelBuilder> & iBuilder,
            std::vector<parabix::StreamSetBuffer*> bitStreamSets
    );


    // BufferSize related Helper Function
    virtual int getInputBufferBlocks(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    virtual int getDecompressedBufferBlocks(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    int get4MbBufferBlocks();

    //// Data Member
    // Driver
    ParabixDriver mPxDriver;
    size_t mLzParabixBlockSize;


    // Runtime Arguments
    llvm::Value * mInputStream;
    llvm::Value * mHeaderSize;
    llvm::Value * mFileSize;
    llvm::Value * mHasBlockChecksum;


    // StreamSetBuffers
    parabix::StreamSetBuffer * mCompressedByteStream;
    parabix::StreamSetBuffer * mCompressedBasisBits;
};


#endif //ICGREP_LZPARABIXGENERATOR_H
