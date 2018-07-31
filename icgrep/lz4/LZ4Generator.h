
#ifndef ICGREP_LZ4GENERATOR_H
#define ICGREP_LZ4GENERATOR_H
#include <llvm/IR/Module.h>
#include <llvm/IR/Function.h>

#include <llvm/Support/Signals.h>
#include <llvm/Support/ManagedStatic.h>
#include <IR_Gen/idisa_target.h>

#include <toolchain/toolchain.h>

#include <toolchain/cpudriver.h>
#include <string>

namespace re { class CC; }


typedef void (*MainFunctionType)(char * byte_data, size_t headerSize, size_t filesize, bool hasBlockChecksum);

struct LZ4BlockInfo {
    parabix::StreamSetBuffer* blockStart;
    parabix::StreamSetBuffer* blockEnd;
    parabix::StreamSetBuffer* isCompress;
};

class LZ4Generator {

public:
    LZ4Generator();
    MainFunctionType getMainFunc();

    void generateDecompressionPipeline(const std::string &outputFile);
protected:

    //// Protected Method
    std::vector<parabix::StreamSetBuffer*> convertCompressedBitsStreamWithBitStreamAioApproach(
            std::vector<parabix::StreamSetBuffer*> compressedBitStreams, std::string prefix);

    void generateMainFunc(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    // Pipeline
    virtual void generateLoadByteStream(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    virtual void generateLoadByteStreamAndBitStream(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    virtual parabix::StreamSetBuffer * generateParallelAIODecompression(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, bool enableGather, bool enableScatter, int minParallelLevel);
    virtual parabix::StreamSetBuffer * generateAIODecompression(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    virtual parabix::StreamSetBuffer * generateSwizzledAIODecompression(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    virtual parabix::StreamSetBuffer * generateBitStreamAIODecompression(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);

    // BufferSize related Helper Function
    virtual int getInputBufferBlocks(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    virtual int getDecompressedBufferBlocks(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    int get4MbBufferBlocks();


    //// Data Member
    // Driver
    ParabixDriver mPxDriver;

    // Runtime Arguments
    llvm::Value * mInputStream;
    llvm::Value * mHeaderSize;
    llvm::Value * mFileSize;
    llvm::Value * mHasBlockChecksum;

    // StreamSetBuffers
    parabix::StreamSetBuffer * mCompressedByteStream;
    parabix::StreamSetBuffer * mCompressedBasisBits;

    unsigned mLz4BlockSize;

    LZ4BlockInfo getBlockInfo(const std::unique_ptr<kernel::KernelBuilder> & b);
};


#endif //ICGREP_LZ4GENERATOR_H
