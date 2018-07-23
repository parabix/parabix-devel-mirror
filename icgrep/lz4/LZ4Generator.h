
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

    void generatePipeline(const std::string &outputFile);
    void generateSwizzledPipeline(const std::string &outputFile);
    void generateNewExtractOnlyPipeline(const std::string &outputFile);
    void generateExtractOnlyPipeline(const std::string &outputFile);
    void generateSwizzledExtractOnlyPipeline(const std::string &outputFile);
    void generateExtractAndDepositOnlyPipeline(const std::string &outputFile);
    void generateSwizzledExtractAndDepositOnlyPipeline(const std::string &outputFile);

protected:
    //// Protected Method
    std::vector<parabix::StreamSetBuffer*> convertCompressedBitsStreamWithBitStreamAioApproach(
            std::vector<parabix::StreamSetBuffer*> compressedBitStreams, std::string prefix);

    void generateMainFunc(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    // Pipeline
    virtual void generateLoadByteStream(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    virtual void generateLoadByteStreamAndBitStream(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    virtual void generateExtractAndDepositMarkers(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    virtual parabix::StreamSetBuffer * generateParallelAIODecompression(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, bool enableGather, bool enableScatter, int minParallelLevel);
    virtual parabix::StreamSetBuffer * generateAIODecompression(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    virtual parabix::StreamSetBuffer * generateSwizzledAIODecompression(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    virtual parabix::StreamSetBuffer * generateBitStreamAIODecompression(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);

    virtual std::pair<parabix::StreamSetBuffer*, parabix::StreamSetBuffer*> generateSwizzleExtractData(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    virtual parabix::StreamSetBuffer* generateBitStreamExtractData(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);

    void generateCompressionMarker(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);

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
    parabix::StreamSetBuffer * mDeletionMarker;
    parabix::StreamSetBuffer * mCompressionMarker;
    parabix::StreamSetBuffer * mDepositMarker;
    parabix::StreamSetBuffer * mMatchOffsetMarker;

    // M0CountMarker will not contain anything, it will only be used to pass producedItemCount and manage processedItemCount between different kernel
    parabix::StreamSetBuffer * mM0Marker;

    unsigned mLz4BlockSize;

    LZ4BlockInfo getBlockInfo(const std::unique_ptr<kernel::KernelBuilder> & b);
};


#endif //ICGREP_LZ4GENERATOR_H
