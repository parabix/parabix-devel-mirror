
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

class LZ4Generator {

public:
    LZ4Generator();

    MainFunctionType getMainFunc();

    void generatePipeline(const std::string &outputFile);
    void generateSwizzledPipeline(const std::string &outputFile);
    void generateExtractOnlyPipeline(const std::string &outputFile);
    void generateSwizzledExtractOnlyPipeline(const std::string &outputFile);
    void generateExtractAndDepositOnlyPipeline(const std::string &outputFile);
    void generateSwizzledExtractAndDepositOnlyPipeline(const std::string &outputFile);

protected:
    //// Protected Method
    void generateMainFunc(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    // Pipeline
    virtual void generateLoadByteStreamAndBitStream(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    virtual void generateExtractAndDepositMarkers(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    virtual std::pair<parabix::StreamSetBuffer*, parabix::StreamSetBuffer*> generateSwizzleExtractData(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    virtual parabix::StreamSetBuffer* generateBitStreamExtractData(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);

    // BufferSize related Helper Function
    virtual int getInputBufferBlocks();
    virtual int getDecompressedBufferBlocks();
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
    parabix::StreamSetBuffer * mDeletionMarker; //TODO rename to ExtarctMarker
    parabix::StreamSetBuffer * mDepositMarker;
    parabix::StreamSetBuffer * mMatchOffsetMarker;

    // M0CountMarker will not contain anything, it will only be used to pass producedItemCount and manage processedItemCount between different kernel
    parabix::StreamSetBuffer * mM0Marker;
};


#endif //ICGREP_LZ4GENERATOR_H
