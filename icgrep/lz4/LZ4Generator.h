
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

    void generatePipeline(const std::string& outputFile);
    void generateExtractOnlyPipeline(const std::string& outputFile);
    void generateExtractAndDepositOnlyPipeline(const std::string& outputFile);

protected:
    //// Protected Method
    inline void generateMainFunc(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    // Pipeline
    virtual void generateLoadByteStreamAndBitStream(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    virtual void generateExtractAndDepositMarkers(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    virtual std::pair<parabix::StreamSetBuffer*, parabix::StreamSetBuffer*> generateSwizzleExtractData(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);

    // BufferSize related Helper Function
    virtual int getInputBufferBlocks();
    virtual int getDecompressedBufferBlocks();
    int get4MbBufferBlocks();


    //// Data Member
    // Driver
    ParabixDriver pxDriver;

    // Runtime Arguments
    llvm::Value * inputStream;
    llvm::Value * headerSize;
    llvm::Value * fileSize;
    llvm::Value * hasBlockChecksum;


    // StreamSetBuffers
    parabix::StreamSetBuffer * ByteStream;
    parabix::StreamSetBuffer * BasisBits;
    parabix::StreamSetBuffer * DeletionMarker; //TODO rename to ExtarctMarker
    parabix::StreamSetBuffer * DepositMarker;
    parabix::StreamSetBuffer * Match_Offset;

    parabix::StreamSetBuffer * M0_Start;  // TODO M0_Start and M0_End should be changed to Deposit_Start and Deposit_End
    parabix::StreamSetBuffer * M0_End;
};


#endif //ICGREP_LZ4GENERATOR_H
