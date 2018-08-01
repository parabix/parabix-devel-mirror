

#ifndef ICGREP_LZ4BASEGENERATOR_H
#define ICGREP_LZ4BASEGENERATOR_H

#include <llvm/IR/Module.h>
#include <llvm/IR/Function.h>

#include <llvm/Support/Signals.h>
#include <llvm/Support/ManagedStatic.h>
#include <IR_Gen/idisa_target.h>

#include <toolchain/toolchain.h>

#include <toolchain/cpudriver.h>
#include <string>

struct LZ4BlockInfo {
    parabix::StreamSetBuffer* blockStart;
    parabix::StreamSetBuffer* blockEnd;
    parabix::StreamSetBuffer* isCompress;
};

class LZ4BaseGenerator {
public:
    LZ4BaseGenerator();

protected:
    //// Member Function
    // Input
    parabix::StreamSetBuffer* loadByteStream();
    std::pair<parabix::StreamSetBuffer*, parabix::StreamSetBuffer*> loadByteStreamAndBitStream();

    // Stream Conversion
    parabix::StreamSetBuffer* s2p(parabix::StreamSetBuffer* byteStream);


            // LZ4 Decoder
    LZ4BlockInfo getBlockInfo(parabix::StreamSetBuffer* compressedByteStream);
    parabix::StreamSetBuffer * byteStreamDecompression(
            parabix::StreamSetBuffer* compressedByteStream
    );
    parabix::StreamSetBuffer * swizzledDecompression(
            parabix::StreamSetBuffer* compressedByteStream,
            parabix::StreamSetBuffer* compressedBasisBits
    );
    parabix::StreamSetBuffer * bitStreamDecompression(
            parabix::StreamSetBuffer* compressedByteStream,
            parabix::StreamSetBuffer* compressedBasisBits
    );

    parabix::StreamSetBuffer *parallelByteStreamDecompression(
            parabix::StreamSetBuffer *compressedByteStream,
            bool enableGather, bool enableScatter, int minParallelLevel
    );

    std::vector<parabix::StreamSetBuffer*> convertCompressedBitsStreamWithBitStreamAioApproach(
            parabix::StreamSetBuffer* compressedByteStream,
            std::vector<parabix::StreamSetBuffer*> compressedBitStreams
    );



    // BufferSize related Helper Function
    unsigned getDefaultBufferBlocks();
    unsigned getBlockSizeBufferBlocks();


    //// Data Member
    // Driver
    ParabixDriver mPxDriver;

    // Runtime Arguments
    llvm::Value * mInputStream;
    llvm::Value * mHeaderSize;
    llvm::Value * mFileSize;
    llvm::Value * mHasBlockChecksum;

    unsigned mLz4BlockSize;
};


#endif //ICGREP_LZ4BASEGENERATOR_H
