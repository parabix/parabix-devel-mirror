

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

namespace kernel { class StreamSet; }
namespace kernel { class Scalar; }

struct LZ4BlockInfo {
    kernel::StreamSet * blockStart;
    kernel::StreamSet * blockEnd;
    kernel::StreamSet * isCompress;

    LZ4BlockInfo() : blockStart(nullptr), blockEnd(nullptr), isCompress(nullptr) { }
};

class LZ4BaseGenerator {
public:
    LZ4BaseGenerator();
    virtual ~LZ4BaseGenerator();
protected:
    //// Member Function
    // Input
    kernel::StreamSet * loadByteStream();
    std::pair<kernel::StreamSet*, kernel::StreamSet*> loadByteStreamAndBitStream();

    // Stream Conversion
    kernel::StreamSet* s2p(kernel::StreamSet* byteStream);
    kernel::StreamSet* p2s(kernel::StreamSet* bitStream);


            // LZ4 Decoder
    LZ4BlockInfo getBlockInfo(kernel::StreamSet* compressedByteStream);

    kernel::StreamSet * byteStreamDecompression(kernel::StreamSet* compressedByteStream);

    kernel::StreamSet * swizzledDecompression(kernel::StreamSet* compressedByteStream, kernel::StreamSet* compressedBasisBits);

    kernel::StreamSet * bitStreamDecompression(kernel::StreamSet* compressedByteStream, kernel::StreamSet* compressedBasisBits);

    kernel::StreamSets convertCompressedBitsStreamWithBitStreamAioApproach(kernel::StreamSet* compressedByteStream, kernel::StreamSets compressedBitStreams);

    // BufferSize related Helper Function
    unsigned getDefaultBufferBlocks();
    unsigned getBlockSizeBufferBlocks();


    //// Data Member
    // Driver
    CPUDriver mPxDriver;
    std::unique_ptr<kernel::ProgramBuilder> mPipeline;

    // Runtime Arguments
    kernel::Scalar * mInputStream;
    kernel::Scalar * mHeaderSize;
    kernel::Scalar * mFileSize;
    kernel::Scalar * mHasBlockChecksum;

    const unsigned mLz4BlockSize;

    LZ4BlockInfo mBlockInfo;
};


#endif //ICGREP_LZ4BASEGENERATOR_H
