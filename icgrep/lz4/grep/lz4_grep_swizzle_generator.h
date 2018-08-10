
#ifndef ICGREP_LZ4_GREP_SWIZZLE_GENERATOR_H
#define ICGREP_LZ4_GREP_SWIZZLE_GENERATOR_H


#include "lz4_grep_base_generator.h"

class LZ4GrepSwizzleGenerator: public LZ4GrepBaseGenerator {
protected:
    virtual parabix::StreamSetBuffer* generateUncompressedBitStreams() override;
    virtual parabix::StreamSetBuffer* decompressBitStream(parabix::StreamSetBuffer* compressedByteStream, parabix::StreamSetBuffer* compressedBitStream) override;

private:
    parabix::StreamSetBuffer * convertCompressedBitsStreamWithSwizzledAioApproach(
            parabix::StreamSetBuffer *compressedByteStream,
            parabix::StreamSetBuffer *compressedBitStream,
            std::string prefix
    );
};


#endif //ICGREP_LZ4_GREP_SWIZZLE_GENERATOR_H
