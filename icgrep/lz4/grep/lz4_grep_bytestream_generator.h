
#ifndef ICGREP_LZ4_GREP_BYTESTREAM_GENERATOR_H
#define ICGREP_LZ4_GREP_BYTESTREAM_GENERATOR_H

#include "lz4_grep_base_generator.h"

class LZ4GrepByteStreamGenerator: public LZ4GrepBaseGenerator {
protected:
    virtual parabix::StreamSetBuffer* generateUncompressedBitStreams() override;
    virtual parabix::StreamSetBuffer* decompressBitStream(parabix::StreamSetBuffer* compressedByteStream, parabix::StreamSetBuffer* compressedBitStream) override;

private:
    parabix::StreamSetBuffer *convertCompressedBitsStreamWithTwistApproach(
            parabix::StreamSetBuffer *compressedByteStream,
            parabix::StreamSetBuffer *compressedBitStream,
            std::string prefix
    );
};


#endif //ICGREP_LZ4_GREP_BYTESTREAM_GENERATOR_H
