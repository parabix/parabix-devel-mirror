

#ifndef ICGREP_LZ4_GREP_BITSTREAM_GENERATOR_H
#define ICGREP_LZ4_GREP_BITSTREAM_GENERATOR_H

#include "lz4_grep_base_generator.h"

class LZ4GrepBitStreamGenerator: public LZ4GrepBaseGenerator {
protected:
    virtual parabix::StreamSetBuffer* generateUncompressedBitStreams() override;
    virtual parabix::StreamSetBuffer* decompressBitStream(parabix::StreamSetBuffer* compressedByteStream, parabix::StreamSetBuffer* compressedBitStream) override;
    virtual std::vector<parabix::StreamSetBuffer*> decompressBitStreams(parabix::StreamSetBuffer* compressedByteStream, std::vector<parabix::StreamSetBuffer*> compressedBitStreams) override;
};


#endif //ICGREP_LZ4_GREP_BITSTREAM_GENERATOR_H
