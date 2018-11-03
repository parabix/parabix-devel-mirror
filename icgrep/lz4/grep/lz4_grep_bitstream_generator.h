

#ifndef ICGREP_LZ4_GREP_BITSTREAM_GENERATOR_H
#define ICGREP_LZ4_GREP_BITSTREAM_GENERATOR_H

#include "lz4_grep_base_generator.h"

class LZ4GrepBitStreamGenerator final : public LZ4GrepBaseGenerator {
public:
    LZ4GrepBitStreamGenerator(const FunctionType type) : LZ4GrepBaseGenerator(type) { }
protected:
    kernel::StreamSet * generateUncompressedBitStreams() override;
    kernel::StreamSet * decompressBitStream(kernel::StreamSet * compressedByteStream, kernel::StreamSet * compressedBitStream) override;
    kernel::StreamSets decompressBitStreams(kernel::StreamSet * compressedByteStream, kernel::StreamSets compressedBitStreams) override;
};


#endif //ICGREP_LZ4_GREP_BITSTREAM_GENERATOR_H
