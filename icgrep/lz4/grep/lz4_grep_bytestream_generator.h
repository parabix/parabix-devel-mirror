
#ifndef ICGREP_LZ4_GREP_BYTESTREAM_GENERATOR_H
#define ICGREP_LZ4_GREP_BYTESTREAM_GENERATOR_H

#include "lz4_grep_base_generator.h"

class LZ4GrepByteStreamGenerator final : public LZ4GrepBaseGenerator {
public:
    LZ4GrepByteStreamGenerator(const FunctionType type) : LZ4GrepBaseGenerator(type) { }
protected:
    kernel::StreamSet * generateUncompressedByteStream() override;
    kernel::StreamSet * generateUncompressedBitStreams() override;
    kernel::StreamSet * decompressBitStream(kernel::StreamSet* compressedByteStream, kernel::StreamSet* compressedBitStream) override;
    kernel::StreamSets decompressBitStreams(kernel::StreamSet* compressedByteStream, kernel::StreamSets compressedBitStreams) override;
private:
    kernel::StreamSet* twist(const kernel::StreamSets &inputStreams, const unsigned twistWidth);
    kernel::StreamSets untwist(kernel::StreamSet * inputStream, const unsigned twistWidth, const std::vector<unsigned> & numOfStreams);
};


#endif //ICGREP_LZ4_GREP_BYTESTREAM_GENERATOR_H
