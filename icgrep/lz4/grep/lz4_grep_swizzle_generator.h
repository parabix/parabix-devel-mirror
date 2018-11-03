
#ifndef ICGREP_LZ4_GREP_SWIZZLE_GENERATOR_H
#define ICGREP_LZ4_GREP_SWIZZLE_GENERATOR_H


#include "lz4_grep_base_generator.h"

class LZ4GrepSwizzleGenerator final : public LZ4GrepBaseGenerator {
public:
    LZ4GrepSwizzleGenerator(const FunctionType type) : LZ4GrepBaseGenerator(type) { }
protected:
    kernel::StreamSet* generateUncompressedBitStreams() override;
    kernel::StreamSet* decompressBitStream(kernel::StreamSet* compressedByteStream, kernel::StreamSet* compressedBitStream) override;
private:
    kernel::StreamSet * convertCompressedBitsStreamWithSwizzledAioApproach(kernel::StreamSet *compressedByteStream, kernel::StreamSet *compressedBitStream);
};


#endif //ICGREP_LZ4_GREP_SWIZZLE_GENERATOR_H
