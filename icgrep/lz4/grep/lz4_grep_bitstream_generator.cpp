#include "lz4_grep_bitstream_generator.h"

using namespace kernel;

StreamSet* LZ4GrepBitStreamGenerator::generateUncompressedBitStreams() {
    StreamSet *compressedByteStream = nullptr, *compressedBasisBits = nullptr;
    std::tie(compressedByteStream, compressedBasisBits) = loadByteStreamAndBitStream();
    return bitStreamDecompression(compressedByteStream, compressedBasisBits);
}

StreamSet *LZ4GrepBitStreamGenerator::decompressBitStream(StreamSet *compressedByteStream, StreamSet * compressedBitStream) {
    const auto ret = convertCompressedBitsStreamWithBitStreamAioApproach(compressedByteStream, {compressedBitStream});
    assert (ret.size() == 1);
    return ret.front();
}

StreamSets LZ4GrepBitStreamGenerator::decompressBitStreams(StreamSet * compressedByteStream, StreamSets compressedBitStreams) {
    return convertCompressedBitsStreamWithBitStreamAioApproach(compressedByteStream, compressedBitStreams);
}
