

#include "lz4_grep_bitstream_generator.h"


namespace re { class CC; }

using namespace llvm;
using namespace parabix;
using namespace kernel;
using namespace grep;

parabix::StreamSetBuffer* LZ4GrepBitStreamGenerator::generateUncompressedBitStreams() {
    StreamSetBuffer *compressedByteStream = nullptr, *compressedBasisBits = nullptr;
    std::tie(compressedByteStream, compressedBasisBits) = this->loadByteStreamAndBitStream();
    return this->bitStreamDecompression(compressedByteStream, compressedBasisBits);
}

parabix::StreamSetBuffer *LZ4GrepBitStreamGenerator::decompressBitStream(parabix::StreamSetBuffer *compressedByteStream,
                                                                         parabix::StreamSetBuffer *compressedBitStream) {
    auto ret = this->convertCompressedBitsStreamWithBitStreamAioApproach(compressedByteStream, {compressedBitStream});
    return ret[0];
}

std::vector<parabix::StreamSetBuffer *>
LZ4GrepBitStreamGenerator::decompressBitStreams(parabix::StreamSetBuffer *compressedByteStream,
                                                std::vector<parabix::StreamSetBuffer *> compressedBitStreams) {
    return this->convertCompressedBitsStreamWithBitStreamAioApproach(compressedByteStream, compressedBitStreams);
}
