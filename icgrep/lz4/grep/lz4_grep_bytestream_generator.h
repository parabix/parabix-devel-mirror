
#ifndef ICGREP_LZ4_GREP_BYTESTREAM_GENERATOR_H
#define ICGREP_LZ4_GREP_BYTESTREAM_GENERATOR_H

#include "lz4_grep_base_generator.h"

class LZ4GrepByteStreamGenerator: public LZ4GrepBaseGenerator {
protected:
    virtual parabix::StreamSetBuffer* generateUncompressedBitStreams() override;
    virtual parabix::StreamSetBuffer* decompressBitStream(parabix::StreamSetBuffer* compressedByteStream, parabix::StreamSetBuffer* compressedBitStream) override;
    virtual std::vector<parabix::StreamSetBuffer*> decompressBitStreams(parabix::StreamSetBuffer* compressedByteStream, std::vector<parabix::StreamSetBuffer*> compressedBitStreams) override;

private:
    unsigned calculateTwistWidth(unsigned numOfStreams);
    parabix::StreamSetBuffer* twist(const std::unique_ptr<kernel::KernelBuilder> &b,
                                    std::vector<parabix::StreamSetBuffer*> inputStreams,
                                    unsigned twistWidth);
    std::vector<parabix::StreamSetBuffer*> untwist(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, parabix::StreamSetBuffer* inputStream, unsigned twistWidth, std::vector<unsigned> numOfStreams);
};


#endif //ICGREP_LZ4_GREP_BYTESTREAM_GENERATOR_H
