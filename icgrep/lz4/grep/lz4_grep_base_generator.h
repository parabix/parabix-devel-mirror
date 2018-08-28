
#ifndef ICGREP_LZ4GREPGENERATOR_H
#define ICGREP_LZ4GREPGENERATOR_H

#include "lz4/lz4_base_generator.h"

#include <grep_interface.h>
#include <kernels/streamset.h>
#include <cc/multiplex_CCs.h>
#include <string>
#include <vector>
#include <sstream>
#include <atomic>
#include <grep/grep_engine.h>

typedef void (*ScanMatchGrepMainFunctionType)(char * byte_data, size_t headerSize, size_t filesize, bool hasBlockChecksum, intptr_t match_accumulator);
typedef uint64_t (*CountOnlyGrepMainFunctionType)(char * byte_data, size_t headerSize, size_t filesize, bool hasBlockChecksum);



class LZ4GrepBaseGenerator : public LZ4BaseGenerator {
public:

    LZ4GrepBaseGenerator();

    void generateScanMatchGrepPipeline(re::RE* regex, bool enableMultiplexing, bool utf8CC);
    void generateCountOnlyGrepPipeline(re::RE* regex, bool enableMultiplexing, bool utf8CC);


    void invokeScanMatchGrep(char* fileBuffer, size_t blockStart, size_t blockEnd, bool hasBlockChecksum);

    ScanMatchGrepMainFunctionType getScanMatchGrepMainFunction();
    CountOnlyGrepMainFunctionType getCountOnlyGrepMainFunction();



protected:
    virtual parabix::StreamSetBuffer* generateUncompressedByteStream() {
        parabix::StreamSetBuffer* bitStreams = this->generateUncompressedBitStreams();
        return this->p2s(bitStreams);
    }
    virtual parabix::StreamSetBuffer* generateUncompressedBitStreams() = 0;
    virtual parabix::StreamSetBuffer* decompressBitStream(parabix::StreamSetBuffer* compressedByteStream, parabix::StreamSetBuffer* compressedBitStream) = 0;
    virtual std::vector<parabix::StreamSetBuffer*> decompressBitStreams(parabix::StreamSetBuffer* compressedByteStream, std::vector<parabix::StreamSetBuffer*> compressedBitStreams);

    std::vector<parabix::StreamSetBuffer*> generateFakeStreams(
            const std::unique_ptr<kernel::KernelBuilder> & iBuilder,
            parabix::StreamSetBuffer* refStream,
            std::vector<unsigned> numOfStreams
    );

private:
    grep::GrepRecordBreakKind mGrepRecordBreak;
    void initREs(re::RE * REs);


    re::CC * mBreakCC;
    re:: RE * mRE;
    std::set<re::Name *> mUnicodeProperties;
    bool mMoveMatchesToEOL;
    re::RE* u8NonFinalRe;
    re::RE* u8FinalRe;


    std::vector<std::ostringstream> mResultStrs;

    void generateCountOnlyMainFunc(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    void generateScanMatchMainFunc(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);


    llvm::Value * match_accumulator;


    parabix::StreamSetBuffer * linefeedStreamFromUncompressedBits(parabix::StreamSetBuffer *uncompressedBasisBits);


    void generateFullyDecompressionScanMatchGrepPipeline(re::RE *regex);
    void generateMultiplexingScanMatchGrepPipeline(re::RE *regex, bool utf8CC);


    void generateFullyDecompressionCountOnlyGrepPipeline(re::RE *regex);
    void generateMultiplexingCountOnlyGrepPipeline(re::RE *regex, bool utf8CC);


    std::pair<parabix::StreamSetBuffer *, parabix::StreamSetBuffer *> grep(re::RE *RE, parabix::StreamSetBuffer *byteStream,
                                                                           parabix::StreamSetBuffer *uncompressedBasisBits, bool ccMultiplexing = false);
    std::pair<parabix::StreamSetBuffer *, parabix::StreamSetBuffer *> multiplexingGrep(
            re::RE *RE,
            parabix::StreamSetBuffer *compressedByteStream,
            parabix::StreamSetBuffer *compressedBitStream,
            bool utf8CC
    );
    std::unique_ptr<cc::MultiplexedAlphabet> mpx;

};


#endif //ICGREP_LZ4GREPGENERATOR_H
