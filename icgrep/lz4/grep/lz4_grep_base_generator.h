
#ifndef ICGREP_LZ4GREPGENERATOR_H
#define ICGREP_LZ4GREPGENERATOR_H

#include "lz4/lz4_base_generator.h"

#include <grep_interface.h>
#include <kernels/core/relationship.h>
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

    enum FunctionType { CountOnly, Match };

    void generateScanMatchGrepPipeline(re::RE* regex, bool enableMultiplexing, bool utf8CC);
    void generateCountOnlyGrepPipeline(re::RE* regex, bool enableMultiplexing, bool utf8CC);


    void invokeScanMatchGrep(char* fileBuffer, size_t blockStart, size_t blockEnd, bool hasBlockChecksum);

    ScanMatchGrepMainFunctionType getScanMatchGrepMainFunction();
    CountOnlyGrepMainFunctionType getCountOnlyGrepMainFunction();

protected:

    LZ4GrepBaseGenerator(const FunctionType type);

    virtual kernel::StreamSet* generateUncompressedByteStream() {
        kernel::StreamSet* bitStreams = generateUncompressedBitStreams();
        return p2s(bitStreams);
    }

    virtual kernel::StreamSet* generateUncompressedBitStreams() = 0;
    virtual kernel::StreamSet* decompressBitStream(kernel::StreamSet* compressedByteStream, kernel::StreamSet* compressedBitStream) = 0;
    virtual kernel::StreamSets decompressBitStreams(kernel::StreamSet* compressedByteStream, kernel::StreamSets compressedBitStreams);

    kernel::StreamSets generateFakeStreams(kernel::StreamSet * refStream, std::vector<unsigned> numOfStreams);

    unsigned calculateTwistWidth(unsigned numOfStreams);

private:

    std::unique_ptr<kernel::ProgramBuilder> makeInternalPipeline(const FunctionType type);

    grep::GrepRecordBreakKind mGrepRecordBreak;

    void initREs(re::RE * re);


    re::CC * mBreakCC;
    re:: RE * mRE;
    std::set<re::Name *> mUnicodeProperties;
    bool mMoveMatchesToEOL;
    re::RE* u8NonFinalRe;
    re::RE* u8FinalRe;
    kernel::Scalar * match_accumulator;
    void * mMainMethod;

    std::vector<std::ostringstream> mResultStrs;

    kernel::StreamSet * linefeedStreamFromUncompressedBits(kernel::StreamSet *uncompressedBasisBits);


    void generateFullyDecompressionScanMatchGrepPipeline(re::RE *regex);
    void generateMultiplexingScanMatchGrepPipeline(re::RE *regex, bool utf8CC);


    void generateFullyDecompressionCountOnlyGrepPipeline(re::RE *regex);
    void generateMultiplexingCountOnlyGrepPipeline(re::RE *regex, bool utf8CC);


    std::pair<kernel::StreamSet *, kernel::StreamSet *> grep(
            re::RE * re, kernel::StreamSet *byteStream, kernel::StreamSet *uncompressedBasisBits, bool ccMultiplexing = false);

    std::pair<kernel::StreamSet *, kernel::StreamSet *> multiplexingGrep(
            re::RE * re, kernel::StreamSet *compressedByteStream, kernel::StreamSet *compressedBitStream, bool utf8CC);

};


#endif //ICGREP_LZ4GREPGENERATOR_H
