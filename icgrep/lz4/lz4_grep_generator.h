
#ifndef ICGREP_LZ4GREPGENERATOR_H
#define ICGREP_LZ4GREPGENERATOR_H

#include "lz4_base_generator.h"

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


class LZ4GrepGenerator : public LZ4BaseGenerator {
public:

    LZ4GrepGenerator(bool enableMultiplexing = false);

    void generateScanMatchGrepPipeline(re::RE* regex);
    std::pair<parabix::StreamSetBuffer *, parabix::StreamSetBuffer *> grepPipeline(std::vector<re::RE *> &REs,
                                                                                   parabix::StreamSetBuffer *uncompressedBasisBits);

    std::pair<parabix::StreamSetBuffer *, parabix::StreamSetBuffer *> multiplexingGrepPipeline(
            std::vector<re::RE *> &REs,
            parabix::StreamSetBuffer *compressedByteStream,
            parabix::StreamSetBuffer *compressedBitStream,
            bool useSwizzled = true,
            bool useByteStream = false
    );


    void invokeScanMatchGrep(char* fileBuffer, size_t blockStart, size_t blockEnd, bool hasBlockChecksum);

    void generateMultiplexingSwizzledAioPipeline(re::RE *regex);
    void generateSwizzledAioPipeline(re::RE* regex);
    void generateMultiplexingBitStreamAioPipeline(re::RE* regex);
    void generateBitStreamAioPipeline(re::RE* regex);

    void generateByteStreamMultiplexingAioPipeline(re::RE* regex);
    void generateAioPipeline(re::RE* regex);
    void generateParallelAioPipeline(re::RE* regex, bool enableGather, bool enableScatter, int minParallelLevel);

    ScanMatchGrepMainFunctionType getScanMatchGrepMainFunction();
    CountOnlyGrepMainFunctionType getCountOnlyGrepMainFunction();

private:
    bool mEnableMultiplexing;

    grep::GrepRecordBreakKind mGrepRecordBreak;
    void initREs(std::vector<re::RE *> & REs);


    re::CC * mBreakCC;
    std::vector<re:: RE *> mREs;
    std::set<re::Name *> mUnicodeProperties;
    bool mMoveMatchesToEOL;


    std::vector<std::ostringstream> mResultStrs;

    void generateCountOnlyMainFunc(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    void generateScanMatchMainFunc(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);


    llvm::Value * match_accumulator;



    std::unique_ptr<cc::MultiplexedAlphabet> mpx;

    parabix::StreamSetBuffer * linefeedStreamFromUncompressedBits(parabix::StreamSetBuffer *uncompressedBasisBits);



    parabix::StreamSetBuffer * convertCompressedBitsStreamWithSwizzledAioApproach(
            parabix::StreamSetBuffer *compressedByteStream,
            parabix::StreamSetBuffer *compressedBitStream,
            std::string prefix
    );

    parabix::StreamSetBuffer *convertCompressedBitsStreamWithByteStreamAioApproach(
            parabix::StreamSetBuffer *compressedByteStream,
            parabix::StreamSetBuffer *compressedBitStream,
            std::string prefix
    );

};


#endif //ICGREP_LZ4GREPGENERATOR_H
