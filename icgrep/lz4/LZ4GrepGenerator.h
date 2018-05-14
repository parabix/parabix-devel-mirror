
#ifndef ICGREP_LZ4GREPGENERATOR_H
#define ICGREP_LZ4GREPGENERATOR_H

#include "LZ4Generator.h"
#include <grep_interface.h>
#include <kernels/streamset.h>
#include <cc/multiplex_CCs.h>
#include <string>
#include <vector>
#include <sstream>
#include <atomic>
#include <grep/grep_engine.h>

typedef void (*ScanMatchGrepMainFunctionType)(char * byte_data, size_t headerSize, size_t filesize, bool hasBlockChecksum, intptr_t match_accumulator);

class LZ4GrepGenerator : public LZ4Generator{
public:
    LZ4GrepGenerator(bool enableMultiplexing = false);
    void generateSwizzledCountOnlyGrepPipeline(re::RE *regex);
    void generateCountOnlyGrepPipeline(re::RE *regex);

    void generateScanMatchGrepPipeline(re::RE* regex);
    std::pair<parabix::StreamSetBuffer *, parabix::StreamSetBuffer *> grepPipeline(std::vector<re::RE *> &REs,
                                                                                   parabix::StreamSetBuffer *decompressedBasisBits);
    std::pair<parabix::StreamSetBuffer *, parabix::StreamSetBuffer *> multiplexingGrepPipeline(std::vector<re::RE *> &REs,
                                                                                   parabix::StreamSetBuffer *matchCopiedBasisBits);


    void invokeScanMatchGrep(char* fileBuffer, size_t blockStart, size_t blockEnd, bool hasBlockChecksum);

private:
    bool mEnableMultiplexing;

    grep::GrepRecordBreakKind mGrepRecordBreak;
    void initREs(std::vector<re::RE *> & REs);


    re::CC * mBreakCC;
    std::vector<re:: RE *> mREs;
    std::set<re::Name *> mUnicodeProperties;
    bool mMoveMatchesToEOL;


    std::vector<std::ostringstream> mResultStrs;


    void generateScanMatchMainFunc(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);


    llvm::Value * match_accumulator;

    ScanMatchGrepMainFunctionType getScanMatchGrepMainFunction();

    std::unique_ptr<cc::MultiplexedAlphabet> mpx;

    parabix::StreamSetBuffer * linefeedStreamFromDecompressedBits(parabix::StreamSetBuffer *decompressedBasisBits);
    parabix::StreamSetBuffer * linefeedStreamFromCompressedBits();
};


#endif //ICGREP_LZ4GREPGENERATOR_H
