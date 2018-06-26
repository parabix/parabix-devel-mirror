//
// Created by wxy325 on 2018/6/19.
//

#ifndef ICGREP_LZPARABIXGREPGENERATOR_H
#define ICGREP_LZPARABIXGREPGENERATOR_H

#include "LZParabixGenerator.h"
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

class LZParabixGrepGenerator: public LZParabixGenerator {

public:
    LZParabixGrepGenerator(bool enableMultiplexing = false);
    void generateCountOnlyAioPipeline(re::RE* regex);
    CountOnlyGrepMainFunctionType getCountOnlyGrepMainFunction();
protected:
    bool mEnableMultiplexing;
    grep::GrepRecordBreakKind mGrepRecordBreak;
    re::CC * mBreakCC;
    std::vector<re:: RE *> mREs;
    std::set<re::Name *> mUnicodeProperties;
    bool mMoveMatchesToEOL;
    std::unique_ptr<cc::MultiplexedAlphabet> mpx;

    void initREs(std::vector<re::RE *> & REs);

    std::pair<parabix::StreamSetBuffer *, parabix::StreamSetBuffer *> grepPipeline(std::vector<re::RE *> &REs);
    std::pair<parabix::StreamSetBuffer *, parabix::StreamSetBuffer *> multiplexingGrepPipeline(std::vector<re::RE *> &REs);

    parabix::StreamSetBuffer * linefeedStreamFromDecompressedBits(parabix::StreamSetBuffer *decompressedBasisBits);

    void generateCountOnlyMainFunc(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
};


#endif //ICGREP_LZPARABIXGREPGENERATOR_H
