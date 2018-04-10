//
// Created by wxy325 on 2018/3/15.
//

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
    LZ4GrepGenerator();
    void generateCountOnlyGrepPipeline(re::RE* regex);
    void generateScanMatchGrepPipeline(re::RE* regex);
    std::pair<parabix::StreamSetBuffer *, parabix::StreamSetBuffer *> grepPipeline(std::vector<re::RE *> &REs,
                                                                                   parabix::StreamSetBuffer *ByteStream);

    void invokeScanMatchGrep(char* fileBuffer, size_t blockStart, size_t blockEnd, bool hasBlockChecksum);

private:
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
};


#endif //ICGREP_LZ4GREPGENERATOR_H
