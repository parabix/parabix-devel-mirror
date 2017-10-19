/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#ifndef GREP_ENGINE_H
#define GREP_ENGINE_H
#include <grep_interface.h>
#include <toolchain/grep_pipeline.h>
#include <string>       // for string
#include <vector>
#include <sstream>
#include <mutex>

namespace re { class CC; }
namespace re { class RE; }
namespace llvm { namespace cl { class OptionCategory; } }
class Driver;


namespace grep {

// Thread function only.
void *DoGrepThreadFunction(void *args);
    

class NonNormalizingReportMatch : public MatchAccumulator {
public:
    NonNormalizingReportMatch(std::string linePrefix) : mLinePrefix(linePrefix), mLineCount(0), mPrevious_line_end(nullptr) {}
    void accumulate_match(const size_t lineNum, char * line_start, char * line_end) override;
    void finalize_match(char * buffer_end) override;
    std::string mLinePrefix;
    size_t mLineCount;
    char * mPrevious_line_end;
    std::stringstream mResultStr;
    
};


class GrepEngine {
public:

    GrepEngine() : mGrepDriver(nullptr), grepMatchFound(false), fileCount(0) {}


    virtual ~GrepEngine();
    
    void initFileResult(std::vector<std::string> filenames);
    
    virtual void grepCodeGen(std::vector<re::RE *> REs);

    uint64_t doGrep(const std::string & fileName, const uint32_t fileIdx);

    uint64_t doGrep(const int32_t fileDescriptor, const uint32_t fileIdx);
    
    void PrintResults();
    
    

    Driver * mGrepDriver;
    bool grepMatchFound;

    std::vector<std::unique_ptr<NonNormalizingReportMatch>> resultAccums;
    std::vector<std::string> inputFiles;
    
    std::mutex count_mutex;
    size_t fileCount;
    
};

class CountOnlyGrepEngine : public GrepEngine {
public:
    
    CountOnlyGrepEngine() : GrepEngine() {}
    void grepCodeGen(std::vector<re::RE *> REs) override;
    
};

}

#endif
