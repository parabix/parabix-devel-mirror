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

class GrepEngine {
public:

    GrepEngine() : mGrepDriver(nullptr), mFileSuffix(InitialTabFlag ? "\t:" : ":"), grepMatchFound(false), fileCount(0) {}

    virtual ~GrepEngine();
    
    void initFileResult(std::vector<std::string> filenames);
    
    virtual void grepCodeGen(std::vector<re::RE *> REs);

    void run();
    
    void PrintResults();
    
    
    
protected:
    static void * DoGrepThreadFunction(void *args);
    virtual uint64_t doGrep(const std::string & fileName, const uint32_t fileIdx);
    std::string linePrefix(std::string fileName);
    
    Driver * mGrepDriver;

    std::vector<std::string> inputFiles;
    std::vector<std::stringstream> mResultStrs;
    
    std::string mFileSuffix;
    
    bool grepMatchFound;
    std::mutex count_mutex;
    size_t fileCount;
};

class CountOnlyGrepEngine : public GrepEngine {
public:
    
    CountOnlyGrepEngine() : GrepEngine() {mFileSuffix = ":";}
    void grepCodeGen(std::vector<re::RE *> REs) override;
private:
    uint64_t doGrep(const std::string & fileName, const uint32_t fileIdx) override;
    
};

class MatchOnlyGrepEngine : public CountOnlyGrepEngine {
public:
    
    MatchOnlyGrepEngine() : CountOnlyGrepEngine(), mRequiredCount(Mode != FilesWithoutMatch) {mFileSuffix = NullFlag ? std::string("\0", 1) : "\n";}
private:
    uint64_t doGrep(const std::string & fileName, const uint32_t fileIdx) override;
    unsigned mRequiredCount;        
};
    
}

#endif
