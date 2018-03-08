/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#ifndef GREP_ENGINE_H
#define GREP_ENGINE_H
#include <grep_interface.h>
#include <kernels/streamset.h>
#include <cc/multiplex_CCs.h>
#include <string>
#include <vector>
#include <sstream>
#include <atomic>

namespace re { class CC; }
namespace re { class RE; }
namespace llvm { namespace cl { class OptionCategory; } }
class Driver;


namespace grep {
    class MatchAccumulator {
    public:
        MatchAccumulator() {}
        virtual void accumulate_match(const size_t lineNum, char * line_start, char * line_end) = 0;
        virtual void finalize_match(char * buffer_end) {}  // default: no op
    };
    
    void accumulate_match_wrapper(intptr_t accum_addr, const size_t lineNum, char * line_start, char * line_end);
    
    void finalize_match_wrapper(intptr_t accum_addr, char * buffer_end);
    
    void grepBuffer(re::RE * pattern, const char * buffer, size_t bufferLength, MatchAccumulator * accum);

class GrepEngine {
    enum class FileStatus {Pending, GrepComplete, PrintComplete};
public:

    GrepEngine();
    virtual ~GrepEngine();
    
    void initFileResult(std::vector<std::string> & filenames);
    virtual void grepCodeGen(std::vector<re::RE *> REs);
    bool searchAllFiles();
    void * DoGrepThreadMethod();

protected:
    std::pair<parabix::StreamSetBuffer *, parabix::StreamSetBuffer *> grepPipeline(std::vector<re::RE *> & REs, parabix::StreamSetBuffer * ByteStream);

    virtual uint64_t doGrep(const std::string & fileName, const uint32_t fileIdx);
    std::string linePrefix(std::string fileName);
    int32_t openFile(const std::string & fileName, std::ostringstream & msgstrm);

    Driver * mGrepDriver;

    std::atomic<unsigned> mNextFileToGrep;
    std::atomic<unsigned> mNextFileToPrint;
    std::vector<std::string> inputFiles;
    std::vector<std::ostringstream> mResultStrs;
    std::vector<FileStatus> mFileStatus;
    bool grepMatchFound;

    std::unique_ptr<cc::MultiplexedAlphabet> mpx;
    std::string mFileSuffix;
    bool mMoveMatchesToEOL;
    pthread_t mEngineThread;
};

class EmitMatchesEngine : public GrepEngine {
public:
    EmitMatchesEngine();
    void grepCodeGen(std::vector<re::RE *> REs) override;
private:
    uint64_t doGrep(const std::string & fileName, const uint32_t fileIdx) override;
};

class CountOnlyEngine : public GrepEngine {
public:
    CountOnlyEngine();
private:
    uint64_t doGrep(const std::string & fileName, const uint32_t fileIdx) override;
};

class MatchOnlyEngine : public GrepEngine {
public:
    MatchOnlyEngine(bool showFilesWithoutMatch);
private:
    uint64_t doGrep(const std::string & fileName, const uint32_t fileIdx) override;
    unsigned mRequiredCount;
};

class QuietModeEngine : public GrepEngine {
public:
    QuietModeEngine();
};

}

#endif
