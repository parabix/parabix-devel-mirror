
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
#include <util/aligned_allocator.h>
#include <boost/filesystem.hpp>

namespace re { class CC; }
namespace re { class RE; }
namespace llvm { namespace cl { class OptionCategory; } }
class Driver;


namespace grep {
    
enum class GrepRecordBreakKind {Null, LF, Unicode};

class MatchAccumulator {
public:
    MatchAccumulator() {}
    virtual void accumulate_match(const size_t lineNum, char * line_start, char * line_end) = 0;
    virtual void finalize_match(char * buffer_end) {}  // default: no op
};

extern "C" void accumulate_match_wrapper(intptr_t accum_addr, const size_t lineNum, char * line_start, char * line_end);

extern "C" void finalize_match_wrapper(intptr_t accum_addr, char * buffer_end);


class GrepEngine {
    enum class FileStatus {Pending, GrepComplete, PrintComplete};
public:

    GrepEngine();
    virtual ~GrepEngine();
    
    void setPreferMMap() {mPreferMMap = true;}
    
    void showFileNames() {mShowFileNames = true;}
    void setStdinLabel(std::string lbl) {mStdinLabel = lbl;}
    void showLineNumbers() {mShowLineNumbers = true;}
    void setInitialTab() {mInitialTab = true;}

    void setMaxCount(int m) {mMaxCount = m;}
    void setGrepStdIn() {mGrepStdIn = true;}
    void setInvertMatches() {mInvertMatches = true;}
    void setCaseInsensitive()  {mCaseInsensitive = true;}

    void suppressFileMessages() {mSuppressFileMessages = true;}

    void setRecordBreak(GrepRecordBreakKind b);
    void initFileResult(std::vector<boost::filesystem::path> & filenames);
    void initREs(std::vector<re::RE *> & REs);
    virtual void grepCodeGen();
    bool searchAllFiles();
    void * DoGrepThreadMethod();

protected:
    std::pair<parabix::StreamSetBuffer *, parabix::StreamSetBuffer *> grepPipeline(parabix::StreamSetBuffer * ByteStream);

    virtual uint64_t doGrep(const std::string & fileName, std::ostringstream & strm);
    int32_t openFile(const std::string & fileName, std::ostringstream & msgstrm);

    enum class EngineKind {QuietMode, MatchOnly, CountOnly, EmitMatches};
    EngineKind mEngineKind;
    
    std::string linePrefix(std::string fileName);

    bool mSuppressFileMessages;
    bool mPreferMMap;
    bool mShowFileNames;
    std::string mStdinLabel;
    bool mShowLineNumbers;
    bool mInitialTab;
    bool mCaseInsensitive;
    bool mInvertMatches;
    int mMaxCount;
    bool mGrepStdIn;
    
    Driver * mGrepDriver;

    std::atomic<unsigned> mNextFileToGrep;
    std::atomic<unsigned> mNextFileToPrint;
    std::vector<boost::filesystem::path> inputPaths;
    std::vector<std::ostringstream> mResultStrs;
    std::vector<FileStatus> mFileStatus;
    bool grepMatchFound;
    GrepRecordBreakKind mGrepRecordBreak;
    
    std::vector<re:: RE *> mREs;
    std::set<re::Name *> mUnicodeProperties;
    re::CC * mBreakCC;
    std::unique_ptr<cc::MultiplexedAlphabet> mpx;
    std::string mFileSuffix;
    bool mMoveMatchesToEOL;
    pthread_t mEngineThread;
};


//
// The EmitMatches engine uses an EmitMatchesAccumulator object to concatenate together
// matched lines.

class EmitMatch : public MatchAccumulator {
    friend class EmitMatchesEngine;
public:
    EmitMatch(std::string linePrefix, bool showLineNumbers, bool initialTab, std::ostringstream & strm) : mLinePrefix(linePrefix),
        mShowLineNumbers(showLineNumbers),
        mInitialTab(initialTab),
        mLineCount(0),
        mTerminated(true),
        mResultStr(strm) {}
    void accumulate_match(const size_t lineNum, char * line_start, char * line_end) override;
    void finalize_match(char * buffer_end) override;
protected:
    std::string mLinePrefix;
    bool mShowLineNumbers;
    bool mInitialTab;
    size_t mLineCount;
    bool mTerminated;
    std::ostringstream & mResultStr;
};

class EmitMatchesEngine : public GrepEngine {
public:
    EmitMatchesEngine();
    void grepCodeGen() override;
private:
    uint64_t doGrep(const std::string & fileName, std::ostringstream & strm) override;
};

class CountOnlyEngine : public GrepEngine {
public:
    CountOnlyEngine();
private:
    uint64_t doGrep(const std::string & fileName, std::ostringstream & strm) override;
};

class MatchOnlyEngine : public GrepEngine {
public:
    MatchOnlyEngine(bool showFilesWithoutMatch, bool useNullSeparators);
private:
    uint64_t doGrep(const std::string & fileName, std::ostringstream & strm) override;
    unsigned mRequiredCount;
};

class QuietModeEngine : public GrepEngine {
public:
    QuietModeEngine();
};

    
    
class InternalSearchEngine {
public:
    InternalSearchEngine();
    ~InternalSearchEngine();
    
    void setRecordBreak(GrepRecordBreakKind b) {mGrepRecordBreak = b;}
    void setCaseInsensitive()  {mCaseInsensitive = true;}
    
    void grepCodeGen(re::RE * matchingRE, re::RE * invertedRE, MatchAccumulator * accum);
    
    void doGrep(const char * search_buffer, size_t bufferLength);
    
private:
    GrepRecordBreakKind mGrepRecordBreak;
    bool mCaseInsensitive;

    Driver * mGrepDriver;
};
    
    
#define MAX_SIMD_WIDTH_SUPPORTED 256
#define INITIAL_CAPACITY 64
    
class SearchableBuffer  {
public:
    SearchableBuffer();
    void addSearchCandidate(const char * string_ptr);
    size_t getCandidateCount() {return mEntries;}
    char * getBufferBase() {return mBuffer_base;}
    size_t getBufferSize() {return mSpace_used;}
    ~SearchableBuffer();
private:
    static const unsigned BUFFER_ALIGNMENT = MAX_SIMD_WIDTH_SUPPORTED/8;
    AlignedAllocator<char, BUFFER_ALIGNMENT> mAllocator;
    size_t mAllocated_capacity;
    size_t mSpace_used;
    size_t mEntries;
    char * mBuffer_base;
    alignas(BUFFER_ALIGNMENT) char mInitial_buffer[INITIAL_CAPACITY];
};

}

#endif
