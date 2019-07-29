
/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#ifndef GREP_ENGINE_H
#define GREP_ENGINE_H

#include "../../tools/icgrep/grep_interface.h" // TODO: consider moving grep_interface to this library
#include <string>
#include <vector>
#include <sstream>
#include <atomic>
#include <set>
#include <boost/filesystem.hpp>
#include <re/cc/multiplex_CCs.h>
#include <re/parse/GLOB_parser.h>
#include <kernel/core/callback.h>
#include <kernel/util/linebreak_kernel.h>
#include <grep/grep_kernel.h>

namespace re { class CC; }
namespace re { class RE; }
namespace llvm { namespace cl { class OptionCategory; } }
namespace kernel { class ProgramBuilder; }
namespace kernel { class StreamSet; }
class BaseDriver;


namespace grep {


extern unsigned ByteCClimit;

enum class GrepRecordBreakKind {Null, LF, Unicode};

class InternalSearchEngine;
class InternalMultiSearchEngine;

enum GrepSignal : unsigned {BinaryFile};

class GrepCallBackObject : public kernel::SignallingObject {
public:
    GrepCallBackObject() : SignallingObject(), mBinaryFile(false) {}
    virtual ~GrepCallBackObject() {}
    virtual void handle_signal(unsigned signal);
    bool binaryFileSignalled() {return mBinaryFile;}
private:
    bool mBinaryFile;
};

class MatchAccumulator : public GrepCallBackObject {
public:
    MatchAccumulator() {}
    virtual ~MatchAccumulator() {}
    virtual void accumulate_match(const size_t lineNum, char * line_start, char * line_end) = 0;
    virtual void finalize_match(char * buffer_end) {}  // default: no op
};

extern "C" void accumulate_match_wrapper(intptr_t accum_addr, const size_t lineNum, char * line_start, char * line_end);

extern "C" void finalize_match_wrapper(intptr_t accum_addr, char * buffer_end);

class GrepEngine {
    enum class FileStatus {Pending, GrepComplete, PrintComplete};
    friend class InternalSearchEngine;
    friend class InternalMultiSearchEngine;
public:

    enum class EngineKind {QuietMode, MatchOnly, CountOnly, EmitMatches};

    GrepEngine(BaseDriver & driver);

    virtual ~GrepEngine() = 0;

    void setPreferMMap(bool b = true) {mPreferMMap = b;}

    void showFileNames(bool b = true) {mShowFileNames = b;}
    void setStdinLabel(std::string lbl) {mStdinLabel = lbl;}
    void showLineNumbers(bool b = true) {mShowLineNumbers = b;}
    void setContextLines(unsigned before, unsigned after) {mBeforeContext = before; mAfterContext = after;}
    void setInitialTab(bool b = true) {mInitialTab = b;}

    void setMaxCount(int m) {mMaxCount = m;}
    void setGrepStdIn(bool b = true) {mGrepStdIn = b;}
    void setInvertMatches(bool b = true) {mInvertMatches = b;}
    void setCaseInsensitive(bool b = true)  {mCaseInsensitive = b;}

    void suppressFileMessages(bool b = true) {mSuppressFileMessages = b;}
    void setBinaryFilesOption(argv::BinaryFilesMode mode) {mBinaryFilesMode = mode;}
    void setRecordBreak(GrepRecordBreakKind b);
    void initFileResult(const std::vector<boost::filesystem::path> & filenames);
    void initREs(std::vector<re::RE *> & REs);
    virtual void grepCodeGen();
    bool searchAllFiles();
    void * DoGrepThreadMethod();
    virtual void showResult(uint64_t grepResult, const std::string & fileName, std::ostringstream & strm);

protected:
    // Functional components that may be required for grep searches,
    // depending on search pattern, mode flags, external parameters and
    // implementation strategy.
    typedef uint32_t component_t;
    enum class Component : component_t {
        NoComponents = 0,
        S2P = 0x01,
        UTF8index = 0x02,
        MoveMatchesToEOL = 0x04,
        GraphemeClusterBoundary = 0x08
    };
    bool hasComponent(Component compon_set, Component c);
    void setComponent(Component & compon_set, Component c);

    // Transpose to basis bit streams, if required otherwise return the source byte stream.
    kernel::StreamSet * getBasis(const std::unique_ptr<kernel::ProgramBuilder> &P, kernel::StreamSet * ByteStream);
    
    // Initial grep set-up.
    // Implement any required checking/processing of null characters, determine the
    // line break stream and the U8 index stream (if required).
    void grepPrologue(const std::unique_ptr<kernel::ProgramBuilder> &P, kernel::StreamSet * SourceStream);
    // Prepare external property and GCB streams, if required.
    void prepareExternalStreams(const std::unique_ptr<ProgramBuilder> & P, StreamSet * SourceStream);
    void addExternalStreams(const std::unique_ptr<ProgramBuilder> & P, std::unique_ptr<GrepKernelOptions> & options, re::RE * regexp);
    StreamSet * grepPipeline(const std::unique_ptr<kernel::ProgramBuilder> &P, kernel::StreamSet * ByteStream);
    virtual uint64_t doGrep(const std::string & fileName, std::ostringstream & strm);
    int32_t openFile(const std::string & fileName, std::ostringstream & msgstrm);

    std::string linePrefix(std::string fileName);

protected:

    EngineKind mEngineKind;
    bool mSuppressFileMessages;
    argv::BinaryFilesMode mBinaryFilesMode;
    bool mPreferMMap;
    bool mShowFileNames;
    std::string mStdinLabel;
    bool mShowLineNumbers;
    unsigned mBeforeContext;
    unsigned mAfterContext;
    bool mInitialTab;
    bool mCaseInsensitive;
    bool mInvertMatches;
    int mMaxCount;
    bool mGrepStdIn;
    NullCharMode mNullMode;
    BaseDriver & mGrepDriver;
    void * mMainMethod;

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
    re::RE * mPrefixRE;
    re::RE * mSuffixRE;
    std::string mFileSuffix;
    Component mExternalComponents;
    Component mInternalComponents;
    std::map<std::string, StreamSet *> mPropertyStreamMap;
    StreamSet * mLineBreakStream;
    StreamSet * mU8index;
    StreamSet * mGCB_stream;
    pthread_t mEngineThread;
};


//
// The EmitMatches engine uses an EmitMatchesAccumulator object to concatenate together
// matched lines.

class EmitMatch : public MatchAccumulator {
    friend class EmitMatchesEngine;
public:
    EmitMatch(std::string linePrefix, bool showLineNumbers, bool showContext, bool initialTab, std::ostringstream & strm)
        : mLinePrefix(linePrefix),
        mShowLineNumbers(showLineNumbers),
        mContextGroups(showContext),
        mInitialTab(initialTab),
        mLineCount(0),
        mLineNum(0),
        mTerminated(true),
        mResultStr(strm) {}
    void accumulate_match(const size_t lineNum, char * line_start, char * line_end) override;
    void finalize_match(char * buffer_end) override;
protected:
    std::string mLinePrefix;
    bool mShowLineNumbers;
    bool mContextGroups;
    bool mInitialTab;
    size_t mLineCount;
    size_t mLineNum;
    bool mTerminated;
    std::ostringstream & mResultStr;
};

class EmitMatchesEngine final : public GrepEngine {
public:
    EmitMatchesEngine(BaseDriver & driver);
    void grepCodeGen() override;
private:
    uint64_t doGrep(const std::string & fileName, std::ostringstream & strm) override;
};

class CountOnlyEngine final : public GrepEngine {
public:
    CountOnlyEngine(BaseDriver & driver);
private:
    void showResult(uint64_t grepResult, const std::string & fileName, std::ostringstream & strm) override;
};

class MatchOnlyEngine final : public GrepEngine {
public:
    MatchOnlyEngine(BaseDriver & driver, bool showFilesWithoutMatch, bool useNullSeparators);
private:
    void showResult(uint64_t grepResult, const std::string & fileName, std::ostringstream & strm) override;
    unsigned mRequiredCount;
};

class QuietModeEngine final : public GrepEngine {
public:
    QuietModeEngine(BaseDriver & driver);
};



class InternalSearchEngine {
public:
    InternalSearchEngine(BaseDriver & driver);

    InternalSearchEngine(const std::unique_ptr<grep::GrepEngine> & engine);

    ~InternalSearchEngine();

    void setRecordBreak(GrepRecordBreakKind b) {mGrepRecordBreak = b;}
    void setCaseInsensitive()  {mCaseInsensitive = true;}

    void grepCodeGen(re::RE * matchingRE);

    void doGrep(const char * search_buffer, size_t bufferLength, MatchAccumulator & accum);

private:
    GrepRecordBreakKind mGrepRecordBreak;
    bool mCaseInsensitive;
    BaseDriver & mGrepDriver;
    void * mMainMethod;
    unsigned mNumOfThreads;
};

enum class PatternKind {Include, Exclude};
class InternalMultiSearchEngine {
public:
    InternalMultiSearchEngine(BaseDriver & driver);

    InternalMultiSearchEngine(const std::unique_ptr<grep::GrepEngine> & engine);

    ~InternalMultiSearchEngine();

    void setRecordBreak(GrepRecordBreakKind b) {mGrepRecordBreak = b;}
    void setCaseInsensitive()  {mCaseInsensitive = true;}

    void grepCodeGen(std::vector<std::pair<re::PatternKind, re::RE *>> REs);

    void doGrep(const char * search_buffer, size_t bufferLength, MatchAccumulator & accum);

private:
    GrepRecordBreakKind mGrepRecordBreak;
    bool mCaseInsensitive;
    BaseDriver & mGrepDriver;
    void * mMainMethod;
    unsigned mNumOfThreads;
};

}

#endif
