
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
    virtual unsigned getFileCount() {return 1;}  // default: return 1 for single file
    virtual size_t getFileStartPos(unsigned fileNo) {return 0;}
    virtual void setBatchLineNumber(unsigned fileNo, size_t batchLine) {}  // default: no op
};

extern "C" void accumulate_match_wrapper(intptr_t accum_addr, const size_t lineNum, char * line_start, char * line_end);

extern "C" void finalize_match_wrapper(intptr_t accum_addr, char * buffer_end);

extern "C" unsigned get_file_count_wrapper(intptr_t accum_addr);

extern "C" size_t get_file_start_pos_wrapper(intptr_t accum_addr, unsigned fileNo);

extern "C" void set_batch_line_number_wrapper(intptr_t accum_addr, unsigned fileNo, size_t batchLine);


class GrepEngine {
    enum class FileStatus {Pending, GrepComplete, PrintComplete};
    friend class InternalSearchEngine;
    friend class InternalMultiSearchEngine;
public:

    enum class EngineKind {QuietMode, MatchOnly, CountOnly, EmitMatches};

    GrepEngine(BaseDriver & driver);

    virtual ~GrepEngine() = 0;

    void setPreferMMap(bool b = true) {mPreferMMap = b;}

    void setColoring(bool b = true)  {mColoring = b;}
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
    bool haveFileBatch();
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
        S2P = 1,
        UTF8index = 2,
        MoveMatchesToEOL = 4,
        MatchStarts = 8,
        GraphemeClusterBoundary = 16,
        WordBoundary = 32
    };
    bool hasComponent(Component compon_set, Component c);
    void setComponent(Component & compon_set, Component c);
    bool matchesToEOLrequired();

    // Transpose to basis bit streams, if required otherwise return the source byte stream.
    kernel::StreamSet * getBasis(const std::unique_ptr<kernel::ProgramBuilder> &P, kernel::StreamSet * ByteStream);

    // Initial grep set-up.
    // Implement any required checking/processing of null characters, determine the
    // line break stream and the U8 index stream (if required).
    void grepPrologue(const std::unique_ptr<kernel::ProgramBuilder> &P, kernel::StreamSet * SourceStream);
    // Prepare external property and GCB streams, if required.
    void prepareExternalStreams(const std::unique_ptr<kernel::ProgramBuilder> & P, kernel::StreamSet * SourceStream);
    void addExternalStreams(const std::unique_ptr<kernel::ProgramBuilder> & P, std::unique_ptr<kernel::GrepKernelOptions> & options, re::RE * regexp, kernel::StreamSet * indexMask = nullptr);
    void U8indexedGrep(const std::unique_ptr<kernel::ProgramBuilder> &P, re::RE * re, kernel::StreamSet * Source, kernel::StreamSet * Results);
    void UnicodeIndexedGrep(const std::unique_ptr<kernel::ProgramBuilder> &P, re::RE * re, kernel::StreamSet * Source, kernel::StreamSet * Results);
    kernel::StreamSet * grepPipeline(const std::unique_ptr<kernel::ProgramBuilder> &P, kernel::StreamSet * ByteStream);
    virtual uint64_t doGrep(const std::vector<std::string> & fileNames, std::ostringstream & strm);
    int32_t openFile(const std::string & fileName, std::ostringstream & msgstrm);

    std::string linePrefix(std::string fileName);

protected:

    EngineKind mEngineKind;
    bool mSuppressFileMessages;
    argv::BinaryFilesMode mBinaryFilesMode;
    bool mPreferMMap;
    bool mColoring;
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
    void * mBatchMethod;

    std::atomic<unsigned> mNextFileToGrep;
    std::atomic<unsigned> mNextFileToPrint;
    std::vector<boost::filesystem::path> mInputPaths;
    std::vector<std::vector<std::string>> mFileGroups;
    std::vector<std::ostringstream> mResultStrs;
    std::vector<FileStatus> mFileStatus;
    bool grepMatchFound;
    GrepRecordBreakKind mGrepRecordBreak;

    std::vector<re:: RE *> mREs;
    std::set<re::Name *> mExternalNames;
    re::CC * mBreakCC;
    re::RE * mPrefixRE;
    re::RE * mSuffixRE;
    std::string mFileSuffix;
    Component mExternalComponents;
    Component mInternalComponents;
    std::map<std::string, kernel::StreamSet *> mPropertyStreamMap;
    kernel::StreamSet * mLineBreakStream;
    kernel::StreamSet * mU8index;
    kernel::StreamSet * mGCB_stream;
    kernel::StreamSet * mWordBoundary_stream;
    re::UTF8_Transformer mUTF8_Transformer;
    pthread_t mEngineThread;
};


//
// The EmitMatches engine uses an EmitMatchesAccumulator object to concatenate together
// matched lines.

class EmitMatch : public MatchAccumulator {
    friend class EmitMatchesEngine;
public:
    EmitMatch(bool showFileNames, bool showLineNumbers, bool showContext, bool initialTab)
        : mShowFileNames(showFileNames),
        mShowLineNumbers(showLineNumbers),
        mContextGroups(showContext),
        mInitialTab(initialTab),
        mCurrentFile(0),
        mLineCount(0),
        mLineNum(0),
        mTerminated(true) {}
    void prepareBatch (const std::vector<std::string> & fileNames);
    void accumulate_match(const size_t lineNum, char * line_start, char * line_end) override;
    void finalize_match(char * buffer_end) override;
    void setFileLabel(std::string fileLabel);
    void setStringStream(std::ostringstream * s);
    unsigned getFileCount() override;
    size_t getFileStartPos(unsigned fileNo) override;
    void setBatchLineNumber(unsigned fileNo, size_t batchLine) override;
protected:
    bool mShowFileNames;
    bool mShowLineNumbers;
    bool mContextGroups;
    bool mInitialTab;
    unsigned mCurrentFile;
    size_t mLineCount;
    size_t mLineNum;
    bool mTerminated;
    // An EmitMatch object may be defined to work with a single buffer for a
    // batch of files concatenated together.  The following vectors hold information
    // for each file in the batch, namely, its name, its starting code unit
    // position in the batch and its starting line number within the batch.
    std::vector<std::string> mFileNames;
    std::vector<size_t> mFileStartPositions;
    std::vector<size_t> mFileStartLineNumbers;
    std::string mLinePrefix;
    std::ostringstream * mResultStr;
    char * mBatchBuffer;
};

class EmitMatchesEngine final : public GrepEngine {
public:
    EmitMatchesEngine(BaseDriver & driver);
    void grepPipeline(const std::unique_ptr<kernel::ProgramBuilder> &P, kernel::StreamSet * ByteStream, bool BatchMode = false);
    void grepCodeGen() override;
private:
    uint64_t doGrep(const std::vector<std::string> & fileNames, std::ostringstream & strm) override;
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

    void grepCodeGen(const re::PatternVector & patterns);

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
