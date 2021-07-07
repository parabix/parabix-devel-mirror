#include "grep_engine.h"

namespace grep {

class NestedInternalSearchEngine {
public:
    using MainMethodTy = void *;

    NestedInternalSearchEngine(BaseDriver & driver);

    ~NestedInternalSearchEngine();

    void setRecordBreak(GrepRecordBreakKind b) {mGrepRecordBreak = b;}

    void setCaseInsensitive()  {mCaseInsensitive = true;}

    void init();

    void push(const re::PatternVector & REs);

    void pop();

    void grepCodeGen();

    void doGrep(const char * search_buffer, size_t bufferLength, MatchAccumulator & accum);

    void setNumOfThreads(unsigned numOfThreads) {
        mNumOfThreads = numOfThreads;
    }

private:
    GrepRecordBreakKind mGrepRecordBreak;
    bool mCaseInsensitive;
    BaseDriver & mGrepDriver;
    unsigned mNumOfThreads;
    re::CC * mBreakCC;
    kernel::StreamSet * mBasisBits;
    kernel::StreamSet * mU8index;
    kernel::StreamSet * mBreaks;
    kernel::StreamSet * mMatches;
    std::vector<void *>             mMainMethod;
    std::vector<kernel::Kernel *>   mNested;

};


}
