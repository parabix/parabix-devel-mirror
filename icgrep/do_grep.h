#ifndef DO_GREP_H
#define DO_GREP_H
/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <string>
#include <stdint.h>
#include "basis_bits.h"
#include "include/simd-lib/bitblock.hpp"
#include "include/simd-lib/transpose.hpp"
#include "include/simd-lib/bitblock_iterator.hpp"
#include <re/re_cc.h>

struct Output {
    BitBlock matches;
    BitBlock LF;
};

#if (BLOCK_SIZE == 128)
#define SEGMENT_BLOCKS 7
#endif

#if (BLOCK_SIZE == 256)
#define SEGMENT_BLOCKS 15
#endif

#define SEGMENT_SIZE (BLOCK_SIZE * SEGMENT_BLOCKS)


#if (BLOCK_SIZE == 256)
typedef BitStreamScanner<BitBlock, uint64_t, uint64_t, SEGMENT_BLOCKS> ScannerT;
#endif

#if (BLOCK_SIZE == 128)
typedef BitStreamScanner<BitBlock, uint32_t, uint32_t, SEGMENT_BLOCKS> ScannerT;
#endif

typedef void (*process_block_fcn)(const Basis_bits & basis_bits, Output & output);

namespace llvm { class raw_ostream; }

class GrepExecutor {
public:

    GrepExecutor(void * process_block)
    : mCountOnlyOption(false)
    , mGetCodePointsOption(false)
    , mShowFileNameOption(false)
    , mShowLineNumberingOption(false)
    , mParsedCodePointSet(nullptr)
    , mProcessBlockFcn(reinterpret_cast<process_block_fcn>(process_block)) {

    }
          
    void setCountOnlyOption(bool doCount = true) {mCountOnlyOption = doCount;}
    void setParseCodepointsOption() {
        mGetCodePointsOption = true;
        mParsedCodePointSet = re::makeCC();
    }
    void setShowFileNameOption(bool showF = true) {mShowFileNameOption = showF;}
    void setShowLineNumberOption(bool showN = true) {mShowLineNumberingOption = showN;}
    void setNormalizeLineBreaksOption(bool normLB = true) {mNormalizeLineBreaksOption = normLB;}
    
    void doGrep(const std::string & fileName);
    re::CC * getParsedCodepoints() { return mParsedCodePointSet;}
private:
    ssize_t write_matches(llvm::raw_ostream & out, const char *buffer, ssize_t first_line_start);
    
    bool finalLineIsUnterminated() const;
    ssize_t extract_codepoints(char * buffer, ssize_t first_line_start);

    bool mCountOnlyOption;
    bool mGetCodePointsOption;
    bool mShowFileNameOption;
    bool mShowLineNumberingOption;
    bool mNormalizeLineBreaksOption;
    
    re::CC * mParsedCodePointSet;

    process_block_fcn mProcessBlockFcn;
    
    std::string mFileName;
    size_t mFileSize;
    char * mFileBuffer;
    ScannerT mLineBreak_scanner;
    ScannerT mMatch_scanner;
    size_t mLineNum;
};


#endif
