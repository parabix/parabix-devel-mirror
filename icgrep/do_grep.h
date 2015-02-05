#ifndef DO_GREP_H
#define DO_GREP_H
/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <stdint.h>

#include "include/simd-lib/bitblock.hpp"
#include "include/simd-lib/transpose.hpp"
#include "include/simd-lib/bitblock_iterator.hpp"

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


typedef void (*process_block_fcn)(const Basis_bits &basis_bits, BitBlock carry_q[], BitBlock advance_q[], Output &output);



class GrepExecutor {
public:
    GrepExecutor(int carry_count, int advance_count, process_block_fcn process_block): 
    mCountOnlyOption(false), mShowFileNameOption(false), mShowLineNumberingOption(false),
    mCarries(carry_count), mAdvances(advance_count),
    mProcessBlockFcn(process_block)
    {}
          
    void setCountOnlyOption(bool doCount = true) {mCountOnlyOption = doCount;}
    void setShowFileNameOption(bool showF = true) {mShowFileNameOption = showF;}
    void setShowLineNumberOption(bool showN = true) {mShowLineNumberingOption = showN;}
    void setNormalizeLineBreaksOption(bool normLB = true) {mNormalizeLineBreaksOption = normLB;}
    
    void doGrep(std::string fileName);
private:
    ssize_t write_matches(char * buffer, ssize_t first_line_start);

    bool mCountOnlyOption;
    bool mShowFileNameOption;
    bool mShowLineNumberingOption;
    bool mNormalizeLineBreaksOption;
    int mCarries;
    int mAdvances;
    process_block_fcn mProcessBlockFcn;
    
    std::string currentFileName;
    size_t currentFileSize;
    ScannerT LF_scanner;
    ScannerT match_scanner;
    long line_no;
};


#endif
