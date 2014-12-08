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

struct Output {
    BitBlock matches;
    BitBlock LF;
};

typedef void (*process_block_fcn)(const Basis_bits &basis_bits, BitBlock carry_q[], BitBlock advance_q[], Output &output);



class GrepExecutor {
public:
    GrepExecutor(int carry_count, int advance_count, process_block_fcn process_block): 
    mCarries(carry_count), mAdvances(advance_count), 
    mCountOnlyOption(false), mShowFileNameOption(false), mShowLineNumberingOption(false),
    mProcessBlockFcn(process_block)
    {}
    
    void setCountOnlyOption(bool doCount = true) {mCountOnlyOption = doCount;}
    void setShowFileNameOption(bool showF = true) {mShowFileNameOption = showF;}
    void setShowLineNumberOption(bool showN = true) {mShowLineNumberingOption = showN;}
    
    void doGrep(char * fileName);
private:
    bool mCountOnlyOption;
    bool mShowFileNameOption;
    bool mShowLineNumberingOption;
    int mCarries;
    int mAdvances;
    process_block_fcn mProcessBlockFcn;
    
};


#endif
