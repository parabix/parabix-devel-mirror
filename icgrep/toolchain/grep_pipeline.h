/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#ifndef GREP_PIPELINE_H
#define GREP_PIPELINE_H

#include <stdlib.h>
#include <stdint.h>

namespace re { class RE; }

namespace grep {
    
class MatchAccumulator {
public:
    MatchAccumulator() {};
    virtual void accumulate_match(const size_t lineNum, char * line_start, char * line_end) = 0;
    virtual void finalize_match(char * buffer_end) {}  // default: no op
};

void accumulate_match_wrapper(intptr_t accum_addr, const size_t lineNum, char * line_start, char * line_end);

void finalize_match_wrapper(intptr_t accum_addr, char * buffer_end);
    
void grepBuffer(re::RE * pattern, const char * buffer, size_t bufferLength, MatchAccumulator * accum);

}

#endif
