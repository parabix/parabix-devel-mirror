#ifndef RE_ANALYSIS_H
#define RE_ANALYSIS_H

//Regular Expressions
#include "re_seq.h"
#include <list>

namespace re {

bool isByteLength(const RE *re);
    
bool isUnicodeUnitLength(const RE * re);

std::pair<int, int> getUnicodeUnitLengthRange(const RE * re);

int minMatchLength(RE * re);

}

#endif // RE_ANALYSIS_H
