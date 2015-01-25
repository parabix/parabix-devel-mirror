#ifndef RE_ANALYSIS_H
#define RE_ANALYSIS_H

//Regular Expressions
#include "re_seq.h"
#include <list>

namespace re {

bool isByteLength(RE * re);
    
bool isUnicodeUnitLength(RE * re);

int minMatchLength(RE * re);

}

#endif // RE_ANALYSIS_H
