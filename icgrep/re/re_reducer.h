#ifndef RE_REDUCER_H
#define RE_REDUCER_H

//Regular Expressions
#include "re_re.h"
#include <algorithm>
#include <list>
#include <map>

namespace re {

typedef std::map<std::string, RE*> RENameMap;

class RE_Reducer
{
public:
    static RE* reduce(RE* re, RENameMap & re_map);
};

}

#endif // RE_REDUCER_H
