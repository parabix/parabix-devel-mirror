#ifndef RE_MULTIPLEX_H
#define RE_MULTIPLEX_H

#include <UCD/ucd_compiler.hpp>
#include <cc/multiplex_CCs.h>
#include <re/re_utility.h>

namespace re {

    class RE;
    class Name;
    class CC;

    RE * transformCCs(cc::MultiplexedAlphabet * mpx, RE * r);

    
}
#endif
