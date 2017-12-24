#ifndef RE_NAME_GATHER_H
#define RE_NAME_GATHER_H

#include <UCD/ucd_compiler.hpp>

namespace re {

    class RE;
    class Name;

    UCD::UCDCompiler::NameMap gatherNames(RE * &re);

}
#endif
