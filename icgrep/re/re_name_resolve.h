#ifndef RE_NAME_RESOLVE_H
#define RE_NAME_RESOLVE_H

#include <UCD/ucd_compiler.hpp>

namespace re {

    class RE;
    class Name;

    UCD::UCDCompiler::NameMap resolveNames(RE * &re, Name * &zerowidth);

}
#endif
