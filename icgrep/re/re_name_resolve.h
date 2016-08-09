#ifndef RE_NAME_RESOLVE_H
#define RE_NAME_RESOLVE_H

#include <re/re_name.h>
#include <re/re_re.h>
#include <re/re_name.h>
#include <UCD/ucd_compiler.hpp>
namespace re {
    UCD::UCDCompiler::NameMap resolveNames(RE * &re, Name * &zerowidth);
    Name * resolveOtherNames(RE * re);
    Name * generateGraphemeClusterBoundaryRule();
}
#endif
