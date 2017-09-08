#ifndef RE_COLLECT_UNICODESETS_H
#define RE_COLLECT_UNICODESETS_H

#include <UCD/ucd_compiler.hpp>

namespace re {

    class RE;
    class Name;

    std::vector<UCD::UnicodeSet> collect_UnicodeSets(RE * re);

}
#endif
