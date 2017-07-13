#ifndef RE_MULTIPLEX_H
#define RE_MULTIPLEX_H

#include <UCD/ucd_compiler.hpp>

namespace re {

    class RE;
    class Name;

    RE * multiplex(RE * re, std::vector<UCD::UnicodeSet> UnicodeSets,
                    std::vector<std::vector<unsigned>> exclusiveSetIDs,
                    std::vector<UCD::UnicodeSet> multiplexedCCs);

}
#endif
