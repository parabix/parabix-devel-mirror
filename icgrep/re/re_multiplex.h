#ifndef RE_MULTIPLEX_H
#define RE_MULTIPLEX_H

#include <UCD/ucd_compiler.hpp>

namespace re {

    class RE;
    class Name;

    RE * multiplex(RE * const re, const std::vector<UCD::UnicodeSet> & UnicodeSets,
                    const std::vector<std::vector<unsigned>> & exclusiveSetIDs,
                    const std::vector<UCD::UnicodeSet> & multiplexedCCs);

}
#endif
