#ifndef RE_MULTIPLEX_H
#define RE_MULTIPLEX_H

#include <UCD/ucd_compiler.hpp>

namespace re {

    class RE;
    class Name;
    class CC;

    RE * multiplex(RE * const re,
                   const std::vector<const CC *> & UnicodeSets,
                   const std::vector<std::vector<unsigned>> & exclusiveSetIDs);

}
#endif
