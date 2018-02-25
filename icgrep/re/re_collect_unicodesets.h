#ifndef RE_COLLECT_UNICODESETS_H
#define RE_COLLECT_UNICODESETS_H

#include <vector>
#include <set>

namespace re {

    class RE;
    class CC;
    class Name;

    std::vector<const CC *> collectUnicodeSets(RE * const re, std::set<Name *> external = {});

}
#endif
