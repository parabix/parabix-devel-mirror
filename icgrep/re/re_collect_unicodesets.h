#ifndef RE_COLLECT_UNICODESETS_H
#define RE_COLLECT_UNICODESETS_H

#include <vector>

namespace re {

    class RE;
    class CC;

    std::vector<const CC *> collectUnicodeSets(RE * const re);

}
#endif
