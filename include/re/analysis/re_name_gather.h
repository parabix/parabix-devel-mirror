#ifndef RE_NAME_GATHER_H
#define RE_NAME_GATHER_H

#include <string>
#include <set>

namespace re {

    class RE; class Name;

    void gatherNames(RE * const re, std::set<Name *> & mNameSet);

}
#endif
