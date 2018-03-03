#ifndef RE_NAME_GATHER_H
#define RE_NAME_GATHER_H

#include <string>
#include <set>

namespace re {

    class RE; class Name;

    void gatherUnicodeProperties (RE * r, std::set<Name *> & nameSet);

}
#endif
