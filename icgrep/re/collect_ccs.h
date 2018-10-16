#ifndef COLLECT_CCS_H
#define COLLECT_CCS_H

#include <vector>
#include <set>

namespace cc { class Alphabet; }
namespace re {

class RE;
class CC;
class Name;

std::vector<CC *> collectCCs(RE * const re, const cc::Alphabet * a, std::set<Name *> external = {});

}
#endif
