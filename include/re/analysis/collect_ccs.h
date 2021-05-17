#ifndef COLLECT_CCS_H
#define COLLECT_CCS_H

#include <vector>
#include <set>
#include <re/analysis/re_inspector.h>

namespace cc { class Alphabet; }
namespace re {

class RE;
class CC;

std::vector<CC *> collectCCs(RE * const re, const cc::Alphabet & a,
                             re::NameProcessingMode m = re::NameProcessingMode::None);

}
#endif
