#include "re_collect_unicodesets.h"
#include <re/re_name.h>
#include <re/re_any.h>
#include <re/re_alt.h>
#include <re/re_cc.h>
#include <re/re_seq.h>
#include <re/re_rep.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_assertion.h>
#include <boost/container/flat_set.hpp>


using namespace llvm;

namespace re {
    
struct SetCollector {
    void collect(RE * const re);
public:
    std::vector<const CC *> UnicodeSets;
    boost::container::flat_set<const RE *>  Visited;
};

void SetCollector::collect(RE * const re) {
    assert ("RE object cannot be null!" && re);
    if (Visited.insert(re).second) {
        if (CC * cc = dyn_cast<CC>(re)) {
            UnicodeSets.push_back(cc);
        } else if (isa<Name>(re)) {
            collect(cast<Name>(re)->getDefinition());
        } else if (isa<Seq>(re)) {
            for (auto item : *cast<Seq>(re)) {
                collect(item);
            }
        } else if (isa<Alt>(re)) {
            for (auto item : *cast<Alt>(re)) {
                collect(item);
            }
        } else if (isa<Rep>(re)) {
            collect(cast<Rep>(re)->getRE());
        } else if (isa<Assertion>(re)) {
            collect(cast<Assertion>(re)->getAsserted());
        } else if (isa<Diff>(re)) {
            collect(cast<Diff>(re)->getLH());
            collect(cast<Diff>(re)->getRH());
        } else if (isa<Intersect>(re)) {
            collect(cast<Intersect>(re)->getLH());
            collect(cast<Intersect>(re)->getRH());
        } else if (isa<Any>(re)) {
            UnicodeSets.push_back(makeCC(0x00, 0x10FFFF));
        }
    }
}

std::vector<const CC *> collectUnicodeSets(RE * const re) {
    SetCollector collector;
    collector.collect(re);
    return collector.UnicodeSets;
}



}
