#include "re_collect_unicodesets.h"
#include <re/re_name.h>
#include <re/re_any.h>
#include <re/re_alt.h>
#include <re/re_cc.h>
#include <re/re_seq.h>
#include <re/re_rep.h>
#include <re/re_range.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_assertion.h>
#include <cc/alphabet.h>
#include <boost/container/flat_set.hpp>
#include <llvm/Support/raw_ostream.h>

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
            if (cc->getAlphabet() == &cc::Unicode) {
                UnicodeSets.push_back(cc);
            }
        } else if (isa<Name>(re)) {
            auto def = cast<Name>(re)->getDefinition();
            if (def != nullptr)
                collect(def);
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
        }
    }
}

std::vector<const CC *> collectUnicodeSets(RE * const re) {
    SetCollector collector;
    collector.collect(re);
    return collector.UnicodeSets;
}



}
