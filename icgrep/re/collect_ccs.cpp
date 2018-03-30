#include "collect_ccs.h"
#include <cc/alphabet.h>
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
#include <re/re_memoizer.hpp>

#include <boost/container/flat_set.hpp>
#include <llvm/Support/raw_ostream.h>

using namespace llvm;

namespace re {
    
struct SetCollector : private Memoizer {
    void collect(RE * const re);
public:
    const cc::Alphabet * alphabet;
    std::vector<CC *> theSets;
    std::set<Name *> ignoredExternals;
};

void SetCollector::collect(RE * const re) {
    assert ("RE object cannot be null!" && re);
    if (CC * cc = dyn_cast<CC>(re)) {
        if (cc->getAlphabet() == alphabet) {
            if (find(cc) == end()) {
                cc = memoize(cc);
                theSets.push_back(cc);
            }
        }
    } else if (isa<Name>(re)) {
        if (ignoredExternals.find(cast<Name>(re)) != ignoredExternals.end()) return;
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

std::vector<CC *> collectCCs(RE * const re, const cc::Alphabet * a, std::set<Name *> external) {
    SetCollector collector;
    collector.alphabet = a;
    collector.ignoredExternals = external;
    collector.collect(re);
    return collector.theSets;
}



}
