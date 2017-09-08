#include "re_collect_unicodesets.h"
#include <re/re_name.h>
#include <re/re_any.h>
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_alt.h>
#include <re/re_cc.h>
#include <re/re_seq.h>
#include <re/re_rep.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_assertion.h>
#include <re/re_analysis.h>
#include <re/re_memoizer.hpp>
#include <UCD/ucd_compiler.hpp>
#include <UCD/resolve_properties.h>
#include <boost/container/flat_set.hpp>
#include <cc/multiplex_CCs.h>
#include <sstream>
#include <iostream>

using namespace boost::container;
using namespace llvm;

namespace re {
    
class SetCollector {
public:
    void collect_UnicodeSets(RE * re, std::vector<UCD::UnicodeSet> & UnicodeSets);
private:
    flat_set<Name *>        mVisited;
};

void SetCollector::collect_UnicodeSets(RE * re, std::vector<UCD::UnicodeSet> & UnicodeSets) {
    assert ("RE object cannot be null!" && re);
    if (CC * cc = dyn_cast<CC>(re)) {
        UnicodeSets.push_back(* cast<UCD::UnicodeSet>(cc));
    } else if (isa<Name>(re)) {
        if (mVisited.insert(cast<Name>(re)).second) {
            collect_UnicodeSets(cast<Name>(re)->getDefinition(), UnicodeSets);
        }
    } else if (isa<Seq>(re)) {
        for (RE * item : *cast<Seq>(re)) {
            collect_UnicodeSets(item, UnicodeSets);
        }
    } else if (isa<Alt>(re)) {
        for (RE * item : *cast<Alt>(re)) {
            collect_UnicodeSets(item, UnicodeSets);
        }
    } else if (isa<Rep>(re)) {
        collect_UnicodeSets(cast<Rep>(re)->getRE(), UnicodeSets);
    } else if (isa<Assertion>(re)) {
        collect_UnicodeSets(cast<Assertion>(re)->getAsserted(), UnicodeSets);
    } else if (isa<Diff>(re)) {
        collect_UnicodeSets(cast<Diff>(re)->getLH(), UnicodeSets);
        collect_UnicodeSets(cast<Diff>(re)->getRH(), UnicodeSets);
    } else if (isa<Intersect>(re)) {
        collect_UnicodeSets(cast<Intersect>(re)->getLH(), UnicodeSets);
        collect_UnicodeSets(cast<Intersect>(re)->getRH(), UnicodeSets);
    } else if (isa<Any>(re)) {
        UnicodeSets.push_back(UCD::UnicodeSet(0x00, 0x10FFFF));
    } else if (isa<Start>(re)) {
        // LineBreak set handled globally
    } else if (isa<End>(re)) {
        // LineBreak set handled globally
    }
}
    
std::vector<UCD::UnicodeSet> collect_UnicodeSets(RE * re) {
    SetCollector collector;
    std::vector<UCD::UnicodeSet> UnicodeSets;
    collector.collect_UnicodeSets(re, UnicodeSets);
    return UnicodeSets;
}

}
