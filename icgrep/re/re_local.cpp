#include "re_local.h"
#include <re/re_name.h>
#include <re/re_alt.h>
#include <re/re_cc.h>
#include <re/re_seq.h>
#include <re/re_rep.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_assertion.h>
#include <re/re_any.h>
#include <re/re_analysis.h>
#include <UCD/resolve_properties.h>
#include <boost/container/flat_set.hpp>
#include <boost/range/adaptor/reversed.hpp>
#include <map>

using namespace boost::container;
using namespace llvm;

namespace re {
  
inline void combine(CC *& a, CC * b) {
    if (a && b) {
        a = makeCC(a, b);
    } else if (b) {
        a = b;
    }
}

CC * RE_Local::first(RE * re) {
    if (Name * name = dyn_cast<Name>(re)) {
        if (LLVM_LIKELY(name->getDefinition() != nullptr)) {
            return first(name->getDefinition());
        } else {
            throw std::runtime_error("All non-unicode-property Name objects should have been defined prior to Unicode property resolution.");
        }
    } else if (CC * cc = dyn_cast<CC>(re)) {
        return cc;
    } else if (Seq * seq = dyn_cast<Seq>(re)) {
        CC * cc = nullptr;
        for (auto & si : *seq) {
            combine(cc, first(si));
            if (!isNullable(si)) {
                break;
            }
        }
        return cc;
    } else if (Alt * alt = dyn_cast<Alt>(re)) {
        CC * cc = nullptr;
        for (auto & ai : *alt) {
            combine(cc, first(ai));
        }
        return cc;
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
        return first(rep->getRE());
    } else if (Diff * diff = dyn_cast<Diff>(re)) {
        if (CC * lh = first(diff->getLH())) {
            if (CC * rh = first(diff->getRH())) {
                return subtractCC(lh, rh);
            }
        }
    } else if (Intersect * ix = dyn_cast<Intersect>(re)) {
        if (CC * lh = first(ix->getLH())) {
            if (CC * rh = first(ix->getRH())) {
                return intersectCC(lh, rh);
            }
        }
    }
    return nullptr;
}

CC * RE_Local::final(RE * re) {
    if (Name * name = dyn_cast<Name>(re)) {
        if (LLVM_LIKELY(name->getDefinition() != nullptr)) {
            return final(name->getDefinition());
        } else {
            throw std::runtime_error("All non-unicode-property Name objects should have been defined prior to Unicode property resolution.");
        }
    } else if (CC * cc = dyn_cast<CC>(re)) {
        return cc;
    } else if (Seq * seq = dyn_cast<Seq>(re)) {
        CC * cc = nullptr;
        for (auto & si : boost::adaptors::reverse(*seq)) {
            combine(cc, first(si));
            if (!isNullable(si)) {
                break;
            }
        }
        return cc;
    } else if (Alt * alt = dyn_cast<Alt>(re)) {
        CC * cc = nullptr;
        for (auto & ai : *alt) {
            combine(cc, final(ai));
        }
        return cc;
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
        return final(rep->getRE());
    } else if (Diff * diff = dyn_cast<Diff>(re)) {
        if (CC * lh = final(diff->getLH())) {
            if (CC * rh = final(diff->getRH())) {
                return subtractCC(lh, rh);
            }
        }
    } else if (Intersect * ix = dyn_cast<Intersect>(re)) {
        if (CC * lh = final(ix->getLH())) {
            if (CC * rh = final(ix->getRH())) {
                return intersectCC(lh, rh);
            }
        }
    }
    return nullptr;

}

void RE_Local::follow(RE * re, std::map<CC *, CC*> &follow_map) {
    if (Name * name = dyn_cast<Name>(re)) {
        if (LLVM_LIKELY(name->getDefinition() != nullptr)) {
            return follow(name->getDefinition(), follow_map);
        } else {
            throw std::runtime_error("All non-unicode-property Name objects should have been defined prior to Unicode property resolution.");
        }
    } else if (Seq * seq = dyn_cast<Seq>(re)) {
        RE * re_first = *(seq->begin());
        RE * re_follow = makeSeq(seq->begin() + 1, seq->end());
        auto e1 = final(re_first);
        auto e2 = first(re_follow);
        if (e1 && e2) {
            auto e = follow_map.find(e1);
            if (e != follow_map.end()) {
                e->second = makeCC(e->second, e2);
            } else {
                follow_map.emplace(e1, e2);
            }
        }
        follow(re_first, follow_map);
        follow(re_follow, follow_map);
    } else if (Alt * alt = dyn_cast<Alt>(re)) {
        for (auto ai = alt->begin(); ai != alt->end(); ++ai) {
            follow(*ai, follow_map);
        }
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
        auto e1 = final(rep->getRE());
        auto e2 = first(rep->getRE());
        if (e1 && e2) {
            auto e = follow_map.find(e1);
            if (e != follow_map.end()) {
                e->second = makeCC(e->second, e2);
            } else {
                follow_map.emplace(e1, e2);
            }
        }
        follow(rep->getRE(), follow_map);
    }
}

bool RE_Local::isLocalLanguage(RE * re) {
    std::vector<UCD::UnicodeSet> UnicodeSets;
    collect_UnicodeSets_helper(re, UnicodeSets);
    if (UnicodeSets.size() == 0) return false;
    for (unsigned i = 0; i < UnicodeSets.size(); i++) {
        for (unsigned j = i + 1; j < UnicodeSets.size(); j++) {
            for (const UCD::UnicodeSet::interval_t & range : UnicodeSets[i]) {
                auto lo = re::lo_codepoint(range);
                auto hi = re::hi_codepoint(range);
                if (UnicodeSets[j].intersects(lo, hi)) {
                    return false;
                }
            }
        }
    }
    return true;
}


RE * RE_Local::collect_UnicodeSets_helper(RE * re, std::vector<UCD::UnicodeSet> & UnicodeSets) {
    assert ("RE object cannot be null!" && re);
    if (isa<Name>(re)) {
        if (CC * cc = dyn_cast<CC>(cast<Name>(re)->getDefinition())) {
            UnicodeSets.push_back(* cast<UCD::UnicodeSet>(cc));
        } else {
            collect_UnicodeSets_helper(cast<Name>(re)->getDefinition(), UnicodeSets);
        }
    } else if (isa<Seq>(re)) {
        for (RE * item : *cast<Seq>(re)) {
            collect_UnicodeSets_helper(item, UnicodeSets);
        }
    } else if (isa<Alt>(re)) {
        for (RE * item : *cast<Alt>(re)) {
            collect_UnicodeSets_helper(item, UnicodeSets);
        }
    } else if (isa<Rep>(re)) {
        collect_UnicodeSets_helper(cast<Rep>(re)->getRE(), UnicodeSets);
    } else if (isa<Assertion>(re)) {
        collect_UnicodeSets_helper(cast<Assertion>(re)->getAsserted(), UnicodeSets);
    } else if (isa<Diff>(re)) {
        collect_UnicodeSets_helper(cast<Diff>(re)->getLH(), UnicodeSets);
        collect_UnicodeSets_helper(cast<Diff>(re)->getRH(), UnicodeSets);
    } else if (isa<Intersect>(re)) {
        collect_UnicodeSets_helper(cast<Intersect>(re)->getLH(), UnicodeSets);
        collect_UnicodeSets_helper(cast<Intersect>(re)->getRH(), UnicodeSets);
    } else if (isa<Any>(re)) {
        UnicodeSets.push_back(UCD::UnicodeSet(0x00, 0x10FFFF));
    }
    return re;
}

bool RE_Local::isNullable(const RE * re) {
    if (const Seq * re_seq = dyn_cast<const Seq>(re)) {
        for (const RE * re : *re_seq) {
            if (!isNullable(re)) {
                return false;
            }
        }
        return true;
    } else if (const Alt * re_alt = dyn_cast<const Alt>(re)) {
        for (const RE * re : *re_alt) {
            if (isNullable(re)) {
                return true;
            }
        }
    } else if (const Rep* re_rep = dyn_cast<const Rep>(re)) {
        return re_rep->getLB() == 0 ? true : isNullable(re_rep->getRE());
    } else if (const Diff * diff = dyn_cast<const Diff>(re)) {
        return isNullable(diff->getLH()) && !isNullable(diff->getRH());
    } else if (const Intersect * e = dyn_cast<const Intersect>(re)) {
        return isNullable(e->getLH()) && isNullable(e->getRH());
    } 
    return false;
}

}
