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
#include <map>

using namespace boost::container;
using namespace llvm;

namespace re {
  
UCD::UnicodeSet* RE_Local::first(RE * re) {
    if (Name * name = dyn_cast<Name>(re)) {
        if (LLVM_LIKELY(name->getDefinition() != nullptr)) {
            if (CC * cc = dyn_cast<CC>(name->getDefinition())) {
                UCD::UnicodeSet * sets = cast<UCD::UnicodeSet>(cc);
                return sets;
            } else {
                return first(name->getDefinition());
            }
        } else {
            throw std::runtime_error("All non-unicode-property Name objects should have been defined prior to Unicode property resolution.");
        }
    } else if (CC * cc = dyn_cast<CC>(re)) {
        UCD::UnicodeSet * sets = cast<UCD::UnicodeSet>(cc);
        return sets;
    } else if (Seq * seq = dyn_cast<Seq>(re)) {
        UCD::UnicodeSet * UnicodeSets = new UCD::UnicodeSet();
        UCD::UnicodeSet UnicodeSets_seq = UCD::UnicodeSet();
        for (auto si = seq->begin(); si != seq->end(); ++si) {
            if (isNullable(*si) && first(*si) != nullptr) {
                UnicodeSets_seq = UnicodeSets_seq + *(first(*si));
            } else if (isNullable(*si) && first(*si) == nullptr) {
                continue;
            } else if (!isNullable(*si) && first(*si) != nullptr){
                UnicodeSets_seq = UnicodeSets_seq + *(first(*si));
                break;
            } else {
                break;
            }
        }
        *UnicodeSets = UnicodeSets_seq;
        return UnicodeSets_seq.empty() ? nullptr : UnicodeSets;
    } else if (Alt * alt = dyn_cast<Alt>(re)) {
        UCD::UnicodeSet * UnicodeSets = new UCD::UnicodeSet();
        UCD::UnicodeSet UnicodeSets_alt = UCD::UnicodeSet();
        for (auto ai = alt->begin(); ai != alt->end(); ++ai) {
            if (first(*ai) != nullptr) {
                UnicodeSets_alt = UnicodeSets_alt + *(first(*ai));
            }
        }
        *UnicodeSets = UnicodeSets_alt;
        return UnicodeSets_alt.empty() ? nullptr : UnicodeSets;
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
        return first(rep->getRE());
    } else if (Diff * diff = dyn_cast<Diff>(re)) {
        UCD::UnicodeSet * UnicodeSets = new UCD::UnicodeSet();
        UCD::UnicodeSet UnicodeSets_diff = UCD::UnicodeSet();
        if (first(diff->getLH()) && first(diff->getRH())) {
            UnicodeSets_diff = *(first(diff->getLH())) - *(first(diff->getRH()));
        }
        *UnicodeSets = UnicodeSets_diff;
        return UnicodeSets;
    } else if (Intersect * ix = dyn_cast<Intersect>(re)) {
        UCD::UnicodeSet * UnicodeSets = new UCD::UnicodeSet();
        UCD::UnicodeSet UnicodeSets_inter = UCD::UnicodeSet();
        if (first(ix->getLH()) && first(ix->getRH())) {
            UnicodeSets_inter = *(first(ix->getLH())) & *(first(ix->getRH()));
        }
        *UnicodeSets = UnicodeSets_inter;
        return UnicodeSets;
    }
    return nullptr;

}

UCD::UnicodeSet* RE_Local::final(RE * re) {
    if (Name * name = dyn_cast<Name>(re)) {
        if (LLVM_LIKELY(name->getDefinition() != nullptr)) {
            if (CC * cc = dyn_cast<CC>(name->getDefinition())) {
                UCD::UnicodeSet * sets = cast<UCD::UnicodeSet>(cc);
                return sets;
            } else {
                return final(name->getDefinition());
            }
        } else {
            throw std::runtime_error("All non-unicode-property Name objects should have been defined prior to Unicode property resolution.");
        }
    } else if (CC * cc = dyn_cast<CC>(re)) {
        UCD::UnicodeSet * sets = cast<UCD::UnicodeSet>(cc);
        return sets;
    } else if (Seq * seq = dyn_cast<Seq>(re)) {
        UCD::UnicodeSet * UnicodeSets = new UCD::UnicodeSet();
        UCD::UnicodeSet UnicodeSets_seq = UCD::UnicodeSet();
        for (auto si = seq->rbegin(); si != seq->rend(); ++si) {
            if (isNullable(*si) && final(*si) != nullptr) {
                UnicodeSets_seq = UnicodeSets_seq + *(final(*si));
            } else if (isNullable(*si) && final(*si) == nullptr) {
                continue;
            } else if (!isNullable(*si) && final(*si) != nullptr){
                UnicodeSets_seq = UnicodeSets_seq + *(final(*si));
                break;
            } else {
                break;
            }
        }
        *UnicodeSets = UnicodeSets_seq;
        return UnicodeSets_seq.empty() ? nullptr : UnicodeSets;
    } else if (Alt * alt = dyn_cast<Alt>(re)) {
        UCD::UnicodeSet * UnicodeSets = new UCD::UnicodeSet();
        UCD::UnicodeSet UnicodeSets_alt = UCD::UnicodeSet();
        for (auto ai = alt->begin(); ai != alt->end(); ++ai) {
            if (final(*ai) != nullptr) {
                UnicodeSets_alt = UnicodeSets_alt + *(final(*ai));
            }
        }
        *UnicodeSets = UnicodeSets_alt;
        return UnicodeSets_alt.empty() ? nullptr : UnicodeSets;
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
        return final(rep->getRE());
    } else if (Diff * diff = dyn_cast<Diff>(re)) {
        UCD::UnicodeSet * UnicodeSets = new UCD::UnicodeSet();
        UCD::UnicodeSet UnicodeSets_diff = UCD::UnicodeSet();
        if (final(diff->getLH()) && final(diff->getRH())) {
            UnicodeSets_diff = *(final(diff->getLH())) - *(final(diff->getRH()));
        }
        *UnicodeSets = UnicodeSets_diff;
        return UnicodeSets;
    } else if (Intersect * ix = dyn_cast<Intersect>(re)) {
        UCD::UnicodeSet * UnicodeSets = new UCD::UnicodeSet();
        UCD::UnicodeSet UnicodeSets_inter = UCD::UnicodeSet();
        if (final(ix->getLH()) && final(ix->getRH())) {
            UnicodeSets_inter = *(final(ix->getLH())) & *(final(ix->getRH()));
        }
        *UnicodeSets = UnicodeSets_inter;
        return UnicodeSets;
    }
    return nullptr;

}

void RE_Local::follow(RE * re, std::map<UCD::UnicodeSet*, UCD::UnicodeSet*> &follow_map) {
    if (Name * name = dyn_cast<Name>(re)) {
        if (LLVM_LIKELY(name->getDefinition() != nullptr)) {
            return follow(name->getDefinition(), follow_map);
        } else {
            throw std::runtime_error("All non-unicode-property Name objects should have been defined prior to Unicode property resolution.");
        }
    } else if (Seq * seq = dyn_cast<Seq>(re)) {
        RE * re_first = *(seq->begin());
        std::vector<RE *> list;
        list.reserve(seq->size());
        for (auto i = seq->begin() + 1; i != seq->end(); i++) {
            list.push_back(*i);
        }
        RE * re_follow = makeSeq(list.begin(), list.end());
        follow(re_first, follow_map);
        follow(re_follow, follow_map);
        auto e1 = final(re_first);
        auto e2 = first(re_follow);
        if (e1 && e2) {
            auto e = follow_map.find(e1);
            if (e != follow_map.end()) {
                *(e->second) = *(e->second) + *e2;
            } else {
                follow_map.insert(std::pair<UCD::UnicodeSet*, UCD::UnicodeSet*>(e1, e2));
            }
        }
        return;
    } else if (Alt * alt = dyn_cast<Alt>(re)) {
        for (auto ai = alt->begin(); ai != alt->end(); ++ai) {
            follow(*ai, follow_map);
        }
        return;
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
        follow(rep->getRE(), follow_map);
        auto e1 = final(rep->getRE());
        auto e2 = first(rep->getRE());
        if (e1 && e2) {
            auto e = follow_map.find(e1);
            if (e != follow_map.end()) {
                *(e->second) = *(e->second) + *e2;
            } else {
                follow_map.insert(std::pair<UCD::UnicodeSet*, UCD::UnicodeSet*>(e1, e2));
            }
        }
        return;
    }
    return;
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
