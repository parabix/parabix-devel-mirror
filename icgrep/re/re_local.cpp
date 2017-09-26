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
            UndefinedNameError(name);
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
            UndefinedNameError(name);
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
            UndefinedNameError(name);
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
    UCD::UnicodeSet seen = UCD::UnicodeSet();
    return isLocalLanguage_helper(re, seen);
}

bool RE_Local::isLocalLanguage_helper(const RE * re, UCD::UnicodeSet & codepoints_seen) {
    assert ("RE object cannot be null!" && re);
    if (isa<Any>(re)) {
        bool no_intersect = codepoints_seen.empty();
        codepoints_seen = UCD::UnicodeSet(0x00, 0x10FFFF);
        return no_intersect;
    } else if (const CC * cc = dyn_cast<CC>(re)) {
        bool has_intersect = cast<UCD::UnicodeSet>(cc)->intersects(codepoints_seen);
        codepoints_seen = codepoints_seen + *cast<UCD::UnicodeSet>(cc);
        return !has_intersect;
    } else if (const Name * n = dyn_cast<Name>(re)) {
        return isLocalLanguage_helper(n->getDefinition(), codepoints_seen);
    } else if (const Seq * re_seq = dyn_cast<const Seq>(re)) {
        for (const RE * item : *re_seq) {
            if (!isLocalLanguage_helper(item, codepoints_seen)) return false;
        }
        return true;
    } else if (const Alt * re_alt = dyn_cast<const Alt>(re)) {
        for (RE * item : *re_alt) {
            if (!isLocalLanguage_helper(item, codepoints_seen)) return false;
        }
        return true;
    } else if (const Rep* re_rep = dyn_cast<const Rep>(re)) {
        if (re_rep->getUB() > 1) return false;
        return isLocalLanguage_helper(re_rep->getRE(), codepoints_seen);
    } else {
        // A local language cannot contain Intersects, Differences, Assertions, Start, End.
        return false;
    }
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
