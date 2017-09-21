#include "re_star_normal.h"
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

using namespace llvm;

namespace re {

RE * RE_Star_Normal::star_normal(RE * re) {

    if (Alt * alt = dyn_cast<Alt>(re)) {
        std::vector<RE *> list;
        list.reserve(alt->size());
        for (RE * re : *alt) {
            list.push_back(star_normal(re));
        }
        re = makeAlt(list.begin(), list.end());
    } else if (Seq * seq = dyn_cast<Seq>(re)) {
        std::vector<RE *> list;
        list.reserve(seq->size());
        for (RE * re : *seq) {
            list.push_back(star_normal(re));
        }
        re = makeSeq(list.begin(), list.end());
    } else if (Assertion * a = dyn_cast<Assertion>(re)) {
        re = makeAssertion(star_normal(a->getAsserted()), a->getKind(), a->getSense());
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
         if (rep->getLB() == 0 && rep->getUB() == Rep::UNBOUNDED_REP) {
            RE * expr = helper(rep->getRE());
            re = makeRep(expr, 0, rep->getUB());
        } else {
            RE * expr = star_normal(rep->getRE());
            re = makeRep(expr, rep->getLB(), rep->getUB());
        }
    } else if (Diff * diff = dyn_cast<Diff>(re)) {
        re = makeDiff(star_normal(diff->getLH()), star_normal(diff->getRH()));
    } else if (Intersect * e = dyn_cast<Intersect>(re)) {
        re = makeIntersect(star_normal(e->getLH()), star_normal(e->getRH()));
    } else if (Name * name = dyn_cast<Name>(re)) {
        if (name->getDefinition()) {
            name->setDefinition(star_normal(name->getDefinition()));
        }
    }
    return re;
}

RE * RE_Star_Normal::helper(RE * re) {
    if (Alt * alt = dyn_cast<Alt>(re)) {
        std::vector<RE *> list;
        list.reserve(alt->size());
        for (RE * re : *alt) {
            list.push_back(helper(re));
        }
        re = makeAlt(list.begin(), list.end());
    } else if (Seq * seq = dyn_cast<Seq>(re)) {
        RE * const re_first = *(seq->begin());
        RE * const re_follow = makeSeq(seq->begin() + 1, seq->end());
        const auto isFirstNullable = isNullable(re_first);
        const auto isFollowNullable = isNullable(re_follow);
        if (LLVM_LIKELY(!isFirstNullable && !isFollowNullable)) {
            re = makeSeq({star_normal(re_first), star_normal(re_follow)});
        } else if (!isFirstNullable && isFollowNullable) {
            re = makeSeq({helper(re_first), star_normal(re_follow)});
        } else if (isFirstNullable && !isFollowNullable) {
            re = makeSeq({star_normal(re_first), helper(re_follow)});
        } else {
            re = makeAlt({helper(re_first), helper(re_follow)});
        }
    } else if (Assertion * a = dyn_cast<Assertion>(re)) {
        re = makeAssertion(helper(a->getAsserted()), a->getKind(), a->getSense());
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
        RE * const expr = helper(rep->getRE());
        if (rep->getLB() == 0 && rep->getUB() == Rep::UNBOUNDED_REP) {
            re = expr;
        } else {
            re = makeRep(expr, rep->getLB(), rep->getUB());
        }
    } else if (Diff * diff = dyn_cast<Diff>(re)) {
        re = makeDiff(helper(diff->getLH()), helper(diff->getRH()));
    } else if (Intersect * e = dyn_cast<Intersect>(re)) {
        re = makeIntersect(helper(e->getLH()), helper(e->getRH()));
    } else if (Name * name = dyn_cast<Name>(re)) {
        if (name->getDefinition()) {
            name->setDefinition(helper(name->getDefinition()));
        }
    }
    return re;
}

bool RE_Star_Normal::isNullable(const RE * re) {
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
