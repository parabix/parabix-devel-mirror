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
#include <re/re_nullable.h>

using namespace llvm;

namespace re {

RE * RE_Star_Normal::optimize(RE * re) {
    if (Alt * alt = dyn_cast<Alt>(re)) {
        std::vector<RE *> list;
        list.reserve(alt->size());
        for (RE * re : *alt) {
            list.push_back(optimize(re));
        }
        re = makeAlt(list.begin(), list.end());
    } else if (Seq * seq = dyn_cast<Seq>(re)) {
        RE * head = *(seq->begin());
        RE * tail = makeSeq(seq->begin() + 1, seq->end());
        const auto headNullable = RE_Nullable::isNullable(head);
        head = headNullable ? optimize(head) : star_normal(head);
        const auto tailNullable = RE_Nullable::isNullable(tail);
        tail = tailNullable ? optimize(tail) : star_normal(tail);
        if (LLVM_UNLIKELY(headNullable && tailNullable)) {
            re = makeAlt({head, tail});
        } else {
            re = makeSeq({head, tail});
        }
    } else if (Assertion * a = dyn_cast<Assertion>(re)) {
        re = makeAssertion(optimize(a->getAsserted()), a->getKind(), a->getSense());
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
        RE * const expr = optimize(rep->getRE());
        if (rep->getLB() == 0 && rep->getUB() == Rep::UNBOUNDED_REP) {
            re = expr;
        } else {
            re = makeRep(expr, rep->getLB(), rep->getUB());
        }
    } else if (Diff * diff = dyn_cast<Diff>(re)) {
        re = makeDiff(optimize(diff->getLH()), optimize(diff->getRH()));
    } else if (Intersect * e = dyn_cast<Intersect>(re)) {
        re = makeIntersect(optimize(e->getLH()), optimize(e->getRH()));
    } else if (Name * name = dyn_cast<Name>(re)) {
        if (name->getDefinition()) {
            name->setDefinition(optimize(name->getDefinition()));
        }
    }
    return re;
}

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
            RE * expr = optimize(rep->getRE());
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

}
