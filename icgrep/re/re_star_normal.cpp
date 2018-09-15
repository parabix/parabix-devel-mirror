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

RE * star_rule(RE * re) {
    if (Seq * seq = dyn_cast<Seq>(re)) {
        if (RE_Nullable::isNullable(re)) {
            std::vector<RE *> list;
            list.reserve(seq->size());
            for (RE * r : *seq) {
                if (Rep * rep = dyn_cast<Rep>(r)) {
                    if (rep->getLB() == 0) {
                        list.push_back(rep->getRE());
                    }
                } else if (!isEmptySeq(r)) {
                    list.push_back(r);
                }
            }
            return makeAlt(list.begin(), list.end());
        }
    }
    return re;
}

RE * RE_Star_Normal::transformRep(Rep * rep) {
    RE * e0 = rep->getRE();
    RE * e = transform(e0);
    if (rep->getLB() == 0 && rep->getUB() == Rep::UNBOUNDED_REP) {
        e = star_rule(e);
    }
    if (e == e0) return rep;
    return makeRep(e, rep->getLB(), rep->getUB());
}

}
