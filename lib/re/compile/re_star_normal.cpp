#include <re/compile/re_star_normal.h>

#include <re/adt/re_name.h>
#include <re/adt/re_any.h>
#include <re/adt/re_start.h>
#include <re/adt/re_end.h>
#include <re/adt/re_alt.h>
#include <re/adt/re_cc.h>
#include <re/adt/re_seq.h>
#include <re/adt/re_rep.h>
#include <re/adt/re_diff.h>
#include <re/adt/re_intersect.h>
#include <re/adt/re_assertion.h>
#include <re/compile/re_analysis.h>
#include <re/compile/re_nullable.h>

using namespace llvm;

namespace re {

inline RE * star_rule(RE * re) {
    if (Seq * seq = dyn_cast<Seq>(re)) {
        if (isNullable(re)) {
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
