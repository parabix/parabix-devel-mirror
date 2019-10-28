#include <re/transforms/re_star_normal.h>

#include <re/adt/adt.h>
#include <re/analysis/nullable.h>
#include <re/analysis/re_analysis.h>

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
