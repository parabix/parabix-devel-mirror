#include "re_simplifier.h"
#include <re/re_name.h>
#include <re/re_alt.h>
#include <re/re_seq.h>
#include <re/re_rep.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_assertion.h>
#include <re/re_memoizer.hpp>
#include <boost/container/flat_set.hpp>

using namespace llvm;

namespace re {

using Set = boost::container::flat_set<RE *>;
using List = std::vector<RE *>;

struct PassContainer {
    RE * simplify(RE * re) {
        if (Alt * alt = dyn_cast<Alt>(re)) {
            Set set;
            set.reserve(alt->size());
            for (RE * item : *alt) {
                item = simplify(item);
                if (LLVM_UNLIKELY(isa<Alt>(item) && cast<Alt>(item)->empty())) {
                    continue;
                }
                set.insert(item);
            }
            re = makeAlt(set.begin(), set.end());
        } else if (Seq * seq = dyn_cast<Seq>(re)) {
            List list;
            list.reserve(seq->size());
            for (RE * item : *seq) {
                item = simplify(item);
                if (LLVM_UNLIKELY(isa<Seq>(item) && cast<Seq>(item)->empty())) {
                    continue;
                }
                list.push_back(item);
            }
            re = makeSeq(list.begin(), list.end());
        } else if (Assertion * a = dyn_cast<Assertion>(re)) {
            re = makeAssertion(simplify(a->getAsserted()), a->getKind(), a->getSense());
        } else if (Rep * rep = dyn_cast<Rep>(re)) {
            RE * expr = simplify(rep->getRE());
            re = makeRep(expr, rep->getLB(), rep->getUB());
        } else if (Diff * diff = dyn_cast<Diff>(re)) {
            re = makeDiff(simplify(diff->getLH()), simplify(diff->getRH()));
        } else if (Intersect * e = dyn_cast<Intersect>(re)) {
            re = makeIntersect(simplify(e->getLH()), simplify(e->getRH()));
        }
        return mMemoizer.memoize(re);
    }
private:
    Memoizer mMemoizer;
};

RE * RE_Simplifier::simplify(RE * re) {
    PassContainer pc;
    return pc.simplify(re);
}

}
