#include "re_simplifier.h"
#include "re_cc.h"
#include "re_name.h"
#include "re_start.h"
#include "re_end.h"
#include "re_seq.h"
#include "re_alt.h"
#include "re_rep.h"
#include <algorithm>
#include <memory>
#include <queue>

namespace re {

RE * RE_Simplifier::simplify(RE * re) {
    if (Alt * alt = dyn_cast<Alt>(re)) {
        std::vector<RE*> list;
        list.reserve(alt->size());
        for (RE * re : *alt) {
            list.push_back(simplify(re));
        }
        re = makeAlt(list.begin(), list.end());
    }
    else if (Seq * seq = dyn_cast<Seq>(re)) {
        std::vector<RE*> list;
        list.reserve(seq->size());
        for (RE * re : *seq) {
            list.push_back(simplify(re));
        }
        re = makeSeq(list.begin(), list.end());
    }
    else if (Rep * rep = dyn_cast<Rep>(re)) {
        re = makeRep(simplify(rep->getRE()), rep->getLB(), rep->getUB());
    }
    return re;
}

}
