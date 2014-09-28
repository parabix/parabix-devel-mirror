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
        for (auto i = alt->begin(); i != alt->end(); ++i) {
            *i = simplify(*i);
        }
        re = simplify(alt);
    }
    else if (Seq * seq = dyn_cast<Seq>(re)) {
        for (auto i = seq->begin(); i != seq->end(); ++i) {
            *i = simplify(*i);
        }
        re = simplify(seq);
    }
    else if (Rep * rep = dyn_cast<Rep>(re)) {
        rep->setRE(simplify(rep->getRE()));
        simplify(rep);
    }
    return re;
}

RE * RE_Simplifier::simplify(Seq * seq) {
    /*
      mkSeq - make a sequence, but flatten.  Result might not be a Seq. If
      there is only one component for a sequence, simply return that.
    */

    RE * re = seq;
    if (!seq->empty()) {
        std::vector<RE*> list;
        list.reserve(seq->size());
        // Reverse the order of the input list so we can more efficiently "pull" the first
        // character from the end. Note: this uses a linear inplace reversal.
        std::reverse(seq->begin(), seq->end());

        while (!seq->empty()) {
            RE * next = seq->back();
            seq->pop_back();
            if (Seq * re_seq = dyn_cast<Seq>(next)) {
                if (re_seq->getType() != Seq::Type::Byte) {
                    // like above, insert the "subsequence" to flatten in reverse order
                    std::reverse_copy(re_seq->begin(), re_seq->end(), std::back_inserter(*seq));
                    re_seq->clear();
                    delete re_seq;
                    continue;
                }
            }
            list.push_back(next);
        }
        if (list.size() == 1) {
            re = list.back();
            delete seq;
        }
        else {
            seq->swap(list);
        }
    }
    return re;
}

/**
 * @brief makeAlt
 *
 * Build an Alt, flattening alternative subgroups, and combining character classes and
 * move character classes towards the end of the list to ensure that all combinations are found.
 *
 * @param list
 * @return simplified RE representing the Alt
 */
RE * RE_Simplifier::simplify(Alt * alt) {
    RE * re = alt;
    if (!alt->empty()) {

        std::queue<CC*> ccs;

        std::vector<RE *> list;
        while (!alt->empty()) {
            RE * next = alt->back();
            alt->pop_back();
            if (Alt * re_alt = dyn_cast<Alt>(next)) {
                alt->insert(alt->end(), re_alt->begin(), re_alt->end());
                re_alt->clear();
                delete re_alt;
            }
            else if (CC * cc = dyn_cast<CC>(next)) {
                ccs.push(cc);
            }
            else {
                list.push_back(next);
            }
        }

        if (!ccs.empty()) {
            while (ccs.size() > 1) {
                CC * a = ccs.front(); ccs.pop();
                CC * b = ccs.front(); ccs.pop();
                ccs.push(makeCC(a, b));
            }
            list.push_back(ccs.front());
        }

        if (list.size() == 1) {
            // if only one alternation exists, discard the Alt object itself and return the internal RE.
            re = list.back();
            delete alt;
        }
        else {
            alt->swap(list);
        }
    }
    return re;
}

RE * RE_Simplifier::simplify(Rep * rep) {
    RE * re = rep->getRE();
    const int lb = rep->getLB();
    const int ub = rep->getUB();
    std::unique_ptr<Rep> janitor(rep);
    rep->setRE(nullptr);
    if (Rep * nrep = dyn_cast<Rep>(re)) {
        if (nrep->getUB() == Rep::UNBOUNDED_REP) {
            if ((lb > 0) || (nrep->getLB() <= 1)) {
                nrep->setLB(nrep->getLB() * lb);
                nrep->setUB(Rep::UNBOUNDED_REP);
                return simplify(nrep);
            }
            else if (lb == 0) {
                nrep->setLB(0);
                nrep->setUB(1);
                return simplify(nrep);
            }
        }
        else if ((nrep->getUB() * lb) >= (nrep->getLB() * (lb + 1) - 1)) {
            nrep->setLB(nrep->getUB() * lb);
            nrep->setUB(ubCombine(nrep->getUB(), ub));
            return simplify(nrep);
        }
    }
    else {
        if (Seq * seq = dyn_cast<Seq>(re)) {
            if (seq->empty()) {
                return seq;
            }
        }
        if ((lb == 0) && (ub == 0)) {
            delete re;
            return makeSeq();
        }
        else if ((lb == 1) && (ub == 1)) {
            return re;
        }
    }
    rep->setRE(re);
    return janitor.release();
}

inline int RE_Simplifier::ubCombine(const int h1, const int h2) {
    if ((h1 == Rep::UNBOUNDED_REP) || (h2 == Rep::UNBOUNDED_REP)) {
        return Rep::UNBOUNDED_REP;
    }
    else {
        return h1 * h2;
    }
}

}
