/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "re_assertion.h"
#include "re_rep.h"
#include "re_seq.h"

namespace re {

inline int ubCombine(const int h1, const int h2) {
    if ((h1 == Rep::UNBOUNDED_REP) || (h2 == Rep::UNBOUNDED_REP)) {
        return Rep::UNBOUNDED_REP;
    }
    else {
        return h1 * h2;
    }
}

RE * makeRep(RE * re, const int lb, const int ub) {
    if (Rep * rep = dyn_cast<Rep>(re)) {
        if (rep->getUB() == Rep::UNBOUNDED_REP && ((lb > 0) || (rep->getLB() <= 1))) {
            return new Rep(rep->getRE(), rep->getLB() * lb, Rep::UNBOUNDED_REP);
        }
        else if ((rep->getUB() == Rep::UNBOUNDED_REP) && (lb == 0)) {
            return new Rep(rep, 0, 1);
        }
        else if (lb == ub) {
            return new Rep(rep->getRE(), lb * rep->getLB(), ub * rep->getUB());
        }
        else if ((rep->getUB() * lb) >= (rep->getLB() * (lb + 1) - 1)) {
            return new Rep(rep->getRE(), rep->getUB() * lb, ubCombine(rep->getUB(), ub));
        }
    }
    else if (isa<Assertion>(re)) {
        if (lb > 0) return re;
        else return makeSeq();
    }
    else {
        if (Seq * seq = dyn_cast<Seq>(re)) {
            if (seq->empty()) {
                return seq;
            }
        }
        if ((lb == 0) && (ub == 0)) {
            return makeSeq();
        }
        else if ((lb == 1) && (ub == 1)) {
            return re;
        }
    }
    return new Rep(re, lb, ub);
}

}
