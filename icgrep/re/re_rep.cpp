/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "re_rep.h"
#include "re_assertion.h"
#include "re_seq.h"
#include "re_alt.h"
#include "re_nullable.h"
#include <llvm/Support/Casting.h>
#include <llvm/Support/ErrorHandling.h>

using namespace llvm;

namespace re {

inline int ubCombine(const int h1, const int h2) {
    if ((h1 == Rep::UNBOUNDED_REP) || (h2 == Rep::UNBOUNDED_REP)) {
        return Rep::UNBOUNDED_REP;
    }
    else {
        return h1 * h2;
    }
}
    
RE * makeRep(RE * re, int lb, const int ub) {
    if (LLVM_UNLIKELY(lb == Rep::UNBOUNDED_REP)) {
        report_fatal_error("repetition lower bound must be finite!");
    }
    if (LLVM_UNLIKELY(ub != Rep::UNBOUNDED_REP && ub < lb)) {
        report_fatal_error("lower bound cannot exceed upper bound");
    }
    if (isNullable(re)) {
        if (ub == 1) {
            return re;
        }
        lb = 0;
    }    
    if (Rep * rep = dyn_cast<Rep>(re)) {
        int l = rep->getLB();
        int u = rep->getUB();
        if (lb == ub) {
            return new Rep(rep->getRE(), l * lb, ubCombine(u, ub));
        }
        else if (u == Rep::UNBOUNDED_REP) {
            if (l == 0) {
                /*  R{0,}{lb, ub} = R{0,} */
                return rep;
            } else if (l == 1) {
                /*  R{1,}{lb, ub} = R{lb,} */
                return new Rep(rep->getRE(), lb, Rep::UNBOUNDED_REP);
            } else if (lb == 0) {
                /*  R{l,}{0, ub} = R{l,}? */
                return new Rep(rep, 0, 1);
            } else {
                /* R{l,}{lb, ub} = R{l * lb,} */
                return new Rep(rep->getRE(), l * lb, Rep::UNBOUNDED_REP);
            }
        }
        else if (u > l) {
            // Calculate the smallest number of repetitions n such that n * u + 1 >= (n + 1) * l
            int n = (u - 2)/(u-l);
            if (lb >= n) {
                return new Rep(rep->getRE(), l * lb, ubCombine(u, ub));
            }
            if ((ub == Rep::UNBOUNDED_REP) || (ub >= n)) {
                RE * r1 = new Rep(rep->getRE(), n * l, ubCombine(u, ub));
                RE * r2 = makeRep(rep, lb, n - 1);  // makeRep recursive simplifies.
                return makeAlt({r1, r2});
            }
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

RE * unrollFirst(Rep * rep) {
    RE * e = rep->getRE();
    auto lb = rep->getLB();
    auto ub = rep->getUB();
    if (ub == 0) return makeAlt();  // Can't unroll - return unmatchable regexp.
    // Unroll one copy of the loop and simplify.
    RE * reduced = makeRep(e, lb == 0 ? lb : lb - 1, ub == Rep::UNBOUNDED_REP ? ub : ub - 1);
    RE * unrolled = makeSeq({e, reduced});
    if (lb == 0) return makeAlt({makeSeq(), unrolled});
    else return unrolled;
}
RE * unrollLast(Rep * rep) {
    RE * e = rep->getRE();
    auto lb = rep->getLB();
    auto ub = rep->getUB();
    if (ub == 0) return makeAlt();  // Can't unroll - return unmatchable regexp.
    // Unroll one copy of the loop and simplify.
    RE * reduced = makeRep(e, lb == 0 ? lb : lb - 1, ub == Rep::UNBOUNDED_REP ? ub : ub - 1);
    RE * unrolled = makeSeq({reduced, e});
    if (lb == 0) return makeAlt({makeSeq(), unrolled});
    else return unrolled;
}
}
