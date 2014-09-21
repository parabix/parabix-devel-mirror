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

RE* RE_Simplifier::simplify(RE * re) {
    RE * retVal = re;
    if (Alt * re_alt = dynamic_cast<Alt*>(re)) {
        Vector simplified_alt;
        for (RE * re : *re_alt)
        {
            simplified_alt.push_back(simplify(re));
        }
        retVal = makeAlt(simplified_alt);
    }
    else if (Seq * re_seq = dynamic_cast<Seq*>(re)) {
        Vector simplified_seq;
        for (RE * re : *re_seq)
        {
            simplified_seq.push_back(simplify(re));
        }
        retVal = makeSeq(re_seq->getType(), simplified_seq);
    }
    else if (CC* re_cc = dynamic_cast<CC*>(re)) {
        retVal = re_cc;
    }
    else if (Name* re_name = dynamic_cast<Name*>(re)) {
        Name* name = new Name(re_name->getName());
        name->setType(re_name->getType());
        name->setNegated(re_name->isNegated());   // TODO:  Hide this in the re_name module.
        retVal = name;
    }
    else if (Rep* re_rep = dynamic_cast<Rep*>(re)) {
        retVal = makeRep(simplify(re_rep->getRE()), re_rep->getLB(), re_rep->getUB());
    }
    else if (dynamic_cast<Start*>(re)) {
        retVal = new Start();
    }
    else if (dynamic_cast<End*>(re)) {
        retVal = new End();
    }
    return retVal;
}

RE * RE_Simplifier::makeSeq(const Seq::Type type, Vector & list) {
    /*
      mkSeq - make a sequence, but flatten.  Result might not be a Seq. If
      there is only one component for a sequence, simply return that.
    */

    RE * re = nullptr;
    if (!list.empty()) {
        std::unique_ptr<Seq> seq = std::unique_ptr<Seq>(new Seq(type));
        // Reverse the order of the input list so we can more efficiently "pull" the first
        // character from the end. Note: this ought to be an inplace reversal.
        std::reverse(list.begin(), list.end());

        while (!list.empty()) {
            RE * next = list.back();
            list.pop_back();
            if (Seq * re_seq = dynamic_cast<Seq*>(next)) {
                if (re_seq->getType() != Seq::Byte) {
                    // like above, insert the "subsequence" in reverse order
                    list.reserve(re_seq->size());
                    std::reverse_copy(re_seq->begin(), re_seq->end(), std::back_inserter(list));
                    continue;
                }
            }
            seq->push_back(next);
        }
        if (seq->size() == 1) {
            re = seq->back();
            seq->pop_back();
        }
        else {
            re = seq.release();
        }
    }
    return re;
}

RE * RE_Simplifier::makeAlt(Vector & list) {

    /*
      Build a list for Alt, flattening alternative subgroups, and combining character classes.  We
      move character classes towards the end of the list to ensure that all combinations are found.
    */

    RE * re = nullptr;
    if (!list.empty()) {

        std::unique_ptr<Alt> new_alt = std::unique_ptr<Alt>(new Alt());
        std::queue<CC*> ccs;

        while (!list.empty()) {
            RE * next = list.back();
            list.pop_back();
            if (Alt * re_alt = dynamic_cast<Alt*>(next)) {
                list.insert(list.end(), re_alt->begin(), re_alt->end());
            }
            else if (CC * cc = dynamic_cast<CC*>(next)) {
                ccs.push(cc);
            }
            else {
                new_alt->push_back(next);
            }
        }

        if (!ccs.empty()) {
            while (ccs.size() > 1) {
                CC * a = ccs.front(); ccs.pop();
                CC * b = ccs.front(); ccs.pop();
                ccs.push(new CC(a, b));
            }
            new_alt->push_back(ccs.front());
        }

        if (new_alt->size() == 1) {
            re = new_alt->back();
            new_alt->pop_back();
        }
        else {
            re = new_alt.release();
        }
    }

    return re;
}

RE * RE_Simplifier::makeRep(RE * re, const int lb2, const int ub2)
{
    if (Rep* rep = dynamic_cast<Rep*>(re)) {
        if (((rep->getUB() == UNBOUNDED_REP) && (lb2 > 0)) ||
                ((rep->getUB() == UNBOUNDED_REP) && (rep->getLB() <= 1))) {
            return new Rep(rep->getRE(), rep->getLB() * lb2, UNBOUNDED_REP);
        }
        else if ((rep->getUB() == UNBOUNDED_REP) && (lb2 == 0)) {
            return new Rep(rep, 0, 1);
        }
        else if ((rep->getUB() * lb2) >= (rep->getLB() * (lb2 + 1) - 1)) {
            return new Rep(rep->getRE(), rep->getLB() * lb2, ubCombine(rep->getUB(), ub2));
        }
        else {
            return new Rep(rep, lb2, ub2);
        }
    }
    else {
        if (Seq* seq = dynamic_cast<Seq*>(re)) {
            if (seq->empty()) {
                return seq;
            }
        }

        if ((lb2 == 0) && (ub2 == 0)) {
            return new Seq();
        }
        else if ((lb2 == 1) && (ub2 == 1)) {
            return re;
        }
        else {
            return new Rep(re, lb2, ub2);
        }
    }
}

inline int RE_Simplifier::ubCombine(const int h1, const int h2)
{
    if ((h1 == UNBOUNDED_REP) || (h2 == UNBOUNDED_REP))
    {
        return UNBOUNDED_REP;
    }
    else
    {
        return h1 * h2;
    }
}
