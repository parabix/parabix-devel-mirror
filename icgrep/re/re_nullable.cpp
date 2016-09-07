#include "re_nullable.h"
#include <re/re_cc.h>
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_alt.h>
#include <re/re_rep.h>
#include <re/re_any.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_assertion.h>
#include <re/re_name.h>

/*

 A regular expression is nullable if it (a) matches the empty
 string, and (b) applies everywhere.  Note that Start (^) and
 End ($) match the empty string, but not everywhere).

*/

namespace re {

RE * RE_Nullable::removeNullablePrefix(RE * re) {
    if (Seq * seq = dyn_cast<Seq>(re)) {
        std::vector<RE*> list;
        for (auto i = seq->begin(); i != seq->end(); ++i) {
            if (!isNullable(*i)) {
                list.push_back(removeNullablePrefix(*i));
                std::copy(++i, seq->end(), std::back_inserter(list));
                break;
            }
        }
        re = makeSeq(list.begin(), list.end());
    } else if (Alt * alt = dyn_cast<Alt>(re)) {
        std::vector<RE*> list;
        for (auto i = alt->begin(); i != alt->end(); ++i) {
            list.push_back(removeNullablePrefix(*i));
        }
        re = makeAlt(list.begin(), list.end());
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
        if ((rep->getLB() == 0) || (isNullable(rep->getRE()))) {
            re = makeSeq();
        }
        else if (hasNullablePrefix(rep->getRE())) {
            re = makeSeq({removeNullablePrefix(rep->getRE()), makeRep(rep->getRE(), rep->getLB() - 1, rep->getLB() - 1)});
        }
        else {
            re = makeRep(rep->getRE(), rep->getLB(), rep->getLB());
        }
    } else if (Name * name = dyn_cast<Name>(re)) {
        if (name->getDefinition()) {
            name->setDefinition(removeNullablePrefix(name->getDefinition()));
        }
    }
    return re;
}

RE * RE_Nullable::removeNullableSuffix(RE * re) {
    if (Seq * seq = dyn_cast<Seq>(re)) {
        std::vector<RE*> list;
        for (auto i = seq->rbegin(); i != seq->rend(); ++i) {
            if (!isNullable(*i)) {
                std::copy(seq->begin(), (i + 1).base(), std::back_inserter(list));
                list.push_back(removeNullableSuffix(*i));
                break;
            }
        }
        re = makeSeq(list.begin(), list.end());
    } else if (Alt* alt = dyn_cast<Alt>(re)) {
        std::vector<RE*> list;
        for (auto i = alt->begin(); i != alt->end(); ++i) {
            list.push_back(removeNullableSuffix(*i));
        }
        re = makeAlt(list.begin(), list.end());
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
        if ((rep->getLB() == 0) || (isNullable(rep->getRE()))) {
            re = makeSeq();
        }
        else if (hasNullableSuffix(rep->getRE())) {
            re = makeSeq({makeRep(rep->getRE(), rep->getLB() - 1, rep->getLB() - 1), removeNullableSuffix(rep->getRE())});
        }
        else {
            re = makeRep(rep->getRE(), rep->getLB(), rep->getLB());
        }
    } else if (Name * name = dyn_cast<Name>(re)) {
        if (name->getDefinition()) {
            name->setDefinition(removeNullableSuffix(name->getDefinition()));
        }
    }
    return re;
}

// Deal with case: R1 (Assertion R2) R3
// If R2 is nullable, then R1 R3. 
RE * RE_Nullable::removeNullableAssertion(RE * re) {
    if (Assertion * a = dyn_cast<Assertion>(re)) {
        if (isNullable(a->getAsserted())) {
	    std::vector<RE *> seq;
	    return makeSeq(seq.begin(), seq.end());
        } else {
            return re;
        }
    } else if (Seq * seq = dyn_cast<Seq>(re)) {
        std::vector<RE*> list;
        for (auto i = seq->begin(); i != seq->end(); ++i) {
            list.push_back(removeNullableAssertion(*i));
        }
        re = makeSeq(list.begin(), list.end());
    } else if (Alt * alt = dyn_cast<Alt>(re)) {
        std::vector<RE*> list;
        for (auto i = alt->begin(); i != alt->end(); ++i) {
            list.push_back(removeNullableAssertion(*i));
        }
        re = makeAlt(list.begin(), list.end());
    } 
    return re;
}

// Deal with case: R1 (Assertion R2) R3 
// If R3 is nullable, then R1 R2.
RE * RE_Nullable::removeNullableAfterAssertion(RE * re) {
    if (isNullableAfterAssertion(re)) {
	re = removeNullableAfterAssertion_helper(re);
    }
    return re;
}

bool RE_Nullable::isNullableAfterAssertion(const RE * re) {
    bool nullable = false;
    if (const Seq * seq = dyn_cast<const Seq>(re)) {
        nullable = isa<Assertion>(seq->back()) ? true : isNullableAfterAssertion(seq->back());
    } else if (const Alt * alt = dyn_cast<const Alt>(re)) {
        for (const RE * re : *alt) {
            if (isNullableAfterAssertion(re)) {
                nullable = true;
                break;
            }
        }
    }   
    return nullable;
}

RE * RE_Nullable::removeNullableAfterAssertion_helper(RE * re) {
    if (Assertion * a = dyn_cast<Assertion>(re)) {
        if (a->getSense() == Assertion::Sense::Positive) {
            return a->getAsserted();
        } else {
            return makeDiff(makeAny(), a->getAsserted());
        }
    } else if (Seq * seq = dyn_cast<Seq>(re)) {
        std::vector<RE*> list;
        auto i = seq->begin();
        for (; i != seq->end() - 1; ++i) {
            list.push_back(*i);
        }
        list.push_back(removeNullableAfterAssertion_helper(*i));
        re = makeSeq(list.begin(), list.end());
    } else if (Alt * alt = dyn_cast<Alt>(re)) {
        std::vector<RE*> list;
        for (auto i = alt->begin(); i != alt->end(); ++i) {
            list.push_back(removeNullableAfterAssertion_helper(*i));
        }
        re = makeAlt(list.begin(), list.end());
    } 
    return re;
}

bool RE_Nullable::isNullable(const RE * re) {
    if (const Seq * re_seq = dyn_cast<const Seq>(re)) {
        for (const RE * re : *re_seq) {
            if (!isNullable(re)) {
                return false;
            }
        }
        return true;
    } else if (const Alt * re_alt = dyn_cast<const Alt>(re)) {
        for (const RE * re : *re_alt) {
            if (isNullable(re)) {
                return true;
            }
        }
    } else if (const Rep* re_rep = dyn_cast<const Rep>(re)) {
        return re_rep->getLB() == 0 ? true : isNullable(re_rep->getRE());
    } else if (const Diff * diff = dyn_cast<const Diff>(re)) {
        return isNullable(diff->getLH()) && !isNullable(diff->getRH());
    } else if (const Intersect * e = dyn_cast<const Intersect>(re)) {
        return isNullable(e->getLH()) && isNullable(e->getRH());
    } 
    return false;
}

bool RE_Nullable::hasNullablePrefix(const RE * re) {
    bool nullable = false;
    if (const Seq * seq = dyn_cast<const Seq>(re)) {
        nullable = isNullable(seq->front()) ? true : hasNullablePrefix(seq->front());
    } else if (const Alt * alt = dyn_cast<const Alt>(re)) {
        for (const RE * re : *alt) {
            if (hasNullablePrefix(re)) {
                nullable = true;
                break;
            }
        }
    } else if (const Rep * rep = dyn_cast<const Rep>(re)) {
        nullable = true;
        if (rep->getLB() == rep->getUB()) {
            nullable = hasNullablePrefix(rep->getRE());
        }
    }
    return nullable;
}

bool RE_Nullable::hasNullableSuffix(const RE * re) {
    bool nullable = false;
    if (const Seq * seq = dyn_cast<const Seq>(re)) {
        nullable = isNullable(seq->back()) ? true : hasNullableSuffix(seq->back());
    } else if (const Alt * alt = dyn_cast<const Alt>(re)) {
        for (const RE * re : *alt) {
            if (hasNullableSuffix(re)) {
                nullable = true;
                break;
            }
        }
    } else if (const Rep * rep = dyn_cast<const Rep>(re)) {
        nullable = true;
        if (rep->getLB() == rep->getUB()) {
            nullable = hasNullableSuffix(rep->getRE());
        }
    }
    return nullable;
}

}
