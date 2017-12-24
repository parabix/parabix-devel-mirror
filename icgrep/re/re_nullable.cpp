#include "re_nullable.h"
#include <re/re_alt.h>             // for Alt, makeAlt
#include <re/re_any.h>             // for makeAny, Any
#include <re/re_assertion.h>       // for Assertion, Assertion::Sense, Asser...
#include <re/re_diff.h>            // for Diff, makeDiff
#include <re/re_intersect.h>       // for Intersect
#include <re/re_name.h>            // for Name
#include <re/re_rep.h>             // for Rep, makeRep
#include <re/re_seq.h>             // for Seq, makeSeq
#include <vector>                  // for vector, allocator
#include <llvm/Support/Casting.h>  // for dyn_cast, isa

/*

 A regular expression is nullable if it (a) matches the empty
 string, and (b) applies everywhere.  Note that Start (^) and
 End ($) match the empty string, but not everywhere).

*/

using namespace llvm;

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
