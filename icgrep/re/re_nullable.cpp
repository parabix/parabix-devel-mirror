#include "re_nullable.h"
#include "re_cc.h"
#include "re_start.h"
#include "re_end.h"
#include "re_alt.h"
#include "re_rep.h"
#include "re_simplifier.h"

/*

 A regular expression is nullable if it (a) matches the empty
 string, and (b) applies everywhere.  Note that Start (^) and
 End ($) match the empty string, but not everywhere).

*/

namespace re {

RE * RE_Nullable::removeNullablePrefix(RE * re) {
    if (Seq * seq = dyn_cast<Seq>(re)) {
        re = removeNullablePrefix(seq);
    }
    else if (Alt * alt = dyn_cast<Alt>(re)) {
        for (auto i = alt->begin(); i != alt->end(); ++i) {
            *i = removeNullablePrefix(*i);
        }
        re = alt;
    }
    else if (Rep * rep = dyn_cast<Rep>(re)) {
        if ((rep->getLB() == 0) || (isNullable(rep->getRE()))) {
            re = makeSeq();
        }
        else if (hasNullablePrefix(rep->getRE())) {
            Seq * seq = makeSeq();
            seq->push_back(removeNullablePrefix(rep->getRE()->clone()));
            seq->push_back(makeRep(rep->getRE(), rep->getLB() - 1, rep->getLB() - 1));
            rep->setRE(nullptr);
            delete rep;
            re = RE_Simplifier::simplify(seq);
        }
        else {
            re = RE_Simplifier::simplify(rep);
        }
    }
    return re;
}

inline Seq * RE_Nullable::removeNullablePrefix(Seq * seq) {
    if (!seq->empty()) {
        std::vector<RE *> list;
        auto i = seq->begin();
        // find the first non-nullable prefix
        while (i != seq->end() && isNullable(*i)) {
            delete *i;
            ++i;
        }
        if (i != seq->end()) {
            // push the first non-nullable seq item to the front of the new_seq
            list.push_back(removeNullablePrefix(*i));
            std::copy(++i, seq->end(), std::back_inserter(list));
        }
        seq->swap(list);
    }
    return seq;
}

RE * RE_Nullable::removeNullableSuffix(RE * re) {
    if (Seq * seq = dyn_cast<Seq>(re)) {
        re = removeNullableSuffix(seq);
    }
    else if (Alt* alt = dyn_cast<Alt>(re)) {
        for (auto i = alt->begin(); i != alt->end(); ++i) {
            *i = removeNullableSuffix(*i);
        }
    }
    else if (Rep * rep = dyn_cast<Rep>(re)) {
        if ((rep->getLB() == 0) || (isNullable(rep->getRE()))) {
            delete rep;
            re = makeSeq();
        }
        else if (hasNullableSuffix(rep->getRE())) {
            Seq * seq = makeSeq();
            seq->push_back(RE_Simplifier::simplify(makeRep(rep->getRE()->clone(), rep->getLB() - 1, rep->getLB() - 1)));
            seq->push_back(removeNullableSuffix(rep->getRE()));
            rep->setRE(nullptr);
            delete rep;
            re = RE_Simplifier::simplify(seq);
        }
        else {
            re = RE_Simplifier::simplify(rep);
        }
    }
    return re;
}

inline Seq * RE_Nullable::removeNullableSuffix(Seq * seq) {
    if (!seq->empty()) {
        std::vector<RE *> list;
        auto i = seq->end();
        // find the last non-nullable suffix
        while (i != seq->begin() && isNullable(*--i)) {
            delete *i;
        }
        if (i != seq->begin()) {
            std::copy(seq->begin(), i, std::back_inserter(list));
            list.push_back(removeNullableSuffix(*i));
        }
        seq->swap(list);
    }
    return seq;
}

bool RE_Nullable::isNullable(const RE * re) {
    if (const Seq * re_seq = dyn_cast<const Seq>(re)) {
        return isNullable(re_seq);
    }
    else if (const Alt* re_alt = dyn_cast<const Alt>(re)) {
        return isNullable(re_alt);
    }
    else if (const Rep* re_rep = dyn_cast<const Rep>(re)) {
        return re_rep->getLB() == 0 ? true : isNullable(re_rep->getRE());
    }
    return false;
}

inline bool RE_Nullable::isNullable(const Vector * vec) {
    for (const RE * re : *vec) {
        if (!isNullable(re)) {
            return false;
        }
    }
    return true;
}

bool RE_Nullable::hasNullablePrefix(const RE * re) {
    bool nullable = false;
    if (const Seq * seq = dyn_cast<const Seq>(re)) {
        nullable = isNullable(seq->front()) ? true : hasNullablePrefix(seq->front());
    }
    else if (const Alt * alt = dyn_cast<const Alt>(re)) {
        if (!alt->empty()) {
            nullable = true;
            for (const RE * re : *alt) {
                if (!hasNullablePrefix(re)) {
                    nullable = false;
                    break;
                }
            }
        }
    }
    else if (const Rep * rep = dyn_cast<const Rep>(re)) {
        nullable = hasNullablePrefix(rep->getRE());
    }
    return nullable;
}

bool RE_Nullable::hasNullableSuffix(const RE * re) {
    bool nullable = false;
    if (const Seq * seq = dyn_cast<const Seq>(re)) {
        nullable = isNullable(seq->back()) ? true : hasNullableSuffix(seq->back());
    }
    else if (const Alt * alt = dyn_cast<const Alt>(re)) {
        if (!alt->empty()) {
            nullable = true;
            for (const RE * re : *alt) {
                if (!hasNullableSuffix(re)) {
                    nullable = false;
                    break;
                }
            }
        }
    }
    else if (const Rep * rep = dyn_cast<const Rep>(re)) {
        nullable = hasNullableSuffix(rep->getRE());
    }
    return nullable;
}

}
