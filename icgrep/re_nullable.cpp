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

RE * RE_Nullable::removeNullablePrefix(RE* re) {
    if (Seq * re_seq = dynamic_cast<Seq*>(re)) {
        re = removeNullableSeqPrefix(re_seq);
    }
    else if (Alt * re_alt = dynamic_cast<Alt*>(re)) {
        Alt * new_alt = new Alt();
        for (RE * re : *re_alt) {
            new_alt->push_back(removeNullablePrefix(re));
        }
        re = new_alt;
    }
    else if (Rep * re_rep = dynamic_cast<Rep*>(re)) {
        if ((re_rep->getLB() == 0) || (isNullable(re_rep->getRE()))) {
            re = new Seq();
        }
        else if (hasNullablePrefix(re_rep->getRE())) {
            Vector seq;
            seq.push_back(removeNullablePrefix(re_rep->getRE()));
            seq.push_back(new Rep(re_rep->getRE(), re_rep->getLB() - 1, re_rep->getLB() - 1));
            re = RE_Simplifier::makeSeq(Seq::Normal, seq);
        }
        else {
            re = RE_Simplifier::makeRep(re_rep->getRE(), re_rep->getLB(), re_rep->getLB());
        }
    }
    return re;
}


inline Seq * RE_Nullable::removeNullableSeqPrefix(const Seq * seq) {
    Seq * new_seq = new Seq(seq->getType());
    if (!seq->empty()) {
        auto i = seq->begin();
        // find the first non-nullable prefix
        while (i != seq->end() && isNullable(*i)) {
            ++i;
        }
        if (i == seq->end()) {
            return new_seq;
        }
        // push the first non-nullable seq item to the front of the new_seq
        new_seq->push_back(removeNullablePrefix(*i));
        std::copy(++i, seq->end(), std::back_inserter(*new_seq));
    }
    return new_seq;
}

RE * RE_Nullable::removeNullableSuffix(RE * re) {
    if (Seq * re_seq = dynamic_cast<Seq*>(re)) {
        re = removeNullableSeqSuffix(re_seq);
    }
    else if (Alt* re_alt = dynamic_cast<Alt*>(re)) {
        Alt* new_alt = new Alt();
        for (RE * re : *re_alt) {
            new_alt->push_back(removeNullableSuffix(re));
        }
        re = new_alt;
    }
    else if (Rep * re_rep = dynamic_cast<Rep*>(re)) {
        if ((re_rep->getLB() == 0) || (isNullable(re_rep->getRE()))) {
            re = new Seq();
        }
        else if (hasNullableSuffix(re_rep->getRE())) {
            Vector seq;
            seq.push_back(new Rep(re_rep->getRE(), re_rep->getLB() - 1, re_rep->getLB() - 1));
            seq.push_back(removeNullableSuffix(re_rep->getRE()));
            re = RE_Simplifier::makeSeq(Seq::Normal, seq);
        }
        else {
            re = RE_Simplifier::makeRep(re_rep->getRE(), re_rep->getLB(), re_rep->getLB());
        }
    }
    return re;
}

inline Seq * RE_Nullable::removeNullableSeqSuffix(const Seq * seq) {
    Seq * new_seq = new Seq(seq->getType());
    if (!seq->empty()) {
        auto i = seq->end();
        // find the last non-nullable suffix
        while (i != seq->begin() && isNullable(*--i));

        if (i != seq->begin()) {
            std::copy(seq->begin(), i, std::back_inserter(*new_seq));
            new_seq->push_back(removeNullableSuffix(*i));
        }
    }
    return new_seq;
}

bool RE_Nullable::isNullable(const RE * re) {
    if (const Seq * re_seq = dynamic_cast<const Seq*>(re)) {
        return isNullableVector(re_seq);
    }
    else if (const Alt* re_alt = dynamic_cast<const Alt*>(re)) {
        return isNullableVector(re_alt);
    }
    else if (const Rep* re_rep = dynamic_cast<const Rep*>(re)) {
        return re_rep->getLB() == 0 ? true : isNullable(re_rep->getRE());
    }
    return false;
}

inline bool RE_Nullable::isNullableVector(const Vector * vec) {
    for (const RE * re : *vec) {
        if (!isNullable(re)) {
            return false;
        }
    }
    return true;
}

bool RE_Nullable::hasNullablePrefix(const RE * re) {
    bool nullable = false;
    if (const Seq * seq = dynamic_cast<const Seq*>(re)) {
        nullable = isNullable(seq->front()) ? true : hasNullablePrefix(seq->front());
    }
    else if (const Alt * alt = dynamic_cast<const Alt*>(re)) {
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
    else if (const Rep * rep = dynamic_cast<const Rep*>(re)) {
        nullable = hasNullablePrefix(rep->getRE());
    }
    return nullable;
}

bool RE_Nullable::hasNullableSuffix(const RE * re) {
    bool nullable = false;
    if (const Seq * seq = dynamic_cast<const Seq*>(re)) {
        nullable = isNullable(seq->back()) ? true : hasNullablePrefix(seq->back());
    }
    else if (const Alt * alt = dynamic_cast<const Alt*>(re)) {
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
    else if (const Rep * rep = dynamic_cast<const Rep*>(re)) {
        nullable = hasNullableSuffix(rep->getRE());
    }
    return nullable;
}



