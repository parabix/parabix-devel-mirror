#include "re_nullable.h"
#include <re/re_alt.h>             // for Alt, makeAlt
#include <re/re_any.h>             // for makeAny, Any
#include <re/re_assertion.h>       // for Assertion, Assertion::Sense, Asser...
#include <re/re_diff.h>            // for Diff, makeDiff
#include <re/re_intersect.h>       // for Intersect
#include <re/re_name.h>            // for Name
#include <re/re_rep.h>             // for Rep, makeRep
#include <re/re_seq.h>             // for Seq, makeSeq
#include <re/re_group.h>             // for Seq, makeSeq
#include <vector>                  // for vector, allocator
#include <llvm/Support/Casting.h>  // for dyn_cast, isa

/*

 A regular expression is nullable if it (a) matches the empty
 string, and (b) applies everywhere.  Note that Start (^) and
 End ($) match the empty string, but not everywhere).

*/

using namespace llvm;

namespace re {

    
RE * RE_Nullable::excludeNullable(RE * re) {
    if (!isNullable(re)) return re;
    if (Seq * seq = dyn_cast<Seq>(re)) {
        // All items in the seq must be nullable.  We must allow
        // all possibilities that all but one continue to match empty.
        std::vector<RE*> alts;
        for (auto i = 0; i < seq->size(); i++) {
            std::vector<RE*> list;
            for (auto j = 0; j < seq->size(); j++) {
                if (i == j) {
                    list.push_back(excludeNullable(&seq[j]));
                } else {
                    list.push_back(&seq[j]);
                }
            }
            alts.push_back(makeSeq(list.begin(), list.end()));
        }
        return makeAlt(alts.begin(), alts.end());
    } else if (Alt * alt = dyn_cast<Alt>(re)) {
        std::vector<RE*> list;
        for (auto i = alt->begin(); i != alt->end(); ++i) {
            list.push_back(excludeNullable(*i));
        }
        return makeAlt(list.begin(), list.end());
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
        auto lb = rep->getLB();
        auto ub = rep->getUB();
        auto e = rep->getRE();
        if (!isNullable(e)) {
            assert (lb == 0);  // because isNullable(re) is true
            return makeRep(e, 1, ub);
        }
        auto e1 = excludeNullable(e);
        if (ub == 1) {
            return e1;
        } else {
            return makeSeq({e1, makeRep(e, lb == 0 ? 0 : lb - 1, ub == Rep::UNBOUNDED_REP ? ub : ub - 1)});
        }
    } else if (Group * g = dyn_cast<Group>(re)) {
        return makeGroup(g->getMode(), excludeNullable(g->getRE()), g->getSense());
    } else if (Name * name = dyn_cast<Name>(re)) {
        return excludeNullable(name->getDefinition());
    }
    return re;
}

    
RE * RE_Nullable::removeNullablePrefix(RE * re) {
    if (!hasNullablePrefix(re)) return re;
    if (Seq * seq = dyn_cast<Seq>(re)) {
        std::vector<RE*> list;
        for (auto i = seq->begin(); i != seq->end(); ++i) {
            if (!isNullable(*i)) {
                list.push_back(removeNullablePrefix(*i));
                std::copy(++i, seq->end(), std::back_inserter(list));
                break;
            }
        }
        return makeSeq(list.begin(), list.end());
    } else if (Alt * alt = dyn_cast<Alt>(re)) {
        std::vector<RE*> list;
        for (auto i = alt->begin(); i != alt->end(); ++i) {
            list.push_back(removeNullablePrefix(*i));
        }
        return makeAlt(list.begin(), list.end());
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
        auto lb = rep->getLB();
        auto e = rep->getRE();
        if ((lb == 0) || isNullable(e)) {
            return makeSeq();
        }
        else if (hasNullablePrefix(e)) {
            return makeSeq({removeNullablePrefix(e), makeRep(e, lb - 1, lb - 1)});
        }
        else {
            return makeRep(e, lb, lb);
        }
    } else if (Name * name = dyn_cast<Name>(re)) {
        auto def = name->getDefinition();
        if (hasNullablePrefix(def)) {
            return removeNullablePrefix(def);
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
        return makeSeq(list.begin(), list.end());
    } else if (Alt* alt = dyn_cast<Alt>(re)) {
        std::vector<RE*> list;
        for (auto i = alt->begin(); i != alt->end(); ++i) {
            list.push_back(removeNullableSuffix(*i));
        }
        return makeAlt(list.begin(), list.end());
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
        auto lb = rep->getLB();
        auto e = rep->getRE();
        if ((lb == 0) || isNullable(e)) {
            return makeSeq();
        }
        else if (hasNullableSuffix(e)) {
            return makeSeq({makeRep(e, lb - 1, lb - 1), removeNullableSuffix(e)});
        }
        else {
            return makeRep(e, lb, lb);
        }
    } else if (Name * name = dyn_cast<Name>(re)) {
        auto def = name->getDefinition();
        if (hasNullableSuffix(def)) {
            return removeNullableSuffix(def);
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
        return (re_rep->getLB() == 0) || isNullable(re_rep->getRE());
    } else if (const Diff * diff = dyn_cast<const Diff>(re)) {
        // a Diff of Seq({}) and an Assertion represents a complemented assertion.
        return false;
    } else if (const Intersect * e = dyn_cast<const Intersect>(re)) {
        return isNullable(e->getLH()) && isNullable(e->getRH());
    } else if (const Group * g = dyn_cast<const Group>(re)) {
        return isNullable(g->getRE());
    }
    return false;
}

bool RE_Nullable::hasNullablePrefix(const RE * re) {
    if (const Seq * seq = dyn_cast<const Seq>(re)) {
        if (seq->empty()) return false;
        return isNullable(seq->front()) || hasNullablePrefix(seq->front());
    } else if (const Alt * alt = dyn_cast<const Alt>(re)) {
        for (const RE * a : *alt) {
            if (hasNullablePrefix(a)) {
                return true;
            }
        }
        return false;
    } else if (const Rep * rep = dyn_cast<const Rep>(re)) {
        return (rep->getLB() != rep->getUB()) || hasNullablePrefix(rep->getRE());
    } else if (const Group * g = dyn_cast<const Group>(re)) {
        return hasNullablePrefix(g->getRE());
    }
    return false;
}

bool RE_Nullable::hasNullableSuffix(const RE * re) {
    if (const Seq * seq = dyn_cast<const Seq>(re)) {
        if (seq->empty()) return false;
        return isNullable(seq->back()) || hasNullableSuffix(seq->back());
    } else if (const Alt * alt = dyn_cast<const Alt>(re)) {
        for (const RE * a : *alt) {
            if (hasNullableSuffix(a)) {
                return true;
            }
        }
        return false;
    } else if (const Rep * rep = dyn_cast<const Rep>(re)) {
        return (rep->getLB() != rep->getUB()) || hasNullableSuffix(rep->getRE());
    } else if (const Group * g = dyn_cast<const Group>(re)) {
        return hasNullableSuffix(g->getRE());
    }
    return false;
}

}
