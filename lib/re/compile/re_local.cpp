/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/compile/re_local.h>

#include<re/adt/adt.h>
#include <re/adt/nullable.h>
#include <re/compile/re_analysis.h>
#include <re/compile/re_transformer.h>
#include <boost/container/flat_map.hpp>
#include <boost/range/adaptor/reversed.hpp>

using namespace boost::container;
using namespace llvm;

namespace re {

using FollowMap = flat_map<const CC *, const CC*>;

inline const CC * combine(const CC * a, const CC * b) {
    if (a && b) {
        return makeCC(a, b);
    } else if (b) {
        return b;
    }
    return a;
}

const CC * first(const RE * re) {
    if (const Name * name = dyn_cast<Name>(re)) {
        if (LLVM_LIKELY(name->getDefinition() != nullptr)) {
            return first(name->getDefinition());
        } else {
            UndefinedNameError(name);
        }
    } else if (const CC * cc = dyn_cast<CC>(re)) {
        return cc;
    } else if (const Seq * seq = dyn_cast<Seq>(re)) {
        const CC * cc = nullptr;
        for (auto & si : *seq) {
            cc = combine(cc, first(si));
            if (!isNullable(si)) {
                break;
            }
        }
        return cc;
    } else if (const Alt * alt = dyn_cast<Alt>(re)) {
        const CC * cc = nullptr;
        for (auto & ai : *alt) {
            cc = combine(cc, first(ai));
        }
        return cc;
    } else if (const Rep * rep = dyn_cast<Rep>(re)) {
        return first(rep->getRE());
    } else if (const Diff * diff = dyn_cast<Diff>(re)) {
        if (const CC * lh = first(diff->getLH())) {
            if (const CC * rh = first(diff->getRH())) {
                return subtractCC(lh, rh);
            }
        }
    } else if (const Intersect * ix = dyn_cast<Intersect>(re)) {
        if (const CC * lh = first(ix->getLH())) {
            if (const CC * rh = first(ix->getRH())) {
                return intersectCC(lh, rh);
            }
        }
    }
    return nullptr;
}

const CC * final(const RE * re) {
    if (const Name * name = dyn_cast<Name>(re)) {
        if (LLVM_LIKELY(name->getDefinition() != nullptr)) {
            return final(name->getDefinition());
        } else {
            UndefinedNameError(name);
        }
    } else if (const CC * cc = dyn_cast<CC>(re)) {
        return cc;
    } else if (const Seq * seq = dyn_cast<Seq>(re)) {
        const CC * cc = nullptr;
        for (auto & si : boost::adaptors::reverse(*seq)) {
            cc = combine(cc, final(si));
            if (!isNullable(si)) {
                break;
            }
        }
        return cc;
    } else if (const Alt * alt = dyn_cast<Alt>(re)) {
        const CC * cc = nullptr;
        for (auto & ai : *alt) {
            cc = combine(cc, final(ai));
        }
        return cc;
    } else if (const Rep * rep = dyn_cast<Rep>(re)) {
        return final(rep->getRE());
    } else if (const Diff * diff = dyn_cast<Diff>(re)) {
        if (const CC * lh = final(diff->getLH())) {
            if (const CC * rh = final(diff->getRH())) {
                return subtractCC(lh, rh);
            }
        }
    } else if (const Intersect * ix = dyn_cast<Intersect>(re)) {
        if (const CC * lh = final(ix->getLH())) {
            if (const CC * rh = final(ix->getRH())) {
                return intersectCC(lh, rh);
            }
        }
    }
    return nullptr;

}

void follow(const RE * re, FollowMap & follows) {
    if (const Name * name = dyn_cast<Name>(re)) {
        if (LLVM_LIKELY(name->getDefinition() != nullptr)) {
            return follow(name->getDefinition(), follows);
        } else {
            UndefinedNameError(name);
        }
    } else if (const Seq * seq = dyn_cast<Seq>(re)) {
        if (!seq->empty()) {
            const RE * const re_first = *(seq->begin());
            const RE * const re_follow = makeSeq(seq->begin() + 1, seq->end());
            auto e1 = final(re_first);
            auto e2 = first(re_follow);
            if (e1 && e2) {
                auto e = follows.find(e1);
                if (e != follows.end()) {
                    e->second = makeCC(e->second, e2);
                } else {
                    follows.emplace(e1, e2);
                }
            }
            follow(re_first, follows);
            follow(re_follow, follows);
        }
    } else if (const Alt * alt = dyn_cast<Alt>(re)) {
        for (auto ai = alt->begin(); ai != alt->end(); ++ai) {
            follow(*ai, follows);
        }
    } else if (const Rep * rep = dyn_cast<Rep>(re)) {
        const auto e1 = final(rep->getRE());
        auto e2 = first(rep->getRE());
        if (e1 && e2) {
            auto e = follows.find(e1);
            if (e != follows.end()) {
                e->second = makeCC(e->second, e2);
            } else {
                follows.emplace(e1, e2);
            }
        }
        follow(rep->getRE(), follows);
    }
}

CC * RE_Local::getFirstUniqueSymbol(RE * const re) {
    const CC * const re_first = first(re);
    if (re_first) {
        FollowMap follows;
        follow(re, follows);
        for (const auto & entry : follows) {
            if (entry.second->intersects(*re_first)) {
                return nullptr;
            }
        }
    }
    return const_cast<CC *>(re_first);
}

}
