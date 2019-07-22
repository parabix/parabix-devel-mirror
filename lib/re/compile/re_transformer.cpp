/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/compile/re_transformer.h>

#include <llvm/Support/Casting.h>
#include <llvm/Support/ErrorHandling.h>
#include <llvm/Support/raw_ostream.h>
#include <re/adt/adt.h>
#include <re/toolchain/toolchain.h>

using namespace llvm;

namespace re {

void UndefinedNameError(const Name * n) {
    report_fatal_error("Error: Undefined name in regular expression: \"" + n->getName() + "\".");
}


RE * RE_Transformer::transformRE(RE * re) {
    RE * initialRE = re;
    RE * finalRE = transform(re);
    bool ShowRE = PrintOptionIsSet(ShowAllREs) && !mTransformationName.empty();
    if (PrintOptionIsSet(ShowREs) && (initialRE != finalRE)) { 
        ShowRE |= !mTransformationName.empty() && (mTransformationName[0] != '.');
    }
    if (ShowRE)  {
        errs() << mTransformationName << ":\n" << Printer_RE::PrintRE(finalRE) << '\n';
    }
    return finalRE;
}

RE * RE_Transformer::transform(RE * const from) { assert (from);
    using T = RE::ClassTypeId;
    RE * to = from;
    #define TRANSFORM(Type) \
        case T::Type: to = transform##Type(llvm::cast<Type>(from)); break
    switch (from->getClassTypeId()) {
        TRANSFORM(Alt);
        TRANSFORM(Assertion);
        TRANSFORM(CC);
        TRANSFORM(Range);
        TRANSFORM(Diff);
        TRANSFORM(End);
        TRANSFORM(Intersect);
        TRANSFORM(Name);
        TRANSFORM(Group);
        TRANSFORM(Rep);
        TRANSFORM(Seq);
        TRANSFORM(Start);
        default: llvm_unreachable("Unknown RE type");
    }
    #undef TRANSFORM
    assert (to);

    // Do we already have a memoized version of the transformed RE?
    if (from != to) {
        const auto f = mMap.find(to);
        if (LLVM_LIKELY(f == mMap.end())) {
            mMap.emplace(to, to);
        } else {
            to = f->second;
        }
    }

    return to;
}

RE * RE_Transformer::transformName(Name * nm) {
    if (mNameTransform == NameTransformationMode::None) {
        return nm;
    }
    RE * const defn = nm->getDefinition();
    if (LLVM_UNLIKELY(defn == nullptr)) {
        UndefinedNameError(nm);
    }
    RE * t = transform(defn);
    if (t == defn) return nm;
    return t;
}

RE * RE_Transformer::transformCC(CC * cc) {
    return cc;
}

RE * RE_Transformer::transformStart(Start * s) {
    return s;
}

RE * RE_Transformer::transformEnd(End * e) {
    return e;
}

RE * RE_Transformer::transformSeq(Seq * seq) {
    std::vector<RE *> elems;
    elems.reserve(seq->size());
    bool any_changed = false;
    for (RE * e : *seq) {
        RE * e1 = transform(e);
        if (e1 != e) any_changed = true;
        elems.push_back(e1);
    }
    if (!any_changed) return seq;
    return makeSeq(elems.begin(), elems.end());
}

RE * RE_Transformer::transformAlt(Alt * alt) {
    std::vector<RE *> elems;
    elems.reserve(alt->size());
    bool any_changed = false;
    for (RE * e : *alt) {
        RE * e1 = transform(e);
        if (e1 != e) any_changed = true;
        elems.push_back(e1);
    }
    if (!any_changed) return alt;
    return makeAlt(elems.begin(), elems.end());
}

RE * RE_Transformer::transformRep(Rep * r) {
    RE * x0 = r->getRE();
    RE * x = transform(x0);
    if (x == x0) {
        return r;
    } else {
        return makeRep(x, r->getLB(), r->getUB());
    }
}

RE * RE_Transformer::transformIntersect(Intersect * ix) {
    RE * x0 = ix->getLH();
    RE * y0 = ix->getRH();
    RE * x = transform(x0);
    RE * y = transform(y0);
    if ((x == x0) && (y == y0)) {
        return ix;
    } else {
        return makeIntersect(x, y);
    }
}

RE * RE_Transformer::transformDiff(Diff * d) {
    RE * x0 = d->getLH();
    RE * y0 = d->getRH();
    RE * x = transform(x0);
    RE * y = transform(y0);
    if ((x == x0) && (y == y0)) {
        return d;
    } else {
        return makeDiff(x, y);
    }
}

RE * RE_Transformer::transformRange(Range * rg) {
    RE * x0 = rg->getLo();
    RE * y0 = rg->getHi();
    RE * x = transform(x0);
    RE * y = transform(y0);
    if ((x == x0) && (y == y0)) {
        return rg;
    } else {
        return makeRange(x, y);
    }
}

RE * RE_Transformer::transformGroup(Group * g) {
    RE * x0 = g->getRE();
    RE * x = transform(x0);
    if (x == x0) {
        return g;
    } else {
        return makeGroup(g->getMode(), x, g->getSense());
    }
}

RE * RE_Transformer::transformAssertion(Assertion * a) {
    RE * x0 = a->getAsserted();
    RE * x = transform(x0);
    if (x == x0) {
        return a;
    } else {
        return makeAssertion(x, a->getKind(), a->getSense());
    }
}

} // namespace re
