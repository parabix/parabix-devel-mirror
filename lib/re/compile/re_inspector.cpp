/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/compile/re_inspector.h>

#include <llvm/Support/Casting.h>
#include <re/adt/adt.h>

using namespace llvm;

namespace re {

void RE_Inspector::inspectRE(RE * re) {
    inspect(re);
}

void RE_Inspector::inspect(RE * const re) {
    assert (re);
    if (mIgnoreNonUnique == InspectionMode::IgnoreNonUnique) {
        if (mMap.count(re)) return;
        mMap.emplace(re);
    }
    using T = RE::ClassTypeId;
    #define INSPECT(Type) \
        case T::Type: inspect##Type(llvm::cast<Type>(re)); break
    switch (re->getClassTypeId()) {
        INSPECT(Alt);
        INSPECT(Assertion);
        INSPECT(CC);
        INSPECT(Range);
        INSPECT(Diff);
        INSPECT(End);
        INSPECT(Intersect);
        INSPECT(Name);
        INSPECT(Group);
        INSPECT(Rep);
        INSPECT(Seq);
        INSPECT(Start);
        default: llvm_unreachable("Unknown RE type");
    }
    #undef INSPECT
}

void RE_Inspector::inspectName(Name * nm) {
    RE * const d = nm->getDefinition();
    if (d) inspect(d);
}

void RE_Inspector::inspectCC(CC * cc) {

}

void RE_Inspector::inspectStart(Start * s) {

}

void RE_Inspector::inspectEnd(End * e) {

}

void RE_Inspector::inspectSeq(Seq * seq) {
    for (RE * e : *seq) {
        inspect(e);
    }
}

void RE_Inspector::inspectAlt(Alt * alt) {
    for (RE * e : *alt) {
        inspect(e);
    }
}

void RE_Inspector::inspectRep(Rep * r) {
    inspect(r->getRE());
}

void RE_Inspector::inspectIntersect(Intersect * ix) {
    inspect(ix->getLH());
    inspect(ix->getRH());
}

void RE_Inspector::inspectDiff(Diff * d) {
    inspect(d->getLH());
    inspect(d->getRH());
}

void RE_Inspector::inspectRange(Range * rg) {
    inspect(rg->getLo());
    inspect(rg->getHi());
}

void RE_Inspector::inspectGroup(Group * g) {
    inspect(g->getRE());
}

void RE_Inspector::inspectAssertion(Assertion * a) {
    inspect(a->getAsserted());
}

} // namespace re
