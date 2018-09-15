/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "re_utility.h"
#include <re/re_any.h>
#include <re/re_name.h>
#include <re/re_cc.h>
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_alt.h>
#include <re/re_seq.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_group.h>
#include <re/re_range.h>
#include <re/re_assertion.h>
#include <re/printer_re.h>
#include <llvm/Support/Casting.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/ErrorHandling.h>

namespace re {
    
RE * makeComplement(RE * s) {
  return makeDiff(makeAny(), s);
}

                           
Name * makeDigitSet() {
    return makeName("nd", Name::Type::UnicodeProperty);
}

Name * makeAlphaNumeric() {
    return makeName("alnum", Name::Type::UnicodeProperty);
}

Name * makeWhitespaceSet() {
    return makeName("whitespace", Name::Type::UnicodeProperty);
}

Name * makeWordSet() {
    return makeName("word", Name::Type::UnicodeProperty);
}

RE * makeWordBoundary() {
    Name * wordC = makeWordSet();
    return makeAlt({makeSeq({makeNegativeLookBehindAssertion(wordC), makeLookAheadAssertion(wordC)}),
        makeSeq({makeLookBehindAssertion(wordC), makeNegativeLookAheadAssertion(wordC)})});
}

RE * makeWordNonBoundary() {
    Name * wordC = makeWordSet();
    return makeAlt({makeSeq({makeNegativeLookBehindAssertion(wordC), makeNegativeLookAheadAssertion(wordC)}),
        makeSeq({makeLookBehindAssertion(wordC), makeLookAheadAssertion(wordC)})});
}

RE * makeWordBegin() {
    Name * wordC = makeWordSet();
    return makeNegativeLookBehindAssertion(wordC);
}

RE * makeWordEnd() {
    Name * wordC = makeWordSet();
    return makeNegativeLookAheadAssertion(wordC);
}

RE * makeUnicodeBreak() {
    return makeAlt({makeCC(0x0A, 0x0C), makeCC(0x85), makeCC(0x2028,0x2029), makeSeq({makeCC(0x0D), makeNegativeLookAheadAssertion(makeCC(0x0A))})});
}
    
    
RE * RE_Transformer::transform(RE * re) {
    if (llvm::isa<CC>(re)) return transformCC(llvm::cast<CC>(re));
    else if (llvm::isa<Start>(re)) return transformStart(llvm::cast<Start>(re));
    else if (llvm::isa<End>(re)) return transformEnd(llvm::cast<End>(re));
    else if (llvm::isa<Name>(re)) return transformName(llvm::cast<Name>(re));
    else if (llvm::isa<Seq>(re)) return transformSeq(llvm::cast<Seq>(re));
    else if (llvm::isa<Alt>(re)) return transformAlt(llvm::cast<Alt>(re));
    else if (llvm::isa<Rep>(re)) return transformRep(llvm::cast<Rep>(re));
    else if (llvm::isa<Intersect>(re)) return transformIntersect(llvm::cast<Intersect>(re));
    else if (llvm::isa<Diff>(re)) return transformDiff(llvm::cast<Diff>(re));
    else if (llvm::isa<Range>(re)) return transformRange(llvm::cast<Range>(re));
    else if (llvm::isa<Group>(re)) return transformGroup(llvm::cast<Group>(re));
    else if (llvm::isa<Assertion>(re)) return transformAssertion(llvm::cast<Assertion>(re));
    else {
        llvm_unreachable("Unknown RE type");
        return nullptr;
    }
}
    
RE * RE_Transformer::transformName(Name * nm) {
    if (mNameTransform == NameTransformationMode::None) return nm;
    RE * d = nm->getDefinition();
    if (d) return transform(d);
    UndefinedNameError(nm);
    return nullptr;
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

}
