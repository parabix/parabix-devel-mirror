/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <toolchain/toolchain.h>
#include <grep_interface.h>
#include <re/re_toolchain.h>
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
#include <cc/cc_compiler.h>            // for CC_Compiler
#include <llvm/Support/CommandLine.h>  // for clEnumVal, clEnumValEnd, Optio...
#include <re/re_compiler.h>            // for RE_Compiler
#include <re/re_nullable.h>            // for RE_Nullable
#include <re/re_star_normal.h>         // for RE_Star_Normal
#include <re/re_simplifier.h>          // for RE_Simplifier
#include <re/re_minimizer.h>
#include <re/re_local.h>
#include <re/printer_re.h>
#include <re/re_analysis.h>
#include <re/re_cc.h>
#include <re/casing.h>
#include <re/exclude_CC.h>
#include <re/re_name_resolve.h>

#include <re/grapheme_clusters.h>
#include <re/validation.h>
#include <re/Unicode/decomposition.h>
#include <re/Unicode/equivalence.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/ErrorHandling.h>
#include <toolchain/toolchain.h>

using namespace pablo;
using namespace llvm;

namespace re {

static cl::OptionCategory RegexOptions("Regex Toolchain Options",
                                              "These options control the regular expression transformation and compilation.");
const cl::OptionCategory * LLVM_READONLY re_toolchain_flags() {
    return &RegexOptions;
}

static cl::bits<RE_PrintFlags> 
    PrintOptions(cl::values(clEnumVal(ShowREs, "Show parsed regular expressions and transformations that change them"),
                            clEnumVal(ShowAllREs, "Print all regular expression passes")
                            CL_ENUM_VAL_SENTINEL), cl::cat(RegexOptions));

static cl::bits<RE_AlgorithmFlags>
    AlgorithmOptions(cl::values(clEnumVal(DisableLog2BoundedRepetition, "disable log2 optimizations for bounded repetition of bytes"),
                              clEnumVal(DisableIfHierarchy, "disable nested if hierarchy for generated Unicode classes (not recommended)"), 
                              clEnumVal(DisableMatchStar, "disable MatchStar optimization"), 
                              clEnumVal(DisableUnicodeMatchStar, "disable Unicode MatchStar optimization"),
                              clEnumVal(DisableUnicodeLineBreak, "disable Unicode line breaks - use LF only")
                              CL_ENUM_VAL_SENTINEL), cl::cat(RegexOptions));

    
static cl::opt<bool> UnicodeLevel2("U2", cl::desc("Enable Unicode Level matching under canonical and compatible (?K) equivalence."), cl::cat(RegexOptions));

bool LLVM_READONLY PrintOptionIsSet(RE_PrintFlags flag) {
    return PrintOptions.isSet(flag);
}

bool LLVM_READONLY AlgorithmOptionIsSet(RE_AlgorithmFlags flag) {
    return AlgorithmOptions.isSet(flag);
}

int IfInsertionGap;
static cl::opt<int, true> 
    IfInsertionGapOption("if-insertion-gap",  cl::location(IfInsertionGap), cl::init(3),
                         cl::desc("minimum number of nonempty elements between inserted if short-circuit tests"), 
                         cl::cat(RegexOptions));

RE * resolveModesAndExternalSymbols(RE * r, bool globallyCaseInsensitive) {
    if (PrintOptions.isSet(ShowAllREs) || PrintOptions.isSet(ShowREs)) {
        errs() << "Parser:\n" << Printer_RE::PrintRE(r) << '\n';
    }
    r = resolveGraphemeMode(r, false /* not in grapheme mode at top level*/);
    r = re::resolveUnicodeNames(r);
    validateNamesDefined(r);
    if (UnicodeLevel2 && validateAlphabet(&cc::Unicode, r)) {
        r = UCD::transform(r);
        r = UCD::addClusterMatches(r);
        r = UCD::addEquivalentCodepoints(r);
    } else {
        r = resolveCaseInsensitiveMode(r, globallyCaseInsensitive);
    }
    return r;
}

RE * excludeUnicodeLineBreak(RE * r) {
    r = exclude_CC(r, re::makeCC(re::makeCC(0x0A, 0x0D), re::makeCC(re::makeCC(0x85), re::makeCC(0x2028, 0x2029))));
    if (PrintOptions.isSet(ShowAllREs)) {
        errs() << "excludeUnicodeLineBreak:\n" << Printer_RE::PrintRE(r) << '\n';
    }
    return r;
}

RE * regular_expression_passes(RE * re) {

    //Optimization passes to simplify the AST.
    RE * r = removeNullablePrefix(re);
    r = removeNullableSuffix(r);
    r = RE_Star_Normal().transformRE(r);
    if (codegen::OptLevel > 1) {
        r = RE_Minimizer::minimize(r);
    } else {
        r = RE_Simplifier::simplify(r);
    }
    if (!DefiniteLengthBackReferencesOnly(r)) {
        llvm::report_fatal_error("Future back reference support: references must be within a fixed distance from a fixed-length capture.");
    }
    return r;
}

static bool compare(const RE * const lh, const RE * const rh);

static bool lessThan(const Vector * const lh, const Vector * const rh) {
    assert (lh->getClassTypeId() == rh->getClassTypeId());
    assert (lh->getClassTypeId() == RE::ClassTypeId::Alt || lh->getClassTypeId() == RE::ClassTypeId::Seq);
    if (LLVM_LIKELY(lh->size() != rh->size())) {
        return lh->size() < rh->size();
    }
    for (auto i = lh->begin(), j = rh->begin(); i != lh->end(); ++i, ++j) {
        assert (*i && *j);
        if (compare(*i, *j)) {
            return true;
        } else if (compare(*j, *i)) {
            return false;
        }
    }
    return false;
}

inline bool lessThan(const Name * const lh, const Name * const rh) {
    if (lh->getType() != rh->getType()) {
        return lh->getType() < rh->getType();
    } else if (lh->hasNamespace() != rh->hasNamespace()) {
        return lh->hasNamespace();
    } else if (lh->hasNamespace() && (lh->getNamespace() != rh->getNamespace())) {
        return lh->getNamespace() < rh->getNamespace();
    } else if (lh->getName() != rh->getName()) {
        return lh->getName() < rh->getName();
    } else if (lh->getDefinition() == nullptr) {
        return rh->getDefinition() != nullptr;
    } else if (rh->getDefinition() == nullptr) {
        return false;
    } else {
        return compare(lh->getDefinition(), rh->getDefinition());
    }
}

inline bool lessThan(const Assertion * const lh, const Assertion * const rh) {
    if (lh->getKind() != rh->getKind()) {
        return lh->getKind() < rh->getKind();
    }
    if (lh->getSense() != rh->getSense()) {
        return lh->getSense() < rh->getSense();
    }
    return compare(lh->getAsserted(), rh->getAsserted());
}

inline bool lessThan(const Rep * const lh, const Rep * const rh) {
    if (lh->getLB() != rh->getLB()) {
        return lh->getLB() < rh->getLB();
    }
    if (lh->getUB() != rh->getUB()) {
        return lh->getUB() < rh->getUB();
    }
    return compare(lh->getRE(), rh->getRE());
}

inline bool lessThan(const Diff * const lh, const Diff * const rh) {
    if (compare(lh->getLH(), rh->getLH())) {
        return true;
    } else if (compare(rh->getLH(), lh->getLH())) {
        return false;
    } else if (compare(lh->getRH(), rh->getRH())) {
        return true;
    } else {
        return !compare(rh->getRH(), lh->getRH());
    }
}

inline bool lessThan(const Range * const lh, const Range * const rh) {
    if (compare(lh->getLo(), rh->getLo())) {
        return true;
    } else if (compare(rh->getLo(), lh->getLo())) {
        return false;
    } else if (compare(lh->getHi(), rh->getHi())) {
        return true;
    } else {
        return !compare(rh->getHi(), lh->getHi());
    }
}

static bool lessThan(const Intersect * const lh, const Intersect * const rh) {
    if (compare(lh->getLH(), rh->getLH())) {
        return true;
    } else if (compare(rh->getLH(), lh->getLH())) {
        return false;
    } else if (compare(lh->getRH(), rh->getRH())) {
        return true;
    } else {
        return !compare(rh->getRH(), lh->getRH());
    }
}

inline bool lessThan(const Group * const lh, const Group * const rh) {
    if (lh->getMode() != rh->getMode()) {
        return lh->getMode() < rh->getMode();
    }
    if (lh->getSense() != rh->getSense()) {
        return lh->getSense() < rh->getSense();
    }
    return compare(lh->getRE(), rh->getRE());
}

inline bool compare(const RE * const lh, const RE * const rh) {
    using Type = RE::ClassTypeId;
    assert (lh && rh);
    const auto typeL = lh->getClassTypeId();
    const auto typeR = rh->getClassTypeId();
    if (LLVM_LIKELY(typeL != typeR)) {
        return typeL < typeR;
    }
    switch (typeL) {
        case Type::Alt:
        case Type::Seq:
            return lessThan(cast<Vector>(lh), cast<Vector>(rh));
        case Type::End: case Type::Start:
            return false;
        case Type::Assertion:
            return lessThan(cast<Assertion>(lh), cast<Assertion>(rh));
        case Type::CC:
            return *cast<CC>(lh) < *cast<CC>(rh);
        case Type::Name:
            return lessThan(cast<Name>(lh), cast<Name>(rh));
        case Type::Group:
            return lessThan(cast<Group>(lh), cast<Group>(rh));
        case Type::Range:
            return lessThan(cast<Range>(lh), cast<Range>(rh));
        case Type::Diff:
            return lessThan(cast<Diff>(lh), cast<Diff>(rh));
        case Type::Intersect:
            return lessThan(cast<Intersect>(lh), cast<Intersect>(rh));
        case Type::Rep:
            return lessThan(cast<Rep>(lh), cast<Rep>(rh));
        default:
            llvm_unreachable("RE object of unknown type given to Memoizer");
            return false;
    }
}

inline bool RE_Transformer::MemoizerComparator::operator()(const RE * const lh, const RE * const rh) const {
    return compare(lh, rh);
}

RE * RE_Transformer::transformRE(RE * re) {
    RE * initialRE = re;
    RE * finalRE = transform(re);
    if ((!mTransformationName.empty()) && (PrintOptions.isSet(ShowAllREs) || (PrintOptions.isSet(ShowREs) && (initialRE != finalRE))))  {
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
    RE * const d = nm->getDefinition();
    if (LLVM_UNLIKELY(d == nullptr)) {
        UndefinedNameError(nm);
    }
    return transform(d);
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

inline bool RE_Inspector::MemoizerComparator::operator()(const RE * const lh, const RE * const rh) const {
    return compare(lh, rh);
}

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

}
