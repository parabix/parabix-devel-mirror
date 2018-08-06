/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <string>
#include <vector>
#include <locale>
#include <codecvt>
#include <re/Unicode/decomposition.h>
#include <re/re_cc.h>
#include <re/re_seq.h>
#include <re/re_alt.h>
#include <re/re_group.h>
#include <re/re_range.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_assertion.h>
#include <UCD/unicode_set.h>
#include <UCD/PropertyAliases.h>
#include <UCD/PropertyObjects.h>
#include <UCD/PropertyObjectTable.h>
#include <UCD/PropertyValueAliases.h>
#include <llvm/Support/Casting.h>

using namespace UCD;
using namespace llvm;
using namespace re;

// Constants for computation of Hangul decompositions, see Unicode Standard, section 3.12.
const codepoint_t Hangul_SBase = 0xAC00;
const codepoint_t Hangul_LBase = 0x1100;
const codepoint_t Hangul_VBase = 0x1161;
const codepoint_t Hangul_TBase = 0x11A7;
const unsigned Hangul_TCount = 28;
const unsigned Hangul_NCount = 588;
const unsigned Hangul_SCount = 11172;
static UnicodeSet HangulPrecomposed = UnicodeSet(Hangul_SBase, Hangul_SBase + Hangul_SCount - 1);

static RE * HangulDecomposition(codepoint_t cp) {
    auto SIndex = cp - Hangul_SBase;
    auto LIndex = SIndex / Hangul_NCount;
    auto VIndex = (SIndex % Hangul_NCount) / Hangul_TCount;
    auto TIndex = SIndex % Hangul_TCount;
    auto L = makeCC(Hangul_LBase + LIndex);
    auto V = makeCC(Hangul_VBase + VIndex);
    if (TIndex > 0) {
        return makeSeq({L, V, makeCC(Hangul_TBase + TIndex)});
    } else {
        return makeSeq({L, V});
    }
}

RE * NFD_CC(CC * cc) {
    std::wstring_convert<std::codecvt_utf8<char32_t>, char32_t> conv;
    const auto & decompMappingObj = cast<StringPropertyObject>(property_object_table[dm]);
    const auto & decompTypeObj = cast<EnumeratedPropertyObject>(property_object_table[dt]);
    UnicodeSet canonicalMapped = decompTypeObj->GetCodepointSet(DT_ns::Can);
    UnicodeSet mappingRequired = *cc & (canonicalMapped + HangulPrecomposed);
    if (mappingRequired.empty()) return cc;
    std::vector<RE *> alts;
    CC * finalCC = makeCC(*cc - mappingRequired);
    for (const interval_t & i : mappingRequired) {
        for (codepoint_t cp = lo_codepoint(i); cp <= hi_codepoint(i); cp++) {
            if (HangulPrecomposed.contains(cp)) {
                alts.push_back(HangulDecomposition(cp));
            } else {
                std::u32string dms = conv.from_bytes(decompMappingObj->GetStringValue(cp));
                RE * dm = u32string2re(dms);
                if (Seq * s = dyn_cast<Seq>(dm)) {
                    if (s->size() == 1) {
                        finalCC = makeCC(finalCC, cast<CC>(s->front()));
                    } else {
                        alts.push_back(s);
                    }
                } else {
                    alts.push_back(dm);
                }
            }
        }
    }
    if (!finalCC->empty()) alts.push_back(finalCC);
    return makeAlt(alts.begin(), alts.end());
}


RE * NFKD_CC(CC * cc) {
    std::wstring_convert<std::codecvt_utf8<char32_t>, char32_t> conv;
    const auto & decompMappingObj = cast<StringPropertyObject>(property_object_table[dm]);
    UnicodeSet reflexiveSet = decompMappingObj->GetReflexiveSet() - HangulPrecomposed;
    UnicodeSet mappingRequired = *cc - reflexiveSet;
    if (mappingRequired.empty()) return cc;
    std::vector<RE *> alts;
    CC * finalCC = makeCC(*cc - mappingRequired);
    for (const interval_t & i : mappingRequired) {
        for (codepoint_t cp = lo_codepoint(i); cp <= hi_codepoint(i); cp++) {
            if (HangulPrecomposed.contains(cp)) {
                alts.push_back(HangulDecomposition(cp));
            } else {
                std::u32string dms = conv.from_bytes(decompMappingObj->GetStringValue(cp));
                RE * dm = u32string2re(dms);
                if (Seq * s = dyn_cast<Seq>(dm)) {
                    if (s->size() == 1) {
                        finalCC = makeCC(finalCC, cast<CC>(s->front()));
                    } else {
                        alts.push_back(s);
                    }
                } else {
                    alts.push_back(dm);
                }
            }
        }
    }
    if (!finalCC->empty()) alts.push_back(finalCC);
    return makeAlt(alts.begin(), alts.end());
}

RE * Casefold_CC(CC * cc) {
    std::wstring_convert<std::codecvt_utf8<char32_t>, char32_t> conv;
    const auto & caseFoldObj = cast<StringOverridePropertyObject>(property_object_table[cf]);
    UnicodeSet reflexiveSet = caseFoldObj->GetReflexiveSet();
    UnicodeSet foldingRequired = *cc - reflexiveSet;
    if (foldingRequired.empty()) return cc;
    std::vector<RE *> alts;
    CC * finalCC = makeCC(*cc - foldingRequired);
    for (const interval_t & i : foldingRequired) {
        for (codepoint_t cp = lo_codepoint(i); cp <= hi_codepoint(i); cp++) {
            std::u32string dms = conv.from_bytes(caseFoldObj->GetStringValue(cp));
            RE * dm = u32string2re(dms);
            if (Seq * s = dyn_cast<Seq>(dm)) {
                if (s->size() == 1) {
                    finalCC = makeCC(finalCC, cast<CC>(s->front()));
                } else {
                    alts.push_back(s);
                }
            } else {
                alts.push_back(dm);
            }
        }
    }
    if (!finalCC->empty()) alts.push_back(finalCC);
    return makeAlt(alts.begin(), alts.end());
}

RE * NFD_RE(RE * re) {
    if (Alt * alt = dyn_cast<Alt>(re)) {
        std::vector<RE *> list;
        list.reserve(alt->size());
        for (RE * re : *alt) {
            list.push_back(NFD_RE(re));
        }
        return makeAlt(list.begin(), list.end());
    } else if (CC * cc = dyn_cast<CC>(re)) {
        return NFD_CC(cc);
    } else if (Seq * seq = dyn_cast<Seq>(re)) {
        std::vector<RE *> list;
        list.reserve(seq->size());
        for (RE * re : *seq) {
            list.push_back(NFD_RE(re));
        }
        return makeSeq(list.begin(), list.end());
    } else if (Assertion * a = dyn_cast<Assertion>(re)) {
        return makeAssertion(NFD_RE(a->getAsserted()), a->getKind(), a->getSense());
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
        RE * expr = NFD_RE(rep->getRE());
        return makeRep(expr, rep->getLB(), rep->getUB());
    } else if (Diff * diff = dyn_cast<Diff>(re)) {
        return makeDiff(NFD_RE(diff->getLH()), NFD_RE(diff->getRH()));
    } else if (Intersect * e = dyn_cast<Intersect>(re)) {
        return makeIntersect(NFD_RE(e->getLH()), NFD_RE(e->getRH()));
    } else if (Range * rg = dyn_cast<Range>(re)) {
        return makeRange(NFD_RE(rg->getLo()), NFD_RE(rg->getHi()));
    } else if (Group * g = dyn_cast<Group>(re)) {
        return makeGroup(g->getMode(), NFD_RE(g->getRE()), g->getSense());
    }
    return re;
}
    
RE * NFKD_RE(RE * re) {
    if (Alt * alt = dyn_cast<Alt>(re)) {
        std::vector<RE *> list;
        list.reserve(alt->size());
        for (RE * re : *alt) {
            list.push_back(NFKD_RE(re));
        }
        return makeAlt(list.begin(), list.end());
    } else if (CC * cc = dyn_cast<CC>(re)) {
        return NFKD_CC(cc);
    } else if (Seq * seq = dyn_cast<Seq>(re)) {
        std::vector<RE *> list;
        list.reserve(seq->size());
        for (RE * re : *seq) {
            list.push_back(NFKD_RE(re));
        }
        return makeSeq(list.begin(), list.end());
    } else if (Assertion * a = dyn_cast<Assertion>(re)) {
        return makeAssertion(NFKD_RE(a->getAsserted()), a->getKind(), a->getSense());
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
        RE * expr = NFKD_RE(rep->getRE());
        return makeRep(expr, rep->getLB(), rep->getUB());
    } else if (Diff * diff = dyn_cast<Diff>(re)) {
        return makeDiff(NFKD_RE(diff->getLH()), NFKD_RE(diff->getRH()));
    } else if (Intersect * e = dyn_cast<Intersect>(re)) {
        return makeIntersect(NFKD_RE(e->getLH()), NFKD_RE(e->getRH()));
    } else if (Range * rg = dyn_cast<Range>(re)) {
        return makeRange(NFKD_RE(rg->getLo()), NFKD_RE(rg->getHi()));
    } else if (Group * g = dyn_cast<Group>(re)) {
        return makeGroup(g->getMode(), NFKD_RE(g->getRE()), g->getSense());
    }
    return re;
}

RE * Casefold_RE(RE * re) {
    if (Alt * alt = dyn_cast<Alt>(re)) {
        std::vector<RE *> list;
        list.reserve(alt->size());
        for (RE * re : *alt) {
            list.push_back(Casefold_RE(re));
        }
        return makeAlt(list.begin(), list.end());
    } else if (CC * cc = dyn_cast<CC>(re)) {
        return Casefold_CC(cc);
    } else if (Seq * seq = dyn_cast<Seq>(re)) {
        std::vector<RE *> list;
        list.reserve(seq->size());
        for (RE * re : *seq) {
            list.push_back(Casefold_RE(re));
        }
        return makeSeq(list.begin(), list.end());
    } else if (Assertion * a = dyn_cast<Assertion>(re)) {
        return makeAssertion(Casefold_RE(a->getAsserted()), a->getKind(), a->getSense());
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
        RE * expr = Casefold_RE(rep->getRE());
        return makeRep(expr, rep->getLB(), rep->getUB());
    } else if (Diff * diff = dyn_cast<Diff>(re)) {
        return makeDiff(Casefold_RE(diff->getLH()), Casefold_RE(diff->getRH()));
    } else if (Intersect * e = dyn_cast<Intersect>(re)) {
        return makeIntersect(Casefold_RE(e->getLH()), Casefold_RE(e->getRH()));
    } else if (Range * rg = dyn_cast<Range>(re)) {
        return makeRange(Casefold_RE(rg->getLo()), Casefold_RE(rg->getHi()));
    } else if (Group * g = dyn_cast<Group>(re)) {
        return makeGroup(g->getMode(), Casefold_RE(g->getRE()), g->getSense());
    }
    return re;
}
