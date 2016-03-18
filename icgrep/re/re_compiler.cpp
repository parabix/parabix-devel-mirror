/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#include <re/re_compiler.h>
//Regular Expressions
#include <re/re_name.h>
#include <re/re_any.h>
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_alt.h>
#include <re/re_cc.h>
#include <re/re_seq.h>
#include <re/re_rep.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_assertion.h>
#include <re/re_grapheme_boundary.hpp>
#include <re/re_analysis.h>
#include <re/re_memoizer.hpp>
#include <re/printer_re.h>
#include <pablo/codegenstate.h>
#include <UCD/ucd_compiler.hpp>
#include <UCD/resolve_properties.h>
#ifndef DISABLE_PREGENERATED_UCD_FUNCTIONS
#include <UCD/precompiled_properties.h>
#endif
#include <assert.h>
#include <stdexcept>
#include <iostream>
#include <pablo/printer_pablos.h>
#include "llvm/Support/CommandLine.h"
#include <sstream>
#include <unordered_set>

static cl::OptionCategory fREcompilationOptions("Regex Compilation Options", "These options control the compilation of regular expressions to Pablo.");
static cl::opt<bool> InvertMatches("v", cl::init(false),
                     cl::desc("select non-matching lines"), cl::cat(fREcompilationOptions));
static cl::alias InvertMatchesLong("invert-matches", cl::desc("Alias for -v"), cl::aliasopt(InvertMatches));

static cl::opt<bool> DisableLog2BoundedRepetition("disable-log2-bounded-repetition", cl::init(false),
                     cl::desc("disable log2 optimizations for bounded repetition of bytes"), cl::cat(fREcompilationOptions));
static cl::opt<bool> DisableIfHierarchy("disable-if-hierarchy-strategy", cl::init(false),
                     cl::desc("disable nested if hierarchy for generated Unicode classes (not recommended)"), cl::cat(fREcompilationOptions));
static cl::opt<int> IfInsertionGap("if-insertion-gap", cl::init(3), cl::desc("minimum number of nonempty elements between inserted if short-circuit tests"), cl::cat(fREcompilationOptions));
static cl::opt<bool> DisableMatchStar("disable-matchstar", cl::init(false),
                     cl::desc("disable MatchStar optimization"), cl::cat(fREcompilationOptions));
static cl::opt<bool> DisableUnicodeMatchStar("disable-unicode-matchstar", cl::init(false),
                     cl::desc("disable Unicode MatchStar optimization"), cl::cat(fREcompilationOptions));
static cl::opt<bool> DisableUnicodeLineBreak("disable-unicode-linebreak", cl::init(false),
                     cl::desc("disable Unicode line breaks - use LF only"), cl::cat(fREcompilationOptions));

#ifndef DISABLE_PREGENERATED_UCD_FUNCTIONS
static cl::opt<bool> UsePregeneratedUnicode("use-pregenerated-unicode", cl::init(false),
                     cl::desc("use fixed pregenerated Unicode character class sets instead"), cl::cat(fREcompilationOptions));
#endif

#define UNICODE_LINE_BREAK (!DisableUnicodeLineBreak)

using namespace pablo;

namespace re {

void RE_Compiler::initializeRequiredStreams() {

    Assign * LF = mPB.createAssign("LF", mCCCompiler.compileCC(makeCC(0x0A)));
    mLineFeed = LF;
    PabloAST * CR = mCCCompiler.compileCC(makeCC(0x0D));
    PabloAST * LF_VT_FF_CR = mCCCompiler.compileCC(makeCC(0x0A, 0x0D));

    PabloBuilder crb = PabloBuilder::Create(mPB);
    PabloAST * cr1 = crb.createAdvance(CR, 1, "cr1");
    Assign * acrlf = crb.createAssign("crlf", crb.createAnd(cr1, LF));
    mPB.createIf(CR, {acrlf}, crb);
    mCRLF = acrlf;

    PabloAST * u8pfx = mCCCompiler.compileCC(makeCC(0xC0, 0xFF));
    PabloBuilder it = PabloBuilder::Create(mPB);
    PabloAST * u8pfx2 = mCCCompiler.compileCC(makeCC(0xC2, 0xDF), it);
    PabloAST * u8pfx3 = mCCCompiler.compileCC(makeCC(0xE0, 0xEF), it);
    PabloAST * u8pfx4 = mCCCompiler.compileCC(makeCC(0xF0, 0xF4), it);
    Assign * u8suffix = it.createAssign("u8suffix", mCCCompiler.compileCC(makeCC(0x80, 0xBF)));

    //
    // Two-byte sequences
    PabloBuilder it2 = PabloBuilder::Create(it);
    Assign * u8scope22 = it2.createAssign("u8scope22", it2.createAdvance(u8pfx2, 1));
    Assign * NEL = it2.createAssign("NEL", it2.createAnd(it2.createAdvance(mCCCompiler.compileCC(makeCC(0xC2), it2), 1), mCCCompiler.compileCC(makeCC(0x85), it2)));
    it.createIf(u8pfx2, {u8scope22, NEL}, it2);

    //
    // Three-byte sequences
    PabloBuilder it3 = PabloBuilder::Create(it);
    Assign * u8scope32 = it3.createAssign("u8scope32", it3.createAdvance(u8pfx3, 1));
    PabloAST * u8scope33 = it3.createAdvance(u8pfx3, 2);
    Assign * u8scope3X = it3.createAssign("u8scope3X", it3.createOr(u8scope32, u8scope33));
    PabloAST * E2_80 = it3.createAnd(it3.createAdvance(mCCCompiler.compileCC(makeCC(0xE2), it3), 1), mCCCompiler.compileCC(makeCC(0x80), it3));
    Assign * LS_PS = it3.createAssign("LS_PS", it3.createAnd(it3.createAdvance(E2_80, 1), mCCCompiler.compileCC(makeCC(0xA8,0xA9), it3)));
    PabloAST * E0_invalid = it3.createAnd(it3.createAdvance(mCCCompiler.compileCC(makeCC(0xE0), it3), 1), mCCCompiler.compileCC(makeCC(0x80, 0x9F), it3));
    PabloAST * ED_invalid = it3.createAnd(it3.createAdvance(mCCCompiler.compileCC(makeCC(0xED), it3), 1), mCCCompiler.compileCC(makeCC(0xA0, 0xBF), it3));
    Assign * EX_invalid = it3.createAssign("EX_invalid", it3.createOr(E0_invalid, ED_invalid));
    it.createIf(u8pfx3, {u8scope32, u8scope3X, LS_PS, EX_invalid}, it3);

    //
    // Four-byte sequences
    PabloBuilder it4 = PabloBuilder::Create(it);
    PabloAST * u8scope42 = it4.createAdvance(u8pfx4, 1, "u8scope42");
    PabloAST * u8scope43 = it4.createAdvance(u8scope42, 1, "u8scope43");
    PabloAST * u8scope44 = it4.createAdvance(u8scope43, 1, "u8scope44");
    Assign * u8scope4nonfinal = it4.createAssign("u8scope4nonfinal", it4.createOr(u8scope42, u8scope43));
    Assign * u8scope4X = it4.createAssign("u8scope4X", it4.createOr(u8scope4nonfinal, u8scope44));
    PabloAST * F0_invalid = it4.createAnd(it4.createAdvance(mCCCompiler.compileCC(makeCC(0xF0), it4), 1), mCCCompiler.compileCC(makeCC(0x80, 0x8F), it4));
    PabloAST * F4_invalid = it4.createAnd(it4.createAdvance(mCCCompiler.compileCC(makeCC(0xF4), it4), 1), mCCCompiler.compileCC(makeCC(0x90, 0xBF), it4));
    Assign * FX_invalid = it4.createAssign("FX_invalid", it4.createOr(F0_invalid, F4_invalid));
    it.createIf(u8pfx4, {u8scope4nonfinal, u8scope4X, FX_invalid}, it4);

    //
    // Invalid cases
    PabloAST * anyscope = it.createOr(u8scope22, it.createOr(u8scope3X, u8scope4X));
    PabloAST * legalpfx = it.createOr(it.createOr(u8pfx2, u8pfx3), u8pfx4);
    //  Any scope that does not have a suffix byte, and any suffix byte that is not in
    //  a scope is a mismatch, i.e., invalid UTF-8.
    PabloAST * mismatch = it.createXor(anyscope, u8suffix);
    //
    PabloAST * EF_invalid = it.createOr(EX_invalid, FX_invalid);
    PabloAST * pfx_invalid = it.createXor(u8pfx, legalpfx);
    Assign * u8invalid = it.createAssign("u8invalid", it.createOr(pfx_invalid, it.createOr(mismatch, EF_invalid)));
    Assign * u8valid = it.createAssign("u8valid", it.createNot(u8invalid));
    //
    //

    Assign * valid_pfx = it.createAssign("valid_pfx", it.createAnd(u8pfx, u8valid));
    mNonFinal = it.createAssign("nonfinal", it.createAnd(it.createOr(it.createOr(u8pfx, u8scope32), u8scope4nonfinal), u8valid));

    Assign * NEL_LS_PS = it.createAssign("NEL_LS_PS", it.createOr(NEL, LS_PS));
    mPB.createIf(u8pfx, {u8invalid, valid_pfx, mNonFinal, NEL_LS_PS}, it);

    PabloAST * LB_chars = mPB.createOr(LF_VT_FF_CR, NEL_LS_PS);
    PabloAST * u8single = mPB.createAnd(mCCCompiler.compileCC(makeCC(0x00, 0x7F)), mPB.createNot(u8invalid));
    mInitial = mPB.createOr(u8single, valid_pfx, "initial");
    mFinal = mPB.createNot(mPB.createOr(mNonFinal, u8invalid), "final");
    mUnicodeLineBreak = mPB.createAnd(LB_chars, mPB.createNot(mCRLF));  // count the CR, but not CRLF
    PabloAST * const lb = UNICODE_LINE_BREAK ? mUnicodeLineBreak : mLineFeed;
    mAny = mPB.createNot(lb, "any");
    mFunction.setResult(1, mPB.createAssign("lf", mPB.createAnd(lb, mPB.createNot(mCRLF))));
}

static inline CC * getDefinitionIfCC(RE * re) {
    if (LLVM_LIKELY(isa<Name>(re))) {
        Name * name = cast<Name>(re);
        if (name->getDefinition() && isa<CC>(name->getDefinition())) {
            return cast<CC>(name->getDefinition());
        }
    }
    return nullptr;
}

RE * RE_Compiler::resolveUnicodeProperties(RE * re) {

    Memoizer memoizer;
    Name * graphemeClusterRule = nullptr;

    std::function<RE*(RE*)> resolve = [&](RE * re) -> RE * {
        if (Name * name = dyn_cast<Name>(re)) {
            auto f = memoizer.find(name);
            if (f == memoizer.end()) {
                if (LLVM_LIKELY(name->getDefinition() != nullptr)) {
                    name->setDefinition(resolve(name->getDefinition()));
                } else if (LLVM_LIKELY(name->getType() == Name::Type::UnicodeProperty)) {
                    if (UCD::resolvePropertyDefinition(name)) {
                        resolve(name->getDefinition());
                    } else {
                        #ifndef DISABLE_PREGENERATED_UCD_FUNCTIONS
                        if (UsePregeneratedUnicode) {
                            const std::string functionName = UCD::resolvePropertyFunction(name);
                            const UCD::ExternalProperty & ep = UCD::resolveExternalProperty(functionName);
                            Call * call = mPB.createCall(Prototype::Create(functionName, std::get<1>(ep), std::get<2>(ep), std::get<0>(ep)), mCCCompiler.getBasisBits());
                            name->setCompiled(call);
                        } else {
                        #endif
                            name->setDefinition(makeCC(std::move(UCD::resolveUnicodeSet(name))));
                        #ifndef DISABLE_PREGENERATED_UCD_FUNCTIONS
                        }
                        #endif
                    }
                } else {
                    throw std::runtime_error("All non-unicode-property Name objects should have been defined prior to Unicode property resolution.");
                }
            } else {
                return *f;
            }
        } else if (Seq * seq = dyn_cast<Seq>(re)) {
            for (auto si = seq->begin(); si != seq->end(); ++si) {
                *si = resolve(*si);
            }
        } else if (Alt * alt = dyn_cast<Alt>(re)) {
            CC * unionCC = nullptr;
            std::stringstream name;
            for (auto ai = alt->begin(); ai != alt->end(); ) {
                RE * re = resolve(*ai);
                if (CC * cc = getDefinitionIfCC(re)) {
                    if (unionCC == nullptr) {
                        unionCC = cc;
                    } else {
                        unionCC = makeCC(unionCC, cc);
                        name << '+';
                    }
                    Name * n = cast<Name>(re);
                    if (n->hasNamespace()) {
                        name << n->getNamespace() << ':';
                    }
                    name << n->getName();
                    ai = alt->erase(ai);
                } else {
                    *ai++ = re;
                }
            }
            if (unionCC) {
                alt->push_back(makeName(name.str(), unionCC));
            }
            if (alt->size() == 1) {
                return alt->front();
            }
        } else if (Rep * rep = dyn_cast<Rep>(re)) {
            rep->setRE(resolve(rep->getRE()));
        } else if (Assertion * a = dyn_cast<Assertion>(re)) {
            a->setAsserted(resolve(a->getAsserted()));
        } else if (Diff * diff = dyn_cast<Diff>(re)) {
            diff->setLH(resolve(diff->getLH()));
            diff->setRH(resolve(diff->getRH()));
            CC * lh = getDefinitionIfCC(diff->getLH());
            CC * rh = getDefinitionIfCC(diff->getRH());
            if (lh && rh) {
                return resolve(makeName("diff", subtractCC(lh, rh)));
            }
        } else if (Intersect * ix = dyn_cast<Intersect>(re)) {
            ix->setLH(resolve(ix->getLH()));
            ix->setRH(resolve(ix->getRH()));
            CC * lh = getDefinitionIfCC(ix->getLH());
            CC * rh = getDefinitionIfCC(ix->getRH());
            if (lh && rh) {
                return resolve(makeName("intersect", intersectCC(lh, rh)));
            }
        } else if (GraphemeBoundary * gb = dyn_cast<GraphemeBoundary>(re)) {
            if (LLVM_LIKELY(gb->getBoundaryRule() == nullptr)) {
                switch (gb->getType()) {
                    case GraphemeBoundary::Type::ClusterBoundary:
                        if (graphemeClusterRule == nullptr) {
                            graphemeClusterRule = cast<Name>(resolve(generateGraphemeClusterBoundaryRule()));
                        }
                        gb->setBoundaryRule(graphemeClusterRule);
                        break;
                    default:
                        throw std::runtime_error("Only grapheme cluster boundary rules are supported in icGrep 1.0");
                }
            }
            if (gb->getExpression()) {
                resolve(gb->getExpression());
            }
        }
        return re;
    };

    UCD::UCDCompiler::NameMap nameMap;
    std::unordered_set<Name *> visited;

    std::function<void(RE*)> gather = [&](RE * re) {
        assert ("RE object cannot be null!" && re);
        if (isa<Name>(re)) {
            if (visited.insert(cast<Name>(re)).second) {
                if (isa<CC>(cast<Name>(re)->getDefinition())) {
                    nameMap.emplace(cast<Name>(re), nullptr);
                } else {
                    gather(cast<Name>(re)->getDefinition());
                }
            }
        } else if (isa<Seq>(re)) {
            for (RE * item : *cast<Seq>(re)) {
                gather(item);
            }
        } else if (isa<Alt>(re)) {
            for (RE * item : *cast<Alt>(re)) {
                gather(item);
            }
        } else if (isa<Rep>(re)) {
            gather(cast<Rep>(re)->getRE());
        } else if (isa<Assertion>(re)) {
            gather(cast<Assertion>(re)->getAsserted());
        } else if (isa<Diff>(re)) {
            gather(cast<Diff>(re)->getLH());
            gather(cast<Diff>(re)->getRH());
        } else if (isa<Intersect>(re)) {
            gather(cast<Intersect>(re)->getLH());
            gather(cast<Intersect>(re)->getRH());
        } else if (isa<GraphemeBoundary>(re)) {
            if (cast<GraphemeBoundary>(re)->getExpression()) {
                gather(cast<GraphemeBoundary>(re)->getExpression());
            }
            gather(cast<GraphemeBoundary>(re)->getBoundaryRule());
        }
    };

    re = resolve(re);
    gather(re);

    if (LLVM_LIKELY(nameMap.size() > 0)) {
        UCD::UCDCompiler ucdCompiler(mCCCompiler);
        if (LLVM_UNLIKELY(DisableIfHierarchy)) {
            ucdCompiler.generateWithoutIfHierarchy(nameMap, mPB);
        } else {
            ucdCompiler.generateWithDefaultIfHierarchy(nameMap, mPB);
        }
        for (auto t : nameMap) {
            if (t.second) {
                mCompiledName.insert(std::make_pair(t.first, makeMarker(MarkerPosition::FinalMatchByte, t.second)));
            }
        }
    }

    // Now precompile any grapheme segmentation rules
    if (graphemeClusterRule) {
        auto gcb = compileName(graphemeClusterRule, mPB);
        mCompiledName.insert(std::make_pair(graphemeClusterRule, gcb));
    }
    return re;
}

void RE_Compiler::compileUnicodeNames(RE *& re) {
    re = resolveUnicodeProperties(re);
}

Name * RE_Compiler::generateGraphemeClusterBoundaryRule() {
    // 3.1.1 Grapheme Cluster Boundary Rules
    #define Behind(x) makeLookBehindAssertion(x)
    #define Ahead(x) makeLookAheadAssertion(x)

    RE * GCB_Control = makeName("gcb", "cn", Name::Type::UnicodeProperty);
    RE * GCB_CR = makeName("gcb", "cr", Name::Type::UnicodeProperty);
    RE * GCB_LF = makeName("gcb", "lf", Name::Type::UnicodeProperty);
    RE * GCB_Control_CR_LF = makeAlt({GCB_CR, GCB_LF, GCB_Control});

    // Break at the start and end of text.
    RE * GCB_1 = makeStart();
    RE * GCB_2 = makeEnd();
    // Do not break between a CR and LF.
    RE * GCB_3 = makeSeq({Behind(GCB_CR), Ahead(GCB_LF)});
    // Otherwise, break before and after controls.
    RE * GCB_4 = Behind(GCB_Control_CR_LF);
    RE * GCB_5 = Ahead(GCB_Control_CR_LF);
    RE * GCB_1_5 = makeAlt({GCB_1, GCB_2, makeDiff(makeAlt({GCB_4, GCB_5}), GCB_3)});

    RE * GCB_L = makeName("gcb", "l", Name::Type::UnicodeProperty);
    RE * GCB_V = makeName("gcb", "v", Name::Type::UnicodeProperty);
    RE * GCB_LV = makeName("gcb", "lv", Name::Type::UnicodeProperty);
    RE * GCB_LVT = makeName("gcb", "lvt", Name::Type::UnicodeProperty);
    RE * GCB_T = makeName("gcb", "t", Name::Type::UnicodeProperty);
    RE * GCB_RI = makeName("gcb", "ri", Name::Type::UnicodeProperty);
    // Do not break Hangul syllable sequences.
    RE * GCB_6 = makeSeq({Behind(GCB_L), Ahead(makeAlt({GCB_L, GCB_V, GCB_LV, GCB_LVT}))});
    RE * GCB_7 = makeSeq({Behind(makeAlt({GCB_LV, GCB_V})), Ahead(makeAlt({GCB_V, GCB_T}))});
    RE * GCB_8 = makeSeq({Behind(makeAlt({GCB_LVT, GCB_T})), Ahead(GCB_T)});
    // Do not break between regional indicator symbols.
    RE * GCB_8a = makeSeq({Behind(GCB_RI), Ahead(GCB_RI)});
    // Do not break before extending characters.
    RE * GCB_9 = Ahead(makeName("gcb", "ex", Name::Type::UnicodeProperty));
    // Do not break before SpacingMarks, or after Prepend characters.
    RE * GCB_9a = Ahead(makeName("gcb", "sm", Name::Type::UnicodeProperty));
    RE * GCB_9b = Behind(makeName("gcb", "pp", Name::Type::UnicodeProperty));
    RE * GCB_6_9b = makeAlt({GCB_6, GCB_7, GCB_8, GCB_8a, GCB_9, GCB_9a, GCB_9b});
    // Otherwise, break everywhere.
    RE * GCB_10 = makeSeq({Behind(makeAny()), Ahead(makeAny())});

    Name * gcb = makeName("gcb", Name::Type::UnicodeProperty);
    gcb->setDefinition(makeAlt({GCB_1_5, makeDiff(GCB_10, GCB_6_9b)}));
    return gcb;
}

void RE_Compiler::finalizeMatchResult(MarkerType match_result) {
    PabloAST * match_follow = mPB.createMatchStar(markerVar(match_result), mAny);
    if (InvertMatches) {
        match_follow = mPB.createNot(match_follow);
    }
    mFunction.setResult(0, mPB.createAssign("matches", mPB.createAnd(match_follow, UNICODE_LINE_BREAK ? mUnicodeLineBreak : mLineFeed)));
}

MarkerType RE_Compiler::compile(RE * re, PabloBuilder & pb) {
    return process(re, makeMarker(MarkerPosition::FinalPostPositionByte, pb.createOnes()), pb);
}

MarkerType RE_Compiler::process(RE * re, MarkerType marker, PabloBuilder & pb) {
    if (isa<Name>(re)) {
        return compileName(cast<Name>(re), marker, pb);
    } else if (isa<Seq>(re)) {
        return compileSeq(cast<Seq>(re), marker, pb);
    } else if (isa<Alt>(re)) {
        return compileAlt(cast<Alt>(re), marker, pb);
    } else if (isa<Rep>(re)) {
        return compileRep(cast<Rep>(re), marker, pb);
    } else if (isa<Assertion>(re)) {
        return compileAssertion(cast<Assertion>(re), marker, pb);
    } else if (isa<Any>(re)) {
        return compileAny(marker, pb);
    } else if (isa<Diff>(re)) {
        return compileDiff(cast<Diff>(re), marker, pb);
    } else if (isa<Intersect>(re)) {
        return compileIntersect(cast<Intersect>(re), marker, pb);
    } else if (isa<Start>(re)) {
        return compileStart(marker, pb);
    } else if (isa<End>(re)) {
        return compileEnd(marker, pb);
    } else if (isa<GraphemeBoundary>(re)) {
        return compileGraphemeBoundary(cast<GraphemeBoundary>(re), marker, pb);
    }
    throw std::runtime_error("RE Compiler failed to process " + Printer_RE::PrintRE(re));
}

inline MarkerType RE_Compiler::compileAny(const MarkerType m, PabloBuilder & pb) {
    PabloAST * nextFinalByte = markerVar(AdvanceMarker(m, MarkerPosition::FinalPostPositionByte, pb));
    PabloAST * lb = mLineFeed;
    if (UNICODE_LINE_BREAK) {
        lb = pb.createOr(mUnicodeLineBreak, mCRLF);
    }
    return makeMarker(MarkerPosition::FinalMatchByte, pb.createAnd(nextFinalByte, pb.createNot(lb), "dot"));
}

inline MarkerType RE_Compiler::compileName(Name * name, MarkerType marker, PabloBuilder & pb) {
    MarkerType nameMarker = compileName(name, pb);
    MarkerType nextPos;
    if (markerPos(marker) == MarkerPosition::FinalPostPositionByte) {
        nextPos = marker;
    } else if (name->getType() == Name::Type::Byte) {
        nextPos = AdvanceMarker(marker, MarkerPosition::InitialPostPositionByte, pb);
    } else {
        nextPos = AdvanceMarker(marker, MarkerPosition::FinalPostPositionByte, pb);
    }
    nameMarker.stream = pb.createAnd(markerVar(nextPos), markerVar(nameMarker), name->getName());
    return nameMarker;
}

inline MarkerType RE_Compiler::compileName(Name * name, PabloBuilder & pb) {
    auto f = mCompiledName.find(name);
    if (LLVM_LIKELY(f != mCompiledName.end())) {
        return f->second;
    } else if (LLVM_LIKELY(name->getDefinition() != nullptr)) {
        MarkerType m = compile(name->getDefinition(), pb);
        mCompiledName.insert(std::make_pair(name, m));
        return m;
    }
    throw std::runtime_error("Unresolved name " + name->getName());
}

MarkerType RE_Compiler::compileSeq(Seq * seq, MarkerType marker, PabloBuilder & pb) {
    // if-hierarchies are not inserted within unbounded repetitions
    if (mStarDepth > 0) {
        for (RE * re : *seq) {
            marker = process(re, marker, pb);
        }
        return marker;
    } else {
        return compileSeqTail(seq->begin(), seq->end(), 0, marker, pb);
    }
}

MarkerType RE_Compiler::compileSeqTail(Seq::iterator current, Seq::iterator end, int matchLenSoFar, MarkerType marker, PabloBuilder & pb) {
    if (current == end) return marker;
    if (matchLenSoFar < IfInsertionGap) {
        RE * r = *current;
        marker = process(r, marker, pb);
        current++;
        return compileSeqTail(current, end, matchLenSoFar + minMatchLength(r), marker, pb);
    } else {
        PabloBuilder nested = PabloBuilder::Create(pb);
        MarkerType m1 = compileSeqTail(current, end, 0, marker, nested);
        Assign * m1a = nested.createAssign("m", markerVar(m1));
        pb.createIf(markerVar(marker), {m1a}, nested);
        return makeMarker(m1.pos, m1a);
    }
}

MarkerType RE_Compiler::compileAlt(Alt * alt, MarkerType marker, PabloBuilder & pb) {
    std::vector<PabloAST *>  accum = {pb.createZeroes(), pb.createZeroes(), pb.createZeroes()};
    MarkerType const base = marker;
    // The following may be useful to force a common Advance rather than separate
    // Advances in each alternative.
    // MarkerType const base = makeMarker(InitialPostPositionByte, postPositionVar(marker, pb), pb);
    for (RE * re : *alt) {
        MarkerType rslt = process(re, base, pb);
        MarkerPosition p = markerPos(rslt);
        accum[p] = pb.createOr(accum[p], markerVar(rslt), "alt");
    }
    if (isa<Zeroes>(accum[MarkerPosition::InitialPostPositionByte]) && isa<Zeroes>(accum[MarkerPosition::FinalPostPositionByte])) {
        return makeMarker(MarkerPosition::FinalMatchByte, accum[MarkerPosition::FinalMatchByte]);
    }
    PabloAST * combine = pb.createOr(accum[InitialPostPositionByte], pb.createAdvance(accum[MarkerPosition::FinalMatchByte], 1), "alt");
    if (isa<Zeroes>(accum[FinalPostPositionByte])) {
        return makeMarker(InitialPostPositionByte, combine);
    }
    combine = pb.createOr(pb.createScanThru(pb.createAnd(mInitial, combine), mNonFinal), accum[MarkerPosition::FinalPostPositionByte], "alt");
    return makeMarker(MarkerPosition::FinalPostPositionByte, combine);
}

MarkerType RE_Compiler::compileAssertion(Assertion * a, MarkerType marker, PabloBuilder & pb) {
    RE * asserted = a->getAsserted();
    if (a->getKind() == Assertion::Kind::Lookbehind) {
        MarkerType lookback = compile(asserted, pb);
        AlignMarkers(marker, lookback, pb);
        PabloAST * lb = markerVar(lookback);
        if (a->getSense() == Assertion::Sense::Negative) {
            lb = pb.createNot(lb);
        }
        return makeMarker(markerPos(marker), pb.createAnd(markerVar(marker), lb, "lookback"));
    } else if (isUnicodeUnitLength(asserted)) {
        MarkerType lookahead = compile(asserted, pb);
        if (LLVM_LIKELY(markerPos(lookahead) == MarkerPosition::FinalMatchByte)) {
            PabloAST * la = markerVar(lookahead);
            if (a->getSense() == Assertion::Sense::Negative) {
                la = pb.createNot(la);
            }
            MarkerType fbyte = AdvanceMarker(marker, MarkerPosition::FinalPostPositionByte, pb);
            return makeMarker(MarkerPosition::FinalPostPositionByte, pb.createAnd(markerVar(fbyte), la, "lookahead"));
        }
    }
    throw std::runtime_error("Unsupported lookahead assertion.");
}

inline bool alignedUnicodeLength(const RE * lh, const RE * rh) {
    const auto lhl = getUnicodeUnitLengthRange(lh);
    const auto rhl = getUnicodeUnitLengthRange(rh);
    return (lhl.first == lhl.second && lhl.first == rhl.first && lhl.second == rhl.second);
}

MarkerType RE_Compiler::compileDiff(Diff * diff, MarkerType marker, PabloBuilder & pb) {
    RE * lh = diff->getLH();
    RE * rh = diff->getRH();
    if (alignedUnicodeLength(lh, rh)) {
        MarkerType t1 = process(lh, marker, pb);
        MarkerType t2 = process(rh, marker, pb);
        AlignMarkers(t1, t2, pb);
        return makeMarker(markerPos(t1), pb.createAnd(markerVar(t1), pb.createNot(markerVar(t2)), "diff"));
    }
    throw std::runtime_error("Unsupported Diff operands: " + Printer_RE::PrintRE(diff));
}

MarkerType RE_Compiler::compileIntersect(Intersect * x, MarkerType marker, PabloBuilder & pb) {
    RE * lh = x->getLH();
    RE * rh = x->getRH();
    if (alignedUnicodeLength(lh, rh)) {
        MarkerType t1 = process(lh, marker, pb);
        MarkerType t2 = process(rh, marker, pb);
        AlignMarkers(t1, t2, pb);
        return makeMarker(markerPos(t1), pb.createAnd(markerVar(t1), markerVar(t2), "intersect"));
    }
    throw std::runtime_error("Unsupported Intersect operands: " + Printer_RE::PrintRE(x));
}

MarkerType RE_Compiler::compileRep(Rep * rep, MarkerType marker, PabloBuilder & pb) {
    int lb = rep->getLB();
    int ub = rep->getUB();
    if (lb > 0) {
        marker = processLowerBound(rep->getRE(), lb, marker, pb);
    }
    if (ub == Rep::UNBOUNDED_REP) {
        marker = processUnboundedRep(rep->getRE(), marker, pb);
    } else if (lb < ub) {
        marker = processBoundedRep(rep->getRE(), ub - lb, marker, pb);
    }
    return marker;
}

/*
   Given a stream |repeated| marking positions associated with matches to an item
   of length |repeated_lgth|, compute a stream marking |repeat_count| consecutive
   occurrences of such items.
*/

inline PabloAST * RE_Compiler::consecutive_matches(PabloAST * repeated, int length, int repeat_count, PabloBuilder & pb) {
    int i = length;
    int total = repeat_count * length;
    PabloAST * consecutive_i = repeated;
    while (i * 2 < total) {
        PabloAST * v = consecutive_i;
        PabloAST * v2 =  pb.createAdvance(v, i);
        i *= 2;
        consecutive_i = pb.createAnd(v, v2, "at" + std::to_string(i) + "of" + std::to_string(total));
    }
    if (i < total) {
        PabloAST * v = consecutive_i;
        consecutive_i = pb.createAnd(v, pb.createAdvance(v, total - i), "at" + std::to_string(total));
    }
    return consecutive_i;
}

inline PabloAST * RE_Compiler::reachable(PabloAST * repeated, int length, int repeat_count, PabloBuilder & pb) {
    int i = length;
    int total_lgth = repeat_count * length;
    if (repeat_count == 0) {
        return repeated;
    }
    PabloAST * reachable_i = pb.createOr(repeated, pb.createAdvance(repeated, 1), "within1");
    while (i * 2 < total_lgth) {
        PabloAST * v = reachable_i;
        PabloAST * v2 =  pb.createAdvance(v, i);
        i *= 2;
        reachable_i = pb.createOr(v, v2, "within" + std::to_string(i));
    }
    if (i < total_lgth) {
        PabloAST * v = reachable_i;
        reachable_i = pb.createOr(v, pb.createAdvance(v, total_lgth - i), "within" + std::to_string(total_lgth));
    }
    return reachable_i;
}

MarkerType RE_Compiler::processLowerBound(RE * repeated, int lb, MarkerType marker, PabloBuilder & pb) {
    if (!mGraphemeBoundaryRule && isByteLength(repeated) && !DisableLog2BoundedRepetition) {
        PabloAST * cc = markerVar(compile(repeated, pb));
        PabloAST * cc_lb = consecutive_matches(cc, 1, lb, pb);
        PabloAST * marker_fwd = pb.createAdvance(markerVar(marker), markerPos(marker) == MarkerPosition::FinalMatchByte ? lb : lb - 1);
        return makeMarker(MarkerPosition::FinalMatchByte, pb.createAnd(marker_fwd, cc_lb, "lowerbound"));
    }
    // Fall through to general case.
    for (int i = 1; i <= lb; ++i) {
        marker = process(repeated, marker, pb);
        if (mGraphemeBoundaryRule) {
            marker = AdvanceMarker(marker, MarkerPosition::FinalPostPositionByte, pb);
        }
    }
    return marker;
}

MarkerType RE_Compiler::processBoundedRep(RE * repeated, int ub, MarkerType marker, PabloBuilder & pb) {
    if (!mGraphemeBoundaryRule && isByteLength(repeated) && ub > 1 && !DisableLog2BoundedRepetition) {
        // log2 upper bound for fixed length (=1) class
        // Create a mask of positions reachable within ub from current marker.
        // Use matchstar, then apply filter.
        PabloAST * match = markerVar(AdvanceMarker(marker, MarkerPosition::InitialPostPositionByte, pb));
        PabloAST * upperLimitMask = reachable(match, 1, ub, pb);
        PabloAST * cursor = markerVar(AdvanceMarker(marker, MarkerPosition::InitialPostPositionByte, pb));
        PabloAST * rep_class_var = markerVar(compile(repeated, pb));
        return makeMarker(MarkerPosition::InitialPostPositionByte, pb.createAnd(pb.createMatchStar(cursor, rep_class_var), upperLimitMask, "bounded"));
    }
    // Fall through to general case.
    for (int i = 1; i <= ub; ++i) {
        MarkerType a = process(repeated, marker, pb);
        MarkerType m = marker;
        AlignMarkers(a, m, pb);
        marker = makeMarker(markerPos(a), pb.createOr(markerVar(a), markerVar(m), "upper" + std::to_string(i)));
        if (mGraphemeBoundaryRule) {
            marker = AdvanceMarker(marker, MarkerPosition::FinalPostPositionByte, pb);
        }
    }
    return marker;
}

MarkerType RE_Compiler::processUnboundedRep(RE * repeated, MarkerType marker, PabloBuilder & pb) {
    // always use PostPosition markers for unbounded repetition.
    PabloAST * base = markerVar(AdvanceMarker(marker, MarkerPosition::InitialPostPositionByte, pb));
    if (!mGraphemeBoundaryRule && isByteLength(repeated)  && !DisableMatchStar) {
        PabloAST * cc = markerVar(compile(repeated, pb));
        PabloAST * mstar = nullptr;
        mstar = pb.createMatchStar(base, cc, "unbounded");
        return makeMarker(MarkerPosition::InitialPostPositionByte, mstar);
    } else if (isUnicodeUnitLength(repeated) && !DisableMatchStar && !DisableUnicodeMatchStar) {
        PabloAST * cc = markerVar(compile(repeated, pb));
        PabloAST * mstar = nullptr;
        PabloAST * nonFinal = mNonFinal;
        if (mGraphemeBoundaryRule) {
            nonFinal = pb.createOr(nonFinal, pb.createNot(mGraphemeBoundaryRule, "gext"));
        }
        cc = pb.createOr(cc, nonFinal);
        mstar = pb.createMatchStar(base, cc);
        PabloAST * final = mFinal;
        if (mGraphemeBoundaryRule) {
            final = mGraphemeBoundaryRule;
        }
        return makeMarker(MarkerPosition::FinalPostPositionByte, pb.createAnd(mstar, final, "unbounded"));
    } else if (mStarDepth > 0){
        PabloBuilder * outerb = pb.getParent();
        Assign * starPending = outerb->createAssign("pending", outerb->createZeroes());
        Assign * starAccum = outerb->createAssign("accum", outerb->createZeroes());
        mStarDepth++;
        PabloAST * m1 = pb.createOr(base, starPending);
        PabloAST * m2 = pb.createOr(base, starAccum);
        MarkerType result = process(repeated, makeMarker(MarkerPosition::InitialPostPositionByte, m1), pb);
        result = AdvanceMarker(result, MarkerPosition::InitialPostPositionByte, pb);
        PabloAST * loopComputation = markerVar(result);
        Next * nextPending = pb.createNext(starPending, pb.createAnd(loopComputation, pb.createNot(m2)));
        Next * nextStarAccum = pb.createNext(starAccum, pb.createOr(loopComputation, m2));
        mWhileTest = pb.createOr(mWhileTest, nextPending);
        mLoopVariants.push_back(nextPending);
        mLoopVariants.push_back(nextStarAccum);
        mStarDepth--;
        return makeMarker(markerPos(result), pb.createAssign("unbounded", pb.createOr(base, nextStarAccum)));
    } else {
        Assign * whileTest = pb.createAssign("test", base);
        Assign * whilePending = pb.createAssign("pending", base);
        Assign * whileAccum = pb.createAssign("accum", base);
        mWhileTest = pb.createZeroes();
        PabloBuilder wb = PabloBuilder::Create(pb);
        mStarDepth++;
        MarkerType result = process(repeated, makeMarker(MarkerPosition::InitialPostPositionByte, whilePending), wb);
        result = AdvanceMarker(result, MarkerPosition::InitialPostPositionByte, wb);
        PabloAST * loopComputation = markerVar(result);
        Next * nextWhilePending = wb.createNext(whilePending, wb.createAnd(loopComputation, wb.createNot(whileAccum)));
        Next * nextWhileAccum = wb.createNext(whileAccum, wb.createOr(loopComputation, whileAccum));
        Next * nextWhileTest = wb.createNext(whileTest, wb.createOr(mWhileTest, nextWhilePending));
        mLoopVariants.push_back(nextWhilePending);
        mLoopVariants.push_back(nextWhileAccum);
        mLoopVariants.push_back(nextWhileTest);
        pb.createWhile(nextWhileTest, mLoopVariants, wb);
        mStarDepth--;
        mLoopVariants.clear();
        return makeMarker(markerPos(result), pb.createAssign("unbounded", nextWhileAccum));
    }
}

inline MarkerType RE_Compiler::compileStart(const MarkerType marker, pablo::PabloBuilder & pb) {
    MarkerType m = AdvanceMarker(marker, MarkerPosition::InitialPostPositionByte, pb);
    if (UNICODE_LINE_BREAK) {
        PabloAST * line_end = mPB.createOr(mUnicodeLineBreak, mCRLF);
        PabloAST * sol = pb.createNot(pb.createOr(pb.createAdvance(pb.createNot(line_end), 1), mCRLF));
        return makeMarker(MarkerPosition::InitialPostPositionByte, pb.createAnd(markerVar(m), sol, "sol"));
    } else {
        PabloAST * sol = pb.createNot(pb.createAdvance(pb.createNot(mLineFeed), 1));
        return makeMarker(MarkerPosition::FinalPostPositionByte, pb.createAnd(markerVar(m), sol, "sol"));
    }
}

inline MarkerType RE_Compiler::compileEnd(const MarkerType marker, pablo::PabloBuilder & pb) {
    if (UNICODE_LINE_BREAK) {
        PabloAST * nextPos = markerVar(AdvanceMarker(marker, MarkerPosition::FinalPostPositionByte, pb));
        return makeMarker(MarkerPosition::FinalPostPositionByte, pb.createAnd(nextPos, mUnicodeLineBreak, "eol"));
    } else {
        PabloAST * nextPos = markerVar(AdvanceMarker(marker, MarkerPosition::InitialPostPositionByte, pb));  // For LF match
        return makeMarker(MarkerPosition::FinalPostPositionByte, pb.createAnd(nextPos, mLineFeed, "eol"));
    }
}

inline MarkerType RE_Compiler::compileGraphemeBoundary(GraphemeBoundary * gb, MarkerType marker, pablo::PabloBuilder & pb) {
    auto f = mCompiledName.find(gb->getBoundaryRule());
    assert ("Internal error: failed to locate grapheme boundary rule!" && (f != mCompiledName.end()));
    if (gb->getExpression()) {
        const auto graphemeBoundaryRule = mGraphemeBoundaryRule;
        mGraphemeBoundaryRule = markerVar(f->second);
        marker = process(gb->getExpression(), marker, pb);
        marker = AdvanceMarker(marker, MarkerPosition::FinalPostPositionByte, pb);
        mGraphemeBoundaryRule = graphemeBoundaryRule;
    } else {
        marker = AdvanceMarker(marker, MarkerPosition::FinalPostPositionByte, pb);
        PabloAST * rule = markerVar(f->second);
        if (gb->getSense() == GraphemeBoundary::Sense::Negative) {
            rule = pb.createNot(rule);
        }
        marker = makeMarker(MarkerPosition::FinalPostPositionByte, pb.createAnd(markerVar(marker), rule, "gb"));
    }
    return marker;
}

inline MarkerType RE_Compiler::AdvanceMarker(MarkerType marker, const MarkerPosition newpos, PabloBuilder & pb) {
    if (marker.pos != newpos) {
        if (marker.pos == MarkerPosition::FinalMatchByte) {
            marker.stream = pb.createAdvance(marker.stream, 1, "ipp");
            marker.pos = MarkerPosition::InitialPostPositionByte;
        }
        if (newpos == MarkerPosition::FinalPostPositionByte) {
            PabloAST * nonFinal = mNonFinal;
            if (mGraphemeBoundaryRule) {
                nonFinal = pb.createOr(nonFinal, pb.createNot(mGraphemeBoundaryRule, "gext"));
            }
            marker.stream = pb.createScanThru(pb.createAnd(mInitial, marker.stream), nonFinal, "fpp");
            marker.pos = MarkerPosition::FinalPostPositionByte;
        }
    }
    return marker;
}

inline void RE_Compiler::AlignMarkers(MarkerType & m1, MarkerType & m2, PabloBuilder & pb) {
    if (m1.pos < m2.pos) {
        m1 = AdvanceMarker(m1, m2.pos, pb);
    } else if (m2.pos < m1.pos) {
        m2 = AdvanceMarker(m2, m1.pos, pb);
    }
}

RE_Compiler::RE_Compiler(pablo::PabloFunction & function, cc::CC_Compiler & ccCompiler)
: mCCCompiler(ccCompiler)
, mLineFeed(nullptr)
, mCRLF(nullptr)
, mUnicodeLineBreak(nullptr)
, mAny(nullptr)
, mGraphemeBoundaryRule(nullptr)
, mInitial(nullptr)
, mNonFinal(nullptr)
, mFinal(nullptr)
, mWhileTest(nullptr)
, mStarDepth(0)
, mLoopVariants()
, mPB(ccCompiler.getBuilder().getPabloBlock(), ccCompiler.getBuilder())
, mFunction(function)
{

}

} // end of namespace re
