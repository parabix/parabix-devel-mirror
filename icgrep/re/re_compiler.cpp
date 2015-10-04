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
#include <re/re_analysis.h>
#include <re/printer_re.h>
#include <cc/cc_namemap.hpp>
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
static cl::OptionCategory fREcompilationOptions("Regex Compilation Options",
                                      "These options control the compilation of regular expressions to Pablo.");

static cl::opt<bool> DisableLog2BoundedRepetition("disable-log2-bounded-repetition", cl::init(false),
                     cl::desc("disable log2 optimizations for bounded repetition of bytes"), cl::cat(fREcompilationOptions));
static cl::opt<int> IfInsertionGap("if-insertion-gap", cl::init(3), cl::desc("minimum number of nonempty elements between inserted if short-circuit tests"), cl::cat(fREcompilationOptions));
static cl::opt<bool> DisableMatchStar("disable-matchstar", cl::init(false),
                     cl::desc("disable MatchStar optimization"), cl::cat(fREcompilationOptions));
static cl::opt<bool> DisableUnicodeMatchStar("disable-unicode-matchstar", cl::init(false),
                     cl::desc("disable Unicode MatchStar optimization"), cl::cat(fREcompilationOptions));
static cl::opt<bool> DisableUnicodeLineBreak("disable-unicode-linebreak", cl::init(false),
                     cl::desc("disable Unicode line breaks - use LF only"), cl::cat(fREcompilationOptions));
static cl::opt<bool> SetMod64Approximation("mod64-approximate", cl::init(false),
                     cl::desc("set mod64 approximate mode"), cl::cat(fREcompilationOptions));
#ifndef DISABLE_PREGENERATED_UCD_FUNCTIONS
static cl::opt<bool> UsePregeneratedUnicode("use-pregenerated-unicode", cl::init(false),
                     cl::desc("use fixed pregenerated Unicode character class sets instead"), cl::cat(fREcompilationOptions));
#endif
using namespace pablo;

namespace re {

RE_Compiler::RE_Compiler(pablo::PabloFunction & function, cc::CC_Compiler & ccCompiler)
: mCCCompiler(ccCompiler)
, mLineFeed(nullptr)
, mCRLF(nullptr)
, mUnicodeLineBreak(nullptr)
, mNonLineBreak(nullptr)
, mInitial(nullptr)
, mNonFinal(nullptr)
, mFinal(nullptr)
, mWhileTest(nullptr)
, mStarDepth(0)
, mLoopVariants()
, mPB(*ccCompiler.getBuilder().getPabloBlock(), ccCompiler.getBuilder())
, mFunction(function)
{

}
    
MarkerType RE_Compiler::AdvanceMarker(MarkerType m, MarkerPosition newpos, PabloBuilder & pb) {
    if (m.pos == newpos) return m;
    PabloAST * a = m.stream;
    if (m.pos == MarkerPosition::FinalMatchByte) {
        // Must advance at least to InitialPostPositionByte
        a = pb.createAdvance(a, 1, "adv");
    }
    // Now at InitialPostPositionByte; is a further advance needed?
    if (newpos == MarkerPosition::FinalPostPositionByte) {
        // Must advance through nonfinal bytes
        a = pb.createScanThru(pb.createAnd(mInitial, a), mNonFinal, "scanToFinal");
    }
    return {newpos, a};
}

void RE_Compiler::AlignMarkers(MarkerType & m1, MarkerType & m2, PabloBuilder & pb) {
    if (m1.pos < m2.pos) {
        m1 = AdvanceMarker(m1, m2.pos, pb); 
    }
    else if (m2.pos < m1.pos) {
        m2 = AdvanceMarker(m2, m1.pos, pb); 
    }
}

#define UNICODE_LINE_BREAK (!DisableUnicodeLineBreak)

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
    mNonLineBreak = mPB.createNot(lb);
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

    using PropertyMap = std::map<std::string, RE *>;
    using NamespacedPropertyMap = std::map<std::pair<std::string, std::string>, RE *>;
    using NameMap = UCD::UCDCompiler::NameMap;

    PropertyMap                 propertyMap;
    NamespacedPropertyMap       namespacedPropertyMap;
    NameMap                     nameMap;

    std::function<RE*(RE*)> resolve = [&](RE * re) -> RE * {
        if (Name * name = dyn_cast<Name>(re)) {
            if (LLVM_LIKELY(name->getDefinition() != nullptr)) {
                name->setDefinition(resolve(name->getDefinition()));
            } else if (LLVM_LIKELY(name->getType() == Name::Type::UnicodeProperty)) {
                // Attempt to look up an equivalently named Name object
                if (name->hasNamespace()) {
                    const auto f = namespacedPropertyMap.find(std::make_pair(name->getNamespace(), name->getName()));
                    if (f != namespacedPropertyMap.end()) {
                        if (f->second != name) {
                            return f->second;
                        }
                    }
                    namespacedPropertyMap.insert(std::make_pair(std::make_pair(name->getNamespace(), name->getName()), name));
                } else {
                    const auto f = propertyMap.find(name->getName());
                    if (f != propertyMap.end()) {
                        if (f->second != name) {
                            return f->second;
                        }
                    }
                    propertyMap.insert(std::make_pair(name->getName(), name));
                }
                if (UCD::resolvePropertyDefinition(name)) {
                    resolve(name->getDefinition());
                } else {
                    #ifndef DISABLE_PREGENERATED_UCD_FUNCTIONS
                    if (UsePregeneratedUnicode) {
                        const std::string functionName = UCD::resolvePropertyFunction(name);
                        const UCD::ExternalProperty & ep = UCD::resolveExternalProperty(functionName);
                        Call * call = mPB.createCall(Prototype::Create(functionName, std::get<1>(ep), std::get<2>(ep), std::get<0>(ep)), mCCCompiler.getBasisBits());
                        name->setCompiled(mPB.createAnd(call, mNonLineBreak));
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
        } else if (Seq * seq = dyn_cast<Seq>(re)) {
            for (auto si = seq->begin(); si != seq->end(); ++si) {
                *si = resolve(*si);
            }
        } else if (Alt * alt = dyn_cast<Alt>(re)) {
            CC * unionCC = nullptr;
            for (auto ai = alt->begin(); ai != alt->end(); ) {
                RE * re = resolve(*ai);
                if (CC * cc = getDefinitionIfCC(re)) {
                    unionCC = (unionCC == nullptr) ? cc : makeCC(unionCC, cc);
                    ai = alt->erase(ai);
                } else {
                    *ai++ = re;
                }
            }
            if (unionCC) {
                alt->push_back(makeName("union", unionCC));
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
            CC * lh = getDefinitionIfCC(diff->getLH());
            CC * rh = getDefinitionIfCC(diff->getRH());
            if (lh && rh) {
                return resolve(makeName("intersect", intersectCC(lh, rh)));
            }
        }
        return re;
    };

    std::function<void(RE*)> gather = [&](RE * re) {
        if (Name * name = dyn_cast<Name>(re)) {
            if (name->getCompiled() == nullptr) {
                if (isa<CC>(name->getDefinition())) {
                    nameMap.emplace(name, nullptr);
                } else {
                    gather(name->getDefinition());
                }
            }
        } else if (Seq * seq = dyn_cast<Seq>(re)) {
            for (auto re : *seq) {
                gather(re);
            }
        } else if (Alt * alt = dyn_cast<Alt>(re)) {
            for (auto re : *alt) {
                gather(re);
            }
        } else if (Rep * rep = dyn_cast<Rep>(re)) {
            gather(rep->getRE());
        } else if (Assertion * a = dyn_cast<Assertion>(re)) {
            gather(a->getAsserted());
        } else if (Diff * diff = dyn_cast<Diff>(re)) {
            gather(diff->getLH());
            gather(diff->getRH());
        }
    };

    re = resolve(re);
    gather(re);

    if (nameMap.size() > 0) {
        UCD::UCDCompiler ucdCompiler(mCCCompiler);
        ucdCompiler.generateWithDefaultIfHierarchy(nameMap, mPB);
        for (auto t : nameMap) {
            if (t.second) {
                t.first->setCompiled(mPB.createAnd(t.second, mNonLineBreak));
            }
        }
    }

    return re;
}

void RE_Compiler::compileUnicodeNames(RE *& re) {
    re = resolveUnicodeProperties(re);
}

void RE_Compiler::finalizeMatchResult(MarkerType match_result) {
    mFunction.setResult(0, mPB.createAssign("matches", mPB.createAnd(mPB.createMatchStar(markerVar(match_result), mNonLineBreak), UNICODE_LINE_BREAK ? mUnicodeLineBreak : mLineFeed)));
}

MarkerType RE_Compiler::compile(RE * re, PabloBuilder & pb) {
    return process(re, makeMarker(MarkerPosition::FinalPostPositionByte, pb.createOnes()), pb);
}

PabloAST * RE_Compiler::nextUnicodePosition(MarkerType m, PabloBuilder & pb) {
    if (markerPos(m) == MarkerPosition::FinalPostPositionByte) {
        return markerVar(m);
    } else if (markerPos(m) == MarkerPosition::InitialPostPositionByte) {
        return pb.createScanThru(pb.createAnd(mInitial, markerVar(m)), mNonFinal);
    } else {
        return pb.createScanThru(pb.createAnd(mInitial, pb.createAdvance(markerVar(m), 1)), mNonFinal);
    }
}

MarkerType RE_Compiler::process(RE * re, MarkerType marker, PabloBuilder & pb) {
    if (Name * name = dyn_cast<Name>(re)) {
        return process(name, marker, pb);
    }
    else if (Seq* seq = dyn_cast<Seq>(re)) {
        return process(seq, marker, pb);
    }
    else if (Alt * alt = dyn_cast<Alt>(re)) {
        return process(alt, marker, pb);
    }
    else if (Rep * rep = dyn_cast<Rep>(re)) {
        return process(rep, marker, pb);
    }
    else if (Assertion * a = dyn_cast<Assertion>(re)) {
        return process(a, marker, pb);
    }
    else if (isa<Any>(re)) {
        PabloAST * nextPos = nextUnicodePosition(marker, pb);
        PabloAST * dot = pb.createNot(UNICODE_LINE_BREAK ? pb.createOr(mUnicodeLineBreak, mCRLF) : mLineFeed);
        return makeMarker(MarkerPosition::FinalMatchByte, pb.createAnd(nextPos, dot, "dot"));
    }
    else if (Diff * diff = dyn_cast<Diff>(re)) {
        return process(diff, marker, pb);
    }
    else if (Intersect * ix = dyn_cast<Intersect>(re)) {
        return process(ix, marker, pb);
    }
    else if (isa<Start>(re)) {
        MarkerType m = AdvanceMarker(marker, MarkerPosition::InitialPostPositionByte, pb);
        if (UNICODE_LINE_BREAK) {
            PabloAST * line_end = mPB.createOr(mUnicodeLineBreak, mCRLF);
            PabloAST * sol = pb.createNot(pb.createOr(pb.createAdvance(pb.createNot(line_end), 1), mCRLF));
            return makeMarker(MarkerPosition::InitialPostPositionByte, pb.createAnd(markerVar(m), sol, "sol"));
        }
        else {
            PabloAST * sol = pb.createNot(pb.createAdvance(pb.createNot(mLineFeed), 1));
            return makeMarker(MarkerPosition::FinalPostPositionByte, pb.createAnd(markerVar(m), sol, "sol"));
        }
    }
    else if (isa<End>(re)) {
        if (UNICODE_LINE_BREAK) {
            PabloAST * nextPos = markerVar(AdvanceMarker(marker, MarkerPosition::FinalPostPositionByte, pb));
            return makeMarker(MarkerPosition::FinalPostPositionByte, pb.createAnd(nextPos, mUnicodeLineBreak, "end"));
        }
        PabloAST * nextPos = markerVar(AdvanceMarker(marker, MarkerPosition::InitialPostPositionByte, pb));  // For LF match
        return makeMarker(MarkerPosition::FinalPostPositionByte, pb.createAnd(nextPos, mLineFeed, "eol"));
    }
    return marker;
}

MarkerType RE_Compiler::process(Name * name, MarkerType marker, PabloBuilder & pb) {
    MarkerType nextPos;
    if (markerPos(marker) == MarkerPosition::FinalPostPositionByte) {
        nextPos = marker;
    }
    else if (name->getType() == Name::Type::Byte) {
        nextPos = AdvanceMarker(marker, MarkerPosition::InitialPostPositionByte, pb);
    }
    else {
        nextPos = AdvanceMarker(marker, MarkerPosition::FinalPostPositionByte, pb);
    }
    return makeMarker(MarkerPosition::FinalMatchByte, pb.createAnd(markerVar(nextPos), getNamedCharacterClassStream(name, pb), "m"));
}

PabloAST * RE_Compiler::getNamedCharacterClassStream(Name * name, PabloBuilder & pb) {
    PabloAST * var = name->getCompiled();
    if (LLVM_LIKELY(var != nullptr)) {
        return var;
    } else if (LLVM_LIKELY(name->getDefinition() != nullptr)) {
        MarkerType m = compile(name->getDefinition(), pb);
        assert(markerPos(m) == MarkerPosition::FinalMatchByte);
        var = pb.createAnd(markerVar(m), mNonLineBreak);
        name->setCompiled(var);
        return var;
    } else {
        throw std::runtime_error("Unresolved name " + name->getName());
    }
}

MarkerType RE_Compiler::process(Seq * seq, MarkerType marker, PabloBuilder & pb) {
    // if-hierarchies are not inserted within unbounded repetitions
    if (mStarDepth > 0) {
        for (RE * re : *seq) {
            marker = process(re, marker, pb);
        }
        return marker;
    }
    else {
        return processSeqTail(seq->begin(), seq->end(), 0, marker, pb);
    }
}

MarkerType RE_Compiler::processSeqTail(Seq::iterator current, Seq::iterator end, int matchLenSoFar, MarkerType marker, PabloBuilder & pb) {
    if (current == end) return marker;
    if (matchLenSoFar < IfInsertionGap) {
        RE * r = *current;
        marker = process(r, marker, pb);
        current++;
        return processSeqTail(current, end, matchLenSoFar + minMatchLength(r), marker, pb);
    }
    else {
        PabloBuilder nested = PabloBuilder::Create(pb);
        MarkerType m1 = processSeqTail(current, end, 0, marker, nested);
        Assign * m1a = nested.createAssign("m", markerVar(m1));
        pb.createIf(markerVar(marker), {m1a}, nested);
        return makeMarker(m1.pos, m1a);
    }
}

MarkerType RE_Compiler::process(Alt * alt, MarkerType marker, PabloBuilder & pb) {
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

MarkerType RE_Compiler::process(Assertion * a, MarkerType marker, PabloBuilder & pb) {
    RE * asserted = a->getAsserted();
    if (a->getKind() == Assertion::Kind::Lookbehind) {
        MarkerType m = marker;
        MarkerType lookback = compile(asserted, pb);
        AlignMarkers(m, lookback, pb);
        PabloAST * lb = markerVar(lookback);
        if (a->getSense() == Assertion::Sense::Negative) {
            lb = pb.createNot(lb);
        }
        return makeMarker(markerPos(m), pb.createAnd(markerVar(marker), lb, "lookback"));
    }
    else if (isUnicodeUnitLength(asserted)) {
        MarkerType lookahead = compile(asserted, pb);
        assert(markerPos(lookahead) == MarkerPosition::FinalMatchByte);
        PabloAST * la = markerVar(lookahead);
        if (a->getSense() == Assertion::Sense::Negative) {
            la = pb.createNot(la);
        }
        MarkerType fbyte = AdvanceMarker(marker, MarkerPosition::FinalPostPositionByte, pb);
        return makeMarker(MarkerPosition::FinalPostPositionByte, pb.createAnd(markerVar(fbyte), la, "lookahead"));
    }
    else {
        throw std::runtime_error("Unsupported lookahead assertion.");
    }
}

MarkerType RE_Compiler::process(Diff * diff, MarkerType marker, PabloBuilder & pb) {
    RE * lh = diff->getLH();
    RE * rh = diff->getRH();
    if (isUnicodeUnitLength(lh) && isUnicodeUnitLength(rh)) {
        MarkerType t1 = process(lh, marker, pb);
        MarkerType t2 = process(rh, marker, pb);
        AlignMarkers(t1, t2, pb);
        return makeMarker(markerPos(t1), pb.createAnd(markerVar(t1), pb.createNot(markerVar(t2)), "diff"));
    }
    throw std::runtime_error("Unsupported Diff operands: " + Printer_RE::PrintRE(diff));
}

MarkerType RE_Compiler::process(Intersect * x, MarkerType marker, PabloBuilder & pb) {
    RE * lh = x->getLH();
    RE * rh = x->getRH();
    if (isUnicodeUnitLength(lh) && isUnicodeUnitLength(rh)) {
        MarkerType t1 = process(lh, marker, pb);
        MarkerType t2 = process(rh, marker, pb);
        AlignMarkers(t1, t2, pb);
        return makeMarker(markerPos(t1), pb.createAnd(markerVar(t1), markerVar(t2), "intersect"));
    }
    throw std::runtime_error("Unsupported Intersect operands: " + Printer_RE::PrintRE(x));
}

MarkerType RE_Compiler::process(Rep * rep, MarkerType marker, PabloBuilder & pb) {
    int lb = rep->getLB();
    int ub = rep->getUB();
    if (lb > 0) {
        marker = processLowerBound(rep->getRE(), lb, marker, pb);
    }
    if (ub == Rep::UNBOUNDED_REP) {
        return processUnboundedRep(rep->getRE(), marker, pb);
    }
    else if (ub == lb) { // if (rep->getUB() != Rep::UNBOUNDED_REP)
        return marker;
    }
    else { // if (rep->getUB() != Rep::UNBOUNDED_REP)
        return processBoundedRep(rep->getRE(), ub - lb, marker, pb);
    }
}

/*
   Given a stream |repeated| marking positions associated with matches to an item
   of length |repeated_lgth|, compute a stream marking |repeat_count| consecutive 
   occurrences of such items.
*/

inline PabloAST * RE_Compiler::consecutive1(PabloAST * repeated, int repeated_lgth, int repeat_count, PabloBuilder & pb) {
        int i = repeated_lgth;
        int total_lgth = repeat_count * repeated_lgth;
        PabloAST * consecutive_i = repeated;
        while (i * 2 <= total_lgth) {
            PabloAST * v = consecutive_i;
            PabloAST * v2 =  pb.createAdvance(v, i);
            i *= 2;
            consecutive_i = pb.createAnd(v, v2, "at" + std::to_string(i) + "inarow");
        }        
        if (i < total_lgth) {
            PabloAST * v = consecutive_i;
            consecutive_i = pb.createAnd(v, pb.createAdvance(v, total_lgth - i), "at" + std::to_string(total_lgth) + "inarow");
        }
        return consecutive_i;
}

inline PabloAST * RE_Compiler::reachable(PabloAST *repeated, int repeated_lgth, int repeat_count, PabloBuilder & pb) {
        int i = repeated_lgth;
        int total_lgth = repeat_count * repeated_lgth;
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
    if (isByteLength(repeated) && !DisableLog2BoundedRepetition) {
        PabloAST * cc = markerVar(compile(repeated, pb));
        PabloAST * cc_lb = consecutive1(cc, 1, lb, pb);
        PabloAST * marker_fwd = pb.createAdvance(markerVar(marker), markerPos(marker) == MarkerPosition::FinalMatchByte ? lb : lb - 1);
        return makeMarker(MarkerPosition::FinalMatchByte, pb.createAnd(marker_fwd, cc_lb, "lowerbound"));
    }
    // Fall through to general case.
    while (lb-- != 0) {
        marker = process(repeated, marker, pb);
    }
    return marker;
}

MarkerType RE_Compiler::processBoundedRep(RE * repeated, int ub, MarkerType marker, PabloBuilder & pb) {
    if (isByteLength(repeated) && ub > 1 && !DisableLog2BoundedRepetition) {
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
    while (ub-- != 0) {
        MarkerType a = process(repeated, marker, pb);
        MarkerType m = marker;
        AlignMarkers(a, m, pb);
        marker = makeMarker(markerPos(a), pb.createOr(markerVar(a), markerVar(m), "m"));
    }
    return marker;
}

MarkerType RE_Compiler::processUnboundedRep(RE * repeated, MarkerType marker, PabloBuilder & pb) {
    // always use PostPosition markers for unbounded repetition.
    PabloAST * base = markerVar(AdvanceMarker(marker, MarkerPosition::InitialPostPositionByte, pb));
    
    if (isByteLength(repeated)  && !DisableMatchStar) {
        PabloAST * cc = markerVar(compile(repeated, pb));  
        PabloAST * mstar = SetMod64Approximation ? pb.createMod64MatchStar(base, cc) : pb.createMatchStar(base, cc, "unbounded");
        return makeMarker(MarkerPosition::InitialPostPositionByte, mstar);
    }
    else if (isUnicodeUnitLength(repeated) && !DisableMatchStar && !DisableUnicodeMatchStar) {
        PabloAST * cc = markerVar(compile(repeated, pb));
        PabloAST * mstar = SetMod64Approximation ? pb.createMod64MatchStar(base, pb.createOr(mNonFinal, cc)) : pb.createMatchStar(base, pb.createOr(mNonFinal, cc));
        return makeMarker(MarkerPosition::FinalPostPositionByte, pb.createAnd(mstar, mFinal, "unbounded"));
    }
    else if (mStarDepth > 0){
        
        PabloBuilder * outerb = pb.getParent();
        
        Assign * starPending = outerb->createAssign("pending", outerb->createZeroes());
        Assign * starAccum = outerb->createAssign("accum", outerb->createZeroes());
        
        mStarDepth++;
        PabloAST * m1 = pb.createOr(base, starPending);
        PabloAST * m2 = pb.createOr(base, starAccum);
        PabloAST * loopComputation = markerVar(AdvanceMarker(process(repeated, makeMarker(MarkerPosition::InitialPostPositionByte, m1), pb), MarkerPosition::InitialPostPositionByte, pb));
        Next * nextPending = pb.createNext(starPending, pb.createAnd(loopComputation, pb.createNot(m2)));
        Next * nextStarAccum = pb.createNext(starAccum, pb.createOr(loopComputation, m2));
        mWhileTest = pb.createOr(mWhileTest, nextPending);
        mLoopVariants.push_back(nextPending);
        mLoopVariants.push_back(nextStarAccum);
        mStarDepth--;
        
        return makeMarker(MarkerPosition::InitialPostPositionByte, pb.createAssign("unbounded", pb.createOr(base, nextStarAccum)));
    }    
    else {
        Assign * whileTest = pb.createAssign("test", base);
        Assign * whilePending = pb.createAssign("pending", base);
        Assign * whileAccum = pb.createAssign("accum", base);
        mWhileTest = pb.createZeroes();

        PabloBuilder wb = PabloBuilder::Create(pb);
        mStarDepth++;

        PabloAST * loopComputation = markerVar(AdvanceMarker(process(repeated, makeMarker(MarkerPosition::InitialPostPositionByte, whilePending), wb), MarkerPosition::InitialPostPositionByte, wb));
        Next * nextWhilePending = wb.createNext(whilePending, wb.createAnd(loopComputation, wb.createNot(whileAccum)));
        Next * nextWhileAccum = wb.createNext(whileAccum, wb.createOr(loopComputation, whileAccum));
        Next * nextWhileTest = wb.createNext(whileTest, wb.createOr(mWhileTest, nextWhilePending));
        mLoopVariants.push_back(nextWhilePending);
        mLoopVariants.push_back(nextWhileAccum);
        mLoopVariants.push_back(nextWhileTest);
        pb.createWhile(nextWhileTest, mLoopVariants, wb);
        mStarDepth--;
        mLoopVariants.clear();
        return makeMarker(MarkerPosition::InitialPostPositionByte, pb.createAssign("unbounded", nextWhileAccum));
    }    
} // end of namespace re
}
