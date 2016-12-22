/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#include <re/re_compiler.h>
#include <re/re_toolchain.h>
#include <re/re_name_resolve.h>
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
#include <re/re_memoizer.hpp>
#include <re/printer_re.h>
#include <pablo/codegenstate.h>
#include <pablo/pablo_kernel.h>
#include <UCD/ucd_compiler.hpp>
#include <UCD/resolve_properties.h>
#include <assert.h>
#include <stdexcept>
#include <iostream>
#include <pablo/printer_pablos.h>
#include "llvm/Support/CommandLine.h"
#include <sstream>
#include <unordered_set>


#define UNICODE_LINE_BREAK (!AlgorithmOptionIsSet(DisableUnicodeLineBreak))

using namespace pablo;

namespace re {

void RE_Compiler::initializeRequiredStreams(const unsigned encodingBits) {
    if (encodingBits == 8) {
      RE_Compiler::initializeRequiredStreams_utf8();
    }
    else if (encodingBits == 16) {
      RE_Compiler::initializeRequiredStreams_utf16();
    }
}

void RE_Compiler::initializeRequiredStreams_utf16() {



    PabloAST * LF = mCCCompiler.compileCC("LF", makeCC(0x000A), mPB);
    PabloAST * CR = mCCCompiler.compileCC("CR", makeCC(0x000D), mPB);
    PabloAST * LF_VT_FF_CR = mCCCompiler.compileCC(makeCC(0x000A, 0x000D));
    PabloAST * NEL = mCCCompiler.compileCC("NEL", makeCC(0x0085), mPB);
    PabloAST * LS_PS = mCCCompiler.compileCC("LS_PS", makeCC(0x2028, 0x2029), mPB);
    PabloAST * NEL_LS_PS = mPB.createOr(NEL, LS_PS, "NEL_LS_PS");

    PabloAST * cr1 = mPB.createAdvance(CR, 1, "cr1");
    mCRLF = mPB.createAnd(cr1, LF, "crlf");

    PabloAST * hi_surrogate = mCCCompiler.compileCC(makeCC(0xD800, 0xDBFF));
    //PabloAST * lo_surrogate = mCCCompiler.compileCC(makeCC(0xDC00, 0xDFFF));
    PabloAST * u16hi_hi_surrogate = mCCCompiler.compileCC(makeCC(0xD800, 0xDB00));    //u16hi_hi_surrogate = [\xD8-\xDB]
    PabloAST * u16hi_lo_surrogate = mCCCompiler.compileCC(makeCC(0xDC00, 0xDF00));    //u16hi_lo_surrogate = [\xDC-\xDF]

    PabloAST * invalidTemp = mPB.createAdvance(u16hi_hi_surrogate, 1, "InvalidTemp");
    PabloAST * u16invalid = mPB.createXor(invalidTemp, u16hi_lo_surrogate, "u16invalid");
    // errors.Unicode=pablo.Advance(u16hi_hi_surrogate) ^ u16hi_lo_surrogate
    PabloAST * u16valid = mPB.createNot(u16invalid, "u16valid");

    PabloAST * u16single_temp = mPB.createOr(mCCCompiler.compileCC(makeCC(0x0000, 0xD7FF)), mCCCompiler.compileCC(makeCC(0xE000, 0xFFFF)));
    PabloAST * u16single = mPB.createAnd(u16single_temp, mPB.createNot(u16invalid));
    
    mNonFinal = mPB.createAnd(hi_surrogate, u16valid, "nonfinal");
    mFinal = mPB.createNot(mPB.createOr(mNonFinal, u16invalid), "final");
    mInitial = mPB.createOr(u16single, hi_surrogate, "initial");
    
    PabloAST * LB_chars = mPB.createOr(LF_VT_FF_CR, NEL_LS_PS);
    PabloAST * UnicodeLineBreak = mPB.createAnd(LB_chars, mPB.createNot(mCRLF));  // count the CR, but not CRLF
    PabloAST * lb = UNICODE_LINE_BREAK ? UnicodeLineBreak : LF;
    PabloAST * unterminatedLineAtEOF = mPB.createAtEOF(mPB.createAdvance(mPB.createNot(LB_chars), 1));
    mLineBreak = mPB.createOr(lb, unterminatedLineAtEOF);
    mAny = mPB.createNot(lb, "any");
}
void RE_Compiler::initializeRequiredStreams_utf8() {
    PabloAST * LF = mCCCompiler.compileCC("LF", makeCC(0x0A), mPB);
    PabloAST * CR = mCCCompiler.compileCC(makeCC(0x0D));
    PabloAST * LF_VT_FF_CR = mCCCompiler.compileCC(makeCC(0x0A, 0x0D));

    Zeroes * const zero = mPB.createZeroes();
    Var * crlf = mPB.createVar("crlf", zero);
    PabloBuilder crb = PabloBuilder::Create(mPB);
    PabloAST * cr1 = crb.createAdvance(CR, 1, "cr1");
    crb.createAssign(crlf, crb.createAnd(cr1, LF));
    mPB.createIf(CR, crb);

    mCRLF = crlf;

    Var * u8invalid = mPB.createVar("u8invalid", zero);
    Var * valid_pfx = mPB.createVar("valid_pfx", zero);
    Var * nonFinal = mPB.createVar("nonfinal", zero);
    Var * NEL_LS_PS = mPB.createVar("NEL_LS_PS", zero);

    PabloAST * u8pfx = mCCCompiler.compileCC(makeCC(0xC0, 0xFF));
    PabloBuilder it = PabloBuilder::Create(mPB);
    mPB.createIf(u8pfx, it);

    mNonFinal = nonFinal;

    PabloAST * u8pfx2 = mCCCompiler.compileCC(makeCC(0xC2, 0xDF), it);
    PabloAST * u8pfx3 = mCCCompiler.compileCC(makeCC(0xE0, 0xEF), it);
    PabloAST * u8pfx4 = mCCCompiler.compileCC(makeCC(0xF0, 0xF4), it);
    PabloAST * u8suffix = mCCCompiler.compileCC("u8suffix", makeCC(0x80, 0xBF), it);

    //
    // Two-byte sequences
    Var * u8scope22 = it.createVar("u8scope22", zero);
    Var * NEL = it.createVar("NEL", zero);
    PabloBuilder it2 = PabloBuilder::Create(it);
    it2.createAssign(u8scope22, it2.createAdvance(u8pfx2, 1));
    it2.createAssign(NEL, it2.createAnd(it2.createAdvance(mCCCompiler.compileCC(makeCC(0xC2), it2), 1), mCCCompiler.compileCC(makeCC(0x85), it2)));
    it.createIf(u8pfx2, it2);

    //
    // Three-byte sequences
    Var * u8scope32 = it.createVar("u8scope32", zero);
    Var * u8scope3X = it.createVar("u8scope3X", zero);
    Var * LS_PS = it.createVar("LS_PS", zero);
    Var * EX_invalid = it.createVar("EX_invalid", zero);

    PabloBuilder it3 = PabloBuilder::Create(it);
    it.createIf(u8pfx3, it3);

    it3.createAssign(u8scope32, it3.createAdvance(u8pfx3, 1));
    PabloAST * u8scope33 = it3.createAdvance(u8pfx3, 2);
    it3.createAssign(u8scope3X, it3.createOr(u8scope32, u8scope33));
    PabloAST * E2_80 = it3.createAnd(it3.createAdvance(mCCCompiler.compileCC(makeCC(0xE2), it3), 1), mCCCompiler.compileCC(makeCC(0x80), it3));
    it3.createAssign(LS_PS, it3.createAnd(it3.createAdvance(E2_80, 1), mCCCompiler.compileCC(makeCC(0xA8,0xA9), it3)));
    PabloAST * E0_invalid = it3.createAnd(it3.createAdvance(mCCCompiler.compileCC(makeCC(0xE0), it3), 1), mCCCompiler.compileCC(makeCC(0x80, 0x9F), it3));
    PabloAST * ED_invalid = it3.createAnd(it3.createAdvance(mCCCompiler.compileCC(makeCC(0xED), it3), 1), mCCCompiler.compileCC(makeCC(0xA0, 0xBF), it3));
    it3.createAssign(EX_invalid, it3.createOr(E0_invalid, ED_invalid));

    it.createAssign(NEL_LS_PS, it.createOr(NEL, LS_PS));

    //
    // Four-byte sequences
    Var * u8scope4nonfinal = it.createVar("u8scope4nonfinal", zero);
    Var * u8scope4X = it.createVar("u8scope4X", zero);
    Var * FX_invalid = it.createVar("FX_invalid", zero);
    PabloBuilder it4 = PabloBuilder::Create(it);
    it.createIf(u8pfx4, it4);
    PabloAST * u8scope42 = it4.createAdvance(u8pfx4, 1, "u8scope42");
    PabloAST * u8scope43 = it4.createAdvance(u8scope42, 1, "u8scope43");
    PabloAST * u8scope44 = it4.createAdvance(u8scope43, 1, "u8scope44");
    it4.createAssign(u8scope4nonfinal, it4.createOr(u8scope42, u8scope43));
    it4.createAssign(u8scope4X, it4.createOr(u8scope4nonfinal, u8scope44));
    PabloAST * F0_invalid = it4.createAnd(it4.createAdvance(mCCCompiler.compileCC(makeCC(0xF0), it4), 1), mCCCompiler.compileCC(makeCC(0x80, 0x8F), it4));
    PabloAST * F4_invalid = it4.createAnd(it4.createAdvance(mCCCompiler.compileCC(makeCC(0xF4), it4), 1), mCCCompiler.compileCC(makeCC(0x90, 0xBF), it4));
    it4.createAssign(FX_invalid, it4.createOr(F0_invalid, F4_invalid));

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
    it.createAssign(u8invalid, it.createOr(pfx_invalid, it.createOr(mismatch, EF_invalid)));
    PabloAST * u8valid = it.createNot(u8invalid, "u8valid");
    //
    //

    it.createAssign(valid_pfx, it.createAnd(u8pfx, u8valid));
    it.createAssign(mNonFinal, it.createAnd(it.createOr(it.createOr(u8pfx, u8scope32), u8scope4nonfinal), u8valid));


    PabloAST * LB_chars = mPB.createOr(LF_VT_FF_CR, NEL_LS_PS);
    PabloAST * u8single = mPB.createAnd(mCCCompiler.compileCC(makeCC(0x00, 0x7F)), mPB.createNot(u8invalid));
    mInitial = mPB.createOr(u8single, valid_pfx, "initial");
    mFinal = mPB.createNot(mPB.createOr(mNonFinal, u8invalid), "final");
    PabloAST * UnicodeLineBreak = mPB.createAnd(LB_chars, mPB.createNot(mCRLF));  // count the CR, but not CRLF
    PabloAST * lb = UNICODE_LINE_BREAK ? UnicodeLineBreak : LF;
    PabloAST * unterminatedLineAtEOF = mPB.createAtEOF(mPB.createAdvance(mPB.createNot(LB_chars), 1));
    mLineBreak = mPB.createOr(lb, unterminatedLineAtEOF);
    mAny = mPB.createNot(lb, "any");
}


RE * RE_Compiler::resolveUnicodeProperties(RE * re) {
    Name * ZeroWidth = nullptr;
    mCompiledName = &mBaseMap;

    auto nameMap = resolveNames(re, ZeroWidth);
    if (LLVM_LIKELY(nameMap.size() > 0)) {
        UCD::UCDCompiler ucdCompiler(mCCCompiler);
        if (LLVM_UNLIKELY(AlgorithmOptionIsSet(DisableIfHierarchy))) {
            ucdCompiler.generateWithoutIfHierarchy(nameMap, mPB);
        } else {
            ucdCompiler.generateWithDefaultIfHierarchy(nameMap, mPB);
        }
        for (auto t : nameMap) {
            if (t.second) {
                mCompiledName->add(t.first, makeMarker(MarkerPosition::FinalMatchUnit, mPB.createAnd(t.second, mAny)));
            }
        }
    }

    // Now precompile any grapheme segmentation rules
    if (ZeroWidth) {
        mCompiledName->add(ZeroWidth, compileName(ZeroWidth, mPB));
    }
    return re;
}

void RE_Compiler::compileUnicodeNames(RE *& re) {
    re = resolveUnicodeProperties(re);
}

void RE_Compiler::finalizeMatchResult(MarkerType match_result, bool InvertMatches) {
    PabloAST * match_follow = mPB.createMatchStar(markerVar(match_result), mAny);
    if (InvertMatches) {
        match_follow = mPB.createNot(match_follow);
    }
    PabloAST * matches = mPB.createAnd(match_follow, mLineBreak, "matches");
    if (mCountOnly) {
        Var * const output = mKernel->addOutput("matchedLineCount", mKernel->getSizeTy());
        PabloBuilder nestedCount = PabloBuilder::Create(mPB);
        mPB.createIf(matches, nestedCount);
        nestedCount.createAssign(output, nestedCount.createCount(matches));
    } else {
        Var * const output = mKernel->addOutput("output", mKernel->getStreamSetTy(2));
        mPB.createAssign(mPB.createExtract(output, mPB.getInteger(0)), matches);
        mPB.createAssign(mPB.createExtract(output, mPB.getInteger(1)), mLineBreak);
    }
}

MarkerType RE_Compiler::compile(RE * re, PabloBuilder & pb) {
    return process(re, makeMarker(MarkerPosition::FinalPostPositionUnit, pb.createOnes()), pb);
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
    }
    throw std::runtime_error("RE Compiler failed to process " + Printer_RE::PrintRE(re));
}

inline MarkerType RE_Compiler::compileAny(const MarkerType m, PabloBuilder & pb) {
    PabloAST * nextFinalByte = markerVar(AdvanceMarker(m, MarkerPosition::FinalPostPositionUnit, pb));
    PabloAST * lb = mLineBreak;
    if (UNICODE_LINE_BREAK) {
        lb = pb.createOr(mLineBreak, mCRLF);
    }
    return makeMarker(MarkerPosition::FinalMatchUnit, pb.createAnd(nextFinalByte, pb.createNot(lb), "dot"));
}

inline MarkerType RE_Compiler::compileName(Name * name, MarkerType marker, PabloBuilder & pb) {
    if (isUnicodeUnitLength(name)) {
        MarkerType nameMarker = compileName(name, pb);
        MarkerType nextPos;
        if (markerPos(marker) == MarkerPosition::FinalPostPositionUnit) {
            nextPos = marker;
        } else {
            nextPos = AdvanceMarker(marker, MarkerPosition::FinalPostPositionUnit, pb);
        }
        nameMarker.stream = pb.createAnd(markerVar(nextPos), markerVar(nameMarker), name->getName());
        return nameMarker;
    } else if (name->getType() == Name::Type::ZeroWidth) {
        RE * zerowidth = name->getDefinition();
        MarkerType zero = compile(zerowidth, pb);
        AlignMarkers(marker, zero, pb);
        PabloAST * ze = markerVar(zero);
        const std::string value = name->getName();
        if (value == "NonGCB") {
            ze = pb.createNot(ze);
        }
        return makeMarker(markerPos(marker), pb.createAnd(markerVar(marker), ze, "zerowidth"));
    } else {
        return process(name->getDefinition(), marker, pb);
    }
}

inline MarkerType RE_Compiler::compileName(Name * name, PabloBuilder & pb) {
    MarkerType m;
    if (LLVM_LIKELY(mCompiledName->get(name, m))) {
        return m;
    } else if (LLVM_LIKELY(name->getDefinition() != nullptr)) {
        m = compile(name->getDefinition(), pb);
        mCompiledName->add(name, m);
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
    if (current == end) {
        return marker;
    } else if (matchLenSoFar < IfInsertionGap) {
        RE * r = *current;
        marker = process(r, marker, pb);
        current++;
        return compileSeqTail(current, end, matchLenSoFar + minMatchLength(r), marker, pb);
    } else {
        Var * m = pb.createVar("m", pb.createZeroes());
        NameMap nestedMap(mCompiledName);
        mCompiledName = &nestedMap;
        PabloBuilder nested = PabloBuilder::Create(pb);
        MarkerType m1 = compileSeqTail(current, end, 0, marker, nested);
        nested.createAssign(m, markerVar(m1));
        pb.createIf(markerVar(marker), nested);
        mCompiledName = nestedMap.getParent();
        return makeMarker(m1.pos, m);
    }
}

MarkerType RE_Compiler::compileAlt(Alt * alt, MarkerType marker, PabloBuilder & pb) {
    std::vector<PabloAST *>  accum(3, pb.createZeroes());
    MarkerType const base = marker;
    // The following may be useful to force a common Advance rather than separate
    // Advances in each alternative.
    for (RE * re : *alt) {
        MarkerType m = process(re, base, pb);
        MarkerPosition p = markerPos(m);
        accum[p] = pb.createOr(accum[p], markerVar(m), "alt");
    }
    if (isa<Zeroes>(accum[MarkerPosition::FinalPostPositionUnit])) {
        return makeMarker(MarkerPosition::FinalMatchUnit, accum[MarkerPosition::FinalMatchUnit]);
    }
    PabloAST * combine = pb.createAdvance(accum[MarkerPosition::FinalMatchUnit], 1);
    combine = pb.createOr(pb.createScanThru(pb.createAnd(mInitial, combine), mNonFinal), accum[MarkerPosition::FinalPostPositionUnit], "alt");
    return makeMarker(MarkerPosition::FinalPostPositionUnit, combine);
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
        if (LLVM_LIKELY(markerPos(lookahead) == MarkerPosition::FinalMatchUnit)) {
            PabloAST * la = markerVar(lookahead);
            if (a->getSense() == Assertion::Sense::Negative) {
                la = pb.createNot(la);
            }
            MarkerType fbyte = AdvanceMarker(marker, MarkerPosition::FinalPostPositionUnit, pb);
            return makeMarker(MarkerPosition::FinalPostPositionUnit, pb.createAnd(markerVar(fbyte), la, "lookahead"));
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
    if (lb == 0) return marker;
    if (!mGraphemeBoundaryRule && isByteLength(repeated) && !AlgorithmOptionIsSet(DisableLog2BoundedRepetition)) {
        PabloAST * cc = markerVar(compile(repeated, pb));
        PabloAST * cc_lb = consecutive_matches(cc, 1, lb, pb);
        PabloAST * marker_fwd = pb.createAdvance(markerVar(marker), markerPos(marker) == MarkerPosition::FinalMatchUnit ? lb : lb - 1);
        return makeMarker(MarkerPosition::FinalMatchUnit, pb.createAnd(marker_fwd, cc_lb, "lowerbound"));
    }
    // Fall through to general case.  Process the first item and insert the rest into an if-structure.    
    marker = process(repeated, marker, pb);
    if (mGraphemeBoundaryRule) {
        marker = AdvanceMarker(marker, MarkerPosition::FinalPostPositionUnit, pb);
    }
    if (lb == 1) return marker;
    Var * m = pb.createVar("m", pb.createZeroes());
    PabloBuilder nested = PabloBuilder::Create(pb);
    MarkerType m1 = processLowerBound(repeated, lb - 1, marker, nested);
    nested.createAssign(m, markerVar(m1));
    pb.createIf(markerVar(marker), nested);
    return makeMarker(m1.pos, m);
}
    
MarkerType RE_Compiler::processBoundedRep(RE * repeated, int ub, MarkerType marker, PabloBuilder & pb) {
    if (ub == 0) return marker;
    if (!mGraphemeBoundaryRule && isByteLength(repeated) && ub > 1 && !AlgorithmOptionIsSet(DisableLog2BoundedRepetition)) {
        // log2 upper bound for fixed length (=1) class
        // Create a mask of positions reachable within ub from current marker.
        // Use matchstar, then apply filter.
        PabloAST * match = markerVar(AdvanceMarker(marker, MarkerPosition::FinalPostPositionUnit, pb));
        PabloAST * upperLimitMask = reachable(match, 1, ub, pb);
        PabloAST * cursor = markerVar(AdvanceMarker(marker, MarkerPosition::FinalPostPositionUnit, pb));
        PabloAST * rep_class_var = markerVar(compile(repeated, pb));
        return makeMarker(MarkerPosition::FinalPostPositionUnit, pb.createAnd(pb.createMatchStar(cursor, rep_class_var), upperLimitMask, "bounded"));
    }
    // Fall through to general case.  Process the first item and insert the rest into an if-structure.    
    MarkerType a = process(repeated, marker, pb);
    MarkerType m = marker;
    AlignMarkers(a, m, pb);
    marker = makeMarker(markerPos(a), pb.createOr(markerVar(a), markerVar(m)));
    if (mGraphemeBoundaryRule) {
        marker = AdvanceMarker(marker, MarkerPosition::FinalPostPositionUnit, pb);
    }
    if (ub == 1) return marker;
    Var * m1a = pb.createVar("m", pb.createZeroes());
    PabloBuilder nested = PabloBuilder::Create(pb);
    MarkerType m1 = processBoundedRep(repeated, ub - 1, marker, nested);
    nested.createAssign(m1a, markerVar(m1));
    pb.createIf(markerVar(marker), nested);
    return makeMarker(m1.pos, m1a);
}

MarkerType RE_Compiler::processUnboundedRep(RE * repeated, MarkerType marker, PabloBuilder & pb) {
    // always use PostPosition markers for unbounded repetition.
    PabloAST * base = markerVar(AdvanceMarker(marker, MarkerPosition::FinalPostPositionUnit, pb));
    if (!mGraphemeBoundaryRule && isByteLength(repeated)  && !AlgorithmOptionIsSet(DisableMatchStar)) {
        PabloAST * cc = markerVar(compile(repeated, pb));
        PabloAST * mstar = nullptr;
        mstar = pb.createMatchStar(base, cc, "unbounded");
        return makeMarker(MarkerPosition::FinalPostPositionUnit, mstar);
    } else if (isUnicodeUnitLength(repeated) && !AlgorithmOptionIsSet(DisableMatchStar) && !AlgorithmOptionIsSet(DisableUnicodeMatchStar)) {
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
        return makeMarker(MarkerPosition::FinalPostPositionUnit, pb.createAnd(mstar, final, "unbounded"));
    } else if (mStarDepth > 0){
        PabloBuilder * const outer = pb.getParent();
        Var * starPending = outer->createVar("pending", outer->createZeroes());
        Var * starAccum = outer->createVar("accum", outer->createZeroes());
        mStarDepth++;
        PabloAST * m1 = pb.createOr(base, starPending);
        PabloAST * m2 = pb.createOr(base, starAccum);
        MarkerType result = process(repeated, makeMarker(MarkerPosition::FinalPostPositionUnit, m1), pb);
        result = AdvanceMarker(result, MarkerPosition::FinalPostPositionUnit, pb);
        PabloAST * loopComputation = markerVar(result);
        pb.createAssign(starPending, pb.createAnd(loopComputation, pb.createNot(m2)));
        pb.createAssign(starAccum, pb.createOr(loopComputation, m2));
        mWhileTest = pb.createOr(mWhileTest, starPending);
        mStarDepth--;      
        return makeMarker(markerPos(result), pb.createOr(base, starAccum, "unbounded"));
    } else {
        Var * whileTest = pb.createVar("test", base);
        Var * whilePending = pb.createVar("pending", base);
        Var * whileAccum = pb.createVar("accum", base);
        mWhileTest = pb.createZeroes();
        PabloBuilder wb = PabloBuilder::Create(pb);
        mStarDepth++;
        MarkerType result = process(repeated, makeMarker(MarkerPosition::FinalPostPositionUnit, whilePending), wb);
        result = AdvanceMarker(result, MarkerPosition::FinalPostPositionUnit, wb);
        PabloAST * loopComputation = markerVar(result);
        wb.createAssign(whilePending, wb.createAnd(loopComputation, wb.createNot(whileAccum)));
        wb.createAssign(whileAccum, wb.createOr(loopComputation, whileAccum));
        wb.createAssign(whileTest, wb.createOr(mWhileTest, whilePending));
        pb.createWhile(whileTest, wb);
        mStarDepth--;
        return makeMarker(markerPos(result), whileAccum);
    }
}

inline MarkerType RE_Compiler::compileStart(const MarkerType marker, pablo::PabloBuilder & pb) {
    MarkerType m = AdvanceMarker(marker, MarkerPosition::FinalPostPositionUnit, pb);
    if (UNICODE_LINE_BREAK) {
        PabloAST * line_end = mPB.createOr(mLineBreak, mCRLF);
        PabloAST * sol_init = pb.createNot(pb.createOr(pb.createAdvance(pb.createNot(line_end), 1), mCRLF));
        PabloAST * sol = pb.createScanThru(pb.createAnd(mInitial, sol_init), mNonFinal);
        return makeMarker(MarkerPosition::FinalPostPositionUnit, pb.createAnd(markerVar(m), sol, "sol"));
    } else {
        PabloAST * sol = pb.createNot(pb.createAdvance(pb.createNot(mLineBreak), 1));
        return makeMarker(MarkerPosition::FinalPostPositionUnit, pb.createAnd(markerVar(m), sol, "sol"));
    }
}

inline MarkerType RE_Compiler::compileEnd(const MarkerType marker, pablo::PabloBuilder & pb) {
    PabloAST * nextPos = markerVar(AdvanceMarker(marker, MarkerPosition::FinalPostPositionUnit, pb));
    return makeMarker(MarkerPosition::FinalPostPositionUnit, pb.createAnd(nextPos, mLineBreak, "eol"));
}

inline MarkerType RE_Compiler::AdvanceMarker(MarkerType marker, const MarkerPosition newpos, PabloBuilder & pb) {
    if (marker.pos != newpos) {
        if (marker.pos == MarkerPosition::FinalMatchUnit) {
            marker.stream = pb.createAdvance(marker.stream, 1, "ipp");
            PabloAST * nonFinal = mNonFinal;
            if (mGraphemeBoundaryRule) {
                nonFinal = pb.createOr(nonFinal, pb.createNot(mGraphemeBoundaryRule, "gext"));
            }
            marker.stream = pb.createScanThru(pb.createAnd(mInitial, marker.stream), nonFinal, "fpp");
            marker.pos = MarkerPosition::FinalPostPositionUnit;
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

RE_Compiler::RE_Compiler(PabloKernel * kernel, cc::CC_Compiler & ccCompiler, bool CountOnly)
: mKernel(kernel)
, mCountOnly(CountOnly)
, mCCCompiler(ccCompiler)
, mLineBreak(nullptr)
, mCRLF(nullptr)
, mAny(nullptr)
, mGraphemeBoundaryRule(nullptr)
, mInitial(nullptr)
, mNonFinal(nullptr)
, mFinal(nullptr)
, mWhileTest(nullptr)
, mStarDepth(0)
, mPB(ccCompiler.getBuilder())
, mCompiledName(&mBaseMap) {

}

} // end of namespace re
