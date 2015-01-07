/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "re_compiler.h"
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
#include <cc/cc_namemap.hpp>
#include <pablo/codegenstate.h>

#include <assert.h>
#include <stdexcept>

using namespace pablo;

namespace re {

inline MarkerType makePostPositionMarker(std::string marker_name, PabloAST * s, PabloBlock & pb) {
    return MarkerType{PostPosition, pb.createAssign(marker_name, s)};
}

inline MarkerType wrapPostPositionMarker(Assign * s) {
    return MarkerType{PostPosition, s};
}

inline MarkerType makeFinalPositionMarker(std::string marker_name, PabloAST * s, PabloBlock & pb) {
    return MarkerType{FinalByte, pb.createAssign(marker_name, s)};
}

inline Assign * markerStream(MarkerType m, PabloBlock &) {
    return m.stream;
}

inline Assign * markerVar(MarkerType m, PabloBlock &) {
    return m.stream;
}

inline Assign * postPositionVar(MarkerType m, PabloBlock & pb) {
    if (isFinalPositionMarker(m)) {
        return pb.createAssign("f", pb.createAdvance(markerVar(m, pb), 1));
    }
    return markerVar(m, pb);
}

//Set the 'internal.nonfinal' bit stream for the utf-8 multi-byte encoding.
//#define USE_IF_FOR_NONFINAL



RE_Compiler::RE_Compiler(PabloBlock & baseCG, const cc::CC_NameMap & nameMap)
: mCG(baseCG)
, mLineFeed(nullptr)
, mInitial(nullptr)
, mNonFinal(nullptr)
{

}

#define USE_IF_FOR_NONFINAL 1
#define USE_IF_FOR_CRLF
#define UNICODE_LINE_BREAK true


void RE_Compiler::initializeRequiredStreams(cc::CC_Compiler & ccc) {

    const std::string initial = "initial";
    const std::string nonfinal = "nonfinal";

    Assign * LF = mCG.createAssign("LF", ccc.compileCC(makeCC(0x0A)));
    mLineFeed = LF;
    PabloAST * CR = ccc.compileCC(makeCC(0x0D));
    PabloAST * LF_VT_FF_CR = ccc.compileCC(makeCC(0x0A, 0x0D));
#ifndef USE_IF_FOR_CRLF
    mCRLF = mCG.createAnd(mCG.createAdvance(CR, 1), mLineFeed);
#else
    PabloBlock & crb = PabloBlock::Create(mCG);
    Assign * cr1 = crb.createAssign("cr1", crb.createAdvance(CR, 1));
    Assign * acrlf = crb.createAssign("crlf", crb.createAnd(cr1, LF));
    mCG.createIf(CR, std::move(std::vector<Assign *>{acrlf}), crb);
    mCRLF = acrlf;
#endif

#ifndef USE_IF_FOR_NONFINAL
    PabloAST * u8single = ccc.compileCC(makeCC(0x00, 0x7F));
    PabloAST * u8pfx2 = ccc.compileCC(makeCC(0xC2, 0xDF));
    PabloAST * u8pfx3 = ccc.compileCC(makeCC(0xE0, 0xEF));
    PabloAST * u8pfx4 = ccc.compileCC(makeCC(0xF0, 0xF4));
    PabloAST * u8pfx = mCG.createOr(mCG.createOr(u8pfx2, u8pfx3), u8pfx4);
    mInitial = mCG.createAssign(initial, mCG.createOr(u8pfx, u8single));

    PabloAST * u8scope32 = mCG.createAdvance(u8pfx3, 1);
    PabloAST * u8scope42 = mCG.createAdvance(u8pfx4, 1);
    PabloAST * u8scope43 = mCG.createAdvance(u8scope42, 1);
    PabloAST * NEL = mCG.createAnd(mCG.createAdvance(ccc.compileCC(makeCC(0xC2)), 1), ccc.compileCC(makeCC(0x85)));
    PabloAST * E2_80 = mCG.createAnd(mCG.createAdvance(ccc.compileCC(makeCC(0xE2)), 1), ccc.compileCC(makeCC(0x80)));
    PabloAST * LS_PS = mCG.createAnd(mCG.createAdvance(E2_80, 1), ccc.compileCC(makeCC(0xA8,0xA9)));
    PabloAST * LB_chars = mCG.createOr(LF_VT_FF_CR, mCG.createOr(NEL, LS_PS));
    mNonFinal = mCG.createAssign(nonfinal, mCG.createOr(mCG.createOr(u8pfx, u8scope32), mCG.createOr(u8scope42, u8scope43)));
    mUnicodeLineBreak = mCG.createAnd(LB_chars, mCG.createNot(mCRLF));  // count the CR, but not CRLF
#endif

#ifdef USE_IF_FOR_NONFINAL
    PabloAST * u8single = ccc.compileCC(makeCC(0x00, 0x7F));
    PabloAST * u8pfx = ccc.compileCC(makeCC(0xC0, 0xFF));
    PabloBlock & it = PabloBlock::Create(mCG);
    PabloAST * u8pfx2 = ccc.compileCC(makeCC(0xC2, 0xDF), it);
    PabloAST * u8pfx3 = ccc.compileCC(makeCC(0xE0, 0xEF), it);
    PabloAST * u8pfx4 = ccc.compileCC(makeCC(0xF0, 0xF4), it);
    Assign * valid_pfx = it.createAssign("valid_pfx", it.createOr(it.createOr(u8pfx2, u8pfx3), u8pfx4));
    PabloAST * u8scope32 = it.createAdvance(u8pfx3, 1);
    PabloAST * u8scope42 = it.createAssign("u8scope42", it.createAdvance(u8pfx4, 1));
    PabloAST * u8scope43 = it.createAdvance(u8scope42, 1);
    Assign * a_nonfinal = it.createAssign(nonfinal, it.createOr(it.createOr(u8pfx, u8scope32), it.createOr(u8scope42, u8scope43)));
    PabloAST * NEL = it.createAnd(it.createAdvance(ccc.compileCC(makeCC(0xC2), it), 1), ccc.compileCC(makeCC(0x85), it));
    PabloAST * E2_80 = it.createAnd(it.createAdvance(ccc.compileCC(makeCC(0xE2), it), 1), ccc.compileCC(makeCC(0x80), it));
    PabloAST * LS_PS = it.createAnd(it.createAdvance(E2_80, 1), ccc.compileCC(makeCC(0xA8,0xA9), it));
    Assign * NEL_LS_PS = it.createAssign("NEL_LS_PS", it.createOr(NEL, LS_PS));
    mCG.createIf(u8pfx, std::move(std::vector<Assign *>{valid_pfx, a_nonfinal, NEL_LS_PS}), it);
    PabloAST * LB_chars = mCG.createOr(LF_VT_FF_CR, NEL_LS_PS);
    mInitial = mCG.createAssign(initial, mCG.createOr(u8single, valid_pfx));
    mNonFinal = a_nonfinal;
    mUnicodeLineBreak = mCG.createAnd(LB_chars, mCG.createNot(mCRLF));  // count the CR, but not CRLF
    #endif
}

void RE_Compiler::finalizeMatchResult(MarkerType match_result) {
    //These three lines are specifically for grep.
    PabloAST * lb = UNICODE_LINE_BREAK ? mUnicodeLineBreak : mLineFeed;
    Assign * v = markerVar(match_result, mCG);
    mCG.createAssign("matches", mCG.createAnd(mCG.createMatchStar(v, mCG.createNot(lb)), lb), 0);
    mCG.createAssign("lf", mCG.createAnd(lb, mCG.createNot(mCRLF)), 1);
}

MarkerType RE_Compiler::compile(RE * re, PabloBlock & pb) {
    return process(re, makePostPositionMarker("start", pb.createOnes(), pb), pb);
}

PabloAST * RE_Compiler::character_class_strm(Name * name, PabloBlock & pb) {
    Assign * var = name->getCompiled();
    if (var) {
        return var;
    }
    else {
        RE * def = name->getDefinition();
        if (def != nullptr) {
            MarkerType m = compile(def, mCG);
            assert(isFinalPositionMarker(m));
            Assign * v = markerStream(m, mCG);
            name->setCompiled(markerStream(m, mCG));
            return v;
        }
        else if (name->getType() == Name::Type::UnicodeProperty) {
            return pb.createCall(name->getName());
        }
        else {
            throw std::runtime_error("Unresolved name " + name->getName());
        }
    }
}

PabloAST * RE_Compiler::nextUnicodePosition(MarkerType m, PabloBlock & pb) {
    if (isPostPositionMarker(m)) {
        return pb.createScanThru(pb.createAnd(mInitial, markerVar(m, pb)), mNonFinal);
    }
    else {
        //return pb.createAdvanceThenScanThru(pb.createVar(markerVar(m), pb), mNonFinal);
        return pb.createScanThru(pb.createAnd(mInitial, pb.createAdvance(markerVar(m, pb), 1)), mNonFinal);
    }
}

MarkerType RE_Compiler::process(RE * re, MarkerType marker, PabloBlock & pb) {
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
        return makeFinalPositionMarker("dot", pb.createAnd(nextPos, dot), pb);
    }
    else if (Diff * diff = dyn_cast<Diff>(re)) {
        return process(diff, marker, pb);
    }
    else if (Intersect * ix = dyn_cast<Intersect>(re)) {
        return process(ix, marker, pb);
    }
    else if (isa<Start>(re)) {
        if (UNICODE_LINE_BREAK) {
            PabloAST * line_end = mCG.createOr(mUnicodeLineBreak, mCRLF);
            PabloAST * sol = pb.createNot(pb.createOr(pb.createAdvance(pb.createNot(line_end), 1), mCRLF));
            return makePostPositionMarker("sol", pb.createAnd(postPositionVar(marker, pb), sol), pb);
        }
        else {
            PabloAST * sol = pb.createNot(pb.createAdvance(pb.createNot(mLineFeed), 1));
            return makePostPositionMarker("sol", pb.createAnd(postPositionVar(marker, pb), sol), pb);
        }
    }
    else if (isa<End>(re)) {
        if (UNICODE_LINE_BREAK) {
            PabloAST * nextPos = nextUnicodePosition(marker, pb);
            return makeFinalPositionMarker("end", pb.createAnd(nextPos, mUnicodeLineBreak), pb);
        }
        PabloAST * nextPos = postPositionVar(marker, pb);  // For LF match
        return makePostPositionMarker("eol", pb.createAnd(nextPos, mLineFeed), pb);
    }
    return marker;
}

MarkerType RE_Compiler::process(Name * name, MarkerType marker, PabloBlock & pb) {
    PabloAST * nextPos = (name->getType() == Name::Type::Byte) ? postPositionVar(marker, pb): nextUnicodePosition(marker, pb);
    return makeFinalPositionMarker("m", pb.createAnd(nextPos, character_class_strm(name, pb)), pb);
}

MarkerType RE_Compiler::process(Seq * seq, MarkerType marker, PabloBlock & pb) {
    for (RE * re : *seq) {
        marker = process(re, marker, pb);
    }
    return marker;
}

MarkerType RE_Compiler::process(Alt * alt, MarkerType marker, PabloBlock & pb) {
    PabloAST * atPosnAccumulator = nullptr;
    PabloAST * postPosnAccumulator = nullptr;
    MarkerType const base = marker;
    // The following may be useful to force a common Advance rather than separate
    // Advances in each alternative.
    // MarkerType const base = makePostPositionMarker(postPositionVar(marker, pb), pb);
    for (RE * re : *alt) {
        MarkerType rslt = process(re, base, pb);
        PabloAST * rsltStream = markerVar(rslt, pb); 
        if (isFinalPositionMarker(rslt)) {
            atPosnAccumulator = (atPosnAccumulator == nullptr) ? rsltStream : pb.createOr(atPosnAccumulator, rsltStream);
        }
        else {
            postPosnAccumulator = (postPosnAccumulator == nullptr) ? rsltStream : pb.createOr(postPosnAccumulator, rsltStream);
        }
    }
    if (postPosnAccumulator == nullptr) {
        return makeFinalPositionMarker("alt", atPosnAccumulator == nullptr ? pb.createZeroes() : atPosnAccumulator, pb);
    }
    else {
        if (atPosnAccumulator != nullptr) {
            postPosnAccumulator = pb.createOr(postPosnAccumulator, pb.createAdvance(atPosnAccumulator, 1));
        }
        return makePostPositionMarker("alt", postPosnAccumulator, pb);
    }
}

MarkerType RE_Compiler::process(Assertion * a, MarkerType marker, PabloBlock & pb) {
    if (a->getKind() == Assertion::Kind::Lookbehind) {
        MarkerType lookback = compile(a->getAsserted(), pb);
        if (isFinalPositionMarker(marker) && isFinalPositionMarker(lookback)) {
            PabloAST * lb = markerVar(lookback, pb);
            if (a->getSense() == Assertion::Sense::Negative) lb = pb.createNot(lb);
            return makeFinalPositionMarker("lookback", pb.createAnd(markerVar(marker, pb), lb), pb);
        }
        else {
            Assign * m1 = postPositionVar(marker, pb);
            PabloAST * lb = postPositionVar(lookback, pb);
            if (a->getSense() == Assertion::Sense::Negative) lb = pb.createNot(lb);
            return makePostPositionMarker("lookback", pb.createAnd(m1, lb), pb);
        }
    }
    throw std::runtime_error("Lookahead assertions not implemented.");
}

MarkerType RE_Compiler::process(Diff * diff, MarkerType marker, PabloBlock & pb) {
    RE * lh = diff->getLH();
    RE * rh = diff->getRH();
    if (isUnicodeUnitLength(lh) && isUnicodeUnitLength(rh)) {
        MarkerType t1 = process(lh, marker, pb);
        MarkerType t2 = process(rh, marker, pb);
        assert(isFinalPositionMarker(t1) && isFinalPositionMarker(t2));
        return makeFinalPositionMarker("diff", pb.createAnd(markerVar(t1, pb), pb.createNot(markerVar(t2, pb))), pb);
    }
    throw std::runtime_error("Unsupported Diff operands: " + Printer_RE::PrintRE(diff));
}

MarkerType RE_Compiler::process(Intersect * x, MarkerType marker, PabloBlock & pb) {
    RE * lh = x->getLH();
    RE * rh = x->getRH();
    if (isUnicodeUnitLength(lh) && isUnicodeUnitLength(rh)) {
        MarkerType t1 = process(lh, marker, pb);
        MarkerType t2 = process(rh, marker, pb);
        assert(isFinalPositionMarker(t1) && isFinalPositionMarker(t2));
        return makeFinalPositionMarker("intersect", pb.createAnd(markerVar(t1, pb), markerVar(t2, pb)), pb);
    }
    throw std::runtime_error("Unsupported Intersect operands: " + Printer_RE::PrintRE(x));
}

MarkerType RE_Compiler::process(Rep * rep, MarkerType marker, PabloBlock & pb) {
    int lb = rep->getLB();
    int ub = rep->getUB();
    if (lb > 0) {
        marker = processLowerBound(rep->getRE(), lb, marker, pb);
    }
    if (ub == Rep::UNBOUNDED_REP) {
        return processUnboundedRep(rep->getRE(), marker, pb);
    }
    else { // if (rep->getUB() != Rep::UNBOUNDED_REP)
        return processBoundedRep(rep->getRE(), ub - lb, marker, pb);
    }
}

/*
   Given a stream |repeated| marking positions immediately after matches to an item
   of length |repeated_lgth|, compute a stream marking positions immediately after
   |repeat_count| consecutive occurrences of such items.
*/

inline Assign * RE_Compiler::consecutive(Assign * repeated, int repeated_lgth, int repeat_count, pablo::PabloBlock & pb) {
        int i = repeated_lgth;
        int total_lgth = repeat_count * repeated_lgth;
        Assign * consecutive_i = repeated;
        while (i * 2 < total_lgth) {
            PabloAST * v = consecutive_i;
            consecutive_i = pb.createAssign("consecutive", pb.createAnd(v, pb.createAdvance(v, i)));
            i *= 2;
        }        
        if (i < total_lgth) {
            PabloAST * v = consecutive_i;
            consecutive_i = pb.createAssign("consecutive", pb.createAnd(v, pb.createAdvance(v, total_lgth - i)));
        }
        return consecutive_i;
}

MarkerType RE_Compiler::processLowerBound(RE * repeated, int lb, MarkerType marker, PabloBlock & pb) {
    if (isByteLength(repeated)) {
        PabloAST * cc = markerVar(compile(repeated, pb), pb);
        Assign * cc_lb = consecutive(pb.createAssign("repeated", pb.createAdvance(cc,1)), 1, lb, pb);
        PabloAST * marker_fwd = pb.createAdvance(markerVar(marker, pb), isFinalPositionMarker(marker) ? lb+ 1 : lb);
        return makePostPositionMarker("lowerbound", pb.createAnd(marker_fwd, cc_lb), pb);
    }
    // Fall through to general case.
    while (lb-- != 0) {
        marker = process(repeated, marker, pb);
    }
    return marker;
}

MarkerType RE_Compiler::processBoundedRep(RE * repeated, int ub, MarkerType marker, PabloBlock & pb) {
    if (isByteLength(repeated)) {
        // log2 upper bound for fixed length (=1) class
        // Mask out any positions that are more than ub positions from a current match.
        // Use matchstar, then apply filter.
        Assign * nonMatch = pb.createAssign("nonmatch", pb.createNot(postPositionVar(marker, pb)));
        PabloAST * upperLimitMask = pb.createNot(consecutive(nonMatch, 1, ub + 1, pb));
        PabloAST * rep_class_var = markerVar(compile(repeated, pb), pb);
        return makePostPositionMarker("bounded", pb.createAnd(pb.createMatchStar(postPositionVar(marker, pb), rep_class_var), upperLimitMask), pb);
    }
    // Fall through to general case.
    while (ub-- != 0) {
        MarkerType a = process(repeated, marker, pb);
        if (isFinalPositionMarker(a) && isFinalPositionMarker(marker)) {
            marker = makeFinalPositionMarker("m", pb.createOr(markerVar(marker, pb), markerVar(a, pb)), pb); 
        }
        else {
            marker = makePostPositionMarker("m", pb.createOr(postPositionVar(marker, pb), postPositionVar(a, pb)), pb);
        }
    }
    return marker;
}

MarkerType RE_Compiler::processUnboundedRep(RE * repeated, MarkerType marker, PabloBlock & pb) {
    // always use PostPosition markers for unbounded repetition.
    PabloAST * base = postPositionVar(marker, pb);
    
    if (isByteLength(repeated)) {
        PabloAST * cc = markerVar(compile(repeated, pb), pb);
        return makePostPositionMarker("unbounded", pb.createMatchStar(base, cc), pb);
    }
    else if (isUnicodeUnitLength(repeated)) {
        PabloAST * cc = markerVar(compile(repeated, pb), pb);
        return makePostPositionMarker("unbounded", pb.createAnd(pb.createMatchStar(base, pb.createOr(mNonFinal, cc)), mInitial), pb);
    }
    else {
        Assign * whileTest = pb.createAssign("test", base);
        Assign * whileAccum = pb.createAssign("accum", base);

        PabloBlock & wb = PabloBlock::Create(pb);

        Assign * loopComputation = postPositionVar(process(repeated, wrapPostPositionMarker(whileTest), wb), wb);
        Next * nextWhileTest = wb.createNext(whileTest, wb.createAnd(loopComputation, wb.createNot(whileAccum)));
        wb.createNext(whileAccum, wb.createOr(loopComputation, whileAccum));

        pb.createWhile(nextWhileTest, wb);

        return makePostPositionMarker("unbounded", whileAccum, pb);
    }    
} // end of namespace re
}
