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

#include "llvm/Support/CommandLine.h"
cl::OptionCategory fREcompilationOptions("Regex Compilation Options",
                                      "These options control the compilation of regular expressions to Pablo.");

static cl::opt<bool> DisableLog2BoundedRepetition("disable-log2-bounded-repetition", cl::init(false), 
                     cl::desc("disable log2 optimizations for bounded repetition of bytes"), cl::cat(fREcompilationOptions));
static cl::opt<int> IfInsertionGap("if-insertion-gap", cl::init(3), cl::desc("minimum number of nonempty elements between inserted if short-circuit tests"), cl::cat(fREcompilationOptions));
static cl::opt<bool> DisableMatchStar("disable-matchstar", cl::init(false), 
                     cl::desc("disable MatchStar optimization"), cl::cat(fREcompilationOptions));
static cl::opt<bool> DisableUnicodeMatchStar("disable-Unicode-matchstar", cl::init(false), 
                     cl::desc("disable Unicode MatchStar optimization"), cl::cat(fREcompilationOptions));

static cl::opt<bool> DisableUnicodeLineBreak("disable-Unicode-linebreak", cl::init(false), 
                     cl::desc("disable Unicode line breaks - use LF only"), cl::cat(fREcompilationOptions));


using namespace pablo;

namespace re {


RE_Compiler::RE_Compiler(PabloBlock & baseCG)
: mPB(baseCG)
, mLineFeed(nullptr)
, mInitial(nullptr)
, mNonFinal(nullptr)
, mStarDepth(0)
{

}

    
MarkerType RE_Compiler::AdvanceMarker(MarkerType m, MarkerPosition newpos, PabloBlock & pb) {
    if (m.pos == newpos) return m;
    if (m.pos > newpos) throw std::runtime_error("icgrep internal error: attempt to AdvanceMarker backwards");
    PabloAST * a = m.stream;
    if (m.pos == FinalMatchByte) {
        // Must advance at least to InitialPostPositionByte
        a = pb.createAdvance(a, 1, "adv");
    }
    // Now at InitialPostPositionByte; is a further advance needed?
    if (newpos == FinalPostPositionByte) {
        // Must advance through nonfinal bytes
        a = pb.createScanThru(pb.createAnd(mInitial, a), mNonFinal, "scanToFinal");
    }
    return {newpos, a};
}

void RE_Compiler::AlignMarkers(MarkerType & m1, MarkerType & m2, PabloBlock & pb) {
    if (m1.pos < m2.pos) {
        m1 = AdvanceMarker(m1, m2.pos, pb); 
    }
    else if (m2.pos < m1.pos) {
        m2 = AdvanceMarker(m2, m1.pos, pb); 
    }
}

#define UNICODE_LINE_BREAK (!DisableUnicodeLineBreak)


void RE_Compiler::initializeRequiredStreams(cc::CC_Compiler & ccc) {

    Assign * LF = mPB.createAssign("LF", ccc.compileCC(makeCC(0x0A)));
    mLineFeed = LF;
    PabloAST * CR = ccc.compileCC(makeCC(0x0D));
    PabloAST * LF_VT_FF_CR = ccc.compileCC(makeCC(0x0A, 0x0D));

    PabloBlock & crb = PabloBlock::Create(mPB);
    PabloAST * cr1 = crb.createAdvance(CR, 1, "cr1");
    Assign * acrlf = crb.createAssign("crlf", crb.createAnd(cr1, LF));
    mPB.createIf(CR, std::move(std::vector<Assign *>{acrlf}), crb);
    mCRLF = acrlf;

    PabloAST * u8pfx = ccc.compileCC(makeCC(0xC0, 0xFF));
    PabloBlock & it = PabloBlock::Create(mPB);
    PabloAST * u8pfx2 = ccc.compileCC(makeCC(0xC2, 0xDF), it);
    PabloAST * u8pfx3 = ccc.compileCC(makeCC(0xE0, 0xEF), it);
    PabloAST * u8pfx4 = ccc.compileCC(makeCC(0xF0, 0xF4), it);
    Assign * valid_pfx = it.createAssign("valid_pfx", it.createOr(it.createOr(u8pfx2, u8pfx3), u8pfx4));
    PabloAST * u8scope32 = it.createAdvance(u8pfx3, 1);
    PabloAST * u8scope42 = it.createAdvance(u8pfx4, 1, "u8scope42");
    PabloAST * u8scope43 = it.createAdvance(u8scope42, 1);
    mNonFinal = it.createAssign("nonfinal", it.createOr(it.createOr(u8pfx, u8scope32), it.createOr(u8scope42, u8scope43)));
    PabloAST * NEL = it.createAnd(it.createAdvance(ccc.compileCC(makeCC(0xC2), it), 1), ccc.compileCC(makeCC(0x85), it));
    PabloAST * E2_80 = it.createAnd(it.createAdvance(ccc.compileCC(makeCC(0xE2), it), 1), ccc.compileCC(makeCC(0x80), it));
    PabloAST * LS_PS = it.createAnd(it.createAdvance(E2_80, 1), ccc.compileCC(makeCC(0xA8,0xA9), it));
    Assign * NEL_LS_PS = it.createAssign("NEL_LS_PS", it.createOr(NEL, LS_PS));
    mPB.createIf(u8pfx, std::move(std::vector<Assign *>{valid_pfx, mNonFinal, NEL_LS_PS}), it);
    PabloAST * LB_chars = mPB.createOr(LF_VT_FF_CR, NEL_LS_PS);

    PabloAST * u8single = ccc.compileCC(makeCC(0x00, 0x7F));
    mInitial = mPB.createOr(u8single, valid_pfx, "initial");
    mUnicodeLineBreak = mPB.createAnd(LB_chars, mPB.createNot(mCRLF));  // count the CR, but not CRLF
}

void RE_Compiler::finalizeMatchResult(MarkerType match_result) {
    //These three lines are specifically for grep.
    PabloAST * lb = UNICODE_LINE_BREAK ? mUnicodeLineBreak : mLineFeed;
    PabloAST * v = markerVar(match_result);
    mPB.createAssign("matches", mPB.createAnd(mPB.createMatchStar(v, mPB.createNot(lb)), lb), 0);
    mPB.createAssign("lf", mPB.createAnd(lb, mPB.createNot(mCRLF)), 1);
}

MarkerType RE_Compiler::compile(RE * re, PabloBlock & pb) {
    return process(re, makeMarker(InitialPostPositionByte, pb.createOnes()), pb);
}

PabloAST * RE_Compiler::nextUnicodePosition(MarkerType m, PabloBlock & pb) {
    if (markerPos(m) == FinalPostPositionByte) {
        return markerVar(m);
    }
    else if (markerPos(m) == InitialPostPositionByte) {
        return pb.createScanThru(pb.createAnd(mInitial, markerVar(m)), mNonFinal);
    }
    else {
        return pb.createScanThru(pb.createAnd(mInitial, pb.createAdvance(markerVar(m), 1)), mNonFinal);
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
        return makeMarker(FinalMatchByte, pb.createAnd(nextPos, dot, "dot"));
    }
    else if (Diff * diff = dyn_cast<Diff>(re)) {
        return process(diff, marker, pb);
    }
    else if (Intersect * ix = dyn_cast<Intersect>(re)) {
        return process(ix, marker, pb);
    }
    else if (isa<Start>(re)) {
        MarkerType m = AdvanceMarker(marker, InitialPostPositionByte, pb);
        if (UNICODE_LINE_BREAK) {
            PabloAST * line_end = mPB.createOr(mUnicodeLineBreak, mCRLF);
            PabloAST * sol = pb.createNot(pb.createOr(pb.createAdvance(pb.createNot(line_end), 1), mCRLF));
            return makeMarker(InitialPostPositionByte, pb.createAnd(markerVar(m), sol, "sol"));
        }
        else {
            PabloAST * sol = pb.createNot(pb.createAdvance(pb.createNot(mLineFeed), 1));
            return makeMarker(InitialPostPositionByte, pb.createAnd(markerVar(m), sol, "sol"));
        }
    }
    else if (isa<End>(re)) {
        if (UNICODE_LINE_BREAK) {
            PabloAST * nextPos = nextUnicodePosition(marker, pb);
            return makeMarker(FinalPostPositionByte, pb.createAnd(nextPos, mUnicodeLineBreak, "end"));
        }
        PabloAST * nextPos = markerVar(AdvanceMarker(marker, InitialPostPositionByte, pb));  // For LF match
        return makeMarker(InitialPostPositionByte, pb.createAnd(nextPos, mLineFeed, "eol"));
    }
    return marker;
}

MarkerType RE_Compiler::process(Name * name, MarkerType marker, PabloBlock & pb) {
    MarkerType nextPos;
    if (markerPos(marker) == FinalPostPositionByte) nextPos = marker;
    else if (name->getType() == Name::Type::Byte) {
        nextPos = AdvanceMarker(marker, InitialPostPositionByte, pb);
    }
    else {
        nextPos = AdvanceMarker(marker, FinalPostPositionByte, pb);
    }
    return makeMarker(FinalMatchByte, pb.createAnd(markerVar(nextPos), getNamedCharacterClassStream(name), "m"));
}

PabloAST * RE_Compiler::getNamedCharacterClassStream(Name * name) {
    PabloAST * var = name->getCompiled();
    if (LLVM_LIKELY(var != nullptr)) {
        return var;
    }
    else if (name->getDefinition() != nullptr) {
        MarkerType m = compile(name->getDefinition(), mPB);
        assert(markerPos(m) == FinalMatchByte);
        var = markerVar(m);
    }
    else if (name->getType() == Name::Type::UnicodeProperty) {
        var = mPB.createCall(name->getName());
    }
    else {
        throw std::runtime_error("Unresolved name " + name->getName());
    }

    var = mPB.createAnd(var, mPB.createNot(UNICODE_LINE_BREAK ? mUnicodeLineBreak : mLineFeed));
    name->setCompiled(var);
    return var;
}


MarkerType RE_Compiler::process(Seq * seq, MarkerType marker, PabloBlock & pb) {
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

MarkerType RE_Compiler::processSeqTail(Seq::iterator current, Seq::iterator end, int matchLenSoFar, MarkerType marker, PabloBlock & pb) {
    if (current == end) return marker;
    if (matchLenSoFar < IfInsertionGap) {
        RE * r = *current;
        marker = process(r, marker, pb);
        current++;
        return processSeqTail(current, end, matchLenSoFar + minMatchLength(r), marker, pb);
    }
    else {
        PabloBlock & nested = PabloBlock::Create(pb);
        MarkerType m1 = processSeqTail(current, end, 0, marker, nested);
        Assign * m1a = nested.createAssign("m", markerVar(m1));
        pb.createIf(markerVar(marker), std::move(std::vector<Assign *>{m1a}), nested);
        return makeMarker(m1.pos, m1a);
    }
}

MarkerType RE_Compiler::process(Alt * alt, MarkerType marker, PabloBlock & pb) {
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
    if (isa<Zeroes>(accum[InitialPostPositionByte]) && isa<Zeroes>(accum[FinalPostPositionByte])) {
        return makeMarker(FinalMatchByte, accum[FinalMatchByte]);
    }
    PabloAST * combine = pb.createOr(accum[InitialPostPositionByte], pb.createAdvance(accum[FinalMatchByte], 1), "alt");
    if (isa<Zeroes>(accum[FinalPostPositionByte])) {
        return makeMarker(InitialPostPositionByte, combine);
    }
    combine = pb.createOr(pb.createScanThru(pb.createAnd(mInitial, combine), mNonFinal), accum[FinalPostPositionByte], "alt");
    return makeMarker(FinalPostPositionByte, combine);
}

MarkerType RE_Compiler::process(Assertion * a, MarkerType marker, PabloBlock & pb) {
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
        assert(markerPos(lookahead) == FinalMatchByte);
        PabloAST * la = markerVar(lookahead);
        if (a->getSense() == Assertion::Sense::Negative) {
            la = pb.createNot(la);
        }
        MarkerType fbyte = AdvanceMarker(marker, FinalPostPositionByte, pb);
        return makeMarker(FinalPostPositionByte, pb.createAnd(markerVar(fbyte), la, "lookahead"));
    }
    else {
        throw std::runtime_error("Unsupported lookahead assertion.");
    }
}

MarkerType RE_Compiler::process(Diff * diff, MarkerType marker, PabloBlock & pb) {
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

MarkerType RE_Compiler::process(Intersect * x, MarkerType marker, PabloBlock & pb) {
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

MarkerType RE_Compiler::process(Rep * rep, MarkerType marker, PabloBlock & pb) {
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

inline PabloAST * RE_Compiler::consecutive1(PabloAST * repeated, int repeated_lgth, int repeat_count, pablo::PabloBlock & pb) {
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

inline PabloAST * RE_Compiler::reachable(PabloAST *repeated, int repeated_lgth, int repeat_count, pablo::PabloBlock & pb) {
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

MarkerType RE_Compiler::processLowerBound(RE * repeated, int lb, MarkerType marker, PabloBlock & pb) {
    if (isByteLength(repeated) && !DisableLog2BoundedRepetition) {
        PabloAST * cc = markerVar(compile(repeated, pb));
        PabloAST * cc_lb = consecutive1(cc, 1, lb, pb);
        PabloAST * marker_fwd = pb.createAdvance(markerVar(marker), markerPos(marker) == FinalMatchByte ? lb : lb-1);
        return makeMarker(FinalMatchByte, pb.createAnd(marker_fwd, cc_lb, "lowerbound"));
    }
    // Fall through to general case.
    mStarDepth++;
    while (lb-- != 0) {
        marker = process(repeated, marker, pb);
    }
    mStarDepth--;
    return marker;
}

MarkerType RE_Compiler::processBoundedRep(RE * repeated, int ub, MarkerType marker, PabloBlock & pb) {
    if (isByteLength(repeated) && ub > 1 && !DisableLog2BoundedRepetition) {
        // log2 upper bound for fixed length (=1) class
        // Create a mask of positions reachable within ub from current marker.
        // Use matchstar, then apply filter.
        PabloAST * match = markerVar(AdvanceMarker(marker, InitialPostPositionByte, pb));
        PabloAST * upperLimitMask = reachable(match, 1, ub, pb);
        PabloAST * cursor = markerVar(AdvanceMarker(marker, InitialPostPositionByte, pb));
        PabloAST * rep_class_var = markerVar(compile(repeated, pb));
        return makeMarker(InitialPostPositionByte, pb.createAnd(pb.createMatchStar(cursor, rep_class_var), upperLimitMask, "bounded"));
    }
    // Fall through to general case.
    mStarDepth++;
    while (ub-- != 0) {
        MarkerType a = process(repeated, marker, pb);
        MarkerType m = marker;
        AlignMarkers(a, m, pb);
        marker = makeMarker(markerPos(a), pb.createOr(markerVar(a), markerVar(m), "m"));
    }
    mStarDepth--;
    return marker;
}

MarkerType RE_Compiler::processUnboundedRep(RE * repeated, MarkerType marker, PabloBlock & pb) {
    // always use PostPosition markers for unbounded repetition.
    PabloAST * base = markerVar(AdvanceMarker(marker, InitialPostPositionByte, pb));
    
    if (isByteLength(repeated)  && !DisableMatchStar) {
        PabloAST * cc = markerVar(compile(repeated, pb));  
        return makeMarker(InitialPostPositionByte, pb.createMatchStar(base, cc, "unbounded"));
    }
    else if (isUnicodeUnitLength(repeated) && !DisableMatchStar && !DisableUnicodeMatchStar) {
        PabloAST * cc = markerVar(compile(repeated, pb));
        return makeMarker(InitialPostPositionByte, pb.createAnd(pb.createMatchStar(base, pb.createOr(mNonFinal, cc)), mInitial, "unbounded"));
    }
    else {
        Assign * whileTest = pb.createAssign("test", base);
        Assign * whileAccum = pb.createAssign("accum", base);

        PabloBlock & wb = PabloBlock::Create(pb);
        mStarDepth++;

        PabloAST * loopComputation = markerVar(AdvanceMarker(process(repeated, makeMarker(InitialPostPositionByte, whileTest), wb), InitialPostPositionByte, wb));
        Next * nextWhileTest = wb.createNext(whileTest, wb.createAnd(loopComputation, wb.createNot(whileAccum)));
        Next * nextWhileAccum = wb.createNext(whileAccum, wb.createOr(loopComputation, whileAccum));

        pb.createWhile(nextWhileTest, wb);
        mStarDepth--;

        return makeMarker(InitialPostPositionByte, pb.createAssign("unbounded", nextWhileAccum));
    }    
} // end of namespace re
}
