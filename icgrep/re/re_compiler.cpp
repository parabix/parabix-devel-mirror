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
#include <cc/cc_namemap.hpp>
#include <pablo/codegenstate.h>

#include <assert.h>
#include <stdexcept>

using namespace pablo;

namespace re {

MarkerType makePostPositionMarker(std::string marker_name, PabloAST * s, PabloBlock & pb) {
    return MarkerType{PostPosition, pb.createAssign(marker_name, s)};
}
    
MarkerType wrapPostPositionMarker(Assign * s) {
    return MarkerType{PostPosition, s};
}
    
MarkerType makeFinalPositionMarker(std::string marker_name, PabloAST * s, PabloBlock & pb) {
    return MarkerType{FinalByte, pb.createAssign(marker_name, s)};
}

Assign * markerStream(MarkerType m, PabloBlock & pb) {
    return m.stream;
}

Var * markerVar(MarkerType m, PabloBlock & pb) {
    return pb.createVar(m.stream);
}

Var * postPositionVar(MarkerType m, PabloBlock & pb) {
    if (isFinalPositionMarker(m)) return pb.createVar(pb.createAssign("f", pb.createAdvance(pb.createVar(m.stream), 1)));
    else return pb.createVar(m.stream);
}

//Set the 'internal.nonfinal' bit stream for the utf-8 multi-byte encoding.
//#define USE_IF_FOR_NONFINAL



RE_Compiler::RE_Compiler(PabloBlock & baseCG, const cc::CC_NameMap & nameMap)
: mCG(baseCG)
, mLineFeed(nullptr)
, mInitial(nullptr)
, mNonFinal(nullptr)
, mNameMap(nameMap)
{

}

//#define USE_IF_FOR_NONFINAL 1
#define UNICODE_LINE_BREAK false

    
void RE_Compiler::initializeRequiredStreams(cc::CC_Compiler & ccc) {

    mLineFeed = ccc.compileCC(makeCC(0x0A));
    PabloAST * CR = ccc.compileCC(makeCC(0x0D));
    PabloAST * LF_VT_FF_CR = ccc.compileCC(makeCC(0x0A, 0x0D));
    PabloAST * NEL = mCG.createAnd(mCG.createAdvance(ccc.compileCC(makeCC(0xC2)), 1), ccc.compileCC(makeCC(0x85)));
    PabloAST * E2_80 = mCG.createAnd(mCG.createAdvance(ccc.compileCC(makeCC(0xE2)), 1), ccc.compileCC(makeCC(0x80)));
    PabloAST * LS_PS = mCG.createAnd(mCG.createAdvance(E2_80, 1), ccc.compileCC(makeCC(0xA8,0xA9)));
    PabloAST * LB_chars = mCG.createOr(LF_VT_FF_CR, mCG.createOr(NEL, LS_PS));
    mCRLF = mCG.createAnd(mCG.createAdvance(CR, 1), mLineFeed);
    mUnicodeLineBreak = mCG.createAnd(LB_chars, mCG.createNot(mCRLF));  // count the CR, but not CRLF
    PabloAST * u8single = ccc.compileCC(makeCC(0x00, 0x7F));
    PabloAST * u8pfx2 = ccc.compileCC(makeCC(0xC2, 0xDF));
    PabloAST * u8pfx3 = ccc.compileCC(makeCC(0xE0, 0xEF));
    PabloAST * u8pfx4 = ccc.compileCC(makeCC(0xF0, 0xF4));
    
    const std::string initial = "initial";
    const std::string nonfinal = "nonfinal";

    PabloAST * u8pfx = mCG.createOr(mCG.createOr(u8pfx2, u8pfx3), u8pfx4);
    mInitial = mCG.createVar(mCG.createAssign(initial, mCG.createOr(u8pfx, u8single)));
    #ifdef USE_IF_FOR_NONFINAL
    mNonFinal = mCG.createVar(pb.createAssign(nonfinal, mCG.createZeroes()));
    #endif
    PabloAST * u8scope32 = mCG.createAdvance(u8pfx3, 1);
    PabloAST * u8scope42 = mCG.createAdvance(u8pfx4, 1);
    PabloAST * u8scope43 = mCG.createAdvance(u8scope42, 1);
    #ifdef USE_IF_FOR_NONFINAL
    PabloBlock it(mCG);
    it.createAssign(nonfinal, it.createOr(it.createOr(u8pfx, u8scope32), it.createOr(u8scope42, u8scope43)));
    mCG.createIf(u8pfx, std::move(it));
    #else
    mNonFinal = mCG.createVar(mCG.createAssign(nonfinal, mCG.createOr(mCG.createOr(u8pfx, u8scope32), mCG.createOr(u8scope42, u8scope43))));
    #endif
}

void RE_Compiler::finalizeMatchResult(MarkerType match_result) {
    //These three lines are specifically for grep.
    PabloAST * lb = UNICODE_LINE_BREAK ? mUnicodeLineBreak : mLineFeed;
    mCG.createAssign("matches", mCG.createAnd(mCG.createMatchStar(markerVar(match_result, mCG), mCG.createNot(lb)), lb), 0);
    mCG.createAssign("lf", lb, 1);//mCG.createAnd(lb, mCG.createNot(mCRLF)), 1);
}
    
MarkerType RE_Compiler::compile(RE * re, PabloBlock & pb) {
        return process(re, makePostPositionMarker("start", pb.createOnes(), pb), pb);
}
        
PabloAST * RE_Compiler::character_class_strm(Name * name, PabloBlock & pb) {
    if (name->getType() == Name::Type::UnicodeCategory) {
        return pb.createCall(name->getName());
    }
    else {
        Var * var = name->getCompiled();
        if (var == nullptr) {
            RE * def = name->getDefinition();
	        assert(!isa<CC>(def));  //  Names mapping to CCs should have been compiled.
	        assert(name->getType == Name::Type::Unicode);  // 
	        // compile in top-level block
                MarkerType m = compile(def, mCG);
                assert(isFinalPositionMarker(m));
	        name -> setCompiled(pb.createVar(markerStream(m, mCG)));
        }
        return name->getCompiled();
    }
}

PabloAST * RE_Compiler::nextUnicodePosition(MarkerType m, PabloBlock & pb) {
    if (isPostPositionMarker(m)) {
        return pb.createScanThru(pb.createVar(pb.createAnd(mInitial, markerVar(m, pb))), mNonFinal);
    }
    else {
        //return pb.createAdvanceThenScanThru(pb.createVar(markerVar(m), pb), mNonFinal);
        return pb.createScanThru(pb.createAnd(mInitial, pb.createAdvance(pb.createVar(markerVar(m, pb)), 1)), mNonFinal);
        
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
            // We would have to advance to the end of the Unicode LB category,
            // but that violates our marker assumption (a third marker type: atNextFinal???)
            throw std::runtime_error("Unsupported: $ with Unicode line break");
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

MarkerType RE_Compiler::process(Diff * diff, MarkerType marker, PabloBlock & pb) {
    RE * lh = diff->getLH();
    RE * rh = diff->getRH();
    if ((isa<Any>(lh) || isa<Name>(lh)) && (isa<Any>(rh) || isa<Name>(rh))) {
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
    if ((isa<Any>(lh) || isa<Name>(lh)) && (isa<Any>(rh) || isa<Name>(rh))) {
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
        PabloAST * v = pb.createVar(consecutive_i);
                consecutive_i = pb.createAssign("consecutive", pb.createAnd(v, pb.createAdvance(v, i)));
                i *= 2;
        }
        if (i < total_lgth) {
        PabloAST * v = pb.createVar(consecutive_i);
                consecutive_i = pb.createAssign("consecutive", pb.createAnd(v, pb.createAdvance(v, total_lgth - i)));
        }
        return consecutive_i;
}
                
inline bool RE_Compiler::isFixedLength(RE * regexp) {
    return isa<Name>(regexp) && ((cast<Name>(regexp)->getType()) == Name::Type::Byte);
}

MarkerType RE_Compiler::processLowerBound(RE * repeated, int lb, MarkerType marker, PabloBlock & pb) {
    if (isFixedLength(repeated)) {
        Name * name = cast<Name>(repeated);
        Assign * cc_lb = consecutive(pb.createAssign("repeated", pb.createAdvance(name->getCompiled(),1)), 1, lb, pb);
        PabloAST * marker_fwd = pb.createAdvance(markerVar(marker, pb), isFinalPositionMarker(marker) ? lb+ 1 : lb);
        return makePostPositionMarker("lowerbound", pb.createAnd(marker_fwd, pb.createVar(cc_lb)), pb);
    }
    // Fall through to general case.
    while (lb-- != 0) {
        marker = process(repeated, marker, pb);
    }
    return marker;
}

MarkerType RE_Compiler::processBoundedRep(RE * repeated, int ub, MarkerType marker, PabloBlock & pb) {
    if (isFixedLength(repeated)) {
        // log2 upper bound for fixed length (=1) class
        // Mask out any positions that are more than ub positions from a current match.
        // Use matchstar, then apply filter.
        Assign * nonMatch = pb.createAssign("nonmatch", pb.createNot(postPositionVar(marker, pb)));
        PabloAST * upperLimitMask = pb.createNot(pb.createVar(consecutive(nonMatch, 1, ub + 1, pb)));
        PabloAST * rep_class_var = cast<Name>(repeated)->getCompiled();
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
    
    if (isa<Name>(repeated)) {
        Name * name = cast<Name>(repeated);
        PabloAST * cc = character_class_strm(name, pb);
        if (name->getType() == Name::Type::Byte) {
            return makePostPositionMarker("unbounded", pb.createMatchStar(base, cc), pb);
        }
        else { // Name::Unicode and Name::UnicodeCategory
            return makePostPositionMarker("unbounded", pb.createAnd(pb.createMatchStar(base, pb.createOr(mNonFinal, cc)), mInitial), pb);
        }        
    }
    else if (isa<Any>(repeated)) {
        PabloAST * dot = pb.createNot(UNICODE_LINE_BREAK ? mUnicodeLineBreak : mLineFeed);
        return makePostPositionMarker("unbounded", pb.createAnd(pb.createMatchStar(base, pb.createOr(mNonFinal, dot)), mInitial), pb);
    }
    else if (isa<Diff>(repeated) && isa<Any>(cast<Diff>(repeated)->getLH()) && isa<Name>(cast<Diff>(repeated)->getRH())) {
        Name * name = cast<Name>(cast<Diff>(repeated)->getRH());
        PabloAST * cc = pb.createNot(pb.createOr(character_class_strm(name, pb), mLineFeed));
        return makePostPositionMarker("unbounded", pb.createAnd(pb.createMatchStar(base, pb.createOr(mNonFinal, cc)), mInitial), pb);
    }
    else {
        Assign * whileTest = pb.createAssign("test", base);
        Assign * whileAccum = pb.createAssign("accum", base);

        PabloBlock wb(pb); 

        Var * loopComputation = postPositionVar(process(repeated, wrapPostPositionMarker(whileTest), wb), wb);

        Var * whileAccumVar = wb.createVar(whileAccum);

        Next * nextWhileTest = wb.createNext(whileTest, wb.createAnd(loopComputation, wb.createNot(whileAccumVar)));

        wb.createNext(whileAccum, wb.createOr(loopComputation, whileAccumVar));

        pb.createWhile(wb.createVar(nextWhileTest), std::move(wb));

        return makePostPositionMarker("unbounded", whileAccumVar, pb);
    }    
} // end of namespace re
}
