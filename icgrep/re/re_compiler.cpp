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

//Set the 'internal.nonfinal' bit stream for the utf-8 multi-byte encoding.
//#define USE_IF_FOR_NONFINAL

using namespace pablo;

namespace re {

RE_Compiler::RE_Compiler(PabloBlock & baseCG, const cc::CC_NameMap & nameMap)
: mCG(baseCG)
, mLineFeed(nullptr)
, mInitial(nullptr)
, mNonFinal(nullptr)
, mNameMap(nameMap)
{

}

//#define USE_IF_FOR_NONFINAL 1

void RE_Compiler::compile(RE * re, PabloBlock & pb) {

    mLineFeed = mNameMap["LineFeed"]->getVar();

    const std::string initial = "initial";
    const std::string nonfinal = "nonfinal";

    //Set the 'internal.initial' bit stream for the utf-8 multi-byte encoding.
    PabloAST * u8single = mNameMap["UTF8-SingleByte"]->getVar();
    PabloAST * u8pfx2 = mNameMap["UTF8-Prefix2"]->getVar();
    PabloAST * u8pfx3 = mNameMap["UTF8-Prefix3"]->getVar();
    PabloAST * u8pfx4 = mNameMap["UTF8-Prefix4"]->getVar();
    PabloAST * u8pfx = pb.createOr(pb.createOr(u8pfx2, u8pfx3), u8pfx4);
    mInitial = pb.createVar(pb.createAssign(initial, pb.createOr(u8pfx, u8single)));
    #ifdef USE_IF_FOR_NONFINAL
    mNonFinal = pb.createVar(pb.createAssign(nonfinal, pb.createZeroes()));
    #endif
    PabloAST * u8scope32 = pb.createAdvance(u8pfx3, 1);
    PabloAST * u8scope42 = pb.createAdvance(u8pfx4, 1);
    PabloAST * u8scope43 = pb.createAdvance(u8scope42, 1);
    #ifdef USE_IF_FOR_NONFINAL
    PabloBlock it(pb);
    it.createAssign(nonfinal, it.createOr(it.createOr(u8pfx, u8scope32), it.createOr(u8scope42, u8scope43)));
    pb.createIf(u8pfx, std::move(it));
    #else
    mNonFinal = pb.createVar(pb.createAssign(nonfinal, pb.createOr(pb.createOr(u8pfx, u8scope32), pb.createOr(u8scope42, u8scope43))));
    #endif

    PabloAST * result = process(re, pb.createAssign("start", pb.createOnes()), pb);

    //These three lines are specifically for grep.
    pb.createAssign("matches", pb.createAnd(pb.createMatchStar(pb.createVar(result), pb.createNot(mLineFeed)), mLineFeed), 0);
    pb.createAssign("lf", mLineFeed, 1);
}

PabloAST * RE_Compiler::character_class_strm(Name * name, PabloBlock & pb) {
    if (name->getType() == Name::Type::UnicodeCategory) {
        return pb.createCall(name->getName());
    }
    else {
        return name->getVar();
    }
}

Assign * RE_Compiler::process(RE * re, Assign * marker, PabloBlock & pb) {
    if (Name * name = dyn_cast<Name>(re)) {
        marker = process(name, marker, pb);
    }
    else if (Seq* seq = dyn_cast<Seq>(re)) {
        marker = process(seq, marker, pb);
    }
    else if (Alt * alt = dyn_cast<Alt>(re)) {
        marker = process(alt, marker, pb);
    }
    else if (Rep * rep = dyn_cast<Rep>(re)) {
        marker = process(rep, marker, pb);
    }
    else if (isa<Any>(re)) {
        // Move the markers forward through any nonfinal UTF-8 bytes to the final position of each character.
        PabloAST * markerVar = pb.createVar(marker);
        markerVar = pb.createAnd(markerVar, mInitial);
        markerVar = pb.createScanThru(markerVar, mNonFinal);
        PabloAST * dot = pb.createNot(mLineFeed);
        marker = pb.createAssign("dot", pb.createAdvance(pb.createAnd(markerVar, dot), 1));
    }
    else if (Diff * diff = dyn_cast<Diff>(re)) {
        marker = process(diff, marker, pb);
    }
    else if (Intersect * ix = dyn_cast<Intersect>(re)) {
        marker = process(ix, marker, pb);
    }
    else if (isa<Start>(re)) {
        PabloAST * const sol = pb.createNot(pb.createAdvance(pb.createNot(mLineFeed), 1));
        marker = pb.createAssign("sol", pb.createAnd(pb.createVar(marker), sol));
    }
    else if (isa<End>(re)) {
        marker = pb.createAssign("eol", pb.createAnd(pb.createVar(marker), mLineFeed));
    }
    return marker;
}

inline Assign * RE_Compiler::process(Name * name, Assign * marker, PabloBlock & pb) {
    PabloAST * markerVar = pb.createVar(marker);
    if (name->getType() != Name::Type::FixedLength) {
        // Move the markers forward through any nonfinal UTF-8 bytes to the final position of each character.
        markerVar = pb.createAnd(markerVar, mInitial);
        markerVar = pb.createScanThru(markerVar, mNonFinal);
    }
    return pb.createAssign("m", pb.createAdvance(pb.createAnd(character_class_strm(name, pb), markerVar), 1));
}

inline Assign * RE_Compiler::process(Seq * seq, Assign * marker, PabloBlock & pb) {
    for (RE * re : *seq) {
        marker = process(re, marker, pb);
    }
    return marker;
}

inline Assign * RE_Compiler::process(Alt * alt, Assign * marker, PabloBlock & pb) {
    if (alt->empty()) {
        marker = pb.createAssign("fail", pb.createZeroes()); // always fail (note: I'm not sure this ever occurs. How do I create a 0-element alternation?)
    }
    else {
        auto i = alt->begin();
        Assign * const base = marker;
        marker = process(*i, base, pb);
        while (++i != alt->end()) {
            Assign * other = process(*i, base, pb);
            marker = pb.createAssign("alt", pb.createOr(pb.createVar(marker), pb.createVar(other)));
        }
    }    
    return marker;
}

inline Assign * RE_Compiler::process(Diff * diff, Assign * marker, PabloBlock & pb) {
    RE * lh = diff->getLH();
    RE * rh = diff->getRH();
    if ((isa<Any>(lh) || isa<Name>(lh)) && (isa<Any>(rh) || isa<Name>(rh))) {
        Assign * t1 = process(lh, marker, pb);
        Assign * t2 = process(rh, marker, pb);
        return pb.createAssign("diff", pb.createAnd(pb.createVar(t1), pb.createNot(pb.createVar(t2))));
    }
    throw std::runtime_error("Unsupported Diff operands: " + Printer_RE::PrintRE(diff));
}

inline Assign * RE_Compiler::process(Intersect * x, Assign * marker, PabloBlock & pb) {
    RE * lh = x->getLH();
    RE * rh = x->getRH();
    if ((isa<Any>(lh) || isa<Name>(lh)) && (isa<Any>(rh) || isa<Name>(rh))) {
        Assign * t1 = process(lh, marker, pb);
        Assign * t2 = process(rh, marker, pb);
        return pb.createAssign("intersect", pb.createAnd(pb.createVar(t1), pb.createVar(t2)));
    }
    throw std::runtime_error("Unsupported Intersect operands: " + Printer_RE::PrintRE(x));
}

inline Assign * RE_Compiler::process(Rep * rep, Assign * marker, PabloBlock & pb) {
    int lb = rep->getLB();
    int ub = rep->getUB();
    if (lb > 0) {
        marker = processLowerBound(rep->getRE(), lb, marker, pb);
    }
    if (ub == Rep::UNBOUNDED_REP) {
        marker = processUnboundedRep(rep->getRE(), marker, pb);
    }
    else { // if (rep->getUB() != Rep::UNBOUNDED_REP)
        marker = processBoundedRep(rep->getRE(), ub - lb, marker, pb);
    }    
    return marker;
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
    return isa<Name>(regexp) && ((cast<Name>(regexp)->getType()) == Name::Type::FixedLength);
}

inline Assign * RE_Compiler::processLowerBound(RE * repeated, int lb, Assign * marker, PabloBlock & pb) {
	if (isFixedLength(repeated)) {
        Name * name = cast<Name>(repeated);
        Assign * cc_lb = consecutive(pb.createAssign("repeated", pb.createAdvance(name->getVar(), 1)), 1, lb, pb);
		return pb.createAssign("lowerbound", pb.createAnd(pb.createAdvance(pb.createVar(marker), lb), pb.createVar(cc_lb)));
	}
	// Fall through to general case.
	while (lb-- != 0) {
		marker = process(repeated, marker, pb);
	}
	return marker;
}

inline Assign * RE_Compiler::processBoundedRep(RE * repeated, int ub, Assign * marker, PabloBlock & pb) {
	if (isFixedLength(repeated)) {
		// log2 upper bound for fixed length (=1) class
		// Mask out any positions that are more than ub positions from a current match.
		// Use matchstar, then apply filter.
		Assign * nonMatch = pb.createAssign("nonmatch", pb.createNot(pb.createVar(marker)));
		PabloAST * upperLimitMask = pb.createNot(pb.createVar(consecutive(nonMatch, 1, ub + 1, pb)));
        PabloAST * rep_class_var = cast<Name>(repeated)->getVar();
        return pb.createAssign("bounded", pb.createAnd(pb.createMatchStar(pb.createVar(marker), rep_class_var), upperLimitMask));
	}
	// Fall through to general case.
	while (ub-- != 0) {
		Assign * alt = process(repeated, marker, pb);
		marker = pb.createAssign("m", pb.createOr(pb.createVar(marker), pb.createVar(alt)));
	}
	return marker;
}

inline Assign * RE_Compiler::processUnboundedRep(RE * repeated, Assign * marker, PabloBlock & pb) {

    PabloAST * unbounded = nullptr;

    if (isa<Name>(repeated)) {
        Name * name = cast<Name>(repeated);

        PabloAST * cc = character_class_strm(name, pb);

        unbounded = pb.createVar(marker);
        if (name->getType() == Name::Type::FixedLength) {
            unbounded = pb.createMatchStar(unbounded, cc);
        }
        else { // Name::Unicode and Name::UnicodeCategory
            unbounded = pb.createAnd(pb.createMatchStar(unbounded, pb.createOr(mNonFinal, cc)), mInitial);
        }        
    }
    else if (isa<Any>(repeated)) {
        PabloAST * dot = pb.createNot(mLineFeed);
        unbounded = pb.createVar(marker);
        unbounded = pb.createAnd(pb.createMatchStar(unbounded, pb.createOr(mNonFinal, dot)), mInitial);
    }
    else if (isa<Diff>(repeated) && isa<Any>(cast<Diff>(repeated)->getLH()) && isa<Name>(cast<Diff>(repeated)->getRH())) {
        Name * name = cast<Name>(cast<Diff>(repeated)->getRH());
        PabloAST * cc = pb.createNot(pb.createOr(character_class_strm(name, pb), mLineFeed));
        unbounded = pb.createVar(marker);
        unbounded = pb.createAnd(pb.createMatchStar(unbounded, pb.createOr(mNonFinal, cc)), mInitial);
    }
    else {
        Var * markerVar = pb.createVar(marker);
        Assign * whileTest = pb.createAssign("test", markerVar);
        Assign * whileAccum = pb.createAssign("accum", markerVar);

        PabloBlock wb(pb);

        Var * loopComputation = wb.createVar(process(repeated, whileTest, wb));

        Var * whileAccumVar = wb.createVar(whileAccum);

        Next * nextWhileTest = wb.createNext(whileTest, wb.createAnd(loopComputation, wb.createNot(whileAccumVar)));

        wb.createNext(whileAccum, wb.createOr(loopComputation, whileAccumVar));

        pb.createWhile(wb.createVar(nextWhileTest), std::move(wb));

        unbounded = whileAccumVar;
    }    
    return pb.createAssign("unbounded", unbounded);
}

} // end of namespace re
