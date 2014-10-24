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

void RE_Compiler::compile(RE * re, PabloBlock & pb) {

    mLineFeed = mNameMap["LineFeed"]->getVar();

    const std::string initial = "initial";
    const std::string nonfinal = "nonfinal";

    if (hasUnicode(re)) {
        //Set the 'internal.initial' bit stream for the utf-8 multi-byte encoding.        
        PabloAST * u8single = mNameMap["UTF8-SingleByte"]->getVar();
        PabloAST * u8pfx2 = mNameMap["UTF8-Prefix2"]->getVar();
        PabloAST * u8pfx3 = mNameMap["UTF8-Prefix3"]->getVar();
        PabloAST * u8pfx4 = mNameMap["UTF8-Prefix4"]->getVar();
        PabloAST * u8pfx = pb.createOr(pb.createOr(u8pfx2, u8pfx3), u8pfx4);
        mInitial = pb.createVar(pb.createAssign(initial, pb.createOr(u8pfx, u8single)));
        #ifdef USE_IF_FOR_NONFINAL
        mNonFinal = pb.createVar(pb.createAssign(gs_nonfinal, pb.createZeroes()));
        #endif
        PabloAST * u8scope32 = pb.createAdvance(u8pfx3, 1);
        PabloAST * u8scope42 = pb.createAdvance(u8pfx4, 1);
        PabloAST * u8scope43 = pb.createAdvance(u8scope42, 1);
        #ifdef USE_IF_FOR_NONFINAL
        PabloBlock it(pb);
        it.createAssign(gs_nonfinal, it.createOr(it.createOr(u8pfx, u8scope32), it.createOr(u8scope42, u8scope43)));
        pb.createIf(u8pfx, std::move(it));
        #else        
        mNonFinal = pb.createVar(pb.createAssign(nonfinal, pb.createOr(pb.createOr(u8pfx, u8scope32), pb.createOr(u8scope42, u8scope43))));
        #endif
    }
    else {
        mInitial = pb.createZeroes();
        mNonFinal = pb.createZeroes();
    }

    Assign * start_marker = pb.createAssign("start", pb.createOnes());
    PabloAST * result = process(re, start_marker, pb);

    //These three lines are specifically for grep.
    pb.createAssign("matches", pb.createAnd(pb.createMatchStar(pb.createVar(result), pb.createNot(mLineFeed)), mLineFeed));
}


Assign * RE_Compiler::process(RE * re, Assign * target, PabloBlock & pb) {
    if (Name * name = dyn_cast<Name>(re)) {
        target = process(name, target, pb);
    }
    else if (Seq* seq = dyn_cast<Seq>(re)) {
        target = process(seq, target, pb);
    }
    else if (Alt * alt = dyn_cast<Alt>(re)) {
        target = process(alt, target, pb);
    }
    else if (Rep * rep = dyn_cast<Rep>(re)) {
        target = process(rep, target, pb);
    }
    else if (isa<Any>(re)) {
        // Move the markers forward through any nonfinal UTF-8 bytes to the final position of each character.
        PabloAST * marker = pb.createVar(target);
        marker = pb.createAnd(marker, mInitial);
        marker = pb.createScanThru(marker, mNonFinal);
        PabloAST * dot = pb.createNot(mLineFeed);
        target = pb.createAssign("dot", pb.createAdvance(pb.createAnd(marker, dot), 1));
    }
    else if (Diff * diff = dyn_cast<Diff>(re)) {
        target = process(diff, target, pb);
    }
    else if (isa<Start>(re)) {
        PabloAST * const sol = pb.createNot(pb.createAdvance(pb.createNot(mLineFeed), 1));
        target = pb.createAssign("sol", pb.createAnd(pb.createVar(target), sol));
    }
    else if (isa<End>(re)) {
        target = pb.createAssign("eol", pb.createAnd(pb.createVar(target), mLineFeed));
    }
    return target;
}

inline Assign * RE_Compiler::process(Name * name, Assign * target, PabloBlock & pb) {
    PabloAST * marker = pb.createVar(target);
    if (name->getType() != Name::Type::FixedLength) {
        // Move the markers forward through any nonfinal UTF-8 bytes to the final position of each character.
        marker = pb.createAnd(marker, mInitial);
        marker = pb.createScanThru(marker, mNonFinal);
    }
    PabloAST * cc = nullptr;
    if (name->getType() == Name::Type::UnicodeCategory) {
        cc = pb.createCall(name->getName());
    }
    else {
        cc = pb.createVar(name->getName());
    }
    return pb.createAssign("m", pb.createAdvance(pb.createAnd(cc, marker), 1));
}

inline Assign * RE_Compiler::process(Seq * seq, Assign *target, PabloBlock & pb) {
    for (RE * re : *seq) {
        target = process(re, target, pb);
    }
    return target;
}

inline Assign * RE_Compiler::process(Alt * alt, Assign * target, PabloBlock & pb) {
    if (alt->empty()) {
        target = pb.createAssign("fail", pb.createZeroes()); // always fail (note: I'm not sure this ever occurs. How do I create a 0-element alternation?)
    }
    else {
        auto i = alt->begin();
        Assign * const base = target;
        target = process(*i, base, pb);
        while (++i != alt->end()) {
            Assign * other = process(*i, base, pb);
            target = pb.createAssign("alt", pb.createOr(pb.createVar(target), pb.createVar(other)));
        }
    }    
    return target;
}

inline Assign * RE_Compiler::process(Diff * diff, Assign * target, PabloBlock & pb) {
    RE * lh = diff->getLH();
    RE * rh = diff->getRH();
    if ((isa<Any>(lh) || isa<Name>(lh)) && (isa<Any>(rh) || isa<Name>(rh))) {
        Assign * t1 = process(lh, target, pb);
        Assign * t2 = process(rh, target, pb);
        return pb.createAssign("diff", pb.createAnd(pb.createVar(t1), pb.createNot(pb.createVar(t2))));
    }
    throw std::runtime_error("Unsupported Diff operands: " + Printer_RE::PrintRE(diff));
}

inline Assign * RE_Compiler::process(Rep * rep, Assign * target, PabloBlock & pb) {
    int lb = rep->getLB();
    int ub = rep->getUB();
    if (lb > 0) {
        target = processLowerBound(rep->getRE(), lb, target, pb);
    }
    if (ub == Rep::UNBOUNDED_REP) {
        target = processUnboundedRep(rep->getRE(), target, pb);
    }
    else { // if (rep->getUB() != Rep::UNBOUNDED_REP)
        target = processBoundedRep(rep->getRE(), ub - lb, target, pb);
    }    
    return target;
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
        return isa<Name>(regexp) && ((cast<Name>(regexp) -> getType()) == Name::Type::FixedLength);
}
inline Assign * RE_Compiler::processLowerBound(RE * repeated, int lb, Assign * marker, PabloBlock & pb) {
#ifndef VARIABLE_ADVANCE
    while (lb-- != 0) {
        marker = process(repeated, marker, pb);
    }
    return marker;
#endif
    
#ifdef VARIABLE_ADVANCE
	if (isFixedLength(repeated)) {
        Name * rep_name = cast<Name>(repeated);
		Assign * cc_lb = consecutive(pb.createAssign("repeated", pb.createAdvance(pb.createVar(rep_name->getName()), 1)), 1, lb, pb);
		return pb.createAssign("lowerbound", pb.createAnd(pb.createAdvance(pb.createVar(marker), lb), pb.createVar(cc_lb)));
	}
	else {
		while (lb-- != 0) {
			marker = process(repeated, marker, pb);
		}
		return marker;
	}
#endif

}

inline Assign * RE_Compiler::processUnboundedRep(RE * repeated, Assign * target, PabloBlock & pb) {

    PabloAST * unbounded = nullptr;

    if (isa<Name>(repeated)) {
        Name * rep_name = cast<Name>(repeated);

        PabloAST * cc;
        if (rep_name->getType() == Name::Type::UnicodeCategory) {
            cc = pb.createCall(rep_name->getName());
        }
        else {
            cc = pb.createVar(rep_name->getName());
        }

        unbounded = pb.createVar(target);
        if (rep_name->getType() == Name::Type::FixedLength) {
            unbounded = pb.createMatchStar(unbounded, cc);
        }
        else { // Name::Unicode and Name::UnicodeCategory
            unbounded = pb.createAnd(pb.createMatchStar(unbounded, pb.createOr(mNonFinal, cc)), mInitial);
        }        
    }
    else if (isa<Any>(repeated)) {
        PabloAST * dot = pb.createNot(mLineFeed);
        unbounded = pb.createVar(target);
        unbounded = pb.createAnd(pb.createMatchStar(unbounded, pb.createOr(mNonFinal, dot)), mInitial);
    }
    else {

        Var * targetVar = pb.createVar(target);

        Assign * whileTest = pb.createAssign("test", targetVar);
        Assign * whileAccum = pb.createAssign("accum", targetVar);

        PabloBlock wt(pb);

        Var * loopComputation = wt.createVar(process(repeated, whileTest, wt));

        Var * whileAccumVar = wt.createVar(whileAccum);

        wt.createNext(whileTest, wt.createAnd(loopComputation, wt.createNot(whileAccumVar)));

        wt.createNext(whileAccum, wt.createOr(loopComputation, whileAccumVar));

        pb.createWhile(pb.createVar(whileTest), std::move(wt));

        unbounded = whileAccumVar;
    }    
    return pb.createAssign("unbounded", unbounded);
}

inline Assign * RE_Compiler::processBoundedRep(RE * repeated, int ub, Assign * target, PabloBlock & pb) {
    while (ub-- != 0) {
        Assign * alt = process(repeated, target, pb);
        target = pb.createAssign("m", pb.createOr(pb.createVar(target), pb.createVar(alt)));
    }
    return target;
}


bool RE_Compiler::hasUnicode(const RE * re) {
    bool found = false;
    if (re == nullptr) {
        throw std::runtime_error("Unexpected Null Value passed to RE Compiler!");
    }
    else if (isa<Any>(re)) {
        found = true;
    }
    else if (isa<Diff>(re)) {
        found = true;
    }
    else if (const Name * name = dyn_cast<const Name>(re)) {
        if ((name->getType() == Name::Type::UnicodeCategory) || (name->getType() == Name::Type::Unicode)) {
            found = true;
        }
    }
    else if (const Seq * re_seq = dyn_cast<const Seq>(re)) {
        for (auto i = re_seq->cbegin(); i != re_seq->cend(); ++i) {
            if (hasUnicode(*i)) {
                found = true;
                break;
            }
        }
    }
    else if (const Alt * re_alt = dyn_cast<const Alt>(re)) {
        for (auto i = re_alt->cbegin(); i != re_alt->cend(); ++i) {
            if (hasUnicode(*i)) {
                found = true;
                break;
            }
        }
    }
    else if (const Rep * rep = dyn_cast<const Rep>(re)) {
        found = hasUnicode(rep->getRE());
    }
    return found;
}

} // end of namespace re
