/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */



#include "re_compiler.h"
#include <pablo/pe_ones.h>          // for Ones
#include <pablo/pe_var.h>           // for Var
#include <pablo/pe_zeroes.h>        // for Zeroes
#include <re/printer_re.h>
#include <re/re_alt.h>
#include <re/re_analysis.h>         // for isByteLength, isUnicodeUnitLength
#include <re/re_any.h>
#include <re/re_assertion.h>        // for Assertion, Assertion::Sense, Asse...
#include <re/re_cc.h>               // for makeCC
#include <re/re_diff.h>             // for Diff
#include <re/re_end.h>
#include <re/re_intersect.h>        // for Intersect
#include <re/re_name.h>             // for Name, Name::Type, Name::Type::Zer...
#include <re/re_name_resolve.h>     // for resolveNames
#include <re/re_name_gather.h>      // for gatherNames
#include <re/re_rep.h>              // for Rep, Rep::::UNBOUNDED_REP
#include <re/re_seq.h>              // for Seq
#include <re/re_start.h>
#include <re/re_local.h>
#include <re/re_toolchain.h>        // for AlgorithmOptionIsSet, RE_Algorith...
#include "cc/cc_compiler.h"         // for CC_Compiler
#include "pablo/builder.hpp"        // for PabloBuilder

namespace pablo { class PabloAST; }
namespace pablo { class PabloKernel; }
namespace re { class Alt; }
namespace re { class RE; }


#define UNICODE_LINE_BREAK (!AlgorithmOptionIsSet(DisableUnicodeLineBreak))

using namespace pablo;
using namespace llvm;

namespace re {

RE * RE_Compiler::resolveUnicodeProperties(RE * re) {
    Name * ZeroWidth = nullptr;
    mCompiledName = &mBaseMap;
    gatherNames(re, ZeroWidth);
    // Now precompile any grapheme segmentation rules
    if (ZeroWidth) {
        mCompiledName->add(ZeroWidth, compileName(ZeroWidth, mPB));
    }
    return re;
}

void RE_Compiler::compileUnicodeNames(RE *& re) {
    re = resolveUnicodeProperties(re);
}

void RE_Compiler::compile(RE * re) {
    MarkerType match_results = compile(re, mPB);
    PabloAST * match_post = markerVar(AdvanceMarker(match_results, MarkerPosition::FinalPostPositionUnit, mPB));
    Var * const output = mKernel->getOutputStreamVar("matches");
    
    mPB.createAssign(mPB.createExtract(output, mPB.getInteger(0)), match_post);
}
    
MarkerType RE_Compiler::compile(RE * re, PabloBuilder & pb) {
    return process(re, makeMarker(MarkerPosition::FinalPostPositionUnit, pb.createOnes()), pb);
}
    
MarkerType RE_Compiler::compile_local(RE * re, MarkerType marker, PabloBuilder & pb) {
    CC * first = RE_Local::first(re);
    CC * final = RE_Local::final(re);

    if (first == nullptr || final == nullptr) {
        mLocal = false;
        return process(re, marker, pb);
    }

    PabloAST * pablo_first = mCCCompiler.compileCC(first);
    PabloAST * pablo_final = mCCCompiler.compileCC(final);
    std::map<CC*, CC*> follow_map;
    RE_Local::follow(re, follow_map);

    PabloAST * pablo_follow = pb.createZeroes();
    for (auto i = follow_map.begin(); i != follow_map.end(); i++) {
        CC * one = makeCC(std::move(*i->first));
        CC * two = makeCC(std::move(*i->second));
        PabloAST * pablo_one = pb.createAnd(mCCCompiler.compileCC(one), mAny);
        PabloAST * pablo_two = pb.createAnd(mCCCompiler.compileCC(two), mAny);
        PabloAST * one1 = pb.createAdvance(pablo_one, 1, "one1");
        PabloAST * follow = pb.createAnd(pb.createScanThru(pb.createAnd(mInitial, one1), mNonFinal), pablo_two);
        pablo_follow = pb.createOr(pablo_follow, follow);
    }
    PabloAST * result = pb.createAnd(pb.createMatchStar(pb.createAdvance(pablo_first, 1), pb.createOr(pablo_follow, mNonFinal)), pb.createAdvance(pablo_final, 1));
    return makeMarker(MarkerPosition::FinalPostPositionUnit, result);
}

    
MarkerType RE_Compiler::process(RE * re, MarkerType marker, PabloBuilder & pb) {
    if (mLocal) {
        if (isa<Name>(re) || isa<Seq>(re) || isa<Alt>(re) || isa<Rep>(re) || isa<CC>(re)) {
            return compile_local(re, marker, pb);
        } else if (isa<Any>(re)) {
            return compileAny(marker, pb);
        } else if (isa<Diff>(re)) {
            return compileDiff(cast<Diff>(re), marker, pb);
        } else if (isa<Intersect>(re)) {
            return compileIntersect(cast<Intersect>(re), marker, pb);
        }
        UnsupportedRE("RE Compiler for local language failed to process " + Printer_RE::PrintRE(re));
    } else {
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
        } else if (isa<CC>(re)) {
            // CCs may be passed through the toolchain directly to the compiler.
            return compileCC(cast<CC>(re), marker, pb);
        }
        UnsupportedRE("RE Compiler failed to process " + Printer_RE::PrintRE(re));
    }
}

inline MarkerType RE_Compiler::compileAny(const MarkerType m, PabloBuilder & pb) {
    PabloAST * nextFinalByte = markerVar(AdvanceMarker(m, MarkerPosition::FinalPostPositionUnit, pb));
    PabloAST * lb = mLineBreak;
    if (UNICODE_LINE_BREAK) {
        lb = pb.createOr(mLineBreak, mCRLF);
    }
    return makeMarker(MarkerPosition::FinalMatchUnit, pb.createAnd(nextFinalByte, pb.createNot(lb), "dot"));
}

MarkerType RE_Compiler::compileCC(CC * cc, MarkerType marker, PabloBuilder & pb) {
    MarkerType nextPos;
    if (markerPos(marker) == MarkerPosition::FinalPostPositionUnit) {
        nextPos = marker;
    } else {
        nextPos = AdvanceMarker(marker, MarkerPosition::FinalPostPositionUnit, pb);
    }
    return makeMarker(MarkerPosition::FinalMatchUnit, pb.createAnd(markerVar(marker), pb.createAnd(mCCCompiler.compileCC(cc, pb), mAny)));
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
    UnsupportedRE("Unresolved name " + name->getName());
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
    PabloAST * combine = pb.createAdvance(accum[MarkerPosition::FinalMatchUnit], 1, "combine");
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
    } else if (a->getKind() == Assertion::Kind::Boundary) {
        MarkerType cond = compile(asserted, pb);
        if (LLVM_LIKELY(markerPos(cond) == MarkerPosition::FinalMatchUnit)) {
            MarkerType postCond = AdvanceMarker(cond, MarkerPosition::FinalPostPositionUnit, pb);
            PabloAST * boundaryCond = pb.createAnd(mAny, pb.createXor(markerVar(cond), markerVar(postCond)));
            if (a->getSense() == Assertion::Sense::Negative) {
                boundaryCond = pb.createNot(boundaryCond);
            }
            MarkerType fbyte = AdvanceMarker(marker, MarkerPosition::FinalPostPositionUnit, pb);
            return makeMarker(MarkerPosition::FinalPostPositionUnit, pb.createAnd(markerVar(fbyte), boundaryCond, "boundary"));
        }
        else UnsupportedRE("Unsupported boundary assertion");
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
    UnsupportedRE("Unsupported lookahead assertion.");
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
    UnsupportedRE("Unsupported Diff operands: " + Printer_RE::PrintRE(diff));
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
    UnsupportedRE("Unsupported Intersect operands: " + Printer_RE::PrintRE(x));
}

MarkerType RE_Compiler::compileRep(Rep * rep, MarkerType marker, PabloBuilder & pb) {
    int lb = rep->getLB();
    int ub = rep->getUB();
    if (lb > 0) {
        marker = processLowerBound(rep->getRE(), lb, marker, IfInsertionGap, pb);
    }
    if (ub == Rep::UNBOUNDED_REP) {
        marker = processUnboundedRep(rep->getRE(), marker, pb);
    } else if (lb < ub) {
        marker = processBoundedRep(rep->getRE(), ub - lb, marker, IfInsertionGap, pb);
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
    while ((i * 2) < total) {
        PabloAST * v = consecutive_i;
        PabloAST * v2 =  pb.createAdvance(v, i);
        i *= 2;
        consecutive_i = pb.createAnd(v, v2, "at" + std::to_string(i) + "of" + std::to_string(total));
    }
    if (LLVM_LIKELY(i < total)) {
        PabloAST * v = consecutive_i;
        PabloAST * v2 = pb.createAdvance(v, total - i);
        consecutive_i = pb.createAnd(v, v2, "at" + std::to_string(total));
    }
    return consecutive_i;
}

inline PabloAST * RE_Compiler::reachable(PabloAST * repeated, int length, int repeat_count, PabloBuilder & pb) {
    int i = length;
    int total_lgth = repeat_count * length;
    if (repeat_count == 0) {
        return repeated;
    }
    PabloAST * reachable = pb.createOr(repeated, pb.createAdvance(repeated, 1), "within1");
    while ((i * 2) < total_lgth) {
        PabloAST * v = reachable;
        PabloAST * v2 =  pb.createAdvance(v, i);
        i *= 2;
        reachable = pb.createOr(v, v2, "within" + std::to_string(i));
    }
    if (LLVM_LIKELY(i < total_lgth)) {
        PabloAST * v = reachable;
        PabloAST * v2 = pb.createAdvance(v, total_lgth - i);
        reachable = pb.createOr(v, v2, "within" + std::to_string(total_lgth));
    }
    return reachable;
}

MarkerType RE_Compiler::processLowerBound(RE * repeated, int lb, MarkerType marker, int ifGroupSize, PabloBuilder & pb) {
    if (lb == 0) {
        return marker;
    } else if (!mGraphemeBoundaryRule && isByteLength(repeated) && !AlgorithmOptionIsSet(DisableLog2BoundedRepetition)) {
        PabloAST * cc = markerVar(compile(repeated, pb));
        PabloAST * cc_lb = consecutive_matches(cc, 1, lb, pb);
        const auto pos = markerPos(marker) == MarkerPosition::FinalMatchUnit ? lb : lb - 1;
        PabloAST * marker_fwd = pb.createAdvance(markerVar(marker), pos);
        return makeMarker(MarkerPosition::FinalMatchUnit, pb.createAnd(marker_fwd, cc_lb, "lowerbound"));
    }
    // Fall through to general case.  Process the first item and insert the rest into an if-structure.
    auto group = ifGroupSize < lb ? ifGroupSize : lb;
    for (auto i = 0; i < group; i++) {
        marker = process(repeated, marker, pb);
        if (mGraphemeBoundaryRule) {
            marker = AdvanceMarker(marker, MarkerPosition::FinalPostPositionUnit, pb);
        }
    }
    if (lb == group) {
        return marker;
    }
    Var * m = pb.createVar("m", pb.createZeroes());
    PabloBuilder nested = PabloBuilder::Create(pb);
    NameMap nestedMap(mCompiledName);
    mCompiledName = &nestedMap;
    MarkerType m1 = processLowerBound(repeated, lb - group, marker, ifGroupSize * 2, nested);
    nested.createAssign(m, markerVar(m1));
    pb.createIf(markerVar(marker), nested);
    mCompiledName = nestedMap.getParent();
    return makeMarker(m1.pos, m);
}
    
MarkerType RE_Compiler::processBoundedRep(RE * repeated, int ub, MarkerType marker, int ifGroupSize,  PabloBuilder & pb) {
    if (LLVM_UNLIKELY(ub == 0)) {
        return marker;
    } else if (!mGraphemeBoundaryRule && isByteLength(repeated) && ub > 1 && !AlgorithmOptionIsSet(DisableLog2BoundedRepetition)) {
        // log2 upper bound for fixed length (=1) class
        // Create a mask of positions reachable within ub from current marker.
        // Use matchstar, then apply filter.
        PabloAST * cursor = markerVar(AdvanceMarker(marker, MarkerPosition::FinalPostPositionUnit, pb));
        // If we've advanced the cursor to the post position unit, cursor begins on the first masked bit of the bounded mask.
        // Extend the mask by ub - 1 byte positions to ensure the mask ends on the FinalMatchUnit of the repeated region.
        PabloAST * upperLimitMask = reachable(cursor, 1, ub - 1, pb);
        PabloAST * masked = pb.createAnd(markerVar(compile(repeated, pb)), upperLimitMask);
        PabloAST * nonFinal = mNonFinal;
        if (mGraphemeBoundaryRule) {
            nonFinal = pb.createOr(nonFinal, pb.createNot(mGraphemeBoundaryRule, "gext"));
        }
        // MatchStar deposits any cursors on the post position. However those cursors may may land on the initial "byte" of a
        // "multi-byte" character. Combine the masked range with any nonFinals.
        PabloAST * bounded = pb.createMatchStar(cursor, pb.createOr(masked, nonFinal), "bounded");
        return makeMarker(MarkerPosition::FinalPostPositionUnit, bounded);
    }
    // Fall through to general case.  Process the first item and insert the rest into an if-structure.
    auto group = ifGroupSize < ub ? ifGroupSize : ub;
    for (auto i = 0; i < group; i++) {
        MarkerType a = process(repeated, marker, pb);
        MarkerType m = marker;
        AlignMarkers(a, m, pb);
        marker = makeMarker(markerPos(a), pb.createOr(markerVar(a), markerVar(m)));
        if (mGraphemeBoundaryRule) {
            marker = AdvanceMarker(marker, MarkerPosition::FinalPostPositionUnit, pb);
        }
    }
    if (ub == group) {
        return marker;
    }
    Var * m1a = pb.createVar("m", pb.createZeroes());
    PabloBuilder nested = PabloBuilder::Create(pb);
    NameMap nestedMap(mCompiledName);
    mCompiledName = &nestedMap;
    MarkerType m1 = processBoundedRep(repeated, ub - group, marker, ifGroupSize * 2, nested);
    nested.createAssign(m1a, markerVar(m1));
    pb.createIf(markerVar(marker), nested);
    mCompiledName = nestedMap.getParent();
    return makeMarker(m1.pos, m1a);
}

MarkerType RE_Compiler::processUnboundedRep(RE * repeated, MarkerType marker, PabloBuilder & pb) {
    // always use PostPosition markers for unbounded repetition.
    PabloAST * base = markerVar(AdvanceMarker(marker, MarkerPosition::FinalPostPositionUnit, pb));
    if (!mGraphemeBoundaryRule && isByteLength(repeated)  && !AlgorithmOptionIsSet(DisableMatchStar)) {
        PabloAST * mask = markerVar(compile(repeated, pb));
        PabloAST * nonFinal = mNonFinal;
        if (mGraphemeBoundaryRule) {
            nonFinal = pb.createOr(nonFinal, pb.createNot(mGraphemeBoundaryRule, "gext"));
        }
        // The post position character may land on the initial byte of a multi-byte character. Combine them with the masked range.
        PabloAST * unbounded = pb.createMatchStar(base, pb.createOr(mask, nonFinal), "unbounded");
        return makeMarker(MarkerPosition::FinalPostPositionUnit, unbounded);
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
        NameMap nestedMap(mCompiledName);
        mCompiledName = &nestedMap;
        mStarDepth++;
        MarkerType result = process(repeated, makeMarker(MarkerPosition::FinalPostPositionUnit, whilePending), wb);
        result = AdvanceMarker(result, MarkerPosition::FinalPostPositionUnit, wb);
        PabloAST * loopComputation = markerVar(result);
        wb.createAssign(whilePending, wb.createAnd(loopComputation, wb.createNot(whileAccum)));
        wb.createAssign(whileAccum, wb.createOr(loopComputation, whileAccum));
        wb.createAssign(whileTest, wb.createOr(mWhileTest, whilePending));
        pb.createWhile(whileTest, wb);
        mStarDepth--;
        mCompiledName = nestedMap.getParent();
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
            PabloAST * starts = pb.createAnd(mInitial, marker.stream);
            marker.stream = pb.createScanThru(starts, nonFinal, "fpp");
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
    
LLVM_ATTRIBUTE_NORETURN void RE_Compiler::UnsupportedRE(std::string errmsg) {
    llvm::report_fatal_error(errmsg);
}
    
    

RE_Compiler::RE_Compiler(PabloKernel * kernel, cc::CC_Compiler & ccCompiler, bool local)
: mKernel(kernel)
, mCCCompiler(ccCompiler)
, mLocal(local)
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
    Var * const linebreak = mKernel->getInputStreamVar("linebreak");
    mLineBreak = mPB.createExtract(linebreak, mPB.getInteger(0));
    mAny = mPB.createNot(mLineBreak, "any");
    Var * const required = mKernel->getInputStreamVar("required");
    mInitial = mPB.createExtract(required, mPB.getInteger(0));
    mNonFinal = mPB.createExtract(required, mPB.getInteger(1));
    mFinal = mPB.createExtract(required, mPB.getInteger(2));
    mCRLF = mPB.createExtract(required, mPB.getInteger(3));
}

} // end of namespace re
