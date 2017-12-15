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
#include <IR_Gen/idisa_target.h>    // for AVX2_available
#include <llvm/Support/ErrorHandling.h>

namespace pablo { class PabloAST; }
namespace pablo { class PabloKernel; }
namespace re { class Alt; }
namespace re { class RE; }

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

RE * RE_Compiler::compileUnicodeNames(RE * re) {
    return resolveUnicodeProperties(re);
}

PabloAST * RE_Compiler::compile(RE * re) {
    return markerVar(AdvanceMarker(compile(re, mPB), MarkerPosition::FinalPostPositionUnit, mPB));
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
    } else if (isa<CC>(re)) {
        // CCs may be passed through the toolchain directly to the compiler.
        return compileCC(cast<CC>(re), marker, pb);
    } else {
        UnsupportedRE("RE Compiler failed to process " + Printer_RE::PrintRE(re));
    }
}

inline MarkerType RE_Compiler::compileAny(const MarkerType m, PabloBuilder & pb) {
    PabloAST * const nextFinalByte = markerVar(AdvanceMarker(m, MarkerPosition::FinalPostPositionUnit, pb));
    return makeMarker(MarkerPosition::FinalMatchUnit, nextFinalByte);
}

MarkerType RE_Compiler::compileCC(CC * cc, MarkerType marker, PabloBuilder & pb) {
    PabloAST * const nextPos = markerVar(AdvanceMarker(marker, MarkerPosition::FinalPostPositionUnit, pb));
    return makeMarker(MarkerPosition::FinalMatchUnit, pb.createAnd(nextPos, mCCCompiler.compileCC(cc, pb)));
}

inline MarkerType RE_Compiler::compileName(Name * name, MarkerType marker, PabloBuilder & pb) {
    const std::string nameString = name->getName();
    if (nameString == ".") {
        return compileAny(marker, pb);
    } else if (nameString == "^"){
        return compileStart(marker, pb);
    } else if (nameString == "$"){
        return compileEnd(marker, pb);
    } else if (isUnicodeUnitLength(name)) {
        MarkerType nameMarker = compileName(name, pb);
        MarkerType nextPos = AdvanceMarker(marker, MarkerPosition::FinalPostPositionUnit, pb);
        nameMarker.stream = pb.createAnd(markerVar(nextPos), markerVar(nameMarker), name->getName());
        return nameMarker;
    } else if (name->getType() == Name::Type::ZeroWidth) {
        RE * zerowidth = name->getDefinition();
        MarkerType zero = compile(zerowidth, pb);
        AlignMarkers(marker, zero, pb);
        PabloAST * ze = markerVar(zero);
        if (nameString == "NonGCB") {
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
            PabloAST * boundaryCond = pb.createXor(markerVar(cond), markerVar(postCond));
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
   Given a stream |repeated_j| marking positions associated with |j| conecutive matches to an item
   compute a stream marking |repeat_count| consecutive occurrences of such items.
*/
    
PabloAST * RE_Compiler::consecutive_matches(PabloAST * repeated_j, int j, int repeat_count, PabloAST * indexStream, PabloBuilder & pb) {
    if (j == repeat_count) return repeated_j;
    int i = std::min(j, repeat_count - j);
    int k = j + i;
    if (/*j > IfInsertionGap*/ false) {
        Var * repeated = pb.createVar("repeated", pb.createZeroes());
        PabloBuilder nested = PabloBuilder::Create(pb);
        NameMap nestedMap(mCompiledName);
        mCompiledName = &nestedMap;
        
        PabloAST * adv_i = nullptr;
        if (indexStream == nullptr) adv_i = nested.createAdvance(repeated_j, i);
        else adv_i = nested.createIndexedAdvance(repeated_j, indexStream, i);
        PabloAST * repeated_k = nested.createAnd(repeated_j, adv_i, "at" + std::to_string(k) + "of" + std::to_string(repeat_count));
        nested.createAssign(repeated, consecutive_matches(repeated_k, k, repeat_count, indexStream, nested));
        pb.createIf(repeated_j, nested);
        mCompiledName = nestedMap.getParent();
        return repeated;
    }
    else {
        PabloAST * adv_i = nullptr;
        if (indexStream == nullptr) adv_i = pb.createAdvance(repeated_j, i);
        else adv_i = pb.createIndexedAdvance(repeated_j, indexStream, i);
        PabloAST * repeated_k = pb.createAnd(repeated_j, adv_i, "at" + std::to_string(k) + "of" + std::to_string(repeat_count));
        return consecutive_matches(repeated_k, k, repeat_count, indexStream, pb);
    }
}


inline PabloAST * RE_Compiler::reachable(PabloAST * repeated, int length, int repeat_count, PabloAST * indexStream, PabloBuilder & pb) {
    int i = length;
    int total_lgth = repeat_count * length;
    if (repeat_count == 0) {
        return repeated;
    }
    PabloAST * reachable = nullptr;
    if (indexStream == nullptr) reachable = pb.createOr(repeated, pb.createAdvance(repeated, 1), "within1");
    else reachable = pb.createOr(repeated, pb.createIndexedAdvance(repeated, indexStream, 1), "within1");
    while ((i * 2) < total_lgth) {
        PabloAST * v = reachable;
        PabloAST * v2 = nullptr;
        if (indexStream == nullptr) v2 = pb.createAdvance(v, i);
        else v2 = pb.createIndexedAdvance(v, indexStream, i);
        i *= 2;
        reachable = pb.createOr(v, v2, "within" + std::to_string(i));
    }
    if (LLVM_LIKELY(i < total_lgth)) {
        PabloAST * v = reachable;
        PabloAST * v2 = nullptr;
        if (indexStream == nullptr) v2 = pb.createAdvance(v, total_lgth - i);
        else v2 = pb.createIndexedAdvance(v, indexStream, total_lgth - i);
        reachable = pb.createOr(v, v2, "within" + std::to_string(total_lgth));
    }
    return reachable;
}

MarkerType RE_Compiler::processLowerBound(RE * repeated, int lb, MarkerType marker, int ifGroupSize, PabloBuilder & pb) {
    if (LLVM_UNLIKELY(lb == 0)) return marker;
    else if (LLVM_UNLIKELY(lb == 1)) return process(repeated, marker, pb);
    //
    // A bounded repetition with an upper bound of at least 2.
    if (!mGraphemeBoundaryRule && !AlgorithmOptionIsSet(DisableLog2BoundedRepetition)) {
        // Check for a regular expression that satisfies on of the special conditions that 
        // allow implementation using the log2 technique.
        if (isByteLength(repeated)) {
            PabloAST * cc = markerVar(compile(repeated, pb));
            PabloAST * cc_lb = consecutive_matches(cc, 1, lb, nullptr, pb);
            const auto pos = markerPos(marker) == MarkerPosition::FinalMatchUnit ? lb : lb - 1;
            PabloAST * marker_fwd = pb.createAdvance(markerVar(marker), pos);
            return makeMarker(MarkerPosition::FinalMatchUnit, pb.createAnd(marker_fwd, cc_lb, "lowerbound"));
        }
        else if (isUnicodeUnitLength(repeated)) {
            PabloAST * cc = markerVar(compile(repeated, pb));
            PabloAST * cc_lb = consecutive_matches(cc, 1, lb, mFinal, pb);
            const auto pos = markerPos(marker) == MarkerPosition::FinalMatchUnit ? lb : lb - 1;
            PabloAST * marker_fwd = pb.createIndexedAdvance(markerVar(marker), mFinal, pos);
            return makeMarker(MarkerPosition::FinalMatchUnit, pb.createAnd(marker_fwd, cc_lb, "lowerbound"));
        }
        else if (isTypeForLocal(repeated)) {
            CC * firstSymSet = RE_Local::first(repeated);
            std::map<CC *, CC*> followMap;
            RE_Local::follow(repeated, followMap);
            bool firstSymSet_found_in_follow = false;
            for (auto & entry : followMap) {
                if (entry.second->intersects(*firstSymSet)) {
                    firstSymSet_found_in_follow = true;
                }
            }
            if (!firstSymSet_found_in_follow) {
                // When the first symbol is unique, we can use it as an index stream.
                PabloAST * firstCCstream = markerVar(compile(firstSymSet, pb));
                // Find all matches to the repeated regexp.
                PabloAST * submatch = markerVar(AdvanceMarker(compile(repeated, pb), MarkerPosition::FinalPostPositionUnit, pb));
                // Consecutive submatches require that the symbol following the end of one submatch is the first symbol for
                // the next submatch.   lb-1 such submatches are required.
                PabloAST * consecutive_submatch = consecutive_matches(submatch, 1, lb-1, firstCCstream, pb);
                // Find submatch positions which are lb-2 start symbols forward from the current marker position.
                PabloAST * base = markerVar(AdvanceMarker(marker, MarkerPosition::FinalPostPositionUnit, pb));
                PabloAST * marker_fwd = pb.createIndexedAdvance(base, firstCCstream, lb - 1);
                PabloAST * consecutive_lb_1 = pb.createAnd(marker_fwd, consecutive_submatch);
                // From any of these positions, any position reachable by one more occurrence of the
                // regexp is a match.
                return process(repeated, makeMarker(MarkerPosition::FinalPostPositionUnit, consecutive_lb_1), pb);
            }
        }
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
    if (LLVM_UNLIKELY(ub == 0)) return marker;
    //
    // A bounded repetition with an upper bound of at least 2.
    if (!mGraphemeBoundaryRule && !AlgorithmOptionIsSet(DisableLog2BoundedRepetition) && (ub > 1)) {
        // Check for a regular expression that satisfies on of the special conditions that 
        // allow implementation using the log2 technique.
        if (isByteLength(repeated)) {
            // log2 upper bound for fixed length (=1) class
            // Create a mask of positions reachable within ub from current marker.
            // Use matchstar, then apply filter.
            PabloAST * cursor = markerVar(AdvanceMarker(marker, MarkerPosition::FinalPostPositionUnit, pb));
            // If we've advanced the cursor to the post position unit, cursor begins on the first masked bit of the bounded mask.
            // Extend the mask by ub - 1 byte positions to ensure the mask ends on the FinalMatchUnit of the repeated region.
            PabloAST * upperLimitMask = reachable(cursor, 1, ub - 1, nullptr, pb);
            PabloAST * masked = pb.createAnd(markerVar(compile(repeated, pb)), upperLimitMask);
            // MatchStar deposits any cursors on the post position. However those cursors may may land on the initial "byte" of a
            // "multi-byte" character. Combine the masked range with any nonFinals.
            PabloAST * bounded = pb.createMatchStar(cursor, pb.createOr(masked, mNonFinal), "bounded");
            return makeMarker(MarkerPosition::FinalPostPositionUnit, bounded);
        }
        else if (isUnicodeUnitLength(repeated)) {
            // For a regexp which represent a single Unicode codepoint, we can use the mFinal stream
            // as an index stream for an indexed advance operation.
            PabloAST * cursor = markerVar(AdvanceMarker(marker, MarkerPosition::FinalPostPositionUnit, pb));
            PabloAST * upperLimitMask = reachable(cursor, 1, ub - 1, mFinal, pb);
            PabloAST * masked = pb.createAnd(markerVar(compile(repeated, pb)), upperLimitMask, "masked");
            PabloAST * bounded = pb.createMatchStar(cursor, pb.createOr(masked, mNonFinal), "bounded");
            return makeMarker(MarkerPosition::FinalPostPositionUnit, bounded);
        }
        else if (isTypeForLocal(repeated)) {
            CC * firstSymSet = RE_Local::first(repeated);
            std::map<CC *, CC*> followMap;
            RE_Local::follow(repeated, followMap);
            bool firstSymSet_found_in_follow = false;
            for (auto & entry : followMap) {
                if (entry.second->intersects(*firstSymSet)) {
                    firstSymSet_found_in_follow = true;
                }
            }
            if (!firstSymSet_found_in_follow) {
                // When the first symbol is unique, we can use it as an index stream.
                PabloAST * firstCCstream = markerVar(compile(firstSymSet, pb));
                PabloAST * cursor = markerVar(AdvanceMarker(marker, MarkerPosition::FinalPostPositionUnit, pb));
                PabloAST * upperLimitMask = reachable(cursor, 1, ub - 1, firstCCstream, pb);
                PabloAST * masked = pb.createAnd(markerVar(AdvanceMarker(compile(repeated, pb), MarkerPosition::FinalPostPositionUnit, pb)), upperLimitMask, "masked");
                PabloAST * bounded = pb.createMatchStar(cursor, pb.createOr(masked, mNonFinal), "bounded");
                return makeMarker(MarkerPosition::FinalPostPositionUnit, bounded);
            }
        }
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

inline MarkerType RE_Compiler::compileStart(MarkerType marker, pablo::PabloBuilder & pb) {
    PabloAST * sol = pb.createNot(pb.createAdvance(pb.createNot(mLineBreak), 1));
    if (!AlgorithmOptionIsSet(DisableUnicodeLineBreak)) {
        sol = pb.createScanThru(pb.createAnd(mInitial, sol), mNonFinal);
    }
    MarkerType m = AdvanceMarker(marker, MarkerPosition::FinalPostPositionUnit, pb);
    return makeMarker(MarkerPosition::FinalPostPositionUnit, pb.createAnd(markerVar(m), sol, "sol"));
}

inline MarkerType RE_Compiler::compileEnd(MarkerType marker, pablo::PabloBuilder & pb) {
    PabloAST * const nextPos = markerVar(AdvanceMarker(marker, MarkerPosition::FinalPostPositionUnit, pb));
    return makeMarker(MarkerPosition::FinalPostPositionUnit, pb.createAnd(pb.createScanThru(nextPos, mCRLF), mLineBreak, "eol"));
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

RE_Compiler::RE_Compiler(PabloKernel * kernel, cc::CC_Compiler & ccCompiler)
: mKernel(kernel)
, mCCCompiler(ccCompiler)
, mLineBreak(nullptr)
, mCRLF(nullptr)
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
    Var * const crlf = mKernel->getInputStreamVar("cr+lf");
    mCRLF = mPB.createExtract(crlf, mPB.getInteger(0));
    Var * const required = mKernel->getInputStreamVar("required");
    mInitial = mPB.createExtract(required, mPB.getInteger(0));
    mNonFinal = mPB.createExtract(required, mPB.getInteger(1));
    mFinal = mPB.createExtract(required, mPB.getInteger(2));

}

} // end of namespace re
