/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/compile/re_compiler.h>

#include <llvm/ADT/STLExtras.h>         // for make_unique
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/ErrorHandling.h>
#include <pablo/builder.hpp>            // for PabloBuilder
#include <pablo/pe_ones.h>              // for Ones
#include <pablo/pe_var.h>               // for Var
#include <pablo/pe_zeroes.h>            // for Zeroes
#include <re/adt/adt.h>
#include <re/cc/cc_compiler_target.h>
#include <re/cc/multiplex_CCs.h>
#include <re/cc/cc_compiler.h>
#include <re/transforms/to_utf8.h>
#include <re/analysis/re_analysis.h>     // for isByteLength, isUnicodeUnitLength
#include <re/analysis/re_local.h>
#include <re/toolchain/toolchain.h>
#include <re/ucd/ucd_compiler.hpp>

namespace pablo { class PabloAST; }
namespace pablo { class Var; }
namespace pablo { class PabloKernel; }

using namespace pablo;
using namespace llvm;

namespace re {

PabloAST * ScanToIndex(PabloAST * cursor, PabloAST * indexStrm, PabloBuilder & pb) {
    return pb.createOr(pb.createAnd(cursor, indexStrm),
                       pb.createScanTo(pb.createAnd(pb.createNot(indexStrm), cursor), indexStrm));
}

void RE_Compiler::addIndexingAlphabet(const cc::Alphabet * a, PabloAST * indexStream) {
    mIndexingAlphabet = a;
    mIndexStream = indexStream;
}

void RE_Compiler::addAlphabet(cc::Alphabet * a, std::vector<pablo::PabloAST *> basis_set) {
    mAlphabets.push_back(a);
    mAlphabetCompilers.push_back(make_unique<cc::Parabix_CC_Compiler_Builder>(mEntryScope, basis_set));
}

void RE_Compiler::addPrecompiled(std::string precompiledName, PabloAST * precompiledStream) {
    PabloBuilder pb(mEntryScope);
    mExternalNameMap.insert(std::make_pair(precompiledName, precompiledStream));
}

using MarkerType = RE_Compiler::MarkerType;

PabloAST * RE_Compiler::compile(RE * const re, PabloAST * const initialCursors) {
    pablo::PabloBuilder mPB(mEntryScope);
    const auto markers = initialCursors ? compile(re, initialCursors, mPB) : compile(re, mPB);
    return markerVar(markers);
}

inline MarkerType RE_Compiler::compile(RE * const re, PabloAST * const cursors, PabloBuilder & pb) {
    //  An important use case for an initial set of cursors to be passed in
    //  is that the initial cursors are computed from a prefix of an RE such
    //  that there is a high probability of all cursors remaining in a block
    //  are zeroed.   We therefore embed processing logic in an if-test,
    //  dependent on the initial cursors.
    Var * m = pb.createVar("m", pb.createZeroes());
    NameMap nestedMap(mCompiledName);
    mCompiledName = &nestedMap;
    auto nested = pb.createScope();
    MarkerType m1 = process(re, makeMarker(FinalMatchUnit, cursors), nested);
    nested.createAssign(m, markerVar(m1));
    pb.createIf(cursors, nested);
    mCompiledName = nestedMap.getParent();
    return makeMarker(m1.pos, m);
}

inline MarkerType RE_Compiler::compile(RE * const re, PabloBuilder & pb) {
    return process(re, makeMarker(InitialPostPositionUnit, pb.createOnes()), pb);
}

MarkerType RE_Compiler::process(RE * const re, MarkerType marker, PabloBuilder & pb) {
    if (isa<Name>(re)) {
        return compileName(cast<Name>(re), marker, pb);
    } else if (Capture * c = dyn_cast<Capture>(re)) {
        return process(c->getCapturedRE(), marker, pb);
    } else if (LLVM_UNLIKELY(isa<Reference>(re))) {
        llvm::report_fatal_error("back references not supported in icgrep.");
    } else if (isa<Seq>(re)) {
        return compileSeq(cast<Seq>(re), marker, pb);
    } else if (isa<Seq>(re)) {
        return compileSeq(cast<Seq>(re), marker, pb);
    } else if (isa<Alt>(re)) {
        return compileAlt(cast<Alt>(re), marker, pb);
    } else if (isa<Rep>(re)) {
        return compileRep(cast<Rep>(re), marker, pb);
    } else if (isa<Assertion>(re)) {
        return compileAssertion(cast<Assertion>(re), marker, pb);
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

MarkerType RE_Compiler::compileCC(CC * const cc, MarkerType marker, PabloBuilder & pb) {
    if (cc->empty()) {
        return makeMarker(FinalMatchUnit, pb.createZeroes());
    }
    PabloAST * nextPos = markerVar(marker);
    if (marker.pos == FinalMatchUnit) {
        nextPos = pb.createAdvance(nextPos, 1);
    }
    const cc::Alphabet * a = cc->getAlphabet();
    if ((a == &mCodeUnitAlphabet) || (a == &cc::Byte) || (a == &cc::UTF8)) {
        return makeMarker(FinalMatchUnit, pb.createAnd(nextPos, pb.createInFile(mCCCompiler.compileCC(cc, pb))));
    } else {
        nextPos = ScanToIndex(nextPos, u8Final(pb), pb);
        unsigned i = 0;
        while (i < mAlphabets.size() && (a != mAlphabets[i])) i++;
        if (i == mAlphabets.size()) {
            llvm::report_fatal_error("Alphabet " + a->getName() + " has no CC compiler, codeUnitAlphabet = " + mCodeUnitAlphabet.getName() + "\n in compiling RE: " + Printer_RE::PrintRE(cc) + "\n");
        }
        return makeMarker(FinalMatchUnit, pb.createAnd(nextPos, mAlphabetCompilers[i]->compileCC(cc, pb)));
    }
}

inline MarkerType RE_Compiler::compileName(Name * const name, MarkerType marker, PabloBuilder & pb) {
    const auto & nameString = name->getName();
    MarkerType nameMarker = compileName(name, pb);
    if (isByteLength(name)) {
        MarkerType nextPos = AdvanceMarker(marker, InitialPostPositionUnit, pb);
        nameMarker.stream = pb.createAnd(markerVar(nextPos), markerVar(nameMarker), name->getName());
        return nameMarker;
    } else if (isUnicodeUnitLength(name)) {
        MarkerType nextPos = AdvanceMarker(marker, InitialPostPositionUnit, pb);
        PabloAST * cursor = ScanToIndex(markerVar(nextPos), u8Final(pb), pb);
        nameMarker.stream = pb.createAnd(cursor, markerVar(nameMarker), name->getName());
        return nameMarker;
    } else if (name->getType() == Name::Type::ZeroWidth) {
        AlignMarkers(marker, nameMarker, pb);
        PabloAST * ze = markerVar(nameMarker);
        return makeMarker(markerPos(marker), pb.createAnd(markerVar(marker), ze, "zerowidth"));
    } else {
        llvm::report_fatal_error(nameString + " is neither Unicode unit length nor ZeroWidth.");
    }
}

inline MarkerType RE_Compiler::compileName(Name * const name, PabloBuilder & pb) {
    const auto & nameString = name->getFullName();
    MarkerType m;
    if (LLVM_LIKELY(mCompiledName->get(name, m))) {
        return m;
    }
    const auto f = mExternalNameMap.find(nameString);
    if (f != mExternalNameMap.end()) {
        const auto pos = (name->getType() == Name::Type::ZeroWidth) ? FinalPostPositionUnit : FinalMatchUnit;
        return makeMarker(pos, f->second);
    }
    if (LLVM_LIKELY(name->getDefinition() != nullptr)) {
        m = compile(name->getDefinition(), pb);
        mCompiledName->add(name, m);
        return m;
    }
    UnsupportedRE("Unresolved name " + name->getName());
}

MarkerType RE_Compiler::compileSeq(Seq * const seq, MarkerType marker, PabloBuilder & pb) {

    // if-hierarchies are not inserted within unbounded repetitions
    if (mStarDepth > 0) {
        for (RE * re : *seq) {
            marker = process(re, marker, pb);
        }
        return marker;
    } else {
        return compileSeqTail(seq->cbegin(), seq->cend(), 0, marker, pb);
    }
}

MarkerType RE_Compiler::compileSeqTail(Seq::const_iterator current, const Seq::const_iterator end, const int matchLenSoFar, MarkerType marker, PabloBuilder & pb) {
    if (current == end) {
        return marker;
    } else if (matchLenSoFar < IfInsertionGap) {
        RE * r = *current;
        marker = process(r, marker, pb);
        return compileSeqTail(++current, end, matchLenSoFar + minMatchLength(r), marker, pb);
    } else {
        Var * m = pb.createVar("m", pb.createZeroes());
        NameMap nestedMap(mCompiledName);
        mCompiledName = &nestedMap;
        auto nested = pb.createScope();
        MarkerType m1 = compileSeqTail(current, end, 0, marker, nested);
        nested.createAssign(m, markerVar(m1));
        pb.createIf(markerVar(marker), nested);
        mCompiledName = nestedMap.getParent();
        return makeMarker(m1.pos, m);
    }
}

MarkerType RE_Compiler::compileAlt(Alt * const alt, const MarkerType base, PabloBuilder & pb) {
    std::vector<PabloAST *>  accum(3, pb.createZeroes());
    // The following may be useful to force a common Advance rather than separate
    // Advances in each alternative.
    for (RE * re : *alt) {
        MarkerType m = process(re, base, pb);
        const MarkerPosition p = markerPos(m);
        accum[p] = pb.createOr(accum[p], markerVar(m), "alt");
    }
    if (isa<Zeroes>(accum[InitialPostPositionUnit]) && isa<Zeroes>(accum[FinalPostPositionUnit])) {
        return makeMarker(FinalMatchUnit, accum[FinalMatchUnit]);
    }

    PabloAST * combine = pb.createOr(accum[InitialPostPositionUnit], pb.createAdvance(accum[FinalMatchUnit], 1), "alt");
    if (isa<Zeroes>(accum[FinalPostPositionUnit])) {
        return makeMarker(InitialPostPositionUnit, combine);
    }
    combine = pb.createOr(pb.createOr(pb.createAnd(combine, u8Final(pb)), pb.createScanThru(pb.createAnd(u8NonFinal(pb), combine), u8NonFinal(pb))), accum[FinalPostPositionUnit], "alt");
    return makeMarker(FinalPostPositionUnit, combine);
}

MarkerType RE_Compiler::compileAssertion(Assertion * const a, MarkerType marker, PabloBuilder & pb) {
    RE * asserted = a->getAsserted();
    if (a->getKind() == Assertion::Kind::LookBehind) {
        MarkerType lookback = compile(asserted, pb);
        AlignMarkers(marker, lookback, pb);
        PabloAST * lb = markerVar(lookback);
        if (a->getSense() == Assertion::Sense::Negative) {
            lb = pb.createNot(lb);
        }
        return makeMarker(markerPos(marker), pb.createAnd(markerVar(marker), lb, "lookback"));
    } else if (a->getKind() == Assertion::Kind::Boundary) {
        MarkerType cond = compile(asserted, pb);
        if (LLVM_LIKELY(markerPos(cond) == FinalMatchUnit)) {
            MarkerType postCond = AdvanceMarker(cond, InitialPostPositionUnit, pb);
            PabloAST * boundaryCond = pb.createXor(markerVar(cond), markerVar(postCond));
            if (a->getSense() == Assertion::Sense::Negative) {
                boundaryCond = pb.createNot(boundaryCond);
            }
            MarkerType fbyte = AdvanceMarker(marker, InitialPostPositionUnit, pb);
            return makeMarker(InitialPostPositionUnit, pb.createAnd(markerVar(fbyte), boundaryCond, "boundary"));
        }
        else UnsupportedRE("Unsupported boundary assertion");
    }
    // Lookahead assertions.
    auto lengths = getLengthRange(asserted, &mCodeUnitAlphabet);
    if (lengths.second == 0) {
        MarkerType lookahead = compile(asserted, pb);
        AlignMarkers(marker, lookahead, pb);
        PabloAST * la = markerVar(lookahead);
        if (a->getSense() == Assertion::Sense::Negative) {
            la = pb.createNot(la);
        }
        return makeMarker(markerPos(marker), pb.createAnd(markerVar(marker), la, "lookahead"));
    }
#if 0
    if (((lengths.first == lengths.second) && isa<Name>(asserted))) {
        const auto & nameString = cast<Name>(asserted)->getFullName();
        const auto f = mExternalNameMap.find(nameString);
        if (f != mExternalNameMap.end()) {
            PabloAST * nameMarker = f->second;
            unsigned ahead = lengths.first;
            if (markerPos(marker) != FinalMatchUnit) {
                marker = AdvanceMarker(marker, FinalPostPositionUnit, pb);
                ahead -= 1;
            }
            if (ahead > 0) {
                nameMarker = pb.createLookahead(nameMarker, ahead, nameString + "_ahead");
            }
            if (a->getSense() == Assertion::Sense::Negative) {
                nameMarker = pb.createNot(nameMarker);
            }
            PabloAST * matched = pb.createAnd(markerVar(marker), nameMarker);
            return makeMarker(markerPos(marker), matched);
        }
    }
#endif
    MarkerType lookahead = compile(asserted, pb);
    if (LLVM_LIKELY((lengths.second == 1) && (markerPos(lookahead) == FinalMatchUnit))) {
        MarkerType lookahead = compile(asserted, pb);
        PabloAST * la = markerVar(lookahead);
        if (a->getSense() == Assertion::Sense::Negative) {
            la = pb.createNot(la);
        }
        MarkerType fbyte = AdvanceMarker(marker, InitialPostPositionUnit, pb);
        return makeMarker(InitialPostPositionUnit, pb.createAnd(markerVar(fbyte), la, "lookahead"));
    }
    lengths = getLengthRange(asserted, &cc::Unicode);
    if (LLVM_LIKELY((lengths.second == 1) && (markerPos(lookahead) == FinalMatchUnit))) {
        PabloAST * la = markerVar(lookahead);
        if (a->getSense() == Assertion::Sense::Negative) {
            la = pb.createNot(la);
        }
        MarkerType fbyte = AdvanceMarker(marker, FinalPostPositionUnit, pb);
        return makeMarker(FinalPostPositionUnit, pb.createAnd(markerVar(fbyte), la, "lookahead"));
    }
    llvm::errs() << "lengths.second = " << lengths.second << "\n";
    UnsupportedRE("Unsupported lookahead assertion:" + Printer_RE::PrintRE(a));
}

inline bool alignedUnicodeLength(const RE * const lh, const RE * const rh) {
    const auto lhl = getLengthRange(lh, &cc::Unicode);
    const auto rhl = getLengthRange(rh, &cc::Unicode);
    return (lhl.first == lhl.second && lhl.first == rhl.first && lhl.second == rhl.second);
}

MarkerType RE_Compiler::compileDiff(Diff * diff, MarkerType marker, PabloBuilder & pb) {
    RE * const lh = diff->getLH();
    RE * const rh = diff->getRH();
    if (alignedUnicodeLength(lh, rh)) {
        MarkerType t1 = process(lh, marker, pb);
        MarkerType t2 = process(rh, marker, pb);
        AlignMarkers(t1, t2, pb);
        return makeMarker(markerPos(t1), pb.createAnd(markerVar(t1), pb.createNot(markerVar(t2)), "diff"));
    }
    UnsupportedRE("Unsupported Diff operands: " + Printer_RE::PrintRE(diff));
}

MarkerType RE_Compiler::compileIntersect(Intersect * const x, MarkerType marker, PabloBuilder & pb) {
    RE * const lh = x->getLH();
    RE * const rh = x->getRH();
    if (alignedUnicodeLength(lh, rh)) {
        MarkerType t1 = process(lh, marker, pb);
        MarkerType t2 = process(rh, marker, pb);
        AlignMarkers(t1, t2, pb);
        return makeMarker(markerPos(t1), pb.createAnd(markerVar(t1), markerVar(t2), "intersect"));
    }
    UnsupportedRE("Unsupported Intersect operands: " + Printer_RE::PrintRE(x));
}

MarkerType RE_Compiler::compileRep(Rep * const rep, MarkerType marker, PabloBuilder & pb) {
    const auto lb = rep->getLB();
    const auto ub = rep->getUB();
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
   Given a stream |repeated_j| marking positions associated with |j| consecutive matches to an item
   of length |match_length| compute a stream marking |repeat_count| consecutive occurrences of such items.
*/

PabloAST * RE_Compiler::consecutive_matches(PabloAST * const repeated_j, const int j, const int repeat_count, const int match_length, PabloAST * const indexStream, PabloBuilder & pb) {
    if (j == repeat_count) {
        return repeated_j;
    }
    const int i = std::min(j, repeat_count - j);
    const int k = j + i;
    if (j > IfInsertionGap) {
        Var * repeated = pb.createVar("repeated", pb.createZeroes());
        auto nested = pb.createScope();
        NameMap nestedMap(mCompiledName);
        mCompiledName = &nestedMap;
        PabloAST * adv_i = nested.createIndexedAdvance(repeated_j, indexStream, i * match_length);
        PabloAST * repeated_k = nested.createAnd(repeated_j, adv_i, "at" + std::to_string(k) + "of" + std::to_string(repeat_count));
        nested.createAssign(repeated, consecutive_matches(repeated_k, k, repeat_count, match_length, indexStream, nested));
        pb.createIf(repeated_j, nested);
        mCompiledName = nestedMap.getParent();
        return repeated;
    } else {
        PabloAST * adv_i = pb.createIndexedAdvance(repeated_j, indexStream, i * match_length);
        PabloAST * repeated_k = pb.createAnd(repeated_j, adv_i, "at" + std::to_string(k) + "of" + std::to_string(repeat_count));
        return consecutive_matches(repeated_k, k, repeat_count, match_length, indexStream, pb);
    }
}


inline PabloAST * RE_Compiler::reachable(PabloAST *  const repeated, const int length, const int repeat_count, PabloAST * const indexStream, PabloBuilder & pb) {
    if (repeat_count == 0) {
        return repeated;
    }
    const int total_length = repeat_count * length;
    PabloAST * const v2 = pb.createIndexedAdvance(repeated, indexStream, length);
    PabloAST * reachable = pb.createOr(repeated, v2, "within1");
    int i = length;
    while ((i * 2) < total_length) {
        PabloAST * const extension = pb.createIndexedAdvance(reachable, indexStream, i);
        i *= 2;
        reachable = pb.createOr(reachable, extension, "within" + std::to_string(i));
    }
    if (LLVM_LIKELY(i < total_length)) {
        PabloAST * const extension = pb.createIndexedAdvance(reachable, indexStream, total_length - i);
        reachable = pb.createOr(reachable, extension, "within" + std::to_string(total_length));
    }
    return reachable;
}

MarkerType RE_Compiler::processLowerBound(RE * const repeated, const int lb, MarkerType marker, const int ifGroupSize, PabloBuilder & pb) {
    if (LLVM_UNLIKELY(lb == 0)) {
        return marker;
    } else if (LLVM_UNLIKELY(lb == 1)) {
        return process(repeated, marker, pb);
    }
    //
    // A bounded repetition with a lower bound of at least 2.
    if (LLVM_LIKELY(!AlgorithmOptionIsSet(DisableLog2BoundedRepetition))) {
        // Check for a regular expression that satisfies on of the special conditions that
        // allow implementation using the log2 technique.
        auto lengths = getLengthRange(repeated, &mCodeUnitAlphabet);
        int rpt = lb;
        if ((lengths.first == lengths.second) && isByteLength(repeated)) {
            PabloAST * cc = markerVar(compile(repeated, pb));
            if (markerPos(marker) != FinalMatchUnit) {
                marker = makeMarker(FinalMatchUnit, pb.createAnd(cc, markerVar(marker)));
                rpt -= 1;
            }
            PabloAST * cc_lb = consecutive_matches(cc, 1, rpt, lengths.first, nullptr, pb);
            auto lb_lgth = lengths.first * rpt;
            PabloAST * marker_fwd = pb.createAdvance(markerVar(marker), lb_lgth, "marker_fwd");
            return makeMarker(FinalMatchUnit, pb.createAnd(marker_fwd, cc_lb, "lowerbound"));
        }
        else if (isUnicodeUnitLength(repeated)) {
            PabloAST * cc = markerVar(compile(repeated, pb));
            PabloAST * cursor = markerVar(marker);
            if (markerPos(marker) != FinalMatchUnit) {
                cursor = pb.createAnd(cc, pb.createScanTo(markerVar(marker), u8Final(pb)));
                rpt -= 1;
            }
            PabloAST * cc_lb = consecutive_matches(cc, 1, rpt, 1, u8Final(pb), pb);
            PabloAST * marker_fwd = pb.createIndexedAdvance(cursor, u8Final(pb), rpt);
            return makeMarker(FinalMatchUnit, pb.createAnd(marker_fwd, cc_lb, "lowerbound"));
        }
    }
    // Fall through to general case.  Process the first item and insert the rest into an if-structure.
    const auto group = ifGroupSize < lb ? ifGroupSize : lb;
    for (auto i = 0; i < group; i++) {
        marker = process(repeated, marker, pb);
    }
    if (lb == group) {
        return marker;
    }
    Var * m = pb.createVar("m", pb.createZeroes());
    auto nested = pb.createScope();
    NameMap nestedMap(mCompiledName);
    mCompiledName = &nestedMap;
    MarkerType m1 = processLowerBound(repeated, lb - group, marker, ifGroupSize * 2, nested);
    nested.createAssign(m, markerVar(m1));
    pb.createIf(markerVar(marker), nested);
    mCompiledName = nestedMap.getParent();
    return makeMarker(m1.pos, m);
}

MarkerType RE_Compiler::processBoundedRep(RE * const repeated, const int ub, MarkerType marker, const int ifGroupSize,  PabloBuilder & pb) {
    if (LLVM_UNLIKELY(ub == 0)) {
        return marker;
    }
    //
    // A bounded repetition with an upper bound of at least 2.
    if ((ub > 1) && LLVM_LIKELY(!AlgorithmOptionIsSet(DisableLog2BoundedRepetition))) {
        // Check for a regular expression that satisfies on of the special conditions that
        // allow implementation using the log2 technique.
        auto lengths = getLengthRange(repeated, &mCodeUnitAlphabet);
        // TODO: handle fixed lengths > 1
        if ((lengths.first == lengths.second) && (lengths.first == 1)) {
            PabloAST * repeatMarks = markerVar(compile(repeated, pb));
            // log2 upper bound for fixed length (=1) class
            // Create a mask of positions reachable within ub from current marker.
            // Use matchstar, then apply filter.
            PabloAST * cursor = markerVar(marker);
            PabloAST * upperLimitMask = reachable(cursor, lengths.first, ub, nullptr, pb);
            PabloAST * masked = pb.createAnd(repeatMarks, upperLimitMask, "masked");
            PabloAST * bounded = pb.createAnd(pb.createMatchStar(cursor, masked), upperLimitMask, "bounded");
            return makeMarker(FinalMatchUnit, bounded);
        }
        else if (isUnicodeUnitLength(repeated)) {
            PabloAST * repeatMarks = markerVar(compile(repeated, pb));
            // For a regexp which represent a single Unicode codepoint, we can use the u8Final(pb) stream
            // as an index stream for an indexed advance operation.
            PabloAST * cursor = markerVar(marker);
            PabloAST * upperLimitMask = reachable(cursor, 1, ub, u8Final(pb), pb);
            PabloAST * masked = pb.createAnd(repeatMarks, upperLimitMask, "masked");
            masked = pb.createOr(masked, u8NonFinal(pb));
            PabloAST * bounded = pb.createAnd(pb.createMatchStar(cursor, masked), upperLimitMask, "bounded");
            return makeMarker(FinalMatchUnit, bounded);
        }
    }
    // Fall through to general case.  Process the first item and insert the rest into an if-structure.
    const auto group = ifGroupSize < ub ? ifGroupSize : ub;
    for (auto i = 0; i < group; i++) {
        MarkerType a = process(repeated, marker, pb);
        MarkerType m = marker;
        AlignMarkers(a, m, pb);
        marker = makeMarker(markerPos(a), pb.createOr(markerVar(a), markerVar(m)));
    }
    if (ub == group) {
        return marker;
    }
    Var * const m1a = pb.createVar("m", pb.createZeroes());
    auto nested = pb.createScope();
    NameMap nestedMap(mCompiledName);
    mCompiledName = &nestedMap;
    MarkerType m1 = processBoundedRep(repeated, ub - group, marker, ifGroupSize * 2, nested);
    nested.createAssign(m1a, markerVar(m1));
    pb.createIf(markerVar(marker), nested);
    mCompiledName = nestedMap.getParent();
    return makeMarker(markerPos(m1), m1a);
}

MarkerType RE_Compiler::processUnboundedRep(RE * const repeated, MarkerType marker, PabloBuilder & pb) {
    // always use PostPosition markers for unbounded repetition.
    PabloAST * base = markerVar(AdvanceMarker(marker, InitialPostPositionUnit, pb));
    if (isByteLength(repeated) && LLVM_LIKELY(!AlgorithmOptionIsSet(DisableMatchStar))) {
        PabloAST * mask = markerVar(compile(repeated, pb));
        // The post position character may land on the initial byte of a multi-byte character. Combine them with the masked range.
        PabloAST * unbounded = pb.createMatchStar(base, mask, "unbounded");
        return makeMarker(InitialPostPositionUnit, unbounded);
    } else if (isUnicodeUnitLength(repeated) && LLVM_LIKELY(!AlgorithmOptionIsSet(DisableMatchStar))) {
        PabloAST * mask = markerVar(compile(repeated, pb));
        mask = pb.createOr(mask, u8NonFinal(pb));
        PabloAST * unbounded = pb.createMatchStar(base, mask);
        return makeMarker(InitialPostPositionUnit, pb.createAnd(unbounded, pb.createAdvance(u8Final(pb), 1), "unbounded"));
    } else if (mStarDepth > 0){
        PabloBuilder * const outer = pb.getParent();
        Var * starPending = outer->createVar("pending", outer->createZeroes());
        Var * starAccum = outer->createVar("accum", outer->createZeroes());
        mStarDepth++;
        PabloAST * m1 = pb.createOr(base, starPending);
        PabloAST * m2 = pb.createOr(base, starAccum);
        MarkerType result = process(repeated, makeMarker(InitialPostPositionUnit, m1), pb);
        result = AdvanceMarker(result, InitialPostPositionUnit, pb);
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
        auto wb = pb.createScope();
        NameMap nestedMap(mCompiledName);
        mCompiledName = &nestedMap;
        mStarDepth++;
        MarkerType result = process(repeated, makeMarker(InitialPostPositionUnit, whilePending), wb);
        result = AdvanceMarker(result, InitialPostPositionUnit, wb);
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
    PabloAST * const SOT = pb.createNot(pb.createAdvance(pb.createOnes(), 1));
    MarkerType m = AdvanceMarker(marker, InitialPostPositionUnit, pb);
    return makeMarker(InitialPostPositionUnit, pb.createAnd(markerVar(m), SOT, "SOT"));
}

inline MarkerType RE_Compiler::compileEnd(MarkerType marker, pablo::PabloBuilder & pb) {
    PabloAST * const nextPos = markerVar(AdvanceMarker(marker, InitialPostPositionUnit, pb));
    PabloAST * endOfText = pb.createAtEOF(pb.createAdvance(pb.createOnes(), 1), "EOT");
    PabloAST * const EOT_match = pb.createAnd(endOfText, nextPos, "EOT_match");
    return makeMarker(InitialPostPositionUnit, EOT_match);
}

inline MarkerType RE_Compiler::AdvanceMarker(MarkerType marker, const MarkerPosition newpos, PabloBuilder & pb) {
    if (marker.pos != newpos) {
        if (marker.pos == FinalMatchUnit) {
            marker.stream = pb.createAdvance(marker.stream, 1, "ipp");
            marker.pos = InitialPostPositionUnit;
        }
        if (newpos == FinalPostPositionUnit) {
            marker.stream = ScanToIndex(marker.stream, u8Final(pb), pb);
            marker.pos = FinalPostPositionUnit;
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

pablo::PabloAST * RE_Compiler::u8Final(pablo::PabloBuilder & pb) {
    auto f = mExternalNameMap.find("UTF8_index");
    if (f!= mExternalNameMap.end()) {
        return f->second;
    }
    llvm::report_fatal_error("UTF8_index not found");
}

pablo::PabloAST * RE_Compiler::u8NonFinal(pablo::PabloBuilder & pb) {
    return pb.createInFile(pb.createNot(u8Final(pb)));
}


LLVM_ATTRIBUTE_NORETURN void RE_Compiler::UnsupportedRE(std::string errmsg) {
    llvm::report_fatal_error(errmsg);
}

RE_Compiler::RE_Compiler(PabloBlock * scope,
                         cc::CC_Compiler & ccCompiler,
                         const cc::Alphabet & codeUnitAlphabet)
: mEntryScope(scope)
, mCodeUnitAlphabet(codeUnitAlphabet)
, mCCCompiler(ccCompiler)
, mWhileTest(nullptr)
, mStarDepth(0)
, mCompiledName(&mBaseMap) {
    PabloBuilder pb(mEntryScope);
}

} // end of namespace re
