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
#include <re/analysis/re_analysis.h>
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

void RE_Compiler::addAlphabet(const cc::Alphabet * a, std::vector<pablo::PabloAST *> basis_set) {
    mAlphabets.push_back(a);
    mBasisSets.push_back(basis_set);
    bool useDirectCC = basis_set[0]->getType()->getVectorElementType()->getIntegerBitWidth() > 1;
    std::unique_ptr<cc::CC_Compiler> ccc;
    if (useDirectCC) {
        ccc = make_unique<cc::Direct_CC_Compiler>(mEntryScope, basis_set[0]);
    } else {
        ccc = make_unique<cc::Parabix_CC_Compiler_Builder>(mEntryScope, basis_set);
    }
    mAlphabetCompilers.push_back(std::move(ccc));
}

void RE_Compiler::addIndexingAlphabet(EncodingTransformer * indexingTransformer, PabloAST * indexStream) {
    mIndexingTransformer = indexingTransformer;
    mIndexStream = indexStream;
}

void RE_Compiler::addPrecompiled(std::string precompiledName, PabloAST * precompiledStream) {
    PabloBuilder pb(mEntryScope);
    mExternalNameMap.insert(std::make_pair(precompiledName, precompiledStream));
}

using Marker = RE_Compiler::Marker;

PabloAST * RE_Compiler::compile(RE * const re, PabloAST * const initialCursors) {
    pablo::PabloBuilder mPB(mEntryScope);
    const auto markers = initialCursors ? compile(re, initialCursors, mPB) : compile(re, mPB);
    return markerVar(markers);
}

inline Marker RE_Compiler::compile(RE * const re, PabloAST * const cursors, PabloBuilder & pb) {
    //  An important use case for an initial set of cursors to be passed in
    //  is that the initial cursors are computed from a prefix of an RE such
    //  that there is a high probability of all cursors remaining in a block
    //  are zeroed.   We therefore embed processing logic in an if-test,
    //  dependent on the initial cursors.
    Var * m = pb.createVar("m", pb.createZeroes());
    NameMap nestedMap(mCompiledName);
    mCompiledName = &nestedMap;
    auto nested = pb.createScope();
    Marker m1 = process(re, makeMarker(cursors), nested);
    nested.createAssign(m, markerVar(m1));
    pb.createIf(cursors, nested);
    mCompiledName = nestedMap.getParent();
    return makeMarker(m, m1.offset);
}

inline Marker RE_Compiler::compile(RE * const re, PabloBuilder & pb) {
    return process(re, makeMarker(mIndexStream, 1), pb);
}

Marker RE_Compiler::process(RE * const re, Marker marker, PabloBuilder & pb) {
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

Marker RE_Compiler::compileCC(CC * const cc, Marker marker, PabloBuilder & pb) {
    if (cc->empty()) {
        return makeMarker(pb.createZeroes());
    }
    PabloAST * nextPos = markerVar(marker);
    const cc::Alphabet * a = cc->getAlphabet();
    unsigned i = 0;
    while (i < mAlphabets.size() && (a != mAlphabets[i])) i++;
    if (i < mAlphabets.size()) {
        //llvm::errs() << "Found alphabet: " << i << ", " << mAlphabets[i]->getName() << "\n";
        if (marker.offset == 0) {
            nextPos = pb.createAdvance(nextPos, 1);
        }
        return makeMarker(pb.createAnd(nextPos, mAlphabetCompilers[i]->compileCC(cc, pb)));
    }
    if (mIndexingTransformer && (a == mIndexingTransformer->getIndexingAlphabet())) {
        //llvm::errs() << "Found indexing alphabet: " << i << ", " << mIndexingTransformer->getIndexingAlphabet()->getName() << "\n";
        const cc::Alphabet * encodingAlphabet = mIndexingTransformer->getEncodingAlphabet();
        unsigned i = 0;
        while (i < mAlphabets.size() && (encodingAlphabet != mAlphabets[i])) i++;
        if (i < mAlphabets.size()) {
            RE_Compiler code_unit_compiler(pb.getPabloBlock(), mIndexingTransformer->getEncodingAlphabet());
            code_unit_compiler.addAlphabet(encodingAlphabet, mBasisSets[i]);
            PabloAST * ccm = code_unit_compiler.compile(mIndexingTransformer->transformRE(cc));
            if (marker.offset == 0) {
                nextPos = pb.createIndexedAdvance(nextPos, mIndexStream, 1);
            }
            return makeMarker(pb.createAnd(nextPos, ccm, cc->canonicalName()));
        } else {
            llvm::report_fatal_error("EncodingAlphabet " + encodingAlphabet->getName() + " not registered!\n");
        }
    }
    if (a == &cc::Byte) {
        //llvm::errs() << "Using alphabet 0: for Byte\n";
        if (marker.offset == 0) {
            nextPos = pb.createAdvance(nextPos, 1);
        }
        return makeMarker(pb.createAnd(nextPos, mAlphabetCompilers[0]->compileCC(cc, pb)));
    }
    llvm::report_fatal_error("Alphabet " + a->getName() + " has no CC compiler, codeUnitAlphabet = " + mCodeUnitAlphabet->getName() + "\n in compiling RE: " + Printer_RE::PrintRE(cc) + "\n");
}

inline Marker RE_Compiler::compileName(Name * const name, Marker marker, PabloBuilder & pb) {
    const auto & nameString = name->getName();
    Marker nameMarker = compileName(name, pb);
    auto lengths = getLengthRange(name, mCodeUnitAlphabet);
    if ((lengths.first == 1) && (lengths.second == 1)) {
        PabloAST * nextPos = markerVar(marker);
        if (marker.offset == 0) {
            nextPos = pb.createAdvance(nextPos, 1);
        }
        nameMarker.stream = pb.createAnd(nextPos, markerVar(nameMarker), name->getName());
        return nameMarker;
    }
    if (mIndexingTransformer) {
        auto lengths = getLengthRange(name, mIndexingTransformer->getIndexingAlphabet());
        //llvm::errs() << "getLengthRange(repeated, getIndexingAlphabet) = " << lengths.first << ", " << lengths.second << "\n";
        if ((lengths.first == 1) && (lengths.second == 1)) {
            Marker nextPos = AdvanceMarker(marker, 1, pb);
            //PabloAST * cursor = ScanToIndex(markerVar(nextPos), mIndexStream, pb);
            nameMarker.stream = pb.createAnd(markerVar(nextPos), markerVar(nameMarker), name->getName());
            return nameMarker;
        }
    }
    if (name->getType() == Name::Type::ZeroWidth) {
        AlignMarkers(marker, nameMarker, pb);
        PabloAST * ze = markerVar(nameMarker);
        return makeMarker(pb.createAnd(markerVar(marker), ze, "zerowidth"), markerOffset(marker));
    } else {
        llvm::report_fatal_error(nameString + " is neither Unicode unit length nor ZeroWidth.");
    }
}

inline Marker RE_Compiler::compileName(Name * const name, PabloBuilder & pb) {
    const auto & nameString = name->getFullName();
    Marker m;
    if (LLVM_LIKELY(mCompiledName->get(name, m))) {
        return m;
    }
    const auto f = mExternalNameMap.find(nameString);
    if (f != mExternalNameMap.end()) {
        const auto offset = (name->getType() == Name::Type::ZeroWidth) ? 1 : 0;
        return makeMarker(f->second, offset);
    }
    if (LLVM_LIKELY(name->getDefinition() != nullptr)) {
        //errs() << "compiling definition of name: " << nameString << ": " <<Printer_RE::PrintRE(name->getDefinition()) << "\n";
        m = compile(name->getDefinition(), pb);
        mCompiledName->add(name, m);
        return m;
    }
    UnsupportedRE("Unresolved name " + name->getName());
}

Marker RE_Compiler::compileSeq(Seq * const seq, Marker marker, PabloBuilder & pb) {

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

Marker RE_Compiler::compileSeqTail(Seq::const_iterator current, const Seq::const_iterator end, const int matchLenSoFar, Marker marker, PabloBuilder & pb) {
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
        Marker m1 = compileSeqTail(current, end, 0, marker, nested);
        nested.createAssign(m, markerVar(m1));
        pb.createIf(markerVar(marker), nested);
        mCompiledName = nestedMap.getParent();
        return makeMarker(m, m1.offset);
    }
}

Marker RE_Compiler::compileAlt(Alt * const alt, const Marker base, PabloBuilder & pb) {
    std::vector<PabloAST *>  accum(1, pb.createZeroes());
    // The following may be useful to force a common Advance rather than separate
    // Advances in each alternative.
    for (RE * re : *alt) {
        Marker m = process(re, base, pb);
        const unsigned o = markerOffset(m);
        while (o >= accum.size()) {accum.push_back(pb.createZeroes());}
        accum[o] = pb.createOr(accum[o], markerVar(m), "alt");
    }
    unsigned max_offset = accum.size() - 1;
    if ((max_offset > 0) && !isa<Zeroes>(accum[0])) {
        PabloAST * adjusted = ScanToIndex(pb.createAdvance(accum[0], 1), mIndexStream, pb);
        accum[1] = pb.createOr(accum[1], adjusted);
    }
    for (unsigned offset = 1; offset < max_offset; offset++) {
        if (!isa<Zeroes>(accum[offset])) {
            PabloAST * adjusted = pb.createIndexedAdvance(accum[offset], mIndexStream, max_offset - offset);
            accum[max_offset] = pb.createOr(accum[max_offset], adjusted);
        }
    }
    return makeMarker(accum[max_offset], max_offset);
}

Marker RE_Compiler::compileAssertion(Assertion * const a, Marker marker, PabloBuilder & pb) {
    RE * asserted = a->getAsserted();
    if (a->getKind() == Assertion::Kind::LookBehind) {
        Marker lookback = compile(asserted, pb);
        AlignMarkers(marker, lookback, pb);
        PabloAST * lb = markerVar(lookback);
        if (a->getSense() == Assertion::Sense::Negative) {
            lb = pb.createNot(lb);
        }
        return makeMarker(pb.createAnd(markerVar(marker), lb, "lookback"), markerOffset(marker));
    } else if (a->getKind() == Assertion::Kind::Boundary) {
        Marker cond = compile(asserted, pb);
        if (LLVM_LIKELY(markerOffset(cond) == 0)) {
            Marker postCond = AdvanceMarker(cond, 1, pb);
            PabloAST * boundaryCond = pb.createXor(markerVar(cond), markerVar(postCond));
            if (a->getSense() == Assertion::Sense::Negative) {
                boundaryCond = pb.createNot(boundaryCond);
            }
            Marker fbyte = AdvanceMarker(marker, 1, pb);
            return makeMarker(pb.createAnd(markerVar(fbyte), boundaryCond, "boundary"), 1);
        }
        else UnsupportedRE("Unsupported boundary assertion");
    }
    // Lookahead assertions.
    auto alphabet = mIndexingTransformer ? mIndexingTransformer->getIndexingAlphabet() : mCodeUnitAlphabet;
    auto lengths = getLengthRange(asserted, alphabet);
    if (lengths.second == 0) {
        Marker lookahead = compile(asserted, pb);
        AlignMarkers(marker, lookahead, pb);
        PabloAST * la = markerVar(lookahead);
        if (a->getSense() == Assertion::Sense::Negative) {
            la = pb.createNot(la);
        }
        return makeMarker(pb.createAnd(markerVar(marker), la, "lookahead"), markerOffset(marker));
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
    Marker lookahead = compile(asserted, pb);
    if (LLVM_LIKELY((lengths.second == 1) && (markerOffset(lookahead) == 0))) {
        Marker lookahead = compile(asserted, pb);
        PabloAST * la = markerVar(lookahead);
        if (a->getSense() == Assertion::Sense::Negative) {
            la = pb.createNot(la);
        }
        Marker fbyte = AdvanceMarker(marker, 1, pb);
        return makeMarker(pb.createAnd(markerVar(fbyte), la, "lookahead"), 1);
    }
#if 0
    lengths = getLengthRange(asserted, &cc::Unicode);
    if (LLVM_LIKELY((lengths.second == 1) && (markerPos(lookahead) == FinalMatchUnit))) {
        PabloAST * la = markerVar(lookahead);
        if (a->getSense() == Assertion::Sense::Negative) {
            la = pb.createNot(la);
        }
        Marker fbyte = AdvanceMarker(marker, FinalPostPositionUnit, pb);
        return makeMarker(FinalPostPositionUnit, pb.createAnd(markerVar(fbyte), la, "lookahead"));
    }
#endif
    llvm::errs() << "lengths.second = " << lengths.second << "\n";
    UnsupportedRE("Unsupported lookahead assertion:" + Printer_RE::PrintRE(a));
}

inline bool alignedUnicodeLength(const RE * const lh, const RE * const rh) {
    const auto lhl = getLengthRange(lh, &cc::Unicode);
    const auto rhl = getLengthRange(rh, &cc::Unicode);
    return (lhl.first == lhl.second && lhl.first == rhl.first && lhl.second == rhl.second);
}

Marker RE_Compiler::compileDiff(Diff * diff, Marker marker, PabloBuilder & pb) {
    RE * const lh = diff->getLH();
    RE * const rh = diff->getRH();
    if (alignedUnicodeLength(lh, rh)) {
        Marker t1 = process(lh, marker, pb);
        Marker t2 = process(rh, marker, pb);
        AlignMarkers(t1, t2, pb);
        return makeMarker(pb.createAnd(markerVar(t1), pb.createNot(markerVar(t2)), "diff"), markerOffset(t1));
    }
    UnsupportedRE("Unsupported Diff operands: " + Printer_RE::PrintRE(diff));
}

Marker RE_Compiler::compileIntersect(Intersect * const x, Marker marker, PabloBuilder & pb) {
    RE * const lh = x->getLH();
    RE * const rh = x->getRH();
    if (alignedUnicodeLength(lh, rh)) {
        Marker t1 = process(lh, marker, pb);
        Marker t2 = process(rh, marker, pb);
        AlignMarkers(t1, t2, pb);
        return makeMarker(pb.createAnd(markerVar(t1), markerVar(t2), "intersect"), markerOffset(t1));
    }
    UnsupportedRE("Unsupported Intersect operands: " + Printer_RE::PrintRE(x));
}

Marker RE_Compiler::compileRep(Rep * const rep, Marker marker, PabloBuilder & pb) {
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

Marker RE_Compiler::processLowerBound(RE * const repeated, const int lb, Marker marker, const int ifGroupSize, PabloBuilder & pb) {
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
        auto lengths = getLengthRange(repeated, mCodeUnitAlphabet);
        //llvm::errs() << "getLengthRange(repeated, mCodeUnitAlphabet) = " << lengths.first << ", " << lengths.second << "\n";
        int rpt = lb;
        if ((lengths.first == 1) && (lengths.second == 1)) {
            PabloAST * cc = markerVar(compile(repeated, pb));
            if (markerOffset(marker) != 0) {
                marker = makeMarker(pb.createAnd(cc, markerVar(marker)));
                rpt -= 1;
            }
            PabloAST * cc_lb = consecutive_matches(cc, 1, rpt, lengths.first, nullptr, pb);
            auto lb_lgth = lengths.first * rpt;
            PabloAST * marker_fwd = pb.createAdvance(markerVar(marker), lb_lgth, "marker_fwd");
            return makeMarker(pb.createAnd(marker_fwd, cc_lb, "lowerbound"));
        }
        if (mIndexingTransformer) {
            auto lengths = getLengthRange(repeated, mIndexingTransformer->getIndexingAlphabet());
            //llvm::errs() << "getLengthRange(repeated, getIndexingAlphabet) = " << lengths.first << ", " << lengths.second << "\n";
            if ((lengths.first == 1) && (lengths.second == 1)) {
                PabloAST * cc = markerVar(compile(repeated, pb));
                PabloAST * cursor = markerVar(marker);
                if (markerOffset(marker) != 0) {
                    cursor = pb.createAnd(cc, pb.createScanTo(markerVar(marker), mIndexStream));
                    rpt -= 1;
                }
                PabloAST * cc_lb = consecutive_matches(cc, 1, rpt, 1, mIndexStream, pb);
                PabloAST * marker_fwd = pb.createIndexedAdvance(cursor, mIndexStream, rpt);
                return makeMarker(pb.createAnd(marker_fwd, cc_lb, "lowerbound"));
            }
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
    Marker m1 = processLowerBound(repeated, lb - group, marker, ifGroupSize * 2, nested);
    nested.createAssign(m, markerVar(m1));
    pb.createIf(markerVar(marker), nested);
    mCompiledName = nestedMap.getParent();
    return makeMarker(m, m1.offset);
}

Marker RE_Compiler::processBoundedRep(RE * const repeated, const int ub, Marker marker, const int ifGroupSize,  PabloBuilder & pb) {
    if (LLVM_UNLIKELY(ub == 0)) {
        return marker;
    }
    //
    // A bounded repetition with an upper bound of at least 2.
    if ((ub > 1) && LLVM_LIKELY(!AlgorithmOptionIsSet(DisableLog2BoundedRepetition))) {
        // Check for a regular expression that satisfies on of the special conditions that
        // allow implementation using the log2 technique.
        auto lengths = getLengthRange(repeated, mCodeUnitAlphabet);
        // TODO: handle fixed lengths > 1
        if ((lengths.first == 1) && (lengths.second == 1)) {
            PabloAST * repeatMarks = markerVar(compile(repeated, pb));
            // log2 upper bound for fixed length (=1) class
            // Create a mask of positions reachable within ub from current marker.
            // Use matchstar, then apply filter.
            PabloAST * cursor = markerVar(marker);
            PabloAST * upperLimitMask = reachable(cursor, lengths.first, ub, nullptr, pb);
            PabloAST * masked = pb.createAnd(repeatMarks, upperLimitMask, "masked");
            PabloAST * bounded = pb.createAnd(pb.createMatchStar(cursor, masked), upperLimitMask, "bounded");
            return makeMarker(bounded);
        }
        if (mIndexingTransformer) {
            auto lengths = getLengthRange(repeated, mIndexingTransformer->getIndexingAlphabet());
            //llvm::errs() << "getLengthRange(repeated, getIndexingAlphabet) = " << lengths.first << ", " << lengths.second << "\n";
            if ((lengths.first == 1) && (lengths.second == 1)) {
                PabloAST * repeatMarks = markerVar(compile(repeated, pb));
                // For a regexp which represent a single Unicode codepoint, we can use the mIndexStream stream
                // as an index stream for an indexed advance operation.
                PabloAST * cursor = markerVar(marker);
                PabloAST * upperLimitMask = reachable(cursor, 1, ub, mIndexStream, pb);
                PabloAST * masked = pb.createAnd(repeatMarks, upperLimitMask, "masked");
                masked = pb.createOr(masked, pb.createNot(mIndexStream));
                PabloAST * bounded = pb.createAnd(pb.createMatchStar(cursor, masked), upperLimitMask, "bounded");
                return makeMarker(bounded);
            }
        }
    }
    // Fall through to general case.  Process the first item and insert the rest into an if-structure.
    const auto group = ifGroupSize < ub ? ifGroupSize : ub;
    for (auto i = 0; i < group; i++) {
        Marker a = process(repeated, marker, pb);
        Marker m = marker;
        AlignMarkers(a, m, pb);
        marker = makeMarker(pb.createOr(markerVar(a), markerVar(m)), markerOffset(a));
    }
    if (ub == group) {
        return marker;
    }
    Var * const m1a = pb.createVar("m", pb.createZeroes());
    auto nested = pb.createScope();
    NameMap nestedMap(mCompiledName);
    mCompiledName = &nestedMap;
    Marker m1 = processBoundedRep(repeated, ub - group, marker, ifGroupSize * 2, nested);
    nested.createAssign(m1a, markerVar(m1));
    pb.createIf(markerVar(marker), nested);
    mCompiledName = nestedMap.getParent();
    return makeMarker(m1a, markerOffset(m1));
}

Marker RE_Compiler::processUnboundedRep(RE * const repeated, Marker marker, PabloBuilder & pb) {
    // always use PostPosition markers for unbounded repetition.
    PabloAST * base = markerVar(AdvanceMarker(marker, 1, pb));
    if (LLVM_LIKELY(!AlgorithmOptionIsSet(DisableMatchStar))) {
        auto lengths = getLengthRange(repeated, mCodeUnitAlphabet);
        //llvm::errs() << "getLengthRange(repeated, mCodeUnitAlphabet) = " << lengths.first << ", " << lengths.second << "\n";
        if ((lengths.first == 1) && (lengths.second == 1)) {
            PabloAST * mask = markerVar(compile(repeated, pb));
            mask = pb.createOr(mask, pb.createNot(mIndexStream));
            // The post position character may land on the initial byte of a multi-byte character. Combine them with the masked range.
            PabloAST * unbounded = pb.createMatchStar(base, mask, "unbounded");
            return makeMarker(pb.createAnd(unbounded, mIndexStream, "unbounded"), 1);
        }
        if (mIndexingTransformer) {
            auto lengths = getLengthRange(repeated, mIndexingTransformer->getIndexingAlphabet());
            //llvm::errs() << "getLengthRange(repeated, getIndexingAlphabet) = " << lengths.first << ", " << lengths.second << "\n";
            if ((lengths.first == 1) && (lengths.second == 1)) {
                PabloAST * mask = markerVar(compile(repeated, pb));
                mask = pb.createOr(mask, pb.createNot(mIndexStream));
                PabloAST * unbounded = pb.createMatchStar(base, mask);
                return makeMarker(pb.createAnd(unbounded, mIndexStream, "unbounded"), 1);
            }
        }
    }
    if (mStarDepth > 0){
        PabloBuilder * const outer = pb.getParent();
        Var * starPending = outer->createVar("pending", outer->createZeroes());
        Var * starAccum = outer->createVar("accum", outer->createZeroes());
        mStarDepth++;
        PabloAST * m1 = pb.createOr(base, starPending);
        PabloAST * m2 = pb.createOr(base, starAccum);
        Marker result = process(repeated, makeMarker(m1, 1), pb);
        result = AdvanceMarker(result, 1, pb);
        PabloAST * loopComputation = markerVar(result);
        pb.createAssign(starPending, pb.createAnd(loopComputation, pb.createNot(m2)));
        pb.createAssign(starAccum, pb.createOr(loopComputation, m2));
        mWhileTest = pb.createOr(mWhileTest, starPending);
        mStarDepth--;
        return makeMarker(pb.createOr(base, starAccum, "unbounded"), markerOffset(result));
    } else {
        Var * whileTest = pb.createVar("test", base);
        Var * whilePending = pb.createVar("pending", base);
        Var * whileAccum = pb.createVar("accum", base);
        mWhileTest = pb.createZeroes();
        auto wb = pb.createScope();
        NameMap nestedMap(mCompiledName);
        mCompiledName = &nestedMap;
        mStarDepth++;
        Marker result = process(repeated, makeMarker(whilePending, 1), wb);
        result = AdvanceMarker(result, 1, wb);
        PabloAST * loopComputation = markerVar(result);
        wb.createAssign(whilePending, wb.createAnd(loopComputation, wb.createNot(whileAccum)));
        wb.createAssign(whileAccum, wb.createOr(loopComputation, whileAccum));
        wb.createAssign(whileTest, wb.createOr(mWhileTest, whilePending));
        pb.createWhile(whileTest, wb);
        mStarDepth--;
        mCompiledName = nestedMap.getParent();
        return makeMarker(whileAccum, markerOffset(result));
    }
}

inline Marker RE_Compiler::compileStart(Marker marker, pablo::PabloBuilder & pb) {
    PabloAST * const SOT = pb.createNot(pb.createAdvance(pb.createOnes(), 1));
    Marker m = AdvanceMarker(marker, 1, pb);
    return makeMarker(pb.createAnd(markerVar(m), SOT, "SOT"), 1);
}

inline Marker RE_Compiler::compileEnd(Marker marker, pablo::PabloBuilder & pb) {
    PabloAST * const nextPos = markerVar(AdvanceMarker(marker, 1, pb));
    PabloAST * endOfText = pb.createAtEOF(pb.createAdvance(pb.createOnes(), 1), "EOT");
    PabloAST * const EOT_match = pb.createAnd(endOfText, nextPos, "EOT_match");
    return makeMarker(EOT_match, 1);
}

inline Marker RE_Compiler::AdvanceMarker(Marker marker, const unsigned offset, PabloBuilder & pb) {
    if (marker.offset < offset) {
        marker.stream = pb.createIndexedAdvance(marker.stream, mIndexStream, offset - marker.offset);
        marker.offset = offset;
    }
    return marker;
}

inline void RE_Compiler::AlignMarkers(Marker & m1, Marker & m2, PabloBuilder & pb) {
    if (m1.offset < m2.offset) {
        m1 = AdvanceMarker(m1, m2.offset, pb);
    } else if (m2.offset < m1.offset) {
        m2 = AdvanceMarker(m2, m1.offset, pb);
    }
}

LLVM_ATTRIBUTE_NORETURN void RE_Compiler::UnsupportedRE(std::string errmsg) {
    llvm::report_fatal_error(errmsg);
}

RE_Compiler::RE_Compiler(PabloBlock * scope,
                         const cc::Alphabet * codeUnitAlphabet)
: mEntryScope(scope)
, mCodeUnitAlphabet(codeUnitAlphabet)
, mIndexingTransformer(nullptr)
, mWhileTest(nullptr)
, mStarDepth(0)
, mCompiledName(&mBaseMap) {
    PabloBuilder pb(mEntryScope);
    mIndexStream = pb.createOnes();
}

} // end of namespace re
