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
#include <re/analysis/cc_sequence_search.h>
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
    
using Marker = RE_Compiler::Marker;

void RE_Compiler::addPrecompiled(std::string precompiledName, Marker precompiled) {
    PabloBuilder pb(mEntryScope);
    mExternalNameMap.insert(std::make_pair(precompiledName, precompiled.stream()));
}

Marker RE_Compiler::compileRE(RE * const re) {
    pablo::PabloBuilder mPB(mEntryScope);
    return process(re, Marker(mIndexStream, 1), mPB);
}

Marker RE_Compiler::compileRE(RE * const re, Marker initialMarkers) {
    pablo::PabloBuilder pb(mEntryScope);
    //  An important use case for an initial set of cursors to be passed in
    //  is that the initial cursors are computed from a prefix of an RE such
    //  that there is a high probability of all cursors remaining in a block
    //  are zeroed.   We therefore embed processing logic in an if-test,
    //  dependent on the initial cursors.
    Var * m = pb.createVar("m", pb.createZeroes());
    NameMap nestedMap(mCompiledName);
    mCompiledName = &nestedMap;
    auto nested = pb.createScope();
    Marker m1 = process(re, initialMarkers, nested);
    nested.createAssign(m, m1.stream());
    pb.createIf(initialMarkers.stream(), nested);
    mCompiledName = nestedMap.getParent();
    return Marker(m, m1.offset());
}

inline Marker RE_Compiler::compile(RE * const re, PabloBuilder & pb) {
    return process(re, Marker(mIndexStream, 1), pb);
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
    } else if (isa<Alt>(re)) {
        return compileAlt(cast<Alt>(re), marker, pb);
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
        return compileRep(rep->getLB(), rep->getUB(), rep->getRE(), marker, pb);
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
        return Marker(pb.createZeroes());
    }
    PabloAST * nextPos = marker.stream();
    const cc::Alphabet * a = cc->getAlphabet();
    unsigned i = 0;
    while (i < mAlphabets.size() && (a != mAlphabets[i])) i++;
    if (i < mAlphabets.size()) {
        //llvm::errs() << "Found alphabet: " << i << ", " << mAlphabets[i]->getName() << "\n";
        if (marker.offset() == 0) {
            nextPos = pb.createIndexedAdvance(nextPos, mIndexStream, 1);
        }
        return Marker(pb.createAnd(nextPos, mAlphabetCompilers[i]->compileCC(cc, pb)));
    }
    if (mIndexingTransformer && (a == mIndexingTransformer->getIndexingAlphabet())) {
        //llvm::errs() << "Found indexing alphabet: " << i << ", " << mIndexingTransformer->getIndexingAlphabet()->getName() << "\n";
        const cc::Alphabet * encodingAlphabet = mIndexingTransformer->getEncodingAlphabet();
        unsigned i = 0;
        while (i < mAlphabets.size() && (encodingAlphabet != mAlphabets[i])) i++;
        if (i < mAlphabets.size()) {
            RE_Compiler code_unit_compiler(pb.getPabloBlock(), mIndexingTransformer->getEncodingAlphabet());
            code_unit_compiler.addAlphabet(encodingAlphabet, mBasisSets[i]);
            Marker ccm = code_unit_compiler.compileRE(mIndexingTransformer->transformRE(cc));
            if (marker.offset() == 0) {
                nextPos = pb.createIndexedAdvance(nextPos, mIndexStream, 1);
            }
            return Marker(pb.createAnd(nextPos, ccm.stream(), cc->canonicalName()));
        } else {
            llvm::report_fatal_error("EncodingAlphabet " + encodingAlphabet->getName() + " not registered!\n");
        }
    }
    if (a == &cc::Byte) {
        //llvm::errs() << "Using alphabet 0: for Byte\n";
        if (marker.offset() == 0) {
            nextPos = pb.createAdvance(nextPos, 1);
        }
        return Marker(pb.createAnd(nextPos, mAlphabetCompilers[0]->compileCC(cc, pb)));
    }
    llvm::report_fatal_error("Alphabet " + a->getName() + " has no CC compiler, codeUnitAlphabet = " + mCodeUnitAlphabet->getName() + "\n in compiling RE: " + Printer_RE::PrintRE(cc) + "\n");
}

inline Marker RE_Compiler::compileName(Name * const name, Marker marker, PabloBuilder & pb) {
    const auto & nameString = name->getName();
    Marker nameMarker = compileName(name, pb);
    auto lengths = getLengthRange(name, mCodeUnitAlphabet);
    if ((lengths.first == 1) && (lengths.second == 1)) {
        PabloAST * nextPos = marker.stream();
        if (marker.offset() == 0) {
            nextPos = pb.createAdvance(nextPos, 1);
        }
        return Marker(pb.createAnd(nextPos, nameMarker.stream(), name->getName()), nameMarker.offset());
    }
    if (mIndexingTransformer) {
        auto lengths = getLengthRange(name, mIndexingTransformer->getIndexingAlphabet());
        //llvm::errs() << "getLengthRange(repeated, getIndexingAlphabet) = " << lengths.first << ", " << lengths.second << "\n";
        if ((lengths.first == 1) && (lengths.second == 1)) {
            Marker nextPos = AdvanceMarker(marker, 1, pb);
            //PabloAST * cursor = ScanToIndex(nextPos.stream(), mIndexStream, pb);
            return Marker(pb.createAnd(nextPos.stream(), nameMarker.stream(), name->getName()), nameMarker.offset());
        }
    }
    if (name->getType() == Name::Type::ZeroWidth) {
        AlignMarkers(marker, nameMarker, pb);
        PabloAST * ze = nameMarker.stream();
        return Marker(pb.createAnd(marker.stream(), ze, "zerowidth"), marker.offset());
    } else {
        llvm::report_fatal_error(nameString + " is neither Unicode unit length nor ZeroWidth.");
    }
}

inline Marker RE_Compiler::compileName(Name * const name, PabloBuilder & pb) {
    const auto & nameString = name->getFullName();
    Marker m(nullptr);
    if (LLVM_LIKELY(mCompiledName->get(name, m))) {
        return m;
    }
    const auto f = mExternalNameMap.find(nameString);
    if (f != mExternalNameMap.end()) {
        const auto offset = (name->getType() == Name::Type::ZeroWidth) ? 1 : 0;
        return Marker(f->second, offset);
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
        nested.createAssign(m, m1.stream());
        pb.createIf(marker.stream(), nested);
        mCompiledName = nestedMap.getParent();
        return Marker(m, m1.offset());
    }
}

Marker RE_Compiler::compileAlt(Alt * const alt, const Marker base, PabloBuilder & pb) {
    std::vector<PabloAST *>  accum(1, pb.createZeroes());
    // The following may be useful to force a common Advance rather than separate
    // Advances in each alternative.
    for (RE * re : *alt) {
        Marker m = process(re, base, pb);
        const unsigned o = m.offset();
        while (o >= accum.size()) {accum.push_back(pb.createZeroes());}
        accum[o] = pb.createOr(accum[o], m.stream(), "alt");
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
    return Marker(accum[max_offset], max_offset);
}

Marker RE_Compiler::compileAssertion(Assertion * const a, Marker marker, PabloBuilder & pb) {
    RE * asserted = a->getAsserted();
    if (a->getKind() == Assertion::Kind::LookBehind) {
        Marker lookback = compile(asserted, pb);
        AlignMarkers(marker, lookback, pb);
        PabloAST * lb = lookback.stream();
        if (a->getSense() == Assertion::Sense::Negative) {
            lb = pb.createNot(lb);
        }
        return Marker(pb.createAnd(marker.stream(), lb, "lookback"), marker.offset());
    } else if (a->getKind() == Assertion::Kind::Boundary) {
        Marker cond = compile(asserted, pb);
        if (LLVM_LIKELY(cond.offset() == 0)) {
            Marker postCond = AdvanceMarker(cond, 1, pb);
            PabloAST * boundaryCond = pb.createXor(cond.stream(), postCond.stream());
            if (a->getSense() == Assertion::Sense::Negative) {
                boundaryCond = pb.createNot(boundaryCond);
            }
            Marker fbyte = AdvanceMarker(marker, 1, pb);
            return Marker(pb.createAnd(fbyte.stream(), boundaryCond, "boundary"), 1);
        }
        else UnsupportedRE("Unsupported boundary assertion");
    }
    // Lookahead assertions.
    auto alphabet = mIndexingTransformer ? mIndexingTransformer->getIndexingAlphabet() : mCodeUnitAlphabet;
    auto lengths = getLengthRange(asserted, alphabet);
    if (lengths.second == 0) {
        Marker lookahead = compile(asserted, pb);
        AlignMarkers(marker, lookahead, pb);
        PabloAST * la = lookahead.stream();
        if (a->getSense() == Assertion::Sense::Negative) {
            la = pb.createNot(la);
        }
        return Marker(pb.createAnd(marker.stream(), la, "lookahead"), marker.offset());
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
            PabloAST * matched = pb.createAnd(marker.stream(), nameMarker);
            return Marker(markerPos(marker), matched);
        }
    }
#endif
    Marker lookahead = compile(asserted, pb);
    if (LLVM_LIKELY((lengths.second == 1) && (lookahead.offset() == 0))) {
        Marker lookahead = compile(asserted, pb);
        PabloAST * la = lookahead.stream();
        if (a->getSense() == Assertion::Sense::Negative) {
            la = pb.createNot(la);
        }
        Marker fbyte = AdvanceMarker(marker, 1, pb);
        return Marker(pb.createAnd(fbyte.stream(), la, "lookahead"), 1);
    }
#if 0
    lengths = getLengthRange(asserted, &cc::Unicode);
    if (LLVM_LIKELY((lengths.second == 1) && (markerPos(lookahead) == FinalMatchUnit))) {
        PabloAST * la = lookahead.stream();
        if (a->getSense() == Assertion::Sense::Negative) {
            la = pb.createNot(la);
        }
        Marker fbyte = AdvanceMarker(marker, FinalPostPositionUnit, pb);
        return Marker(FinalPostPositionUnit, pb.createAnd(fbyte.stream(), la, "lookahead"));
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
        return Marker(pb.createAnd(t1.stream(), pb.createNot(t2.stream()), "diff"), t1.offset());
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
        return Marker(pb.createAnd(t1.stream(), t2.stream(), "intersect"), t1.offset());
    }
    UnsupportedRE("Unsupported Intersect operands: " + Printer_RE::PrintRE(x));
}

bool CharacteristicSubexpressionAnalysis(RE * repeated, RE * &E1, RE * &C, RE * &E2) {
    if (isa<CC>(repeated)) {
        E1 = makeSeq();
        E2 = makeSeq();
        C = repeated;
        return true;
    }
    if (Seq * s = dyn_cast<Seq>(repeated)) {
        unsigned i = 0;
        while (i < s->size()) {
            unsigned j = i;
            std::vector<CC *> CC_seq;
            while (j < s->size()) {
                RE * item = (*s)[j];
                if (CC * cc = dyn_cast<CC>(item)) {
                    CC_seq.push_back(cc);
                    j++;
                } else if (const Name * name = dyn_cast<Name>(item)) {
                    RE * defn = name->getDefinition();
                    if (CC * cc = dyn_cast<CC>(defn)) {
                        CC_seq.push_back(cc);
                        j++;
                    } else break;
                } else break;
            }
            // If we found a nonempty CC_seq, determine if it is a characteristic
            // expression.
            if (j > i) {
                E1 = makeSeq(s->begin(), s->begin()+i);
                E2 = makeSeq(s->begin()+j, s->end());
                // Form E2 E1, where the original seq s is E1 CC_seq E2
                RE * E2_E1 = makeSeq({E2, E1});
                if (!CC_Sequence_Search(CC_seq, E2_E1)) {
                    // C is a characteristic subexpression
                    C = makeSeq(s->begin()+i, s->begin()+j);
                    return true;
                }
            }
            i = j+1;
        }
        return false;
    }
    return false;
}

Marker RE_Compiler::compileRep(int lb, int ub, RE * repeated, Marker marker, PabloBuilder & pb) {
    // Always handle cases with small lower bound and no upper bound by expansion.
    if ((lb <= 2) && (ub == Rep::UNBOUNDED_REP)) {
        Marker at_lb = expandLowerBound(repeated, lb, marker, IfInsertionGap, pb);
        return processUnboundedRep(repeated, at_lb, pb);
    }
    // Similarly handle cases with small upper bound by expansion.
    if ((ub != Rep::UNBOUNDED_REP) && (ub <= 2)) {
        Marker at_lb = expandLowerBound(repeated, lb, marker, IfInsertionGap, pb);
        if (lb == ub) return at_lb;
        return expandUpperBound(repeated, ub - lb, marker, IfInsertionGap, pb);
    }
    // Handle the case of lb == 0 specially.
    if (lb == 0) {
        Marker at_least_one = compileRep(1, ub, repeated, marker, pb);
        AlignMarkers(marker, at_least_one, pb);
        return Marker(pb.createOr(marker.stream(), at_least_one.stream()), at_least_one.offset());
    }
    if (LLVM_LIKELY(!AlgorithmOptionIsSet(DisableLog2BoundedRepetition))) {
        // Check for a regular expression that satisfies on of the special conditions that
        // allow implementation using the log2 technique.
        auto lengths = getLengthRange(repeated, mCodeUnitAlphabet);
        //llvm::errs() << "getLengthRange(repeated, mCodeUnitAlphabet) = " << lengths.first << ", " << lengths.second << "\n";
        RE * C;
        RE * E1;
        RE * E2;
        if (CharacteristicSubexpressionAnalysis(repeated, E1, C, E2)) {
            //llvm::errs() << "E1 = " << Printer_RE::PrintRE(E1) << "\n";
            //llvm::errs() << "C = " << Printer_RE::PrintRE(C) << "\n";
            //llvm::errs() << "E2 = " << Printer_RE::PrintRE(E2) << "\n";
            // Process an initial half iteration upto and including a match to C.
            Marker M1 = process(E1, marker, pb);
            M1 = process(C, M1, pb);
            assert(M1.offset() == 0 && "RE compiler error: characteristic subexpression with nonzero offset");
            PabloAST * half_mark = M1.stream();
            //
            // Prepare the stream marking positions represent full repetitions.
            RE * C_E2_E1_C = makeSeq({C, E2, E1, C});
            PabloAST * consecutive = compile(C_E2_E1_C, pb).stream();
            //
            // Prepare the matches to the characteristic subexpression C as the index stream.
            PabloAST * idx = compile(C, pb).stream();
            //
            //  Restrict to positions with at least lb-1 iterations.
            if (lb > 1) {
                PabloAST * at_least_lb = consecutive_matches(consecutive, 1, lb - 1, 1, idx, pb);
                PabloAST * marker_fwd = pb.createIndexedAdvance(half_mark, idx, lb - 1);
                half_mark = pb.createAnd(marker_fwd, at_least_lb, "lowerbound");
            }
            if (ub == Rep::UNBOUNDED_REP) {
                // Finish the half-iteration for the lower bound, then continue with unbounded.
                Marker at_lb = process(E2, Marker(half_mark), pb);
                return processUnboundedRep(repeated, at_lb, pb);
            }
            if (ub == lb) {
                // No additional iterations, but finish the half-iteration.
                return process(E2, Marker(half_mark), pb);
            }
            PabloAST * upperLimitMask = reachable(half_mark, 1, ub - lb, idx, pb);
            PabloAST * masked = pb.createAnd(consecutive, upperLimitMask, "masked");
            masked = pb.createOr(masked, pb.createNot(idx));
            PabloAST * bounded = pb.createAnd(pb.createMatchStar(half_mark, masked), upperLimitMask, "bounded");
            return process(E2, Marker(bounded), pb);
        }
    }
    if (lb > 0) {
        marker = expandLowerBound(repeated, lb, marker, IfInsertionGap, pb);
    }
    if (ub == Rep::UNBOUNDED_REP) {
        marker = processUnboundedRep(repeated, marker, pb);
    } else if (lb < ub) {
        marker = expandUpperBound(repeated, ub - lb, marker, IfInsertionGap, pb);
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

Marker RE_Compiler::expandLowerBound(RE * const repeated, const int lb, Marker marker, const int ifGroupSize, PabloBuilder & pb) {
    if (LLVM_UNLIKELY(lb == 0)) {
        return marker;
    } else if (LLVM_UNLIKELY(lb == 1)) {
        return process(repeated, marker, pb);
    }
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
    Marker m1 = expandLowerBound(repeated, lb - group, marker, ifGroupSize * 2, nested);
    nested.createAssign(m, m1.stream());
    pb.createIf(marker.stream(), nested);
    mCompiledName = nestedMap.getParent();
    return Marker(m, m1.offset());
}

Marker RE_Compiler::expandUpperBound(RE * const repeated, const int ub, Marker marker, const int ifGroupSize,  PabloBuilder & pb) {
    if (LLVM_UNLIKELY(ub == 0)) {
        return marker;
    }
    const auto group = ifGroupSize < ub ? ifGroupSize : ub;
    for (auto i = 0; i < group; i++) {
        Marker a = process(repeated, marker, pb);
        Marker m = marker;
        AlignMarkers(a, m, pb);
        marker = Marker(pb.createOr(a.stream(), m.stream()), a.offset());
    }
    if (ub == group) {
        return marker;
    }
    Var * const m1a = pb.createVar("m", pb.createZeroes());
    auto nested = pb.createScope();
    NameMap nestedMap(mCompiledName);
    mCompiledName = &nestedMap;
    Marker m1 = expandUpperBound(repeated, ub - group, marker, ifGroupSize * 2, nested);
    nested.createAssign(m1a, m1.stream());
    pb.createIf(marker.stream(), nested);
    mCompiledName = nestedMap.getParent();
    return Marker(m1a, m1.offset());
}

Marker RE_Compiler::processUnboundedRep(RE * const repeated, Marker marker, PabloBuilder & pb) {
    // always use PostPosition markers for unbounded repetition.
    PabloAST * base = AdvanceMarker(marker, 1, pb).stream();
    if (LLVM_LIKELY(!AlgorithmOptionIsSet(DisableMatchStar))) {
        auto lengths = getLengthRange(repeated, mCodeUnitAlphabet);
        //llvm::errs() << "getLengthRange(repeated, mCodeUnitAlphabet) = " << lengths.first << ", " << lengths.second << "\n";
        if ((lengths.first == 1) && (lengths.second == 1)) {
            PabloAST * mask = compile(repeated, pb).stream();
            mask = pb.createOr(mask, pb.createNot(mIndexStream));
            // The post position character may land on the initial byte of a multi-byte character. Combine them with the masked range.
            PabloAST * unbounded = pb.createMatchStar(base, mask, "unbounded");
            return Marker(pb.createAnd(unbounded, mIndexStream, "unbounded"), 1);
        }
        if (mIndexingTransformer) {
            auto lengths = getLengthRange(repeated, mIndexingTransformer->getIndexingAlphabet());
            //llvm::errs() << "getLengthRange(repeated, getIndexingAlphabet) = " << lengths.first << ", " << lengths.second << "\n";
            if ((lengths.first == 1) && (lengths.second == 1)) {
                PabloAST * mask = compile(repeated, pb).stream();
                mask = pb.createOr(mask, pb.createNot(mIndexStream));
                PabloAST * unbounded = pb.createMatchStar(base, mask);
                return Marker(pb.createAnd(unbounded, mIndexStream, "unbounded"), 1);
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
        Marker result = process(repeated, Marker(m1, 1), pb);
        result = AdvanceMarker(result, 1, pb);
        PabloAST * loopComputation = result.stream();
        pb.createAssign(starPending, pb.createAnd(loopComputation, pb.createNot(m2)));
        pb.createAssign(starAccum, pb.createOr(loopComputation, m2));
        mWhileTest = pb.createOr(mWhileTest, starPending);
        mStarDepth--;
        return Marker(pb.createOr(base, starAccum, "unbounded"), result.offset());
    } else {
        Var * whileTest = pb.createVar("test", base);
        Var * whilePending = pb.createVar("pending", base);
        Var * whileAccum = pb.createVar("accum", base);
        mWhileTest = pb.createZeroes();
        auto wb = pb.createScope();
        NameMap nestedMap(mCompiledName);
        mCompiledName = &nestedMap;
        mStarDepth++;
        Marker result = process(repeated, Marker(whilePending, 1), wb);
        result = AdvanceMarker(result, 1, wb);
        PabloAST * loopComputation = result.stream();
        wb.createAssign(whilePending, wb.createAnd(loopComputation, wb.createNot(whileAccum)));
        wb.createAssign(whileAccum, wb.createOr(loopComputation, whileAccum));
        wb.createAssign(whileTest, wb.createOr(mWhileTest, whilePending));
        pb.createWhile(whileTest, wb);
        mStarDepth--;
        mCompiledName = nestedMap.getParent();
        return Marker(whileAccum, result.offset());
    }
}

inline Marker RE_Compiler::compileStart(Marker marker, pablo::PabloBuilder & pb) {
    PabloAST * const SOT = pb.createNot(pb.createAdvance(pb.createOnes(), 1));
    Marker m = AdvanceMarker(marker, 1, pb);
    return Marker(pb.createAnd(m.stream(), SOT, "SOT"), 1);
}

inline Marker RE_Compiler::compileEnd(Marker marker, pablo::PabloBuilder & pb) {
    PabloAST * const nextPos = AdvanceMarker(marker, 1, pb).stream();
    PabloAST * endOfText = pb.createAtEOF(pb.createAdvance(pb.createOnes(), 1), "EOT");
    PabloAST * const EOT_match = pb.createAnd(endOfText, nextPos, "EOT_match");
    return Marker(EOT_match, 1);
}

inline Marker RE_Compiler::AdvanceMarker(Marker marker, const unsigned offset, PabloBuilder & pb) {
    if (marker.offset() < offset) {
        return Marker(pb.createIndexedAdvance(marker.stream(), mIndexStream, offset - marker.offset()), offset);
    }
    return marker;
}

inline void RE_Compiler::AlignMarkers(Marker & m1, Marker & m2, PabloBuilder & pb) {
    if (m1.offset() < m2.offset()) {
        m1 = AdvanceMarker(m1, m2.offset(), pb);
    } else if (m2.offset() < m1.offset()) {
        m2 = AdvanceMarker(m2, m1.offset(), pb);
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
