/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "cc_compiler.h"
#include <cc/alphabet.h>
#include <pablo/codegenstate.h>
#include <pablo/boolean.h>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_ones.h>
#include <pablo/pe_infile.h>
#include <pablo/builder.hpp>
#include <pablo/pablo_kernel.h>
#include <llvm/IR/DerivedTypes.h>
#include <llvm/Support/ErrorHandling.h>
#include <llvm/Support/raw_ostream.h>
#include <cmath>

using namespace re;
using namespace pablo;
using namespace llvm;

namespace cc {
using octet_pair_t = Parabix_Ternary_CC_Compiler::octet_pair_t;
using octets_intervals_union_t = Parabix_Ternary_CC_Compiler::octets_intervals_union_t;

CC_Compiler::CC_Compiler(pablo::PabloBlock * scope)
: mBuilder(scope) {
}

CC_Compiler_Common::CC_Compiler_Common(unsigned encodingBits, std::vector<pablo::PabloAST *> basisBit, unsigned encodingMask)
: mEncodingBits(encodingBits)
, mBasisBit(basisBit)
, mEncodingMask(encodingMask) {
}

template<typename PabloBlockOrBuilder>
inline PabloAST * CC_Compiler_Common::getBasisVar(const unsigned i, PabloBlockOrBuilder & pb) const {
    assert (i < mEncodingBits);
    return mBasisBit[i];
}

Parabix_CC_Compiler::Parabix_CC_Compiler(pablo::PabloBlock * scope, std::vector<pablo::PabloAST *> basisBitSet)
: CC_Compiler(scope)
, CC_Compiler_Common(basisBitSet.size(), basisBitSet, (static_cast<unsigned>(1) << basisBitSet.size()) - static_cast<unsigned>(1)) {
}

PabloAST * Parabix_CC_Compiler::compileCC(const std::string & canonicalName, const CC *cc, PabloBlock & block) {
    PabloAST * const var = charset_expr(cc, block);
    if (LLVM_LIKELY(isa<Statement>(var))) {
        cast<Statement>(var)->setName(block.makeName(canonicalName));
    }
    return var;
}

PabloAST * Parabix_CC_Compiler::compileCC(const std::string & canonicalName, const CC *cc, PabloBuilder & builder) {
    PabloAST * const var = charset_expr(cc, builder);
    if (LLVM_LIKELY(isa<Statement>(var))) {
        cast<Statement>(var)->setName(builder.makeName(canonicalName));
    }
    return var;
}
    
template<typename PabloBlockOrBuilder>
PabloAST * Parabix_CC_Compiler::charset_expr(const CC * cc, PabloBlockOrBuilder & pb) {
    if (cc->empty()) {
        return pb.createZeroes();
    }
    if (cc->size() > 2) {
        bool combine = true;
        for (const interval_t & i : *cc) {
            if (lo_codepoint(i) != hi_codepoint(i)) {
                combine = false;
                break;
            }
        }
        if (combine) {
            auto i = cc->begin(), e = cc->end();
            for (auto j = i; ++j != e; i = j) {
                if ((lo_codepoint(i) + 2) != lo_codepoint(j)) {
                    combine  = false;
                    break;
                }
            }
            if (combine) {
                codepoint_t lo = lo_codepoint(cc->front());
                codepoint_t hi = lo_codepoint(cc->back());
                PabloAST * even_odd = getBasisVar(0, pb);
                if ((lo & 1) == 0) {
                    even_odd = pb.createNot(even_odd);
                }
                lo &= (mEncodingMask - 1);
                hi |= (mEncodingMask ^ (mEncodingMask - 1));
                PabloAST * expr = make_range(lo, hi, pb);
                return pb.createAnd(expr, even_odd);
            }
        }
    }
    PabloAST * expr = nullptr;
    for (const interval_t & i : *cc) {
        PabloAST * temp = char_or_range_expr(lo_codepoint(i), hi_codepoint(i), pb);
        expr = (expr == nullptr) ? temp : pb.createOr(expr, temp);
    }
    return pb.createInFile(expr);   
}

template<typename PabloBlockOrBuilder>
PabloAST * Parabix_CC_Compiler::bit_pattern_expr(const unsigned pattern, unsigned selected_bits, PabloBlockOrBuilder & pb) {
    if (LLVM_UNLIKELY(selected_bits == 0)) {
        return pb.createOnes();
    } else {
        std::vector<PabloAST*> terms;
        for (unsigned i = 0; selected_bits; ++i) {
            unsigned test_bit = static_cast<unsigned>(1) << i;
            PabloAST * term = pb.createOnes();
            if (selected_bits & test_bit) {
                term = getBasisVar(i, pb);
                if ((pattern & test_bit) == 0) {
                    term = pb.createNot(term);
                }
                selected_bits ^= test_bit;
            }
            terms.push_back(term);
        }
        if (terms.size() > 1) {
            //Reduce the list so that all of the expressions are contained within a single expression.
            std::vector<PabloAST*> temp;
            temp.reserve(terms.size());
            do {
                for (unsigned i = 0; i < (terms.size() / 2); i++) {
                    temp.push_back(pb.createAnd(terms[2 * i], terms[(2 * i) + 1]));
                }
                if (terms.size() % 2 == 1) {
                    temp.push_back(terms.back());
                }
                terms.swap(temp);
                temp.clear();
            }
            while (terms.size() > 1);
        }
        assert (terms.size() == 1);
        return terms.front();
    }
}

template<typename PabloBlockOrBuilder>
PabloAST * Parabix_CC_Compiler::make_range(const codepoint_t n1, const codepoint_t n2, PabloBlockOrBuilder & pb) {
    codepoint_t diff_count = 0;

    for (codepoint_t diff_bits = n1 ^ n2; diff_bits; diff_count++, diff_bits >>= 1);

    if ((n2 < n1) || (diff_count > mEncodingBits)) {
        llvm::report_fatal_error("Bad Range: [" + std::to_string(n1) + "," +
                                 std::to_string(n2) + "] for " +
                                 std::to_string(mEncodingBits) + "-bit encoding");
    }

    const codepoint_t mask0 = (static_cast<codepoint_t>(1) << diff_count) - 1;

    PabloAST * common = bit_pattern_expr(n1 & ~mask0, mEncodingMask ^ mask0, pb);

    if (diff_count == 0) return common;

    const codepoint_t mask1 = (static_cast<codepoint_t>(1) << (diff_count - 1)) - 1;

    PabloAST* lo_test = GE_Range(diff_count - 1, n1 & mask1, pb);
    PabloAST* hi_test = LE_Range(diff_count - 1, n2 & mask1, pb);

    return pb.createAnd(common, pb.createSel(getBasisVar(diff_count - 1, pb), hi_test, lo_test));
}

template<typename PabloBlockOrBuilder>
PabloAST * Parabix_CC_Compiler::GE_Range(const unsigned N, const unsigned n, PabloBlockOrBuilder &pb) {
    if (N == 0) {
        return pb.createOnes(); //Return a true literal.
    }
    else if (((N % 2) == 0) && ((n >> (N - 2)) == 0)) {
        return pb.createOr(pb.createOr(getBasisVar(N - 1, pb), getBasisVar(N - 2, pb)), GE_Range(N - 2, n, pb));
    }
    else if (((N % 2) == 0) && ((n >> (N - 2)) == 3)) {
        return pb.createAnd(pb.createAnd(getBasisVar(N - 1, pb), getBasisVar(N - 2, pb)), GE_Range(N - 2, n - (3 << (N - 2)), pb));
    }
    else if (N >= 1)
    {
        int hi_bit = n & (1 << (N - 1));
        int lo_bits = n - hi_bit;
        PabloAST * lo_range = GE_Range(N - 1, lo_bits, pb);
        if (hi_bit == 0)
        {
            /*
              If the hi_bit of n is not set, then whenever the corresponding bit
              is set in the target, the target will certaily be >=.  Otherwise,
              the value of GE_range(N-1), lo_range) is required.
            */
            return pb.createOr(getBasisVar(N - 1, pb), lo_range);
        }
        else
        {
            /*
              If the hi_bit of n is set, then the corresponding bit must be set
              in the target for >= and GE_range(N-1, lo_bits) must also be true.
            */
            return pb.createAnd(getBasisVar(N - 1, pb), lo_range);
        }
    }
    throw std::runtime_error("Unexpected input given to ge_range: " + std::to_string(N) + ", " + std::to_string(n));
}

template<typename PabloBlockOrBuilder>
PabloAST * Parabix_CC_Compiler::LE_Range(const unsigned N, const unsigned n, PabloBlockOrBuilder & pb) {
    /*
      If an N-bit pattern is all ones, then it is always true that any n-bit value is LE this pattern.
      Handling this as a special case avoids an overflow issue with n+1 requiring more than N bits.
    */
    if ((n + 1) == (1UL << N)) {
        return pb.createOnes(); //True.
    } else {
        return pb.createNot(GE_Range(N, n + 1, pb));
    }
}

template<typename PabloBlockOrBuilder>
inline PabloAST * Parabix_CC_Compiler::char_test_expr(const codepoint_t ch, PabloBlockOrBuilder &pb) {
    return bit_pattern_expr(ch, mEncodingMask, pb);
}

template<typename PabloBlockOrBuilder>
inline PabloAST * Parabix_CC_Compiler::char_or_range_expr(const codepoint_t lo, const codepoint_t hi, PabloBlockOrBuilder &pb) {
    if (lo == hi) {
        return char_test_expr(lo, pb);
    } else if (lo < hi) {
        return make_range(lo, hi, pb);
    }
    llvm::report_fatal_error(std::string("Invalid Character Set Range: [") + std::to_string(lo) + "," + std::to_string(hi) + "]");
}

PabloAST * Parabix_CC_Compiler::createUCDSequence(const unsigned byte_no, PabloAST * target, PabloAST * var, PabloAST * prefix, PabloBuilder & builder) {
    if (byte_no > 1) {
        var = builder.createAnd(var, builder.createAdvance(prefix, 1));
    }
    return builder.createOr(target, var);
}

PabloAST * Parabix_CC_Compiler::createUCDSequence(const unsigned byte_no, const unsigned len, PabloAST * target, PabloAST * var, PabloAST * prefix, PabloAST * suffix, PabloBuilder & builder) {
    if (byte_no > 1) {
        var = builder.createAnd(builder.createAdvance(prefix, 1), var);
    }
    for (unsigned i = byte_no; i != len; ++i) {
        var = builder.createAnd(suffix, builder.createAdvance(var, 1));
    }
    return builder.createOr(target, var);
}
    
PabloAST * compileCCfromCodeUnitStream(const CC * cc, PabloAST * codeUnitStream, PabloBuilder & pb) {
    const Alphabet * a = cc->getAlphabet();
    if (!isa<CodeUnitAlphabet>(a)) {
        llvm::report_fatal_error("compileCCfromCodeUnitStream must be applied to a CC with a CodeUnitAlphabet");
    }
    unsigned codeUnitWidth = cast<const CodeUnitAlphabet>(a)->getCodeUnitBitWidth();
    unsigned topBit = 1 << (codeUnitWidth - 1);
    unsigned maxCodeVal = (topBit - 1) | topBit;
    //
    // Optimization if there are isolated codepoints that are not in the set.
    UCD::UnicodeSet negatedIsolates = (~(*cc)).isolates();
    UCD::UnicodeSet withNegatedIsolates = (*cc + negatedIsolates);
    PabloAST * ccStrm = pb.createZeroes();
    for (const auto & interval : withNegatedIsolates) {
        unsigned lo = re::lo_codepoint(interval);
        unsigned hi = re::hi_codepoint(interval);
        if (lo == hi) {
            PabloAST * testVal = pb.createRepeat(codeUnitWidth, lo);
            ccStrm = pb.createOr(ccStrm, pb.createEquals(codeUnitStream, testVal));
        } else if (lo == 0) {
            if (hi == maxCodeVal) {
                // All code units
                ccStrm = pb.createOnes();
            } else {
                PabloAST * testVal = pb.createRepeat(codeUnitWidth, hi+1);
                ccStrm = pb.createOr(ccStrm, pb.createLessThan(codeUnitStream, testVal));
            }
        } else if (hi == maxCodeVal) {
            PabloAST * testVal = pb.createRepeat(codeUnitWidth, lo);
            ccStrm = pb.createOr(ccStrm, pb.createNot(pb.createLessThan(codeUnitStream, testVal)));
        } else {
            PabloAST * testVal_lo = pb.createRepeat(codeUnitWidth, lo);
            PabloAST * testVal_hi = pb.createRepeat(codeUnitWidth, hi + 1);
            ccStrm = pb.createOr(ccStrm, pb.createAnd(pb.createNot(pb.createLessThan(codeUnitStream, testVal_lo)),
                                                      pb.createLessThan(codeUnitStream, testVal_hi)));
        }
    }
    if (!negatedIsolates.empty()) {
        PabloAST * toExclude = pb.createZeroes();
        for (const auto & interval : negatedIsolates) {
            PabloAST * testVal = pb.createRepeat(codeUnitWidth, re::lo_codepoint(interval));
            toExclude = pb.createOr(toExclude, pb.createEquals(codeUnitStream, testVal));
        }
        ccStrm = pb.createAnd(ccStrm, pb.createNot(toExclude));
    }
    return pb.createInFile(ccStrm);
}
    
Direct_CC_Compiler::Direct_CC_Compiler(pablo::PabloBlock * scope, pablo::PabloAST * codeUnitStream)
: CC_Compiler(scope)
, mCodeUnitStream(codeUnitStream) {
}

pablo::PabloAST * Direct_CC_Compiler::compileCC(const std::string & name, const re::CC *cc, pablo::PabloBlock & block) {
    PabloBuilder pb(&block);
    return compileCC(name, cc, pb);
}

pablo::PabloAST * Direct_CC_Compiler::compileCC(const std::string & name, const re::CC *cc, pablo::PabloBuilder & b) {
    return compileCCfromCodeUnitStream(cc, mCodeUnitStream, b);
}

Parabix_Ternary_CC_Compiler::Parabix_Ternary_CC_Compiler(pablo::PabloBlock * scope, std::vector<pablo::PabloAST *> basisBitSet)
: CC_Compiler(scope)
, CC_Compiler_Common(basisBitSet.size(), basisBitSet, (static_cast<unsigned>(1) << basisBitSet.size()) - static_cast<unsigned>(1)) {
}

PabloAST * Parabix_Ternary_CC_Compiler::compileCC(const std::string & canonicalName, const CC *cc, PabloBlock & block) {
    PabloAST * const var = charset_expr(cc, block);
    if (LLVM_LIKELY(isa<Statement>(var))) {
        cast<Statement>(var)->setName(block.makeName(canonicalName));
    }
    return var;
}

PabloAST * Parabix_Ternary_CC_Compiler::compileCC(const std::string & canonicalName, const CC *cc, PabloBuilder & builder) {
    PabloAST * const var = charset_expr(cc, builder);
    if (LLVM_LIKELY(isa<Statement>(var))) {
        cast<Statement>(var)->setName(builder.makeName(canonicalName));
    }
    return var;
}

template<typename PabloBlockOrBuilder>
pablo::PabloAST * Parabix_Ternary_CC_Compiler::charset_expr(const re::CC *cc, PabloBlockOrBuilder & pb) {
    if (cc->empty()) {
        return pb.createZeroes();
    }
    octets_intervals_union_t octets_intervals = make_octets_intervals_union(cc);
    std::vector<octet_pair_t> octets = octets_in_union(octets_intervals);
    std::vector<interval_t> intervals = intervals_in_union(octets_intervals);
    PabloAST *expr1 = make_octets_expr(octets, pb);
    PabloAST *expr2 = make_intervals_expr(intervals, pb);
    return pb.createInFile(pb.createOr(expr1, expr2));
}

template<typename PabloBlockOrBuilder>
PabloAST * Parabix_Ternary_CC_Compiler::make_intervals_expr(const std::vector<interval_t> intervals, PabloBlockOrBuilder & pb) {
    PabloAST * expr = nullptr;
    for (auto i : intervals) {
        PabloAST * temp = char_or_range_expr(lo_codepoint(i), hi_codepoint(i), pb);
        expr = (expr == nullptr) ? temp : pb.createOr(expr, temp);
    }
    if (expr == nullptr) return pb.createZeroes();
    return expr;
}

template<typename PabloBlockOrBuilder>
PabloAST * Parabix_Ternary_CC_Compiler::make_octets_expr(const std::vector<octet_pair_t> octets, PabloBlockOrBuilder & pb) {
    std::vector<PabloAST *> terms;
    for (auto octet : octets) {
        uint8_t mask = octet_mask(octet);
        uint8_t mid3_mask = 1 << ((octet_base_codepoint(octet) & 0x38) >> 3);
        uint8_t hi2_mask = 1 << ((octet_base_codepoint(octet) & 0xC0) >> 6);
        PabloAST * mid3 = pb.createTernary(pb.getInteger(mid3_mask), getBasisVar(5, pb), getBasisVar(4, pb), getBasisVar(3, pb));
        PabloAST * hi5 = pb.createTernary(pb.getInteger(hi2_mask << 4), mid3, getBasisVar(7, pb), getBasisVar(6, pb));
        if (mask != 0xFF) {
            PabloAST * lo3 = pb.createTernary(pb.getInteger(mask), getBasisVar(2, pb), getBasisVar(1, pb), getBasisVar(0, pb));
            terms.push_back(pb.createAnd(hi5, lo3));
        } else {
            if (terms.empty()) {
                terms.push_back(hi5);
            } else {
                terms[terms.size()-1] = pb.createOr(terms.back(), hi5);
            }
        }
    }
    if (terms.empty()) return pb.createZeroes();
    //Reduce the list so that all of the expressions are contained within a single expression.
    std::vector<PabloAST *> temp;
    temp.reserve(terms.size());
    do {
        for (unsigned i = 0; i < (terms.size() / 3); i++) {
            temp.push_back(pb.createOr3(terms[3 * i], terms[(3 * i) + 1], terms[(3 * i) + 2]));
        }
        for (unsigned i = 0; i < (terms.size() % 3); i++) {
            temp.push_back(terms[terms.size() - i - 1]);
        }
        terms.swap(temp);
        temp.clear();
    } while (terms.size() >= 3);
    assert (terms.size() == 2 || terms.size() == 1);
    if (terms.size() == 2) {
        return pb.createOr(terms.front(), terms.back());
    } else {
        return terms.front();
    }
}

octets_intervals_union_t Parabix_Ternary_CC_Compiler::make_octets_intervals_union(const re::CC *cc) {
    // The low value of an octet is always 0x0 or 0x8
    // 0x60 -> 0110 0000, 0x67 -> 0110 0111
    // 0x68 -> 0110 1000, 0x6F -> 0110 1111
    // Example of input: <0x67, 0x69>, <0x6B, 0x6B>, <0x70, 0x70>
    // Output: <0x60, 10000000>, <0x68, 00001011>, <0x70, 00000001>
    std::vector<octet_pair_t> octets;
    std::vector<interval_t> intervals;
    for (const interval_t & i : *cc) {
        codepoint_t lo_cp = lo_codepoint(i);
        codepoint_t hi_cp = hi_codepoint(i);
        codepoint_t fst_octet = lo_cp & 0xFFFFFFF8;
        codepoint_t lst_octet = hi_cp & 0xFFFFFFF8;
        if (fst_octet != lst_octet) {
            intervals.push_back(i);
            continue;
        }
        for (codepoint_t octet = fst_octet; octet <= lst_octet; octet += 0x8) {
            // lo_bit and hi_bit values are always in the range 0..7
            // as these are all possible values for selected bits in an octet
            uint8_t mask = 0;
            uint8_t lo_bit = 0;
            uint8_t hi_bit = 0x7;
            if (octet > fst_octet && octet < lst_octet) mask = 0xFF;
            else if (octet == fst_octet && octet < lst_octet) lo_bit = lo_cp - octet;
            else if (octet > fst_octet && octet == lst_octet) hi_bit = hi_cp - octet;
            else if (fst_octet == lst_octet) {
                lo_bit = lo_cp - octet;
                hi_bit = hi_cp - octet;
            }
            if (mask == 0) {
                for (auto j = lo_bit; j <= hi_bit; j++)
                    mask |= 1 << j;
            }
            if (!octets.empty()) {
                octet_pair_t * lst_selected = &(octets.back());
                if (octet == octet_base_codepoint(*lst_selected)) {
                    mask |= octet_mask(*lst_selected);
                    lst_selected->second = mask;
                    continue;
                }
            }
            octets.push_back(std::make_pair(octet, mask));
        }
    }
    return std::make_pair(octets, intervals);
}

template<typename PabloBlockOrBuilder>
PabloAST * Parabix_Ternary_CC_Compiler::bit_pattern_expr(const unsigned pattern, unsigned selected_bits, PabloBlockOrBuilder & pb) {
    if (LLVM_UNLIKELY(selected_bits == 0)) {
        return pb.createOnes();
    } else {
        std::vector<PabloAST*> terms;
        for (unsigned i = 0; selected_bits; ++i) {
            unsigned test_bit = static_cast<unsigned>(1) << i;
            PabloAST * term = pb.createOnes();
            if (selected_bits & test_bit) {
                term = getBasisVar(i, pb);
                if ((pattern & test_bit) == 0) {
                    term = pb.createNot(term);
                }
                selected_bits ^= test_bit;
            }
            terms.push_back(term);
        }
        if (terms.size() > 1) {
            //Reduce the list so that all of the expressions are contained within a single expression.
            std::vector<PabloAST*> temp;
            temp.reserve(terms.size());
            do {
                for (unsigned i = 0; i < (terms.size() / 2); i++) {
                    temp.push_back(pb.createAnd(terms[2 * i], terms[(2 * i) + 1]));
                }
                if (terms.size() % 2 == 1) {
                    temp.push_back(terms.back());
                }
                terms.swap(temp);
                temp.clear();
            }
            while (terms.size() > 1);
        }
        assert (terms.size() == 1);
        return terms.front();
    }
}

template<typename PabloBlockOrBuilder>
PabloAST * Parabix_Ternary_CC_Compiler::make_range(const codepoint_t n1, const codepoint_t n2, PabloBlockOrBuilder & pb) {
    codepoint_t diff_count = 0;

    for (codepoint_t diff_bits = n1 ^ n2; diff_bits; diff_count++, diff_bits >>= 1);

    if ((n2 < n1) || (diff_count > mEncodingBits)) {
        llvm::report_fatal_error("Bad Range: [" + std::to_string(n1) + "," +
                                 std::to_string(n2) + "] for " +
                                 std::to_string(mEncodingBits) + "-bit encoding");
    }

    const codepoint_t mask0 = (static_cast<codepoint_t>(1) << diff_count) - 1;

    PabloAST * common = bit_pattern_expr(n1 & ~mask0, mEncodingMask ^ mask0, pb);

    if (diff_count == 0) return common;

    const codepoint_t mask1 = (static_cast<codepoint_t>(1) << (diff_count - 1)) - 1;

    PabloAST* lo_test = GE_Range(diff_count - 1, n1 & mask1, pb);
    PabloAST* hi_test = LE_Range(diff_count - 1, n2 & mask1, pb);

    return pb.createAnd(common, pb.createSel(getBasisVar(diff_count - 1, pb), hi_test, lo_test));
}

template<typename PabloBlockOrBuilder>
PabloAST * Parabix_Ternary_CC_Compiler::GE_Range(const unsigned N, const unsigned n, PabloBlockOrBuilder &pb) {
    if (N == 0) {
        return pb.createOnes(); //Return a true literal.
    }
    else if (((N % 2) == 0) && ((n >> (N - 2)) == 0)) {
        return pb.createOr(pb.createOr(getBasisVar(N - 1, pb), getBasisVar(N - 2, pb)), GE_Range(N - 2, n, pb));
    }
    else if (((N % 2) == 0) && ((n >> (N - 2)) == 3)) {
        return pb.createAnd(pb.createAnd(getBasisVar(N - 1, pb), getBasisVar(N - 2, pb)), GE_Range(N - 2, n - (3 << (N - 2)), pb));
    }
    else if (N >= 1)
    {
        int hi_bit = n & (1 << (N - 1));
        int lo_bits = n - hi_bit;
        PabloAST * lo_range = GE_Range(N - 1, lo_bits, pb);
        if (hi_bit == 0)
        {
            /*
              If the hi_bit of n is not set, then whenever the corresponding bit
              is set in the target, the target will certaily be >=.  Otherwise,
              the value of GE_range(N-1), lo_range) is required.
            */
            return pb.createOr(getBasisVar(N - 1, pb), lo_range);
        }
        else
        {
            /*
              If the hi_bit of n is set, then the corresponding bit must be set
              in the target for >= and GE_range(N-1, lo_bits) must also be true.
            */
            return pb.createAnd(getBasisVar(N - 1, pb), lo_range);
        }
    }
    throw std::runtime_error("Unexpected input given to ge_range: " + std::to_string(N) + ", " + std::to_string(n));
}

template<typename PabloBlockOrBuilder>
PabloAST * Parabix_Ternary_CC_Compiler::LE_Range(const unsigned N, const unsigned n, PabloBlockOrBuilder & pb) {
    /*
      If an N-bit pattern is all ones, then it is always true that any n-bit value is LE this pattern.
      Handling this as a special case avoids an overflow issue with n+1 requiring more than N bits.
    */
    if ((n + 1) == (1UL << N)) {
        return pb.createOnes(); //True.
    } else {
        return pb.createNot(GE_Range(N, n + 1, pb));
    }
}

template<typename PabloBlockOrBuilder>
inline PabloAST * Parabix_Ternary_CC_Compiler::char_test_expr(const codepoint_t ch, PabloBlockOrBuilder &pb) {
    return bit_pattern_expr(ch, mEncodingMask, pb);
}

template<typename PabloBlockOrBuilder>
inline PabloAST * Parabix_Ternary_CC_Compiler::char_or_range_expr(const codepoint_t lo, const codepoint_t hi, PabloBlockOrBuilder &pb) {
    if (lo == hi) {
        return char_test_expr(lo, pb);
    } else if (lo < hi) {
        return make_range(lo, hi, pb);
    }
    llvm::report_fatal_error(std::string("Invalid Character Set Range: [") + std::to_string(lo) + "," + std::to_string(hi) + "]");
}

PabloAST * Parabix_Ternary_CC_Compiler::createUCDSequence(const unsigned byte_no, PabloAST * target, PabloAST * var, PabloAST * prefix, PabloBuilder & builder) {
    if (byte_no <= 1) return builder.createOr(target, var);
    else return builder.createTernary(builder.getInteger(0xF8), target, var, builder.createAdvance(prefix, 1));
}

PabloAST * Parabix_Ternary_CC_Compiler::createUCDSequence(const unsigned byte_no, const unsigned len, PabloAST * target, PabloAST * var, PabloAST * prefix, PabloAST * suffix, PabloBuilder & builder) {
    if (byte_no > 1) {
        var = builder.createAnd(builder.createAdvance(prefix, 1), var);
    }
    for (unsigned i = byte_no; i != len - 1; ++i) {
        var = builder.createAnd(suffix, builder.createAdvance(var, 1));
    }
    return builder.createTernary(builder.getInteger(0xF8), target, suffix, builder.createAdvance(var, 1));
}

} // end of namespace cc
