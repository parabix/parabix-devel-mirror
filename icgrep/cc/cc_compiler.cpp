/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "cc_compiler.h"
#include "utf_encoding.h"

//Pablo Expressions
#include <re/re_alt.h>
#include <re/re_cc.h>
#include <re/re_seq.h>
#include <re/re_rep.h>
#include <re/re_name.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_assertion.h>
#include <pablo/codegenstate.h>
#include <pablo/builder.hpp>
#include <pablo/function.h>
#include <stdexcept>

using namespace re;
using namespace pablo;

namespace cc {

CC_Compiler::CC_Compiler(PabloFunction & function, const Encoding & encoding, const std::string prefix)
: mBuilder(function.getEntryBlock())
, mBasisBit(encoding.getBits())
, mEncoding(encoding) {
    for (unsigned i = 0; i != encoding.getBits(); i++) {
        Var * var = mBuilder.createVar(prefix + std::to_string(i));
        function.setParameter(i, var);
        mBasisBit[i] = var;
    }
}

Assign * CC_Compiler::compileCC(const std::string && canonicalName, const CC *cc, PabloBlock & block) {
    return block.createAssign(std::move(canonicalName), charset_expr(cc, block));
}

Assign * CC_Compiler::compileCC(const std::string && canonicalName, const CC *cc, PabloBuilder & builder) {
    return builder.createAssign(std::move(canonicalName), charset_expr(cc, builder));
}

template<typename PabloBlockOrBuilder>
PabloAST * CC_Compiler::charset_expr(const CC * cc, PabloBlockOrBuilder & pb) {
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
                const codepoint_t mask = mEncoding.getMask();
                lo &= (mask - 1);
                hi |= (mask ^ (mask - 1));
                PabloAST * expr = make_range(lo, hi, pb);
                PabloAST * bit0 = getBasisVar(0);
                if ((lo & 1) == 0) {
                    bit0 = pb.createNot(bit0);
                }
                return pb.createAnd(expr, bit0);
            }
        }
    }
    PabloAST * expr = nullptr;
    for (const interval_t & i : *cc) {
        PabloAST * temp = char_or_range_expr(lo_codepoint(i), hi_codepoint(i), pb);
        expr = (expr == nullptr) ? temp : pb.createOr(expr, temp);
    }
    return expr;
}

template<typename PabloBlockOrBuilder>
PabloAST * CC_Compiler::bit_pattern_expr(const unsigned pattern, unsigned selected_bits, PabloBlockOrBuilder & pb) {
    if (LLVM_UNLIKELY(selected_bits == 0)) {
        return PabloBlock::createOnes();
    } else {
        std::vector<PabloAST*> terms;
        for (unsigned i = 0; selected_bits; ++i) {
            unsigned test_bit = static_cast<unsigned>(1) << i;
            PabloAST * term = PabloBlock::createOnes();
            if (selected_bits & test_bit) {
                term = getBasisVar(i);
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
inline PabloAST * CC_Compiler::char_test_expr(const codepoint_t ch, PabloBlockOrBuilder &pb) {
    return bit_pattern_expr(ch, mEncoding.getMask(), pb);
}

template<typename PabloBlockOrBuilder>
PabloAST * CC_Compiler::make_range(const codepoint_t n1, const codepoint_t n2, PabloBlockOrBuilder & pb) {
    codepoint_t diff_count = 0;

    for (codepoint_t diff_bits = n1 ^ n2; diff_bits; diff_count++, diff_bits >>= 1);

    if ((n2 < n1) || (diff_count > mEncoding.getBits()))
    {
        throw std::runtime_error("Bad Range: [" + std::to_string(n1) + "," + std::to_string(n2) + "]");
    }

    const codepoint_t mask0 = (static_cast<codepoint_t>(1) << diff_count) - 1;

    PabloAST * common = bit_pattern_expr(n1 & ~mask0, mEncoding.getMask() ^ mask0, pb);

    if (diff_count == 0) return common;

    const codepoint_t mask1 = (static_cast<codepoint_t>(1) << (diff_count - 1)) - 1;

    PabloAST* lo_test = GE_Range(diff_count - 1, n1 & mask1, pb);
    PabloAST* hi_test = LE_Range(diff_count - 1, n2 & mask1, pb);

    return pb.createAnd(common, pb.createSel(getBasisVar(diff_count - 1), hi_test, lo_test));
}

template<typename PabloBlockOrBuilder>
PabloAST * CC_Compiler::GE_Range(const unsigned N, const unsigned n, PabloBlockOrBuilder &pb) {
    if (N == 0) {
        return pb.createOnes(); //Return a true literal.
    }
    else if (((N % 2) == 0) && ((n >> (N - 2)) == 0)) {
        return pb.createOr(pb.createOr(getBasisVar(N - 1), getBasisVar(N - 2)), GE_Range(N - 2, n, pb));
    }
    else if (((N % 2) == 0) && ((n >> (N - 2)) == 3)) {
        return pb.createAnd(pb.createAnd(getBasisVar(N - 1), getBasisVar(N - 2)), GE_Range(N - 2, n - (3 << (N - 2)), pb));
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
            return pb.createOr(getBasisVar(N - 1), lo_range);
        }
        else
        {
            /*
              If the hi_bit of n is set, then the corresponding bit must be set
              in the target for >= and GE_range(N-1, lo_bits) must also be true.
            */
            return pb.createAnd(getBasisVar(N - 1), lo_range);
        }
    }
    throw std::runtime_error("Unexpected input given to ge_range: " + std::to_string(N) + ", " + std::to_string(n));
}

template<typename PabloBlockOrBuilder>
PabloAST * CC_Compiler::LE_Range(const unsigned N, const unsigned n, PabloBlockOrBuilder &pb)
{
    /*
      If an N-bit pattern is all ones, then it is always true that any n-bit value is LE this pattern.
      Handling this as a special case avoids an overflow issue with n+1 requiring more than N bits.
    */
    if ((n + 1) == (1 << N)) {
        return pb.createOnes(); //True.
    }
    else {
        return pb.createNot(GE_Range(N, n + 1, pb));
    }
}

template<typename PabloBlockOrBuilder>
inline PabloAST * CC_Compiler::char_or_range_expr(const codepoint_t lo, const codepoint_t hi, PabloBlockOrBuilder &pb) {
    if (lo == hi) {
        return char_test_expr(lo, pb);
    }
    else if (lo < hi) {
        return make_range(lo, hi, pb);
    }
    throw std::runtime_error(std::string("Invalid Character Set Range: [") + std::to_string(lo) + "," + std::to_string(hi) + "]");
}

inline Var * CC_Compiler::getBasisVar(const int i) const {
    assert (i >= 0 && i < mEncoding.getBits());
    return mBasisBit[mEncoding.getBits() - i - 1];
}

} // end of namespace cc
