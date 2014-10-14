/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "cc_compiler.h"
#include "utf_encoding.h"

//Pablo Expressions
#include <pablo/codegenstate.h>
#include <re/re_alt.h>
#include <re/re_cc.h>
#include <re/re_seq.h>
#include <re/re_rep.h>
#include <re/re_name.h>

#include <utility>
#include <string>
#include <list>
#include <map>
#include <algorithm>

#include <cassert>
#include <stdlib.h>
#include <stdexcept>

using namespace re;
using namespace pablo;

namespace cc {

CC_Compiler::CC_Compiler(PabloBlock & cg, const Encoding encoding, const std::string basis_pattern, const std::string gensym_pattern)
: mCG(cg)
, mBasisBit(encoding.getBits())
, mEncoding(encoding)
, mGenSymPattern(gensym_pattern)
, mBasisPattern(basis_pattern)
{
    for (int i = 0; i < mEncoding.getBits(); i++) {
        mBasisBit[i] = mCG.createVar(mBasisPattern + std::to_string((mEncoding.getBits() - 1) - i));
    }
}

inline Var * CC_Compiler::getBasisVar(const int i) const {
    return mBasisBit[i];
}

void CC_Compiler::compile(const REMap & re_map) {
    for (auto i =  re_map.cbegin(); i != re_map.cend(); ++i) {
        process_re(i->second);
    }
    for (auto i =  re_map.cbegin(); i != re_map.cend(); ++i) {
        //This is specifically for the utf8 multibyte character classes.
        if (Seq * seq = dyn_cast<Seq>(i->second)) {
            if (seq->getType() == Seq::Type::Byte) {
                Assign * assignment = nullptr;
                auto j = seq->begin();
                while (true) {
                    Name * name = dyn_cast<Name>(*j);
                    assert (name);
                    CharClass * cc = mCG.createCharClass(name->getName());
                    PabloE * sym = assignment ? mCG.createAnd(mCG.createVar(assignment->getName()), cc) : cc;
                    if (++j != seq->end()) {
                        assignment = mCG.createAssign(mCG.ssa("marker"), mCG.createAdvance(sym));
                        continue;
                    }
                    mCG.createAssign(seq->getName(), sym);
                    break;
                }
            }
        }
    }
}

void CC_Compiler::process_re(const RE * re) {
    if (const Alt * alt = dyn_cast<const Alt>(re)) {
        for (const RE * re : *alt) {
            process_re(re);
        }
    }
    else if (const CC * cc = dyn_cast<const CC>(re)) {
        process(cc);
    }
    else if (const Rep* re_rep = dyn_cast<const Rep>(re)) {
        process_re(re_rep->getRE());
    }
    else if (const Seq* re_seq = dyn_cast<const Seq>(re)) {
        for (const RE * re : *re_seq) {
            process_re(re);
        }
    }
}

inline void CC_Compiler::process(const CC * cc) {
    if (mComputedSet.insert(cc->getName()).second) {
        // Add the new mapping to the list of pablo statements:
        mCG.createAssign(cc->getName(), charset_expr(cc));
    }
}

PabloE * CC_Compiler::bit_pattern_expr(const unsigned pattern, unsigned selected_bits)
{
    if (selected_bits == 0) {
        return mCG.createAll(1);
    }

    std::vector<PabloE*> bit_terms;
    unsigned i = 0;

    while (selected_bits)
    {
        unsigned test_bit = 1 << i;
        if (selected_bits & test_bit)
        {
            if ((pattern & test_bit) == 0)
            {
                bit_terms.push_back(mCG.createNot(getBasisVar(i)));
            }
            else
            {
                bit_terms.push_back(getBasisVar(i));
            }
        }
        else
        {
            bit_terms.push_back(mCG.createAll(1));
        }
        selected_bits &= ~test_bit;
        i++;
    }

    //Reduce the list so that all of the expressions are contained within a single expression.
    while (bit_terms.size() > 1)
    {
        std::vector<PabloE*> new_terms;
        for (unsigned long i = 0; i < (bit_terms.size()/2); i++)
        {
            new_terms.push_back(mCG.createAnd(bit_terms[(2 * i) + 1], bit_terms[2 * i]));
        }
        if (bit_terms.size() % 2 == 1)
        {
            new_terms.push_back(bit_terms[bit_terms.size() -1]);
        }
        bit_terms.assign(new_terms.begin(), new_terms.end());
    }
    return bit_terms[0];
}

PabloE * CC_Compiler::char_test_expr(const CodePointType ch)
{
    return bit_pattern_expr(ch, mEncoding.getMask());
}

PabloE * CC_Compiler::make_range(const CodePointType n1, const CodePointType n2)
{
    CodePointType diff_count = 0;

    for (CodePointType diff_bits = n1 ^ n2; diff_bits; diff_count++, diff_bits >>= 1);

    if ((n2 < n1) || (diff_count > mEncoding.getBits()))
    {
        throw std::runtime_error(std::string("Bad Range: [") + std::to_string(n1) + "," + std::to_string(n2) + "]");
    }

    const CodePointType mask0 = (static_cast<CodePointType>(1) << diff_count) - 1;

    PabloE * common = bit_pattern_expr(n1 & ~mask0, mEncoding.getMask() ^ mask0);

    if (diff_count == 0) return common;

    const CodePointType mask1 = (static_cast<CodePointType>(1) << (diff_count - 1)) - 1;

    PabloE* lo_test = GE_Range(diff_count - 1, n1 & mask1);
    PabloE* hi_test = LE_Range(diff_count - 1, n2 & mask1);

    return mCG.createAnd(common, mCG.createSel(getBasisVar(diff_count - 1), hi_test, lo_test));
}

PabloE * CC_Compiler::GE_Range(const unsigned N, const unsigned n) {
    if (N == 0)
    {
        return mCG.createAll(1); //Return a true literal.
    }
    else if (((N % 2) == 0) && ((n >> (N - 2)) == 0))
    {
        return mCG.createOr(mCG.createOr(getBasisVar(N - 1), getBasisVar(N - 2)), GE_Range(N - 2, n));
    }
    else if (((N % 2) == 0) && ((n >> (N - 2)) == 3))
    {
        return mCG.createAnd(mCG.createAnd(getBasisVar(N - 1), getBasisVar(N - 2)), GE_Range(N - 2, n - (3 << (N - 2))));
    }
    else if (N >= 1)
    {
        int hi_bit = n & (1 << (N - 1));
        int lo_bits = n - hi_bit;
        PabloE * lo_range = GE_Range(N - 1, lo_bits);
        if (hi_bit == 0)
        {
            /*
              If the hi_bit of n is not set, then whenever the corresponding bit
              is set in the target, the target will certaily be >=.  Oterwise,
              the value of GE_range(N-1), lo_range) is required.
            */
            return mCG.createOr(getBasisVar(N - 1), lo_range);
        }
        else
        {
            /*
              If the hi_bit of n is set, then the corresponding bit must be set
              in the target for >= and GE_range(N-1, lo_bits) must also be true.
            */
            return mCG.createAnd(getBasisVar(N - 1), lo_range);
        }
    }
    throw std::runtime_error("Unexpected input given to ge_range: " + std::to_string(N) + ", " + std::to_string(n));
}

PabloE * CC_Compiler::LE_Range(const unsigned N, const unsigned n)
{
    /*
      If an N-bit pattern is all ones, then it is always true that any n-bit value is LE this pattern.
      Handling this as a special case avoids an overflow issue with n+1 requiring more than N bits.
    */
    if ((n + 1) == (1 << N)) {
        return mCG.createAll(1); //True.
    }
    else {
        return mCG.createNot(GE_Range(N, n + 1));
    }
}

inline PabloE * CC_Compiler::charset_expr(const CC * cc) {
    if (cc->empty()) {
        return mCG.createAll(0);
    }
    if (cc->size() > 2) {
        bool combine = true;
        for (const CharSetItem & item : *cc) {
            if (item.lo_codepoint != item.hi_codepoint) {
                combine = false;
                break;
            }
        }
        if (combine) {
            auto i = cc->cbegin();
            for (auto j = i; ++j != cc->cend(); i = j) {
                const CharSetItem & curr_item = *i;
                const CharSetItem & next_item = *j;
                if ((curr_item.lo_codepoint + 2) != next_item.lo_codepoint) {
                    combine  = false;
                    break;
                }
            }
            if (combine) {
                CodePointType lo = cc->front().lo_codepoint;
                CodePointType hi = cc->back().lo_codepoint;
                const CodePointType mask = mEncoding.getMask();
                lo &= (mask - 1);
                hi |= (mask ^ (mask - 1));
                PabloE * expr = make_range(lo, hi);
                PabloE * bit0 = getBasisVar(0);
                if ((lo & 1) == 0) {
                    bit0 = mCG.createNot(bit0);
                }
                return mCG.createAnd(expr, bit0);
            }
        }
    }
    PabloE * expr = nullptr;
    for (const CharSetItem & item : *cc) {
        PabloE * temp = char_or_range_expr(item.lo_codepoint, item.hi_codepoint);
        expr = (expr == nullptr) ? temp : mCG.createOr(expr, temp);
    }
    return expr;
}

inline PabloE * CC_Compiler::char_or_range_expr(const CodePointType lo, const CodePointType hi) {
    if (lo == hi) {
        return char_test_expr(lo);
    }
    else if (lo < hi) {
        return make_range(lo, hi);
    }
    throw std::runtime_error(std::string("Invalid Character Set Range: [") + std::to_string(lo) + "," + std::to_string(hi) + "]");
}

} // end of namespace cc
