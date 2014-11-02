/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "cc_compiler.h"
#include "utf_encoding.h"

//Pablo Expressions
#include <pablo/codegenstate.h>
#include <pablo/pe_metadata.h>
#include <re/re_alt.h>
#include <re/re_cc.h>
#include <re/re_seq.h>
#include <re/re_rep.h>
#include <re/re_name.h>
#include <re/printer_re.h>
#include <cc/cc_namemap.hpp>

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

CC_Compiler::CC_Compiler(PabloBlock & cg, const Encoding encoding, const bool annotateVariableConstraints, const std::string basis_pattern)
: mCG(cg)
, mAnnotateVariableConstraints(annotateVariableConstraints)
, mBasisBit(encoding.getBits())
, mEncoding(encoding)
{
    for (int i = 0; i < mEncoding.getBits(); i++) {
        mBasisBit[i] = mCG.createVar(basis_pattern + std::to_string(i));
    }
}

std::vector<Var *> CC_Compiler::compile(const CC_NameMap & nameMap) {
    for (Name * name : nameMap) {
        compile_re(name);
    }
    return std::move(mBasisBit);
}

PabloAST * CC_Compiler::compile_re(RE * re) {
    if (isa<Name>(re)) {
        return compile_re(cast<Name>(re));
    }
    else if (isa<Alt>(re)) {
        return compile_re(cast<Alt>(re));
    }
    else if (isa<Seq>(re)) {
        return compile_re(cast<Seq>(re));
    }
    throw std::runtime_error("Unexpected RE node given to CC_Compiler: " + Printer_RE::PrintRE(re));
}

PabloAST * CC_Compiler::compile_re(Name * name) {
    assert(name);
    Var * var = name->getVar();
    if (var == nullptr) {
        if (name->getType() == Name::Type::FixedLength) {
            RE * cc = name->getCC();
            assert (cc);
            PabloAST * value = nullptr;
            if (isa<CC>(cc)) {
                value = charset_expr(cast<CC>(cc));
            }
            else if (isa<Seq>(cc)) {
                value = compile_re(cast<Seq>(cc));
            }
            else if (isa<Alt>(cc)) {
                value = compile_re(cast<Alt>(cc));
            }
            if (value == nullptr) {
                throw std::runtime_error("Unexpected CC node given to CC_Compiler: " + Printer_RE::PrintRE(name) + " : " + Printer_RE::PrintRE(cc));
            }
            var = mCG.createVar(mCG.createAssign(name->getName(), value));
        }
        else {
            var = mCG.createVar(name->getName());
        }
        name->setVar(var);
    }
    return var;
}

PabloAST * CC_Compiler::compile_re(const Seq * seq) {
    Assign * assignment = nullptr;
    PabloAST * result = nullptr;
    auto i = seq->begin();
    while (true) {
        PabloAST * cc = compile_re(*i);
        result = assignment ? mCG.createAnd(mCG.createVar(assignment), cc) : cc;
        if (++i == seq->end()) {
            break;
        }
        assignment = mCG.createAssign("seq", mCG.createAdvance(result, 1));
    }
    return result;
}

PabloAST * CC_Compiler::compile_re(const Alt *alt) {
    Assign * assignment = nullptr;
    PabloAST * result = nullptr;
    auto i = alt->begin();
    while (true) {
        PabloAST * cc = compile_re(*i);
        result = assignment ? mCG.createOr(mCG.createVar(assignment), cc) : cc;
        if (++i == alt->end()) {
            break;
        }
        assignment = mCG.createAssign("alt", result);
    }
    return result;
}


PabloAST * CC_Compiler::charset_expr(const CC * cc) {
    if (cc->empty()) {
        return mCG.createZeroes();
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
                PabloAST * expr = make_range(lo, hi);
                PabloAST * bit0 = getBasisVar(0);
                if ((lo & 1) == 0) {
                    bit0 = mCG.createNot(bit0);
                }
                return tempify(mCG.createAnd(expr, bit0));
            }
        }
    }
    PabloAST * expr = nullptr;
    for (const CharSetItem & item : *cc) {
        PabloAST * temp = char_or_range_expr(item.lo_codepoint, item.hi_codepoint);
        expr = (expr == nullptr) ? temp : tempify(mCG.createOr(expr, temp));
    }
    return expr;
}

PabloAST * CC_Compiler::bit_pattern_expr(const unsigned pattern, unsigned selected_bits)
{
    if (selected_bits == 0) {
        return mCG.createOnes();
    }

    std::vector<PabloAST*> bit_terms;
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
            bit_terms.push_back(mCG.createOnes());
        }
        selected_bits &= ~test_bit;
        i++;
    }

    //Reduce the list so that all of the expressions are contained within a single expression.
    while (bit_terms.size() > 1)
    {
        std::vector<PabloAST*> new_terms;
        for (auto i = 0; i < (bit_terms.size()/2); i++)
        {
            new_terms.push_back(tempify(mCG.createAnd(bit_terms[(2 * i) + 1], bit_terms[2 * i])));
        }
        if (bit_terms.size() % 2 == 1)
        {
            new_terms.push_back(bit_terms[bit_terms.size() -1]);
        }
        bit_terms.swap(new_terms);
    }
    return bit_terms[0];
}

inline PabloAST * CC_Compiler::char_test_expr(const CodePointType ch)
{
    return bit_pattern_expr(ch, mEncoding.getMask());
}

PabloAST * CC_Compiler::make_range(const CodePointType n1, const CodePointType n2)
{
    CodePointType diff_count = 0;

    for (CodePointType diff_bits = n1 ^ n2; diff_bits; diff_count++, diff_bits >>= 1);

    if ((n2 < n1) || (diff_count > mEncoding.getBits()))
    {
        throw std::runtime_error(std::string("Bad Range: [") + std::to_string(n1) + "," + std::to_string(n2) + "]");
    }

    const CodePointType mask0 = (static_cast<CodePointType>(1) << diff_count) - 1;

    PabloAST * common = bit_pattern_expr(n1 & ~mask0, mEncoding.getMask() ^ mask0);

    if (diff_count == 0) return common;

    const CodePointType mask1 = (static_cast<CodePointType>(1) << (diff_count - 1)) - 1;

    PabloAST* lo_test = GE_Range(diff_count - 1, n1 & mask1);
    PabloAST* hi_test = LE_Range(diff_count - 1, n2 & mask1);

    return tempify(mCG.createAnd(common, mCG.createSel(getBasisVar(diff_count - 1), hi_test, lo_test)));
}

PabloAST * CC_Compiler::GE_Range(const unsigned N, const unsigned n) {
    if (N == 0) {
        return mCG.createOnes(); //Return a true literal.
    }
    else if (((N % 2) == 0) && ((n >> (N - 2)) == 0)) {
        return tempify(mCG.createOr(tempify(mCG.createOr(getBasisVar(N - 1), getBasisVar(N - 2))), GE_Range(N - 2, n)));
    }
    else if (((N % 2) == 0) && ((n >> (N - 2)) == 3)) {
        return tempify(mCG.createAnd(tempify(mCG.createAnd(getBasisVar(N - 1), getBasisVar(N - 2))), GE_Range(N - 2, n - (3 << (N - 2)))));
    }
    else if (N >= 1)
    {
        int hi_bit = n & (1 << (N - 1));
        int lo_bits = n - hi_bit;
        PabloAST * lo_range = GE_Range(N - 1, lo_bits);
        if (hi_bit == 0)
        {
            /*
              If the hi_bit of n is not set, then whenever the corresponding bit
              is set in the target, the target will certaily be >=.  Oterwise,
              the value of GE_range(N-1), lo_range) is required.
            */
            return tempify(mCG.createOr(getBasisVar(N - 1), lo_range));
        }
        else
        {
            /*
              If the hi_bit of n is set, then the corresponding bit must be set
              in the target for >= and GE_range(N-1, lo_bits) must also be true.
            */
            return tempify(mCG.createAnd(getBasisVar(N - 1), lo_range));
        }
    }
    throw std::runtime_error("Unexpected input given to ge_range: " + std::to_string(N) + ", " + std::to_string(n));
}

PabloAST * CC_Compiler::LE_Range(const unsigned N, const unsigned n)
{
    /*
      If an N-bit pattern is all ones, then it is always true that any n-bit value is LE this pattern.
      Handling this as a special case avoids an overflow issue with n+1 requiring more than N bits.
    */
    if ((n + 1) == (1 << N)) {
        return mCG.createOnes(); //True.
    }
    else {
        return tempify(mCG.createNot(GE_Range(N, n + 1)));
    }
}

inline PabloAST * CC_Compiler::char_or_range_expr(const CodePointType lo, const CodePointType hi) {
    if (lo == hi) {
        return char_test_expr(lo);
    }
    else if (lo < hi) {
        return make_range(lo, hi);
    }
    throw std::runtime_error(std::string("Invalid Character Set Range: [") + std::to_string(lo) + "," + std::to_string(hi) + "]");
}

inline Var * CC_Compiler::getBasisVar(const int i) const {
    return mBasisBit[(mEncoding.getBits() - 1) - i];
}

inline PabloAST * CC_Compiler::tempify(PabloAST * value) {
//    if (isa<Var>(value)) {
//        return cast<Var>(value);
//    }
//    return mCG.createVar(mCG.createAssign("t", value));
    return value;
}

} // end of namespace cc
