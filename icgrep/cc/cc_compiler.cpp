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
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/printer_re.h>
#include <cc/cc_namemap.hpp>
#include <pablo/printer_pablos.h>
#include <utility>
#include <string>
#include <list>
#include <map>
#include <algorithm>
#include <boost/graph/adjacency_list.hpp>
#include <cassert>
#include <stdlib.h>
#include <stdexcept>

using namespace re;
using namespace pablo;
using namespace boost;

namespace cc {

CC_Compiler::CC_Compiler(PabloBlock & cg, const Encoding encoding, const bool annotateVariableConstraints, const std::string basis_pattern)
: mCG(cg)
//, mAnnotateVariableConstraints(annotateVariableConstraints)
, mBasisBit(encoding.getBits())
, mEncoding(encoding)
{
    for (int i = 0; i < mEncoding.getBits(); i++) {
        mBasisBit[i] = mCG.createVar(basis_pattern + std::to_string(i));
    }
}

pablo::Var * CC_Compiler::compileCC(const re::CC *cc) { 
     return compileCC(cc, mCG); 
}

pablo::Var * CC_Compiler::compileCC(const re::CC *cc, pablo::PabloBlock & pb) { 
     return pb.createVar(pb.createAssign(cc->canonicalName(ByteClass), charset_expr(cc, pb))); 
}

std::vector<Var *> CC_Compiler::getBasisBits(const CC_NameMap & nameMap) {
    return mBasisBit;
}

void CC_Compiler::compileByteClasses(RE * re) {
    if (Alt * alt = dyn_cast<Alt>(re)) {
        for (auto i = alt->begin(); i != alt->end(); ++i) {
            compileByteClasses(*i);
        }
    }
    else if (Seq * seq = dyn_cast<Seq>(re)) {
        for (auto i = seq->begin(); i != seq->end(); ++i) {
            compileByteClasses(*i);
        }
    }
    else if (Rep * rep = dyn_cast<Rep>(re)) {
        compileByteClasses(rep->getRE());
    }
    else if (Diff * diff = dyn_cast<Diff>(re)) {
        compileByteClasses(diff->getRH());
        compileByteClasses(diff->getLH());
    }
    else if (Intersect * e = dyn_cast<Intersect>(re)) {
        compileByteClasses(e->getRH());
        compileByteClasses(e->getLH());
    }
    else if (Name * name = dyn_cast<Name>(re)) {
        RE * d = name->getDefinition();
        if (d && !isa<CC>(d)) {
            compileByteClasses(d);
        }
        else if (d && isa<CC>(d)) {
	    name->setCompiled(compileCC(cast<CC>(d), mCG));
	}
    }
    else if (isa<CC>(re)) {
        std::cerr << "Shouldn't get here\n";
	exit(-1);
    }
}



PabloAST * CC_Compiler::charset_expr(const CC * cc, pablo::PabloBlock & pb) {
    if (cc->empty()) {
        return pb.createZeroes();
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
    for (const CharSetItem & item : *cc) {
        PabloAST * temp = char_or_range_expr(item.lo_codepoint, item.hi_codepoint, pb);
        expr = (expr == nullptr) ? temp : pb.createOr(expr, temp);
    }
    return expr;
}

PabloAST * CC_Compiler::bit_pattern_expr(const unsigned pattern, unsigned selected_bits, pablo::PabloBlock & pb)
{
    if (selected_bits == 0) {
        return pb.createOnes();
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
            bit_terms.push_back(pb.createOnes());
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
            new_terms.push_back(pb.createAnd(bit_terms[(2 * i) + 1], bit_terms[2 * i]));
        }
        if (bit_terms.size() % 2 == 1)
        {
            new_terms.push_back(bit_terms[bit_terms.size() -1]);
        }
        bit_terms.swap(new_terms);
    }
    return bit_terms[0];
}

inline PabloAST * CC_Compiler::char_test_expr(const CodePointType ch, pablo::PabloBlock & pb)
{
    return bit_pattern_expr(ch, mEncoding.getMask(), pb);
}

PabloAST * CC_Compiler::make_range(const CodePointType n1, const CodePointType n2, pablo::PabloBlock & pb)
{
    CodePointType diff_count = 0;

    for (CodePointType diff_bits = n1 ^ n2; diff_bits; diff_count++, diff_bits >>= 1);

    if ((n2 < n1) || (diff_count > mEncoding.getBits()))
    {
        throw std::runtime_error("Bad Range: [" + std::to_string(n1) + "," + std::to_string(n2) + "]");
    }

    const CodePointType mask0 = (static_cast<CodePointType>(1) << diff_count) - 1;

    PabloAST * common = bit_pattern_expr(n1 & ~mask0, mEncoding.getMask() ^ mask0, pb);

    if (diff_count == 0) return common;

    const CodePointType mask1 = (static_cast<CodePointType>(1) << (diff_count - 1)) - 1;

    PabloAST* lo_test = GE_Range(diff_count - 1, n1 & mask1, pb);
    PabloAST* hi_test = LE_Range(diff_count - 1, n2 & mask1, pb);

    return pb.createAnd(common, pb.createSel(getBasisVar(diff_count - 1), hi_test, lo_test));
}

PabloAST * CC_Compiler::GE_Range(const unsigned N, const unsigned n, pablo::PabloBlock & pb) {
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
              is set in the target, the target will certaily be >=.  Oterwise,
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

PabloAST * CC_Compiler::LE_Range(const unsigned N, const unsigned n, pablo::PabloBlock & pb)
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

inline PabloAST * CC_Compiler::char_or_range_expr(const CodePointType lo, const CodePointType hi, pablo::PabloBlock & pb) {
    if (lo == hi) {
        return char_test_expr(lo, pb);
    }
    else if (lo < hi) {
        return make_range(lo, hi, pb);
    }
    throw std::runtime_error(std::string("Invalid Character Set Range: [") + std::to_string(lo) + "," + std::to_string(hi) + "]");
}

inline Var * CC_Compiler::getBasisVar(const int i) const {
    return mBasisBit[(mEncoding.getBits() - 1) - i];
}

void CC_Compiler::computeVariableConstraints() {
    typedef adjacency_list<vecS, vecS, directedS> ConstraintGraph;
    typedef adjacency_list<vecS, vecS, directedS> SubsetGraph;

    typedef graph_traits<ConstraintGraph>::out_edge_iterator ConstraintEdgeIterator;
    typedef graph_traits<SubsetGraph>::out_edge_iterator SubsetEdgeIterator;

    const auto n = mVariableVector.size();

    if (n == 0) {
        return;
    }

    // Compute the constraint and subset graphs.
    ConstraintGraph C(n);
    SubsetGraph S(n);

    for (auto i = 0; i != n; ++i) {
        const CC * const cc1 = mVariableVector[i].first;

        for (auto j = i + 1; j != n; ++j) {
            const CC * const cc2 = mVariableVector[j].first;
            switch(cc1->compare(cc2)) {
                case CC::SetRelationship::OVERLAPPING:
                    add_edge(i, j, C);
                    add_edge(j, i, C);
                    break;
                case CC::SetRelationship::SUBSET:
                    add_edge(i, j, S);
                    break;
                case CC::SetRelationship::SUPERSET:
                    add_edge(j, i, S);
                    break;
                default:
                    /* do nothing; here just to prevent warning */
                    break;
            }
        }
    }

    // Write out the constraints and subset relationships as metadata
    for (auto i = 0; i != n; ++i) {
        std::vector<PabloAST *> constraints;
        ConstraintEdgeIterator ci, ci_end;
        for (std::tie(ci, ci_end) = out_edges(i, C); ci != ci_end; ++ci) {
            constraints.push_back(mVariableVector[target(*ci, C)].second);
        }

        std::vector<PabloAST *> subsets;
        SubsetEdgeIterator si, si_end;
        for (std::tie(si, si_end) = out_edges(i, S); si != si_end; ++si) {
            subsets.push_back(mVariableVector[target(*si, S)].second);
        }

        Assign * assign = mVariableVector[i].second;
        if (!constraints.empty()) {
            assign->setMetadata("constraints", PMDSet::get(constraints.begin(), constraints.end()));
        }
        if (!subsets.empty()) {
            assign->setMetadata("subsets", PMDSet::get(subsets.begin(), subsets.end()));
        }
    }
}

} // end of namespace cc
