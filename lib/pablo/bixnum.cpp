/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <pablo/bixnum.h>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_ones.h>
#include <re/alphabet/alphabet.h>
#include <re/cc/cc_compiler.h>
#include <re/cc/cc_compiler_target.h>
#include <llvm/Support/raw_ostream.h>

using namespace cc;
using namespace re;

namespace pablo {

PabloAST * BixNumCompiler::UGE(BixNum value, unsigned floor) {
    if (floor == 0) return mPB.createOnes();
    return UGT(value, floor - 1);
}

PabloAST * BixNumCompiler::UGT(BixNum value, unsigned floor) {
    if (floor >> value.size() != 0) return mPB.createZeroes();
    PabloAST * UGT_so_far = mPB.createZeroes();
    for (unsigned i = 0; i < value.size(); i++) {
        auto floor_bit = (floor >> i) & 1;
        if (floor_bit == 1) {
            UGT_so_far = mPB.createAnd(value[i], UGT_so_far);
        } else {
            UGT_so_far = mPB.createOr(value[i], UGT_so_far);
        }
    }
    return UGT_so_far;
}

PabloAST * BixNumCompiler::ULE(BixNum value, unsigned floor) {
    return mPB.createNot(UGT(value, floor));
}

PabloAST * BixNumCompiler::ULT(BixNum value, unsigned floor) {
    return mPB.createNot(UGE(value, floor));
}

PabloAST * BixNumCompiler::UGE(BixNum value, BixNum floor) {
    PabloAST * UGE_so_far = mPB.createOnes();
    unsigned common_bits = std::min(value.size(), floor.size());
    for (unsigned i = 0; i < common_bits; i++) {
        UGE_so_far = mPB.createSel(floor[i], mPB.createAnd(value[i], UGE_so_far),
                                   mPB.createOr(value[i], UGE_so_far));
    }
    for (unsigned i = common_bits; i < floor.size(); i++) {
        UGE_so_far = mPB.createAnd(UGE_so_far, mPB.createNot(floor[i]));
    }
    for (unsigned i = common_bits; i < value.size(); i++) {
        UGE_so_far = mPB.createOr(UGE_so_far, value[i]);
    }
    return UGE_so_far;
}

PabloAST * BixNumCompiler::UGT(BixNum value, BixNum floor) {
    PabloAST * UGT_so_far = mPB.createZeroes();
    unsigned common_bits = std::min(value.size(), floor.size());
    for (unsigned i = 0; i < common_bits; i++) {
        UGT_so_far = mPB.createSel(floor[i], mPB.createAnd(value[i], UGT_so_far),
                                   mPB.createOr(value[i], UGT_so_far));
    }
    for (unsigned i = common_bits; i < floor.size(); i++) {
        UGT_so_far = mPB.createAnd(UGT_so_far, mPB.createNot(floor[i]));
    }
    for (unsigned i = common_bits; i < value.size(); i++) {
        UGT_so_far = mPB.createOr(UGT_so_far, value[i]);
    }
    return UGT_so_far;
}

PabloAST * BixNumCompiler::ULE(BixNum value, BixNum floor) {
    return mPB.createNot(UGT(value, floor));
}

PabloAST * BixNumCompiler::ULT(BixNum value, BixNum floor) {
    return mPB.createNot(UGE(value, floor));
}

PabloAST * BixNumCompiler::EQ(BixNum value, unsigned floor) {
    if (floor >> value.size() != 0) return mPB.createZeroes();
    PabloAST * EQ_1 = mPB.createOnes();
    PabloAST * EQ_0 = mPB.createZeroes();
    for (unsigned i = 0; i < value.size(); i++) {
        auto floor_bit = (floor >> i) & 1;
        if (floor_bit == 1) {
            EQ_1 = mPB.createAnd(value[i], EQ_1);
        } else {
            EQ_0 = mPB.createOr(value[i], EQ_0);
        }
    }
    return mPB.createAnd(EQ_1, mPB.createNot(EQ_0));
}

PabloAST * BixNumCompiler::NEQ(BixNum value, unsigned floor) {
    return mPB.createNot(EQ(value, floor));
}

PabloAST * BixNumCompiler::NEQ(BixNum value, BixNum test) {
    PabloAST * any_NEQ = mPB.createZeroes();
    unsigned common_bits = std::min(value.size(), test.size());
    for (unsigned i = 0; i < common_bits; i++) {
        any_NEQ = mPB.createOr(any_NEQ, mPB.createXor(value[i], test[i]));
    }
    for (unsigned i = common_bits; i < test.size(); i++) {
        any_NEQ = mPB.createOr(any_NEQ, test[i]);
    }
    for (unsigned i = common_bits; i < value.size(); i++) {
        any_NEQ = mPB.createOr(any_NEQ, value[i]);
    }
    return any_NEQ;
}

BixNum BixNumCompiler::ZeroExtend(BixNum value, unsigned extended_size) {
    assert(extended_size >= value.size());
    BixNum extended(extended_size);
    for (unsigned i = 0; i < value.size(); i++) {
        extended[i] = value[i];
    }
    for (unsigned i = value.size(); i < extended_size; i++) {
        extended[i] = mPB.createZeroes();
    }
    return extended;
}

BixNum BixNumCompiler::SignExtend(BixNum value, unsigned extended_size) {
    assert(extended_size >= value.size());
    BixNum extended(extended_size);
    for (unsigned i = 0; i < value.size(); i++) {
        extended[i] = value[i];
    }
    for (unsigned i = value.size(); i < extended_size; i++) {
        extended[i] = value.back();
    }
    return extended;
}

BixNum BixNumCompiler::Truncate(BixNum value, unsigned truncated_size) {
    assert(truncated_size <= value.size());
    BixNum truncated(truncated_size);
    for (unsigned i = 0; i < truncated_size; i++) {
        truncated[i] = value[i];
    }
    return truncated;
}

BixNum BixNumCompiler::HighBits(BixNum value, unsigned highBitCount) {
    assert(highBitCount <= value.size());
    unsigned offset = value.size() - highBitCount;
    BixNum extracted(highBitCount);
    for (unsigned i = 0; i < highBitCount; i++) {
        extracted[i] = value[i + offset];
    }
    return extracted;
}

PabloAST * BixNumCompiler::EQ(BixNum value, BixNum test) {
    return mPB.createNot(NEQ(value, test));
}

BixNum BixNumCompiler::AddModular(BixNum augend, unsigned addend) {
    addend = addend & ((1 << augend.size()) - 1);
    if (addend == 0) return augend;
    BixNum sum(augend.size());
    unsigned i = 0;
    while ((addend & (1 << i)) == 0) {
        sum[i] = augend[i];
        i++;
    }
    PabloAST * carry = mPB.createZeroes();
    while (i < augend.size()) {
        if ((addend & (1 << i)) == 0) {
            sum[i] = mPB.createXor(augend[i], carry);
            carry = mPB.createAnd(augend[i], carry);
        } else {
            sum[i] = mPB.createNot(mPB.createXor(augend[i], carry));
            carry = mPB.createOr(augend[i], carry);
        }
        i++;
    }
    return sum;
}

BixNum BixNumCompiler::AddModular(BixNum augend, BixNum addend) {
    BixNum sum(augend.size());
    PabloAST * carry = mPB.createZeroes();
    for (unsigned i = 0; i < augend.size(); i++) {
        if (i < addend.size()) {
            sum[i] = mPB.createXor3(augend[i], addend[i], carry);
            carry = mPB.createMajority3(augend[i], addend[i], carry);
        } else {
            sum[i] = mPB.createXor(augend[i], carry);
            carry = mPB.createAnd(augend[i], carry);
        }
    }
    return sum;
}

BixNum BixNumCompiler::SubModular(BixNum minuend, unsigned subtrahend) {
    unsigned complement = (~subtrahend) + 1;
    return AddModular(minuend, complement);
}

BixNum BixNumCompiler::SubModular(BixNum minuend, BixNum subtrahend) {
    BixNum diff(minuend.size());
    PabloAST * borrow = mPB.createZeroes();
    for (unsigned i = 0; i < minuend.size(); i++) {
        if (i < subtrahend.size()) {
            diff[i] = mPB.createXor3(minuend[i], subtrahend[i], borrow);
            borrow = mPB.createMajority3(mPB.createNot(minuend[i]), subtrahend[i], borrow);
        } else {
            diff[i] = mPB.createXor(minuend[i], borrow);
            borrow = mPB.createAnd(mPB.createNot(minuend[i]), borrow);
        }
    }
    return diff;
}

BixNum BixNumCompiler::MulModular(BixNum multiplicand, unsigned multiplier) {
    if (multiplier == 0) {
        return {mPB.createZeroes()};
    }
    unsigned multiplier_bits = std::log2(multiplier)+1;
    BixNum product(multiplicand.size(), mPB.createZeroes());
    for (unsigned i = 0; i < multiplier_bits; i++) {
        if ((multiplier & (1 << i)) != 0) {
            PabloAST * carry = mPB.createZeroes();
            for (unsigned j = 0; j + i < product.size(); j++) {
                PabloAST * tmp = mPB.createXor3(product[j + i], multiplicand[j], carry);
                carry = mPB.createMajority3(product[j + i], multiplicand[j], carry);
                product[j + i] = tmp;
            }
            product[multiplicand.size() + i] = carry;
        }
    }
    return product;
}

BixNum BixNumCompiler::AddFull(BixNum augend, BixNum addend) {
    if (augend.size() < addend.size()) return AddFull(addend, augend);
    BixNum sum(augend.size() + 1);
    PabloAST * carry = mPB.createZeroes();
    for (unsigned i = 0; i < addend.size(); i++) {
        sum[i] = mPB.createXor3(augend[i], addend[i], carry);
        carry = mPB.createMajority3(augend[i], addend[i], carry);
    }
    for (unsigned i = addend.size(); i < augend.size(); i++) {
        sum[i] = mPB.createXor(augend[i], carry);
        carry = mPB.createAnd(augend[i], carry);
    }
    sum[augend.size()] = carry;
    return sum;
}

BixNum BixNumCompiler::MulFull(BixNum multiplicand, unsigned multiplier) {
    unsigned multiplier_bits = std::log2(multiplier)+1;
    BixNum product(multiplicand.size() + multiplier_bits, mPB.createZeroes());
    // Choose between the addition-based and subtraction-based strategies based
    // on the number of 1 bits in the multiplier.
    if (__builtin_popcount(multiplier) <= multiplier_bits/2) {
        //  Perform an addition for every set bit in the multiplier.
        for (unsigned i = 0; i < multiplier_bits; i++) {
            if ((multiplier & (1 << i)) != 0) {
                PabloAST * carry = mPB.createZeroes();
                for (unsigned j = 0; j < multiplicand.size(); j++) {
                    PabloAST * tmp = mPB.createXor3(product[j + i], multiplicand[j], carry);
                    carry = mPB.createMajority3(product[j + i], multiplicand[j], carry);
                    product[j + i] = tmp;
                }
                product[multiplicand.size() + i] = carry;
            }
        }
    } else {
        unsigned complement = (~multiplier) + 1;
        for (unsigned i = 0; i < multiplier_bits; i++) {
            if ((complement & (1 << i)) != 0) {
                PabloAST * borrow = mPB.createZeroes();
                for (unsigned j = 0; j < multiplicand.size(); j++) {
                    PabloAST * tmp = mPB.createXor3(product[j + i], multiplicand[j], borrow);
                    borrow = mPB.createMajority3(mPB.createNot(product[j + i]), multiplicand[j], borrow);
                    product[j + i] = tmp;
                }
                for (unsigned j = multiplicand.size(); j < product.size()-i; j++) {
                    PabloAST * tmp = mPB.createXor(product[j + i], borrow);
                    borrow = mPB.createAnd(mPB.createNot(product[j + i]), borrow);
                    product[j + i] = tmp;
                }
            }
        }
        PabloAST * carry = mPB.createZeroes();
        for (unsigned j = 0; j < multiplicand.size(); j++) {
            PabloAST * tmp = mPB.createXor3(product[j + multiplier_bits], multiplicand[j], carry);
            carry = mPB.createMajority3(product[j + multiplier_bits], multiplicand[j], carry);
            product[j + multiplier_bits] = tmp;
        }
    }
    return product;
}

const unsigned CONSECUTIVE_SEQ_OPTIMIZATION_MINIMUM = 4;
unsigned BixNumTableCompiler::getTableVal(unsigned inputVal) {
    return mTable[inputVal];
}

unsigned BixNumTableCompiler::consecutiveFrom(unsigned inputVal) {
    unsigned consec = 1;
    unsigned lastVal = mTable[inputVal];
    unsigned nextVal = inputVal + 1;
    while ((nextVal <= mInputMax) && (mTable[nextVal] == lastVal + 1)) {
        consec++;
        lastVal = mTable[nextVal];
        nextVal++;
    }
    return consec;
}

unsigned BixNumTableCompiler::computeOutputBitsForRange(unsigned lo, unsigned hi) {
    unsigned OrAccum = mTable[lo];
    unsigned AndAccum = mTable[lo];
    for (unsigned i = lo+1; i <= hi; i++) {
        OrAccum |= mTable[i];  // zero bits will be zero for all lo..hi
        AndAccum &= mTable[i]; // one bits will be one for all lo..hi
    }
    return std::log2(OrAccum & ~AndAccum) + 1;
}

void BixNumTableCompiler::compileSubTable(PabloBuilder & pb, unsigned lo, PabloAST * subtableSelect) {
    tablePartitionLogic(pb, 0, lo, subtableSelect, mOutput.size());
}



void BixNumTableCompiler::innerLogic(PabloBuilder & pb,
                                          unsigned lo,
                                          PabloAST * subtableSelect,
                                          unsigned outputBitsToSet) {
    unsigned hi = std::min(mInputMax, lo + (1 << mPartitionBits.back()) - 1);
    assert (hi > lo);
    const unsigned bitsPerInputUnit = std::log2(hi-lo)+1;
    assert(mInput.size() >= bitsPerInputUnit);
    const unsigned xfrmBits = bitsPerInputUnit;
    std::vector<CC *> bitXfrmClasses;
    bitXfrmClasses.reserve(xfrmBits);
    for (unsigned i = 0; i < xfrmBits; i++) {
        bitXfrmClasses.push_back(makeCC(&cc::Byte));
    }
    std::vector<CC *> outputBitClasses;
    outputBitClasses.reserve(outputBitsToSet-xfrmBits);
    for (unsigned i = xfrmBits; i < outputBitsToSet; i++) {
        outputBitClasses.push_back(makeCC(&cc::Byte));
    }

    // Determine the longest consecutive run.
    int cur_offset = static_cast<int>(mTable[lo]);
    unsigned cur_seq_lgth = 1;
    int best_offset = cur_offset;
    unsigned max_seq_lgth = 1;
    for (unsigned i = lo+1; i <= hi; i++) {
        int offset = static_cast<int>(mTable[i]) - i;
        if (offset == cur_offset) {
            cur_seq_lgth++;
        } else {
            if (cur_seq_lgth > max_seq_lgth) {
                max_seq_lgth = cur_seq_lgth;
                best_offset = cur_offset;
            }
            cur_offset = offset;
            cur_seq_lgth = 1;
        }
    }
    if (max_seq_lgth < CONSECUTIVE_SEQ_OPTIMIZATION_MINIMUM) {
        best_offset = 0;  // no offsetting
    }
    for (unsigned ch_code = lo; ch_code <= hi; ch_code++) {
        unsigned transcodedCh = static_cast<unsigned>(static_cast<int>(mTable[ch_code]) - best_offset);
        unsigned changedBits = transcodedCh ^ ch_code;
        unsigned subTableIdx = ch_code % (1<<bitsPerInputUnit);
        for (unsigned i = 0; i < xfrmBits; i++) {
            unsigned bit = 1 << i;
            if ((changedBits & bit) == bit) {
                bitXfrmClasses[i]->insert(subTableIdx);
            }
        }
        for (unsigned i = xfrmBits; i < outputBitsToSet; i++) {
            unsigned bit = 1 << i;
            if ((transcodedCh & bit) == bit) {
                outputBitClasses[i-xfrmBits]->insert(subTableIdx);
            }
        }
    }
    BixNumCompiler bnc(pb);
    assert(bitsPerInputUnit <= 8);
    cc::Parabix_CC_Compiler_Builder inputUnitCompiler(pb.getPabloBlock(), bnc.ZeroExtend(bnc.Truncate(mInput, bitsPerInputUnit),8));
    BixNum output(outputBitsToSet, pb.createZeroes());
    for (unsigned i = 0; i < xfrmBits; i++) {
        PabloAST * xfrmStrm = inputUnitCompiler.compileCC(bitXfrmClasses[i]);
        output[i] = pb.createXor(xfrmStrm, mInput[i], "tbl_xfrm[" + std::to_string(i) + "]");
    }
    for (unsigned i = xfrmBits; i < outputBitsToSet; i++) {
        output[i] = inputUnitCompiler.compileCC(outputBitClasses[i - xfrmBits]);
    }
    if (max_seq_lgth >= CONSECUTIVE_SEQ_OPTIMIZATION_MINIMUM) {
        output = BixNumCompiler(pb).AddModular(output, static_cast<unsigned>(best_offset));
    }
    for (unsigned i = 0; i < outputBitsToSet; i++) {
        PabloAST * bit = pb.createAnd(subtableSelect, output[i], "tbl_bit[" + std::to_string(i) + "]");
        pb.createAssign(mOutput[i], pb.createOr(mOutput[i], bit));
    }
}

unsigned BixNumRangeTableCompiler::getTableVal(unsigned inputVal) {
    unsigned idx = getTableIndex(inputVal);
    return mRangeTable[idx].second + inputVal - mRangeTable[idx].first;
}

unsigned BixNumRangeTableCompiler::getTableIndex(unsigned inputVal) {
    unsigned loIdx = 0;
    unsigned hiIdx = mRangeTable.size() - 1;
    while (loIdx != hiIdx) {
        unsigned midIdx = (loIdx + hiIdx + 1)/2;
        if (inputVal < mRangeTable[midIdx].first) {
            hiIdx = midIdx-1;
        } else {
            loIdx = midIdx;
        }
    }
    return loIdx;
}

unsigned BixNumRangeTableCompiler::consecutiveFrom(unsigned inputVal) {
    unsigned idx = getTableIndex(inputVal);
    if ((idx == mRangeTable.size()-1) || (mRangeTable[idx+1].first >= mInputMax)) {
        return mInputMax - inputVal + 1;
    }
    return mRangeTable[idx+1].first - inputVal;
}

unsigned BixNumRangeTableCompiler::computeOutputBitsForRange(unsigned lo, unsigned hi) {
    return std::log2(getTableVal(lo) ^ getTableVal(hi)) + 1;
}

void BixNumRangeTableCompiler::compileTable(PabloBuilder & pb, PabloAST * partitionSelect) {
    tablePartitionLogic(pb, 0, 0, partitionSelect, mOutput.size());

}

void BixNumRangeTableCompiler::innerLogic(PabloBuilder & pb,
                                           unsigned partitionBase,
                                           PabloAST * partitionSelect,
                                           unsigned outputBitsToSet) {
    BixNumCompiler bnc(pb);
    PabloAST * GE_lo_bound = partitionSelect;
    unsigned tblIndex1 = getTableIndex(partitionBase);
    unsigned tblIndexLast = getTableIndex(partitionBase + (1 << mPartitionBits.back()) - 1);
    for (unsigned i = tblIndex1; i <= tblIndexLast; i++) {
        unsigned base = mRangeTable[i].first;
        unsigned cp = mRangeTable[i].second;
        unsigned offset = cp - base;
        PabloAST * GE_hi_bound;
        if (i < tblIndexLast) {
            GE_hi_bound = bnc.UGE(mInput, mRangeTable[i+1].first);
        } else {
            GE_hi_bound = pb.createNot(partitionSelect);
        }
        PabloAST * in_range = pb.createAnd(GE_lo_bound, pb.createNot(GE_hi_bound));
        BixNum mapped = bnc.AddModular(bnc.Truncate(mInput, outputBitsToSet), offset % (1 << outputBitsToSet));
        for (unsigned i = 0; i < outputBitsToSet; i++) {
            pb.createAssign(mOutput[i], pb.createOr(mOutput[i], pb.createAnd(in_range, mapped[i])));
        }
        GE_lo_bound = GE_hi_bound;
    }
}

void BixNumTableCompilerInterface::tablePartitionLogic(PabloBuilder & pb,
                                                   unsigned nestingDepth,
                                                   unsigned partitionBase,
                                                   PabloAST * partitionSelect,
                                                   unsigned outputBitsToSet) {
    if (nestingDepth == mPartitionBits.size() - 1) {
        innerLogic(pb, partitionBase, partitionSelect, outputBitsToSet);
        return;
    }

    BixNumCompiler bnc(pb);
    unsigned partitionBits = mPartitionBits[nestingDepth];
    unsigned partitionSize = 1 << partitionBits;
    // Upon entry, the obligation is to deal with a partition of the
    // overall table consisting of the partitionSize entries starting
    // with partitionBase.   The PabloAST expression partitionSelect,
    // is assumed to represent those positions that are properly
    // within the partition.

    // Partition into subPartitions...
    // The first subpartition starts at the beginning of the overall partition.
    unsigned subPartitionBase = partitionBase;
    unsigned subPartitionLimit = std::min(partitionBase + partitionSize, mInputMax + 1);
    unsigned subPartitionBits = mPartitionBits[nestingDepth+1];
    PabloAST * subPartitionLB_test = partitionSelect;
    while (subPartitionBase < subPartitionLimit) {
        //llvm::errs() << std::string(nestingDepth, ' ') << "subPartitionBase = " << subPartitionBase << "\n";
        unsigned subPartitionNo = subPartitionBase >> subPartitionBits;
        // Now determine the subpartition upper bound expression.  If the current table entry
        // entry spans one or more subpartitions, then we combine the subpartitions which
        // have a single offset calculation.   Otherwise, we advance only one subpartition.
        unsigned consecutiveLimit = std::min(subPartitionLimit - subPartitionBase, consecutiveFrom(subPartitionBase));
        unsigned consecSubPartitions = consecutiveLimit >> subPartitionBits;
        unsigned nextSubPartitionNo = subPartitionNo + (consecSubPartitions > 0 ? consecSubPartitions : 1);
        unsigned nextSubPartitionBase = nextSubPartitionNo << subPartitionBits;
        PabloAST * subpartitionUB_test = bnc.ULT(bnc.HighBits(mInput, mInput.size() - subPartitionBits), nextSubPartitionNo);
        PabloAST * inSubPartition = pb.createAnd(subPartitionLB_test, subpartitionUB_test, "tbl_partition_" + std::to_string(subPartitionBase) + "-" + std::to_string(nextSubPartitionBase));
        //
        // Determine which of the upper output bits are unchanging through the
        // entire subpartition, so that they can be set explicitly.
        //
        unsigned changeableBits = computeOutputBitsForRange(subPartitionBase, subPartitionLimit-1);
        unsigned cpBase = getTableVal(subPartitionBase);
        PabloBuilder nested = pb.createScope();
        pb.createIf(inSubPartition, nested);
        for (unsigned i = changeableBits; i < outputBitsToSet; i++) {
            if ((cpBase >> i) & 1) {
                nested.createAssign(mOutput[i], nested.createOr(mOutput[i], inSubPartition));
            }
        }
        //
        // If we have multiple table entries for a single subpartition, we make a
        // recursive call to consider further division into subsubpartitions.
        //
        if (consecSubPartitions == 0) {
            tablePartitionLogic(nested, nestingDepth+1, subPartitionBase, inSubPartition, changeableBits);
        } else {
            BixNumCompiler bnc(nested);
            // The current table entry spans one or more subpartitions.
            // Compute the mapped values.
            unsigned offset = getTableVal(subPartitionBase) - subPartitionBase;
            BixNum idxBasis;
            if (mInput.size() < changeableBits) idxBasis = bnc.ZeroExtend(mInput, changeableBits);
            else idxBasis = bnc.Truncate(mInput, changeableBits);
            BixNum mapped = bnc.AddModular(idxBasis, offset % (1 << changeableBits));
            for (unsigned i = 0; i < changeableBits; i++) {
                PabloAST * bit = nested.createAnd(inSubPartition, mapped[i], "tbl_bit[" + std::to_string(i) + "]");
                nested.createAssign(mOutput[i], nested.createOr(mOutput[i], bit));
            }
        }
        // Update for the next iteration and continue;
        subPartitionBase = nextSubPartitionBase;
        subPartitionLB_test = pb.createNot(subpartitionUB_test);
    }
}

}
