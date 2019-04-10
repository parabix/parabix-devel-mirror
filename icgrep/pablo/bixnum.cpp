/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <pablo/bixnum.h>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_ones.h>
#include <cc/alphabet.h>
#include <cc/cc_compiler.h>
#include <cc/cc_compiler_target.h>

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
    if (addend == 0) return augend;
    BixNum sum(augend.size());
    addend = addend & ((1 << augend.size()) - 1);
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

BixNum BixNumTableCompiler::compileSubTable(PabloBuilder & pb, unsigned lo, unsigned hi) {
    assert (hi > lo);
    const unsigned bitsPerInputUnit = std::log2(hi-lo)+1;
    assert(mInput.size() >= bitsPerInputUnit);
    std::vector<CC *> bitXfrmClasses;
    bitXfrmClasses.reserve(bitsPerInputUnit);
    for (unsigned i = 0; i < bitsPerInputUnit; i++) {
        bitXfrmClasses.push_back(makeCC(&cc::Byte));
    }
    std::vector<CC *> outputBitClasses;
    outputBitClasses.reserve(mBitsPerOutputUnit-bitsPerInputUnit);
    for (unsigned i = bitsPerInputUnit; i < mBitsPerOutputUnit; i++) {
        outputBitClasses.push_back(makeCC(&cc::Byte));
    }

    // Determine the longest consecutive run.
    int cur_offset = static_cast<int>(mTable[lo]);
    unsigned cur_seq_lgth = 1;
    int best_offset = cur_offset;
    unsigned max_seq_lgth = 1;
    for (int i = lo+1; i <= hi; i++) {
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
        codepoint_t transcodedCh = static_cast<codepoint_t>(static_cast<int>(mTable[ch_code]) - best_offset);
        codepoint_t changedBits = transcodedCh ^ ch_code;
        unsigned subTableIdx = ch_code % (1<<bitsPerInputUnit);
        for (unsigned i = 0; i < bitsPerInputUnit; i++) {
            unsigned bit = 1 << i;
            if ((changedBits & bit) == bit) {
                bitXfrmClasses[i]->insert(subTableIdx);
            }
        }
        for (unsigned i = bitsPerInputUnit; i < mBitsPerOutputUnit; i++) {
            unsigned bit = 1 << i;
            if ((transcodedCh & bit) == bit) {
                outputBitClasses[i-bitsPerInputUnit]->insert(subTableIdx);
            }
        }
    }
    cc::Parabix_CC_Compiler_Builder inputUnitCompiler(pb.getPabloBlock(), mInput);
    BixNum output(mBitsPerOutputUnit, pb.createZeroes());
    for (unsigned i = 0; i < bitsPerInputUnit; i++) {
        PabloAST * xfrmStrm = inputUnitCompiler.compileCC(bitXfrmClasses[i]);
        output[i] = pb.createXor(xfrmStrm, mInput[i]);
    }
    for (unsigned i = bitsPerInputUnit; i < mBitsPerOutputUnit; i++) {
        output[i] = inputUnitCompiler.compileCC(outputBitClasses[i - bitsPerInputUnit]);
    }
    if (max_seq_lgth >= CONSECUTIVE_SEQ_OPTIMIZATION_MINIMUM) {
        output = BixNumCompiler(pb).AddModular(output, static_cast<unsigned>(best_offset));  // no offsetting
    }
    return output;
}


}
