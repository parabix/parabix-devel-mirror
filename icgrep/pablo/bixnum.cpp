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

PabloAST * createXor3(PabloBuilder & pb, PabloAST * op1, PabloAST * op2, PabloAST * op3) {
    return pb.createTernary(0x96, op1, op2, op3);
}

PabloAST * createMajority3(PabloBuilder & pb, PabloAST * op1, PabloAST * op2, PabloAST * op3) {
    return pb.createTernary(0xE8, op1, op2, op3);
}

PabloAST * createIf3(PabloBuilder & pb, PabloAST * op1, PabloAST * op2, PabloAST * op3) {
    return pb.createTernary(0xCA, op1, op2, op3);
}

PabloAST * BixNumArithmetic::UGE(BixNum value, unsigned floor) {
    unsigned floor_bits = std::log2(floor)+1;
    if (value.size() < floor_bits) return mPB.createZeroes();
    PabloAST * UGE_so_far = mPB.createOnes();
    for (unsigned i = 0; i < floor_bits; i++) {
        auto floor_bit = (floor >> i) & 1;
        if (floor_bit == 1) {
            UGE_so_far = mPB.createAnd(value[i], UGE_so_far);
        } else {
            UGE_so_far = mPB.createOr(value[i], UGE_so_far);
        }
    }
    for (unsigned i = floor_bits; i < value.size(); i++) {
        UGE_so_far = mPB.createOr(value[i], UGE_so_far);
    }
    return UGE_so_far;
}

PabloAST * BixNumArithmetic::UGE(BixNum value, BixNum floor) {
    PabloAST * UGE_so_far = mPB.createOnes();
    unsigned common_bits = std::min(value.size(), floor.size());
    for (unsigned i = 0; i < common_bits; i++) {
        UGE_so_far = createIf3(mPB, floor[i], mPB.createAnd(value[i], UGE_so_far),
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

PabloAST * BixNumArithmetic::EQ(BixNum value, unsigned floor) {
    unsigned floor_bits = std::log2(floor)+1;
    if (value.size() < floor_bits) return mPB.createZeroes();
    PabloAST * EQ_1 = mPB.createOnes();
    PabloAST * EQ_0 = mPB.createZeroes();
    for (unsigned i = 0; i < floor_bits; i++) {
        auto floor_bit = (floor >> i) & 1;
        if (floor_bit == 1) {
            EQ_1 = mPB.createAnd(value[i], EQ_1);
        } else {
            EQ_0 = mPB.createOr(value[i], EQ_0);
        }
    }
    for (unsigned i = floor_bits; i < value.size(); i++) {
        EQ_0 = mPB.createOr(value[i], EQ_0);
    }
    return mPB.createAnd(EQ_1, mPB.createNot(EQ_0));
}

PabloAST * BixNumArithmetic::NEQ(BixNum value, unsigned floor) {
    return mPB.createNot(EQ(value, floor));
}

PabloAST * BixNumArithmetic::NEQ(BixNum value, BixNum test) {
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

BixNum BixNumArithmetic::ZeroExtend(BixNum value, unsigned extended_size) {
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

BixNum BixNumArithmetic::SignExtend(BixNum value, unsigned extended_size) {
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

BixNum BixNumArithmetic::Truncate(BixNum value, unsigned truncated_size) {
    assert(truncated_size <= value.size());
    BixNum truncated(truncated_size);
    for (unsigned i = 0; i < truncated_size; i++) {
        truncated[i] = value[i];
    }
    return truncated;
}

BixNum BixNumArithmetic::HighBits(BixNum value, unsigned highBitCount) {
    assert(highBitCount <= value.size());
    unsigned offset = value.size() - highBitCount;
    BixNum extracted(highBitCount);
    for (unsigned i = 0; i < highBitCount; i++) {
        extracted[i] = value[i + offset];
    }
    return extracted;
}

PabloAST * BixNumArithmetic::EQ(BixNum value, BixNum test) {
    return mPB.createNot(NEQ(value, test));
}

BixNum BixNumModularArithmetic::Add(BixNum augend, unsigned addend) {
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

BixNum BixNumModularArithmetic::Add(BixNum augend, BixNum addend) {
    BixNum sum(augend.size());
    PabloAST * carry = mPB.createZeroes();
    for (unsigned i = 0; i < augend.size(); i++) {
        if (i < addend.size()) {
            sum[i] = createXor3(mPB, augend[i], addend[i], carry);
            carry = createMajority3(mPB, augend[i], addend[i], carry);
        } else {
            sum[i] = mPB.createXor(augend[i], carry);
            carry = mPB.createAnd(augend[i], carry);
        }
    }
    return sum;
}

BixNum BixNumModularArithmetic::Sub(BixNum minuend, unsigned subtrahend) {
    unsigned complement = (~subtrahend) + 1;
    return Add(minuend, complement);
}

BixNum BixNumModularArithmetic::Sub(BixNum minuend, BixNum subtrahend) {
    BixNum diff(minuend.size());
    PabloAST * borrow = mPB.createZeroes();
    for (unsigned i = 0; i < minuend.size(); i++) {
        if (i < subtrahend.size()) {
            diff[i] = createXor3(mPB, minuend[i], subtrahend[i], borrow);
            borrow = createMajority3(mPB, mPB.createNot(minuend[i]), subtrahend[i], borrow);
        } else {
            diff[i] = mPB.createXor(minuend[i], borrow);
            borrow = mPB.createAnd(mPB.createNot(minuend[i]), borrow);
        }
    }
    return diff;
}

BixNum BixNumModularArithmetic::Mul(BixNum multiplicand, unsigned multiplier) {
    unsigned multiplier_bits = std::log2(multiplier)+1;
    BixNum product(multiplicand.size(), mPB.createZeroes());
    for (unsigned i = 0; i < multiplier_bits; i++) {
        if ((multiplier & (1 << i)) != 0) {
            PabloAST * carry = mPB.createZeroes();
            for (unsigned j = 0; j + i < product.size(); j++) {
                PabloAST * tmp = createXor3(mPB, product[j + i], multiplicand[j], carry);
                carry = createMajority3(mPB, product[j + i], multiplicand[j], carry);
                product[j + i] = tmp;
            }
            product[multiplicand.size() + i] = carry;
        }
    }
    return product;
}

BixNum BixNumFullArithmetic::Add(BixNum augend, BixNum addend) {
    if (augend.size() < addend.size()) return Add(addend, augend);
    BixNum sum(augend.size() + 1);
    PabloAST * carry = mPB.createZeroes();
    for (unsigned i = 0; i < addend.size(); i++) {
        sum[i] = createXor3(mPB, augend[i], addend[i], carry);
        carry = createMajority3(mPB, augend[i], addend[i], carry);
    }
    for (unsigned i = addend.size(); i < augend.size(); i++) {
        sum[i] = mPB.createXor(augend[i], carry);
        carry = mPB.createAnd(augend[i], carry);
    }
    sum[augend.size()] = carry;
    return sum;
}

BixNum BixNumFullArithmetic::Mul(BixNum multiplicand, unsigned multiplier) {
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
                    PabloAST * tmp = createXor3(mPB, product[j + i], multiplicand[j], carry);
                    carry = createMajority3(mPB, product[j + i], multiplicand[j], carry);
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
                    PabloAST * tmp  = createXor3(mPB, product[j + i], multiplicand[j], borrow);
                    borrow = createMajority3(mPB, mPB.createNot(product[j + i]), multiplicand[j], borrow);
                    product[j + i] = tmp;
                }
                for (unsigned j = multiplicand.size(); j < product.size()-i; j++) {
                    PabloAST * tmp  = mPB.createXor(product[j + i], borrow);
                    borrow = mPB.createAnd(mPB.createNot(product[j + i]), borrow);
                    product[j + i] = tmp;
                }
            }
        }
        PabloAST * carry = mPB.createZeroes();
        for (unsigned j = 0; j < multiplicand.size(); j++) {
            PabloAST * tmp = createXor3(mPB, product[j + multiplier_bits], multiplicand[j], carry);
            carry = createMajority3(mPB, product[j + multiplier_bits], multiplicand[j], carry);
            product[j + multiplier_bits] = tmp;
        }
    }
    return product;
}

BixNum BixNumTableCompiler::compileSubTableLookup(unsigned lo, unsigned hi, unsigned bitsPerOutputUnit, BixNum input) {
    assert(hi-lo > 0);
    const unsigned bitsPerInputUnit = std::log2(hi-lo)+1;
    assert(input.size() >= bitsPerInputUnit);
    std::vector<CC *> bitXfrmClasses;
    bitXfrmClasses.reserve(bitsPerInputUnit);
    for (unsigned i = 0; i < bitsPerInputUnit; i++) {
        bitXfrmClasses.push_back(makeCC(&cc::Byte));
    }
    std::vector<CC *> outputBitClasses;
    outputBitClasses.reserve(bitsPerOutputUnit-bitsPerInputUnit);
    for (unsigned i = bitsPerInputUnit; i < bitsPerOutputUnit; i++) {
        outputBitClasses.push_back(makeCC(&cc::Byte));
    }
    for (unsigned ch_code = lo; ch_code <= hi; ch_code++) {
        codepoint_t transcodedCh = mTable[ch_code];
        codepoint_t changedBits = transcodedCh ^ ch_code;
        for (unsigned i = 0; i < bitsPerInputUnit; i++) {
            unsigned bit = 1 << i;
            if ((changedBits & bit) == bit) {
                bitXfrmClasses[i]->insert(ch_code);
            }
        }
        for (unsigned i = bitsPerInputUnit; i < bitsPerOutputUnit; i++) {
            unsigned bit = 1 << i;
            if ((transcodedCh & bit) == bit) {
                outputBitClasses[i-bitsPerInputUnit]->insert(ch_code);
            }
        }
    }
    cc::Parabix_CC_Compiler_Builder inputUnitCompiler(mPB.getPabloBlock(), input);
    BixNum output;
    output.reserve(bitsPerOutputUnit);
    for (unsigned i = 0; i < bitsPerInputUnit; i++) {
        PabloAST * xfrmStrm = inputUnitCompiler.compileCC(bitXfrmClasses[i]);
        output[i] = mPB.createXor(xfrmStrm, input[i]);
    }
    for (unsigned i = bitsPerInputUnit; i < bitsPerOutputUnit; i++) {
        output[i] = inputUnitCompiler.compileCC(outputBitClasses[i - bitsPerInputUnit]);
    }
    return output;
}


}
