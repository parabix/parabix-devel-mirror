/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <pablo/bixnum.h>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_ones.h>


namespace pablo {

PabloAST * createXor3(PabloBuilder & pb, PabloAST * op1, PabloAST * op2, PabloAST * op3) {
    return pb.createXor(op1, pb.createXor(op2, op3));
}

PabloAST * createMajority3(PabloBuilder & pb, PabloAST * op1, PabloAST * op2, PabloAST * op3) {
    return pb.createOr(pb.createAnd(op1, op2), pb.createAnd(op3, pb.createOr(op1, op2)));
}

PabloAST * createIf3(PabloBuilder & pb, PabloAST * op1, PabloAST * op2, PabloAST * op3) {
    return pb.createOr(pb.createAnd(op1, op2), pb.createAnd(op3, pb.createNot(op1)));
}

PabloAST * BixNumArithmetic::UGE(BixNum value, unsigned floor) {
    unsigned floor_bits = std::log2(floor);
    if (value.size() < floor_bits) return mPB.createZeroes();
    PabloAST * UGE_so_far = mPB.createOnes();
    for (auto i = 0; i < floor_bits; i++) {
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
    for (auto i = 0; i < common_bits; i++) {
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
    unsigned floor_bits = std::log2(floor);
    if (value.size() < floor_bits) return mPB.createZeroes();
    PabloAST * EQ_1 = mPB.createOnes();
    PabloAST * EQ_0 = mPB.createZeroes();
    for (auto i = 0; i < floor_bits; i++) {
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
    for (auto i = 0; i < common_bits; i++) {
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
    unsigned multiplier_bits = std::log2(multiplier);
    BixNum product(multiplicand.size(), mPB.createZeroes());
    for (unsigned i = 0; i < multiplier_bits; i++) {
        PabloAST * carry = mPB.createZeroes();
        if ((multiplier & (1 << i)) != 0) {
            for (unsigned j = 0; j + i < product.size(); j++) {
                product[j + i] = createXor3(mPB, product[j + i], multiplicand[i], carry);
                carry = createMajority3(mPB, product[j + i], multiplicand[i], carry);
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
    unsigned multiplier_bits = std::log2(multiplier);
    BixNum product(multiplicand.size() + multiplier_bits, mPB.createZeroes());
    // Choose between the addition-based and subtraction-based strategies based
    // on the number of 1 bits in the multiplier.
    if (__builtin_popcount(multiplier) <= multiplier_bits/2) {
        //  Perform an addition for every set bit in the multiplier.
        for (unsigned i = 0; i < multiplier_bits; i++) {
            PabloAST * carry = mPB.createZeroes();
            if ((multiplier & (1 << i)) != 0) {
                for (unsigned j = 0; j < multiplicand.size(); j++) {
                    product[j + i] = createXor3(mPB, product[j + i], multiplicand[i], carry);
                    carry = createMajority3(mPB, product[j + i], multiplicand[i], carry);
                }
                product[multiplicand.size() + i] = carry;
            }
        }
    } else {
        unsigned complement = (~multiplier) + 1;
        for (unsigned i = 0; i < multiplier_bits; i++) {
            PabloAST * borrow = mPB.createZeroes();
            if ((complement & (1 << i)) != 0) {
                for (unsigned j = 0; j < multiplicand.size(); j++) {
                    product[j + i] = createXor3(mPB, product[j + i], multiplicand[i], borrow);
                    borrow = createMajority3(mPB, mPB.createNot(product[j + i]), multiplicand[i], borrow);
                }
                product[multiplicand.size() + i] = borrow;
            }
        }
    }
    return product;
}

}
