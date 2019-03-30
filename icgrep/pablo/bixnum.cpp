/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <pablo/bixnum.h>
#include <pablo/pe_zeroes.h>


namespace pablo {

PabloAST * createXor3(PabloBuilder & pb, PabloAST * op1, PabloAST * op2, PabloAST * op3) {
    return pb.createXor(op1, pb.createXor(op2, op3));
}
    
 PabloAST * createMajority3(PabloBuilder & pb, PabloAST * op1, PabloAST * op2, PabloAST * op3) {
    return pb.createOr(pb.createAnd(op1, op2), pb.createAnd(op3, pb.createOr(op1, op2)));
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
