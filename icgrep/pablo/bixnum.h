/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef PARABIX_ARITHMETIC_COMPILER_H
#define PARABIX_ARITHMETIC_COMPILER_H

#include <pablo/pabloAST.h>
#include <pablo/builder.hpp>

namespace pablo {

using BixNum = std::vector<PabloAST *>;
    
    
class BixNumArithmetic {
public:
    BixNumArithmetic(PabloBuilder & pb) : mPB(pb) {}
    PabloAST * EQ(BixNum value, unsigned test);
    PabloAST * EQ(BixNum value, BixNum test);
    PabloAST * NEQ(BixNum value, unsigned test);
    PabloAST * NEQ(BixNum value, BixNum test);
    PabloAST * UGE(BixNum value, unsigned floor);
    PabloAST * UGE(BixNum value, BixNum floor);
    BixNum ZeroExtend(BixNum value, unsigned extended_size);
    BixNum SignExtend(BixNum value, unsigned extended_size);
    BixNum Truncate(BixNum value, unsigned truncated_size);


private:
    PabloBuilder & mPB;
};

class BixNumModularArithmetic {
public:    
    BixNumModularArithmetic(PabloBuilder & pb) : mPB(pb) {}
    BixNum Add(BixNum augend, unsigned addend);
    BixNum Add(BixNum augend, BixNum addend);
    BixNum Sub(BixNum minuend, unsigned subtrahend);
    BixNum Sub(BixNum minuend, BixNum subtrahend);
    BixNum Mul(BixNum multiplicand, unsigned multiplier);
private:
    PabloBuilder & mPB;
};

class BixNumFullArithmetic {
public:
    BixNumFullArithmetic(PabloBuilder & pb) : mPB(pb) {}
    
    BixNum Add(BixNum augend, unsigned addend);
    BixNum Add(BixNum augend, BixNum addend);
    BixNum Mul(BixNum multiplicand, unsigned multiplier);
private:
    PabloBuilder & mPB;
};

}

#endif

