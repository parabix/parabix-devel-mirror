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

