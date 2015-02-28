#ifndef ICGREP_H
#define ICGREP_H

/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "include/simd-lib/bitblock.hpp"
#include "include/simd-lib/carryQ.hpp"
#include "include/simd-lib/pabloSupport.hpp"

struct Basis_bits {
    BitBlock bit_0;
    BitBlock bit_1;
    BitBlock bit_2;
    BitBlock bit_3;
    BitBlock bit_4;
    BitBlock bit_5;
    BitBlock bit_6;
    BitBlock bit_7;
};


#endif // ICGREP_H
