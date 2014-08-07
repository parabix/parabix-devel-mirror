#ifndef UNICODE_CATEGORIES_H
#define UNICODE_CATEGORIES_H

//#include "categories_Cc.h"
#include "categories_Nd.h"

#include <simd-lib/bitblock.hpp>
#include <simd-lib/carryQ.hpp>
#include <simd-lib/pabloSupport.hpp>
#include <simd-lib/s2p.hpp>
#include <simd-lib/buffer.hpp>
#include <simd-lib/bitblock_iterator.hpp>

#include <cstring>
#include <string>

//BitBlock None = simd<1>::constant<0>();

class Unicode_Categories
{
public:
    //Unicode_Categories();
    static BitBlock getCategory(Basis_bits &basis_bits, const char* category);
};

#endif // UNICODE_CATEGORIES_H
