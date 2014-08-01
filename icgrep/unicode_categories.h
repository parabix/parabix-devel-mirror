#ifndef UNICODE_CATEGORIES_H
#define UNICODE_CATEGORIES_H

#include <simd-lib/bitblock.hpp>
#include <simd-lib/carryQ.hpp>
#include <simd-lib/pabloSupport.hpp>
#include <simd-lib/s2p.hpp>
#include <simd-lib/buffer.hpp>
#include <simd-lib/bitblock_iterator.hpp>

class Unicode_Categories
{
public:
    //Unicode_Categories();
    static BitBlock getCategory(const char* category);
};

#endif // UNICODE_CATEGORIES_H
