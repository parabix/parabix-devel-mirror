#include "unicode_categories.h"

BitBlock Unicode_Categories::getCategory(const char* category)
{
    BitBlock test_vector = simd<1>::constant<0>();

    std::cout << "FROM Unicode_Categories, CATEGORY: " << category << std::endl;

    return test_vector;
}

