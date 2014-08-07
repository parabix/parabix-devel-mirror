#include "unicode_categories.h"

Unicode_Categories::Unicode_Categories(){}

BitBlock Unicode_Categories::getCategory(Basis_bits &basis_bits, const char* category)
{
    if (std::strcmp(category, "Cc") == 0)
    {
        /*
        Cc cc;
        Struct_Cc cc_output;
        cc.do_block(basis_bits, cc_output);

        return cc_output.cc;
        */
    }
    else if (std::strcmp(category, "Lu") == 0)
    {
        /*
        Lu lu;
        Struct_Lu lu_output;
        lu.do_block(basis_bits, lu_output);

        return lu_output.cc;
        */
    }
    else if (std::strcmp(category, "Nd") == 0)
    {
        /*
        Nd nd;
        Struct_Nd nd_output;
        nd.do_block(basis_bits, nd_output);

        return nd_output.cc;
        */
    }
    else
    {
        BitBlock retVal = simd<1>::constant<0>();

        return retVal;
    }
}

