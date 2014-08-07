#include "unicode_categories.h"

BitBlock Unicode_Categories::getCategory(Basis_bits &basis_bits, const char* category)
{

    /*
    if (category == "Lu")
    {
        Lu lu;
        Struct_Lu lu_output;
        lu.do_block(basis_bits, lu_output);

        return lu_output.cc;
    }
    else if (std::strcmp(category,"Cc\n") != 0)
    {
        Cc cc;
        Struct_Cc cc_output;
        cc.do_block(basis_bits, cc_output);

        return cc_output.cc;
    }
    else
    {
        std::cout << "Category not implemented yet." << std::endl;
        BitBlock retVal = simd<1>::constant<0>();

        return retVal;
    }
    */

    if (std::strcmp(category, "Cc\n") != 0)
    {
        //Cc cc;
        //Struct_Cc cc_output;
        //cc.do_block(basis_bits, cc_output);

        //return cc_output.cc;
    }
    else if (std::strcmp(category, "Nd\n") !=0)
    {
        Nd nd;
        Struct_Nd nd_output;
        nd.do_block(basis_bits, nd_output);

        return nd_output.cc;
    }
    else
    {
        std::cout << "Category not implemented yet." << std::endl;
        BitBlock retVal = simd<1>::constant<0>();

        return retVal;
    }

}

