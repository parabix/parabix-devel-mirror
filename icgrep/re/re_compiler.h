/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_COMPILER_H
#define RE_COMPILER_H

#include "re_re.h"
#include "../utf_encoding.h"
#include "../llvm_gen.h"
#include <string>

namespace re {

struct processed_parsetree_results{
    RE* re;
    std::string remaining;
};

class RE_Compiler
{
public:
    RE_Compiler();
    LLVM_Gen_RetVal compile(bool show_compile_time,
                            bool ascii_only,
                            std::string basis_pattern,
                            std::string gensym_pattern,
                            UTF_Encoding encoding ,
                            std::string input_string);
};

}

#endif // RE_COMPILER_H
