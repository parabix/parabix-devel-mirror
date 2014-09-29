/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef COMPILER_H
#define COMPILER_H

#include "utf_encoding.h"
#include "llvm_gen.h"
#include <string>

namespace icgrep {

LLVM_Gen_RetVal compile(bool show_compile_time, bool ascii_only, std::string basis_pattern, std::string gensym_pattern, UTF_Encoding encoding, std::string input_string);

}

#endif
