/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef COMPILER_H
#define COMPILER_H

#include <utf_encoding.h>
#include <pablo/pablo_compiler.h>
#include <string>

namespace icgrep {

pablo::LLVM_Gen_RetVal compile(const Encoding encoding, const std::string input_string, const bool show_compile_time = false);

}

#endif
