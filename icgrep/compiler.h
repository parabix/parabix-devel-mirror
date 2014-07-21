/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef COMPILER_H
#define COMPILER_H

#include "compiler.h"
#include "re_parser.h"
#include "pbix_compiler.h"
#include "llvm_gen.h"

class Compiler
{
public:
    Compiler();
};

#endif // COMPILER_H

/*

  TODO: This will become the main driver for the application.  The parser, the cc compiler, the pbix compiler
  and the ir generator will all be called from here.

*/
