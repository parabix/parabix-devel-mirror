/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_COMPILER_H
#define RE_COMPILER_H

#include "hrtime.h"

//Regular Expressions
#include "re_re.h"
#include "re_alt.h"
#include "re_cc.h"
#include "re_end.h"
#include "re_rep.h"
#include "re_seq.h"
#include "re_start.h"

#include "rl_replimit.h"
#include "rl_unbounded.h"
#include "rl_upperbound.h"

#include "printer_pablos.h"
#include "printer_re.h"

#include "utf8_encoder.h"
#include "utf_encoding.h"

#include "parseresult.h"
#include "parsesuccess.h"
#include "parsefailure.h"
#include "re_parser.h"
#include "cc_compiler.h"
#include "cc_codegenobject.h"
#include "cc_compiler.h"

#include "pbix_compiler.h"
#include "symbol_generator.h"
#include "llvm_gen.h"

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <ctype.h>

struct processed_parsetree_results{
    RE* re;
    std::string remaining;
};

class RE_Compiler
{
public:
    RE_Compiler();
    LLVM_Gen_RetVal compile(bool show_compile_time, std::string basis_pattern, std::string gensym_pattern, UTF_Encoding encoding ,std::string input_string);
private:

};

#endif // RE_COMPILER_H
