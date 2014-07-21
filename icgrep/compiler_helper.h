/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef COMPILER_HELPER_H
#define COMPILER_HELPER_H

#include "compiler.h"
#include "cc_compiler.h"

class Compiler_Helper //: public CC_Compiler
{
public:
//protected:
    //Compiler_Helper();
    static PabloE* make_not(PabloE* expr);
    static PabloE* make_and(PabloE* expr1, PabloE* expr2);
    static PabloE* make_or(PabloE* expr1, PabloE* expr2);
    static PabloE* make_sel(PabloE* if_expr, PabloE* t_expr, PabloE* f_expr);
    static PabloE* make_xor(PabloE* expr1, PabloE* expr2);
    static bool equal_exprs(PabloE* expr1, PabloE* expr2);
private:
    Compiler_Helper();
};

#endif // COMPILER_HELPER_H
