/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef LLVM_GENERATOR_HELPER_H
#define LLVM_GENERATOR_HELPER_H

#include <pablo/pe_pabloe.h>

class LLVM_Generator_Helper
{
public:
    static int CarryCount_PabloStatements(const pablo::ExpressionList &stmts);
private:
    static int CarryCount_PabloS(pablo::PabloE* stmt);
    static int CarryCount_PabloE(pablo::PabloE* expr);
    LLVM_Generator_Helper();
};

#endif // LLVM_GENERATOR_HELPER_H
