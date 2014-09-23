/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef LLVM_GENERATOR_HELPER_H
#define LLVM_GENERATOR_HELPER_H

#include <list>

class PabloS;
class PabloE;

class LLVM_Generator_Helper
{
public:
    static int CarryCount_PabloStatements(std::list<PabloS*> stmts);
private:
    static int CarryCount_PabloS(PabloS* stmt);
    static int CarryCount_PabloE(PabloE* expr);
    LLVM_Generator_Helper();
};

#endif // LLVM_GENERATOR_HELPER_H
