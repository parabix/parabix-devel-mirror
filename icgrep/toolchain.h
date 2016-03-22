/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef TOOLCHAIN_H
#define TOOLCHAIN_H

#include "utf_encoding.h"
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/Type.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>

#include <re/re_re.h>
#include <pablo/function.h>

re::RE * regular_expression_passes(const Encoding encoding, re::RE * re_ast);

pablo::PabloFunction * re2pablo_compiler(const Encoding encoding, re::RE * re_ast);

void pablo_function_passes(pablo::PabloFunction * function);

Module * grepIRGen(std::string moduleName, re::RE * re_ast, bool isNameExpression = false);

ExecutionEngine * JIT_to_ExecutionEngine (Module * m);

void icgrep_Linking(Module * m, ExecutionEngine * e);

re::CC * getParsedCodePointSet();
void setParsedCodePointSet();

void initResult(std::vector<std::string> filenames, const int n);
void PrintResult();

#endif
