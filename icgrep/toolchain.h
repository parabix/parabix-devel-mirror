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
#include <IDISA/idisa_builder.h>

IDISA::IDISA_Builder * GetIDISA_Builder(Module * m);

re::RE * regular_expression_passes(const Encoding encoding, re::RE * re_ast);

pablo::PabloFunction * re2pablo_compiler(const Encoding encoding, re::RE * re_ast);

void pablo_function_passes(pablo::PabloFunction * function);

ExecutionEngine * JIT_to_ExecutionEngine (Module * m);

void icgrep_Linking(Module * m, ExecutionEngine * e);

void PrintTotalCount();
re::CC * getParsedCodePointSet();
void setParsedCodePointSet();

#endif
